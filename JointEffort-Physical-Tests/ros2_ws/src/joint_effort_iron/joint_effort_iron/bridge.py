"""Iron-only cold mock: fresh world XYZ/pointing commands -> JointJog -> Servo."""
import json,time,math,os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import ListHardwareComponents
from moveit_msgs.srv import ServoCommandType
from joint_effort_robot.kinematics import Robot,differential_step
class Bridge(Node):
 def __init__(self):
  super().__init__('iron_task_bridge');p=Path(get_package_share_directory('joint_effort_iron'))
  self.profile=self.declare_parameter('runtime_profile','mock').value
  robot_path=self.declare_parameter('robot_path',str(p/'urdf/iron.urdf')).value
  self.expected_plugin='mock_components/GenericSystem';self.physical=False
  self.model=json.loads((p/'config/model.json').read_text())
  if self.profile=='hardware':
   from joint_effort_hardware.configuration import read,PLUGIN,emulator_port
   config=read(self.declare_parameter('hardware_calibration','').value,p/'urdf/iron.urdf')
   emulated=self.declare_parameter('emulated_bus',False).value
   if emulated:emulator_port(config['serial_port'])
   if os.environ.get('ROS_DOMAIN_ID')!=('90' if emulated else '89'):raise RuntimeError('Wrong ROS domain for physical/emulated bridge')
   self.expected_plugin=PLUGIN;self.physical=True
   self.model['mock_max_joint_velocity_rad_s']=min(r['max_velocity_rad_s'] for r in config['joints'])
  elif self.profile!='mock':raise ValueError('Unknown runtime_profile')
  self.robot=Robot(robot_path);self.names=self.robot.names('iron')
  self.enabled=False;self.was_enabled=False;self.command=None;self.state=None;self.last_stamp=-1;self.command_received=self.state_received=-math.inf;self.state_stamp=self.command_stamp=-1;self.hw_future=self.servo_future=None
  self.hw_result=None;self.hw_received=self.hw_poll=-math.inf
  self.hw=self.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components');self.servo=self.create_client(ServoCommandType,'/iron_servo/switch_command_type')
  self.pub=self.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1);self.status=self.create_publisher(String,'~/status',10)
  self.create_subscription(JointState,'/joint_states',self.feedback,1);self.create_subscription(TwistStamped,'/iron/task_velocity',self.receive,1);self.create_service(SetBool,'~/enable',self.enable);self.create_timer(.02,self.tick)
 def stamp(self,stamp):return stamp.sec*10**9+stamp.nanosec
 def fresh(self,received,stamp,timeout):return time.monotonic()-received<=timeout and 0<=(self.get_clock().now().nanoseconds-stamp)*1e-9<=timeout
 def feedback(self,msg):
  self.state=None
  if len(msg.name)!=5 or len(msg.position)!=5 or set(msg.name)!=set(self.names):return
  values=dict(zip(msg.name,msg.position));q=np.array([values[n] for n in self.names]);bounds=self.robot.limits('iron');stamp=self.stamp(msg.header.stamp)
  if not np.isfinite(q).all() or np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]) or not self.fresh(time.monotonic(),stamp,.3):return
  self.state=values;self.state_stamp=stamp;self.state_received=time.monotonic()
 def receive(self,msg):
  stamp=self.stamp(msg.header.stamp);v=np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
  if not self.enabled or msg.header.frame_id!='world' or not self.fresh(time.monotonic(),stamp,.12) or stamp<=self.last_stamp or not np.isfinite(v).all():self.command=None;return
  self.command=v;self.command_stamp=stamp;self.command_received=time.monotonic();self.last_stamp=stamp
 def mock_ready(self):
  if self.hw_result is None or time.monotonic()-self.hw_received>.5:return False
  hw=self.hw_result.component
  return len(hw)==1 and hw[0].plugin_name==self.expected_plugin and (not self.physical or hw[0].state.id==3)
 def enable(self,req,res):
  self.command=None;self.last_stamp=-1
  ready=self.mock_ready() and self.servo_future is not None and self.servo_future.done() and not self.servo_future.exception() and self.servo_future.result().success and self.state is not None and self.fresh(self.state_received,self.state_stamp,.3)
  self.enabled=bool(req.data and ready);res.success=self.enabled==req.data;res.message='enabled for iron '+self.profile if self.enabled else 'disabled; verified hardware and fresh feedback required';return res
 def tick(self):
  now=time.monotonic()
  if self.hw_future is not None and self.hw_future.done():
   self.hw_result=None if self.hw_future.exception() else self.hw_future.result();self.hw_received=now;self.hw_future=None
  if self.hw_future is None and now-self.hw_poll>.2 and self.hw.service_is_ready():self.hw_future=self.hw.call_async(ListHardwareComponents.Request());self.hw_poll=now
  if self.servo_future is None and self.servo.service_is_ready() and self.mock_ready():
   req=ServoCommandType.Request();req.command_type=req.JOINT_JOG;self.servo_future=self.servo.call_async(req)
  v=np.zeros(5);status='disabled'
  if self.enabled:
   if not self.mock_ready():self.enabled=False;self.command=None;status='hardware_fault_reenable_required'
   elif self.state is None or not self.fresh(self.state_received,self.state_stamp,.3):self.enabled=False;self.command=None;status='feedback_fault_reenable_required'
   elif self.command is None or not self.fresh(self.command_received,self.command_stamp,.12):self.command=None;status='command_timeout'
   else:
    q=[self.state[n] for n in self.names];v,info=differential_step(self.robot,'iron',q,self.command[:3],self.command[3:],self.model);status=info['status']
  if self.enabled or self.was_enabled:
   msg=JointJog();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='world';msg.joint_names=self.names;msg.velocities=v.tolist();msg.duration=.02;self.pub.publish(msg)
  self.was_enabled=self.enabled;self.status.publish(String(data=status))
def main(args=None):
 rclpy.init(args=args);node=Bridge()
 try:rclpy.spin(node)
 except (KeyboardInterrupt,ExternalShutdownException):pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
