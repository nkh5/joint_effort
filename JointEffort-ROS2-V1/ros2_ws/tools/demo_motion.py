#!/usr/bin/env python3
"""Small, bounded mock demonstration. Always verifies GenericSystem first."""
import argparse,time,math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import ListHardwareComponents
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer,TransformListener
from moveit_msgs.msg import ServoStatus

SUFFIXES=['base_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_yaw']
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('mode',choices=['joint','up']);p.add_argument('--arm',choices=['iron','wire'],default='iron')
p.add_argument('--joint',choices=SUFFIXES,default='base_yaw');p.add_argument('--speed',type=float,default=.05)
p.add_argument('--seconds',type=float,default=3.0)
a=p.parse_args()
if not 0<a.seconds<=3: p.error('--seconds must be greater than 0 and at most 3')
if not -.05<=a.speed<=.05:p.error('--speed must be in [-0.05,0.05] rad/s')
rclpy.init();n=Node('joint_effort_demo');state={};received=-float('inf');source_ns=-1

def feedback(msg):
 global received,source_ns
 expected={arm+'_'+suffix for arm in ['iron','wire'] for suffix in SUFFIXES}
 if len(msg.name)!=10 or len(msg.position)!=10 or set(msg.name)!=expected or not all(math.isfinite(v) for v in msg.position):
  state.clear();received=-float('inf');source_ns=-1;return
 state.clear();state.update(zip(msg.name,msg.position));received=time.monotonic();source_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
n.create_subscription(JointState,'/joint_states',feedback,1)
buffer=Buffer();listener=TransformListener(buffer,n)
servo_codes=set()
n.create_subscription(ServoStatus,f'/{a.arm}_servo/status',lambda m:servo_codes.add(m.code),10)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def call(kind,name,request):
 client=n.create_client(kind,name)
 if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing '+name+'. Start the model first.')
 future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=10)
 if not future.done():raise RuntimeError('Service did not answer: '+name)
 result=future.result();n.destroy_client(client);return result

def fresh():
 age=(n.get_clock().now().nanoseconds-source_ns)*1e-9
 return len(state)==10 and time.monotonic()-received<.3 and 0<=age<.3

try:
 hardware=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
 if len(hardware.component)!=1 or hardware.component[0].plugin_name!='mock_components/GenericSystem':
  raise RuntimeError('Refusing motion: this is not the expected GenericSystem mock.')
 deadline=time.monotonic()+10
 while not fresh() or not buffer.can_transform('world',f'{a.arm}_tcp',rclpy.time.Time()):
  if time.monotonic()>deadline:raise RuntimeError('No complete fresh joint feedback and TCP transform after 10 seconds. Check model_status.py.')
  spin(.05)
 enable=SetBool.Request();enable.data=False
 call(SetBool,'/task_bridge/enable',enable) # One writer per arm; stop old bridge input.
 if a.mode=='joint':
  request=ServoCommandType.Request();request.command_type=request.JOINT_JOG
  if not call(ServoCommandType,f'/{a.arm}_servo/switch_command_type',request).success:raise RuntimeError('Servo did not accept JointJog mode')
  publisher=n.create_publisher(JointJog,f'/{a.arm}_servo/delta_joint_cmds',1)
  def message(zero=False):
   m=JointJog();m.header.stamp=n.get_clock().now().to_msg();m.joint_names=[a.arm+'_'+s for s in SUFFIXES];m.velocities=[0.]*5
   if not zero:m.velocities[SUFFIXES.index(a.joint)]=a.speed
   m.duration=.02;return m
 else:
  request=GetParameters.Request();request.names=['selected_arm']
  selected=call(GetParameters,'/task_bridge/get_parameters',request).values[0].string_value
  if selected!=a.arm:raise RuntimeError(f'The task bridge selects {selected}. Restart launch with selected_arm:={a.arm}.')
  publisher=n.create_publisher(TwistStamped,f'/{a.arm}/task_velocity',1)
  def message(zero=False):
   m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='world'
   if not zero:m.twist.linear.z=.002 # 2 mm/s request, subject to Servo scaling.
   return m
 spin(.3)
 if a.mode=='up':
  enable.data=True
  if not call(SetBool,'/task_bridge/enable',enable).success:raise RuntimeError('Bridge could not enable. Check fresh feedback and Servo readiness.')
 before=state.copy();before_tf=buffer.lookup_transform('world',f'{a.arm}_tcp',rclpy.time.Time()).transform.translation
 servo_codes.clear()
 print(f'MOCK ONLY: {a.mode} command for {a.arm}, duration {a.seconds:.1f}s',flush=True)
 try:
  end=time.monotonic()+a.seconds
  while time.monotonic()<end:
   if not fresh():raise RuntimeError('Feedback became stale; stopping the demo.')
   publisher.publish(message());spin(.02)
 finally:
  # Stop explicit requests, then let Servo/bridge timeouts settle the mock.
  for _ in range(3):publisher.publish(message(True));spin(.02)
  if a.mode=='up':enable.data=False;call(SetBool,'/task_bridge/enable',enable)
 spin(.7)
 delta={s:round(state[a.arm+'_'+s]-before[a.arm+'_'+s],6) for s in SUFFIXES}
 print('Observed mock joint change (radians):',delta)
 print('Observed mock joint change (degrees):', {s:round(math.degrees(state[a.arm+'_'+s]-before[a.arm+'_'+s]),2) for s in SUFFIXES})
 after_tf=buffer.lookup_transform('world',f'{a.arm}_tcp',rclpy.time.Time()).transform.translation
 print('TCP movement from live TF (mm):', {axis:round(1000*(getattr(after_tf,axis)-getattr(before_tf,axis)),2) for axis in ['x','y','z']})
 if 4 in servo_codes:print('Servo slowed motion near a modeled collision. Try the opposite joint direction or inspect the clearance.')
 if 5 in servo_codes:print('Servo stopped motion for a modeled collision.')
 print('Command complete. The mock arm is holding its new position.')
except (RuntimeError,KeyboardInterrupt) as error:
 print('Demo stopped:',error)
 raise SystemExit(1)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
