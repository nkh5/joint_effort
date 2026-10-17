"""Cold mock-only reduced task bridge. Disabled on startup, one selected arm.

Task commands are TwistStamped in world, interpreted as XYZ plus tool pointing,
with roll about local tool +X omitted. Output is JointJog to Servo, never direct
controller output. This is an experimental interface, not a hardware supervisor.
"""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import SetBool
from std_msgs.msg import String
from moveit_msgs.srv import ServoCommandType
from .kinematics import Robot, differential_step

class TaskBridge(Node):
    def __init__(self):
        super().__init__('task_bridge')
        p = Path(get_package_share_directory('joint_effort_robot'))
        self.robot = Robot(p/'urdf/joint_effort.urdf')
        self.model = json.loads((p/'config/model.json').read_text())
        self.arm = self.declare_parameter('selected_arm', 'iron').value
        if self.arm not in ['iron','wire']:
            raise ValueError('selected_arm must be iron or wire')
        self.feedback_timeout = self.declare_parameter('mock_feedback_timeout_s',0.3).value
        self.enabled = False
        self.was_enabled = False
        self.servo_clients = {a:self.create_client(ServoCommandType, f'/{a}_servo/switch_command_type') for a in ['iron','wire']}
        self.servo_futures = {}
        self.states = None
        self.state_time = self.command_time = -float('inf')
        self.command = None
        self.last_stamp = -1
        self.state_stamp = self.command_stamp = -1
        self.pub = self.create_publisher(JointJog, f'/{self.arm}_servo/delta_joint_cmds', 1)
        self.status = self.create_publisher(String, '~/status', 1)
        self.create_subscription(JointState,'/joint_states',self.on_state,1)
        self.create_subscription(TwistStamped,f'/{self.arm}/task_velocity',self.on_command,1)
        self.create_service(SetBool,'~/enable',self.enable)
        self.create_timer(0.02,self.tick)

    def fresh_stamp(self, stamp, timeout=0.12):
        ns = stamp.sec*1_000_000_000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds-ns)*1e-9
        return ns if 0 <= age <= timeout else None

    def on_state(self,msg):
        names = self.robot.names('iron')+self.robot.names('wire')
        if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):
            self.states = None
            return
        values = dict(zip(msg.name,msg.position))
        if self.fresh_stamp(msg.header.stamp,self.feedback_timeout) is None or any(n not in values for n in names):
            self.states = None
            return
        q = np.array([values[n] for n in names])
        bounds = np.vstack([self.robot.limits('iron'),self.robot.limits('wire')])
        if not np.isfinite(q).all() or np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):
            self.states = None
            return
        self.states = values
        self.state_time = time.monotonic()
        self.state_stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec

    def on_command(self,msg):
        stamp = self.fresh_stamp(msg.header.stamp)
        values = np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,
                           msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        if not self.enabled or stamp is None or stamp <= self.last_stamp or msg.header.frame_id != 'world' or not np.isfinite(values).all():
            self.command = None
            return
        self.command = values
        self.command_time = time.monotonic()
        self.last_stamp = stamp
        self.command_stamp = stamp

    def fresh(self, received, stamp, timeout=0.12):
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        return time.monotonic()-received <= timeout and 0 <= age <= timeout

    def enable(self,request,response):
        self.command = None
        self.last_stamp = -1
        servo_ready = self.arm in self.servo_futures and self.servo_futures[self.arm].done() and self.servo_futures[self.arm].result().success
        self.enabled = bool(request.data and servo_ready and self.states is not None and self.fresh(self.state_time,self.state_stamp,self.feedback_timeout))
        response.success = self.enabled == request.data
        response.message = 'enabled for cold mock testing' if self.enabled else 'disabled; fresh complete feedback required to enable'
        return response

    def tick(self):
        for arm,client in self.servo_clients.items():
            if arm not in self.servo_futures and client.service_is_ready():
                request = ServoCommandType.Request()
                request.command_type = ServoCommandType.Request.JOINT_JOG
                self.servo_futures[arm] = client.call_async(request)
        now = time.monotonic()
        velocity = np.zeros(5)
        status = 'disabled'
        if self.enabled:
            if self.states is None or not self.fresh(self.state_time,self.state_stamp,self.feedback_timeout):
                self.enabled = False
                self.command = None
                status = 'feedback_fault_reenable_required'
                self.get_logger().warning(f'Feedback timeout: receipt age {now-self.state_time:.4f}s, stamp age {(self.get_clock().now().nanoseconds-self.state_stamp)*1e-9:.4f}s; re-enable required')
            elif self.command is None or not self.fresh(self.command_time,self.command_stamp):
                self.command = None
                status = 'command_timeout'
            else:
                q = [self.states[n] for n in self.robot.names(self.arm)]
                velocity, info = differential_step(self.robot,self.arm,q,self.command[:3],self.command[3:],self.model)
                status = info['status']
        # Send zero only when enabled or when a fault/disable must stop a previous
        # request. The bridge is the sole writer to this selected Servo input.
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.joint_names = self.robot.names(self.arm)
        msg.velocities = velocity.tolist()
        msg.duration = 0.02
        if self.enabled or self.was_enabled:
            self.pub.publish(msg)
        self.was_enabled = self.enabled
        self.status.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    node = TaskBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
