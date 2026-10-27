#!/usr/bin/env python3
import math,time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from moveit_msgs.srv import GetStateValidity
from std_msgs.msg import String
rclpy.init();n=Node('iron_prototype_status');state=None;status='not received'
def receive(msg):
 global state
 state=msg
n.create_subscription(JointState,'/joint_states',receive,1)
def bridge(msg):
 global status
 status=msg.data
n.create_subscription(String,'/iron_task_bridge/status',bridge,1)
def call(kind,path):
 c=n.create_client(kind,path)
 if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Start scripts/run_iron.sh first; missing '+path)
 f=c.call_async(kind.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=10);n.destroy_client(c)
 if not f.done():raise RuntimeError('Service timeout')
 return f.result()
try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components');print('Hardware:',', '.join(c.plugin_name for c in hw.component))
 for c in call(ListControllers,'/controller_manager/list_controllers').controller:print('Controller:',c.name,'→',c.state)
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if state is None:raise RuntimeError('No joint feedback')
 age=(n.get_clock().now().nanoseconds-state.header.stamp.sec*10**9-state.header.stamp.nanosec)*1e-9;print('Feedback:',len(state.name),'joints; age',round(age*1000,1),'ms');print('Joint degrees:',{name:round(math.degrees(v),2) for name,v in zip(state.name,state.position)});print('Task bridge:',status)
 c=n.create_client(GetStateValidity,'/check_state_validity');assert c.wait_for_service(timeout_sec=10);req=GetStateValidity.Request();req.robot_state.joint_state=state;f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10);print('Modeled collision:', 'CLEAR' if f.result().valid else 'COLLISION')
finally:n.destroy_node();rclpy.shutdown()
