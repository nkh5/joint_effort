#!/usr/bin/env python3
"""Readable, read-only summary for a running mock launch."""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters
rclpy.init();n=Node('joint_effort_status');state={};status='not received';stamp=-1

def on_state(m):
 global stamp
 state.update(zip(m.name,m.position));stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

def on_status(m):
 global status
 status=m.data
n.create_subscription(JointState,'/joint_states',on_state,1)
n.create_subscription(String,'/task_bridge/status',on_status,1)
def call(kind,name,request):
 c=n.create_client(kind,name)
 if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Start the model first; unavailable service '+name)
 f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
 if not f.done():raise RuntimeError('Service did not respond: '+name)
 result=f.result();n.destroy_client(c);return result
try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
 controllers=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
 end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 print('Hardware:',', '.join(c.plugin_name for c in hw.component))
 for c in controllers.controller:print('Controller:',c.name,'→',c.state)
 print('Joint feedback:',len(state),'joints; age',round((n.get_clock().now().nanoseconds-stamp)/1e6,1),'ms')
 request=GetParameters.Request();request.names=['selected_arm']
 arm=call(GetParameters,'/task_bridge/get_parameters',request).values[0].string_value
 print('Selected task arm:',arm,'; bridge state:',status)
 if len(state)==10:
  req=GetStateValidity.Request();req.robot_state.joint_state.name=list(state);req.robot_state.joint_state.position=list(state.values())
  res=call(GetStateValidity,'/check_state_validity',req)
  print('Current modeled collision check:', 'CLEAR' if res.valid else 'COLLISION')
  for c in res.contacts:print('  Contact:',c.contact_body_1,'with',c.contact_body_2)
except RuntimeError as e:
 print(e);raise SystemExit(1)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
