#!/usr/bin/env python3
"""Publish CAD-review joint states only, on a separate topic/domain."""
import math,xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
class Pose(Node):
 def __init__(self):
  super().__init__('cad_review_pose')
  root=ET.parse(Path(get_package_share_directory('joint_effort_cad_review'))/'urdf/cad_arm.urdf').getroot()
  self.bounds={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in root.findall('joint') if j.get('type')=='revolute'}
  for name in self.bounds:self.declare_parameter(name,0.)
  checked=self.validate([self.get_parameter(n) for n in self.bounds])
  if not checked.successful:raise ValueError(checked.reason)
  self.add_on_set_parameters_callback(self.validate)
  self.pub=self.create_publisher(JointState,'/cad_review/joint_states',10)
  self.create_timer(.05,self.tick)
 def validate(self,params):
  for p in params:
   if p.name in self.bounds:
    lo,hi=self.bounds[p.name]
    if not isinstance(p.value,float) or not math.isfinite(p.value) or not lo<=p.value<=hi:return SetParametersResult(successful=False,reason='Review angles must be finite radians within draft slider bounds.')
  return SetParametersResult(successful=True)
 def tick(self):
  msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=list(self.bounds)
  msg.position=[self.get_parameter(n).value for n in msg.name];self.pub.publish(msg)
rclpy.init();node=Pose()
try:rclpy.spin(node)
except (KeyboardInterrupt,ExternalShutdownException):pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
