"""Protocol rejection tests without controller output or motor access."""
import unittest
import time
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from joint_effort_robot.task_bridge import TaskBridge
class BridgeTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):self.n=TaskBridge()
 def tearDown(self):self.n.destroy_node()
 def command(self):
  m=TwistStamped();m.header.stamp=self.n.get_clock().now().to_msg();m.header.frame_id='world';m.twist.linear.z=.002;return m
 def test_disabled_rejects_input(self):
  self.n.on_command(self.command());self.assertIsNone(self.n.command)
 def test_wrong_frame_stale_nonfinite_and_replay_rejected(self):
  self.n.enabled=True
  msg=self.command();self.n.on_command(msg);self.assertIsNotNone(self.n.command)
  self.n.on_command(msg);self.assertIsNone(self.n.command)
  msg=self.command();msg.header.frame_id='iron_tcp';self.n.on_command(msg);self.assertIsNone(self.n.command)
  msg=self.command();msg.header.stamp.sec-=1;self.n.on_command(msg);self.assertIsNone(self.n.command)
  msg=self.command();msg.twist.linear.x=float('nan');self.n.on_command(msg);self.assertIsNone(self.n.command)
 def test_incomplete_duplicate_and_bad_feedback_rejected(self):
  msg=JointState();msg.header.stamp=self.n.get_clock().now().to_msg();msg.name=['iron_base_yaw'];msg.position=[0.]
  self.n.on_state(msg);self.assertIsNone(self.n.states)
  msg.name=self.n.robot.names('iron')+self.n.robot.names('wire');msg.position=[0.]*10
  self.n.on_state(msg);self.assertIsNotNone(self.n.states)
  msg.position[0]=float('nan');self.n.on_state(msg);self.assertIsNone(self.n.states)
  msg.position=[0.]*10;msg.name[1]=msg.name[0];self.n.on_state(msg);self.assertIsNone(self.n.states)
 def test_stale_feedback_latches_disabled(self):
  self.n.enabled=True;self.n.state_time=time.monotonic()-1;self.n.tick();self.assertFalse(self.n.enabled)
if __name__=='__main__':unittest.main()
