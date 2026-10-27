"""Iron task protocol and feedback guards without controllers or motor access."""
import unittest,time
import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from joint_effort_iron.bridge import Bridge
class IronBridgeTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):rclpy.init()
 @classmethod
 def tearDownClass(cls):rclpy.shutdown()
 def setUp(self):self.node=Bridge()
 def tearDown(self):self.node.destroy_node()
 def command(self):
  m=TwistStamped();m.header.stamp=self.node.get_clock().now().to_msg();m.header.frame_id='world';m.twist.linear.z=.002;return m
 def feedback(self):
  m=JointState();m.header.stamp=self.node.get_clock().now().to_msg();m.name=self.node.names;m.position=[0.,-.5,1.2,0.,1.5];return m  # home pose (joint_3 limit is 0.7..2.3)
 def test_disabled_and_unverified_hardware_reject(self):
  self.node.receive(self.command());self.assertIsNone(self.node.command);self.node.feedback(self.feedback());req=SetBool.Request();req.data=True;res=self.node.enable(req,SetBool.Response());self.assertFalse(res.success);self.assertFalse(self.node.enabled)
 def test_stale_future_frame_nan_and_replay_rejected(self):
  self.node.enabled=True;m=self.command();self.node.receive(m);self.assertIsNotNone(self.node.command);self.node.receive(m);self.assertIsNone(self.node.command)
  for change in [lambda m:setattr(m.header,'frame_id','iron_tcp'),lambda m:setattr(m.header.stamp,'sec',m.header.stamp.sec-1),lambda m:setattr(m.header.stamp,'sec',m.header.stamp.sec+1),lambda m:setattr(m.twist.linear,'z',float('nan'))]:
   m=self.command();change(m);self.node.receive(m);self.assertIsNone(self.node.command)
 def test_incomplete_duplicate_nonfinite_and_out_of_bounds_feedback(self):
  self.node.feedback(self.feedback());self.assertIsNotNone(self.node.state)
  for change in [lambda m:setattr(m,'position',[0.]*4),lambda m:setattr(m,'name',['joint_1']*5),lambda m:setattr(m,'position',[float('nan'),0.,0.,0.,0.]),lambda m:setattr(m,'position',[1.,0.,0.,0.,0.])]:
   m=self.feedback();change(m);self.node.feedback(m);self.assertIsNone(self.node.state)
 def test_loss_of_feedback_latches_disabled(self):
  self.node.enabled=True;self.node.feedback(self.feedback());self.node.state_received=time.monotonic()-1;self.node.tick();self.assertFalse(self.node.enabled);self.assertIsNone(self.node.command)
if __name__=='__main__':unittest.main()
