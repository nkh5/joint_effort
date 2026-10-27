"""Exercise the forwarding boundary with malformed/stale/mismatched ROS messages."""
import math
import time
import unittest
from types import SimpleNamespace
from geometry_msgs.msg import TwistStamped
from joint_effort_input.forwarder import Forwarder


class Boundary:
    def __init__(self):
        self.clock = 100000000000; self.command_timeout = .12
        self.phase = 'active'; self.last_raw_stamp = self.min_raw_stamp = -1
        self.active_start = time.monotonic()-1.; self.last_command = -math.inf
        self.failures = []; self.outputs = []
        self.input = SimpleNamespace(axes=[0., 0., .8], mode='translation', level=20,
                                     allowed=lambda now: True)
    def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.clock))
    def ns(self, stamp): return stamp.sec*10**9+stamp.nanosec
    def fail(self, reason): self.failures.append(reason)
    def ready(self, now): return True
    def publish_velocity(self, values): self.outputs.append(values)


def message():
    msg = TwistStamped(); msg.header.stamp.sec = 100; msg.header.frame_id = 'world'
    msg.twist.linear.z = 0.008
    return msg


class ForwardingTests(unittest.TestCase):
    def test_fresh_matching_raw_scaled_and_old_axis_sample_ignored(self):
        boundary = Boundary(); msg = message(); Forwarder.raw(boundary, msg)
        self.assertEqual(len(boundary.outputs), 1); self.assertAlmostEqual(boundary.outputs[0][2], 0.016)
        self.assertTrue(all(v == 0. for i, v in enumerate(boundary.outputs[0]) if i != 2))
        self.assertFalse(boundary.failures)
        boundary = Boundary(); msg.twist.linear.x = .0015
        Forwarder.raw(boundary, msg)
        self.assertFalse(boundary.outputs); self.assertFalse(boundary.failures)

    def test_wrong_frame_old_future_nan_and_duplicate_never_forward(self):
        for case in ['frame', 'old', 'future', 'nan', 'replay']:
            boundary = Boundary(); msg = message()
            if case == 'frame': msg.header.frame_id = 'base_link'
            elif case == 'old': msg.header.stamp.sec = 99
            elif case == 'future': msg.header.stamp.sec = 101
            elif case == 'nan': msg.twist.angular.z = math.nan
            elif case == 'replay': boundary.last_raw_stamp = boundary.clock
            Forwarder.raw(boundary, msg)
            self.assertTrue(boundary.failures, case); self.assertFalse(boundary.outputs, case)


if __name__ == '__main__': unittest.main()
