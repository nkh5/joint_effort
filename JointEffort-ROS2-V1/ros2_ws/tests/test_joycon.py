"""Joy-Con event parsing (amw_teleop.joycon) and the ZL pointing maths (amw_teleop.hand).
No ROS, evdev or Joy-Con needed."""
import math
import unittest

import numpy as np

import _paths  # noqa: F401  (adds src packages to sys.path)
from amw_teleop.hand import G, Orientation, exp_rotation, hand_tilt, iron_target, point_velocity, swing
from amw_teleop.joycon import (ABS_HAT0Y, ABS_RX, ABS_X, ABS_Y, BUTTONS, EV_ABS, EV_KEY, EV_MSC, EV_SYN,
                               MSC_TIMESTAMP, SYN_REPORT, Calibration, Clock, ImuStream, Pad)


def sample(stream, gyro_counts, stamp_us, arrival=0.0):
    for i, v in enumerate(gyro_counts):
        stream.event(EV_ABS, ABS_RX + i, v, arrival)
    stream.event(EV_MSC, MSC_TIMESTAMP, stamp_us, arrival)
    return stream.event(EV_SYN, SYN_REPORT, 0, arrival)


class PadTest(unittest.TestCase):
    def test_buttons_by_name(self):
        pad = Pad()
        for code, name in [(310, 'L'), (312, 'ZL'), (544, 'DPAD_UP'), (545, 'DPAD_DOWN'), (547, 'DPAD_RIGHT')]:
            pad.event(EV_KEY, code, 1)
            self.assertEqual(pad.buttons()[BUTTONS.index(name)], 1, name)
            pad.event(EV_KEY, code, 0)
        self.assertEqual(sum(pad.buttons()), 0)

    def test_stick_scaled_up_is_positive_and_rest_removed(self):
        pad = Pad(32767, invert_y=True)
        pad.event(EV_ABS, ABS_X, 32767)
        pad.event(EV_ABS, ABS_Y, -32767)            # hid-nintendo: up is negative
        self.assertEqual(pad.stick(), [1.0, 1.0])
        pad.rest = [0.1, 0.0]
        pad.event(EV_ABS, ABS_X, 3277)
        self.assertAlmostEqual(pad.stick()[0], 0.0, places=3)

    def test_pro_controller_hat_is_the_dpad(self):
        pad = Pad()
        pad.event(EV_ABS, ABS_HAT0Y, -1)
        self.assertEqual(pad.buttons()[BUTTONS.index('DPAD_UP')], 1)
        pad.event(EV_ABS, ABS_HAT0Y, 0)
        self.assertEqual(sum(pad.buttons()), 0)


class ImuTest(unittest.TestCase):
    def test_units_and_device_clock(self):
        s = ImuStream()
        t0, _, g = sample(s, [14247, 0, -14247], 1000)
        self.assertAlmostEqual(g[0], math.radians(1.0))
        self.assertAlmostEqual(g[2], -math.radians(1.0))
        t1, _, _ = sample(s, [0, 0, 0], 1000 + 4666)
        self.assertAlmostEqual(t1 - t0, 0.004666)

    def test_timestamp_wrap(self):
        s = ImuStream()
        t0, _, _ = sample(s, [0, 0, 0], 2 ** 32 - 1000)
        t1, _, _ = sample(s, [0, 0, 0], 3666)
        self.assertAlmostEqual(t1 - t0, 0.004666)

    def test_clock_keeps_the_smallest_delay(self):
        c = Clock()
        self.assertAlmostEqual(c.to_ros(0.000, 100.015), 100.015)     # first sample of a bundle, 15 ms late
        self.assertAlmostEqual(c.to_ros(0.010, 100.015), 100.015)     # last of the bundle: offset 100.005
        self.assertAlmostEqual(c.to_ros(0.020, 100.040), 100.025)     # a late arrival keeps that offset
        self.assertAlmostEqual(c.to_ros(5.000, 106.000), 106.000)     # a jump (> 0.2 s) re-syncs


class CalibrationTest(unittest.TestCase):
    def test_still_joycon_gives_bias_and_rest(self):
        cal = Calibration(seconds=1.0)
        bias = [0.01, -0.01, 0.005]
        done = False
        for k in range(250):
            done = cal.add(k * 0.005, bias, [0.05, -0.03])
        self.assertTrue(done)
        np.testing.assert_allclose(cal.bias, bias)
        np.testing.assert_allclose(cal.rest, [0.05, -0.03])

    def test_motion_restarts_and_held_stick_is_not_a_rest(self):
        cal = Calibration(seconds=1.0)
        for k in range(100):
            cal.add(k * 0.005, [0.0, 0.0, 0.0], [0.6, 0.0])
        cal.add(0.5, [0.5, 0.0, 0.0], [0.6, 0.0])          # a twitch
        self.assertFalse(cal.done)
        for k in range(101, 400):
            cal.add(k * 0.005, [0.0, 0.0, 0.0], [0.6, 0.0])
        self.assertTrue(cal.done)
        self.assertEqual(cal.rest, [0.0, 0.0])


class NodeNamesTest(unittest.TestCase):
    def test_joycon_node_does_not_shadow_rclpy_node(self):
        """rclpy.Node uses these names itself; redefining one crashes the node at start (as `handle` did)."""
        import re
        source = (_paths.WS / 'src/amw_teleop/amw_teleop/joycon.py').read_text()
        ours = set(re.findall(r'self\.(\w+)\s*=', source)) | set(re.findall(r'^\s+def (\w+)\(self', source, re.M))
        rclpy_node = {'handle', 'executor', 'context', 'default_callback_group', 'publishers', 'subscriptions',
                      'clients', 'services', 'timers', 'guards', 'waitables', 'name', 'clock', 'namespace'}
        self.assertEqual(ours & rclpy_node, set())


class HandTest(unittest.TestCase):
    def test_gravity_pulls_a_wrong_tilt_back(self):
        o = Orientation(gravity_gain=2.0)
        o.add([0, 0, 0], [0, 0, G], 0.0)                       # flat: body z up
        np.testing.assert_allclose(o.r, np.eye(3), atol=1e-9)
        o.r = exp_rotation([0.2, 0.0, 0.0])                    # pretend 11 deg of drift
        for k in range(1, 2001):                               # 10 s still, at 200 Hz
            o.add([0, 0, 0], [0, 0, G], k * 0.005)
        np.testing.assert_allclose(o.r @ [0, 0, 1], [0, 0, 1], atol=1e-3)

    def test_gyro_rotation_is_tracked(self):
        o = Orientation(gravity_gain=0.0)
        o.add([0, 0, 0], [0, 0, G], 0.0)
        for k in range(1, 201):                                # 1 s at 0.5 rad/s about body x
            o.add([0.5, 0, 0], [0, 0, G], k * 0.005)
        np.testing.assert_allclose(o.r, exp_rotation([0.5, 0, 0]), atol=1e-9)

    def test_twist_about_the_long_axis_is_not_a_tilt(self):
        ref = exp_rotation([0.3, -0.2, 0.1])
        np.testing.assert_allclose(hand_tilt(ref, ref @ exp_rotation([0, 0.7, 0]), long_axis=1), 0, atol=1e-9)
        tilt = hand_tilt(ref, ref @ exp_rotation([0.25, 0, 0]), long_axis=1)
        np.testing.assert_allclose(tilt, [0.25, 0, 0], atol=1e-9)

    def test_iron_target_scaled_limited(self):
        np.testing.assert_allclose(iron_target([0, 0, 0]), [0, 0, -1])
        d = iron_target([0.4, 0.0, 9.9], axes=(0, 1), signs=(1.0, 1.0), scale=0.5)   # z component ignored
        np.testing.assert_allclose(swing(np.array([0, 0, -1.0]), d), [0.2, 0, 0], atol=1e-9)
        d = iron_target([0, 0, 4.0], axes=(0, 2), signs=(1.0, -1.0), scale=1.0, max_angle=math.radians(60))
        np.testing.assert_allclose(swing(np.array([0, 0, -1.0]), d), [0, -math.radians(60), 0], atol=1e-9)

    def test_point_velocity_is_perpendicular_to_the_iron(self):
        d = np.array([0.3, 0.1, -0.9]) / np.linalg.norm([0.3, 0.1, -0.9])
        w = point_velocity(d, [0, 0, -1.0], 2.0)
        self.assertAlmostEqual(float(w @ d), 0.0)
        self.assertAlmostEqual(float(np.linalg.norm(w)), 2.0 * math.acos(-d[2]))


if __name__ == '__main__':
    unittest.main()
