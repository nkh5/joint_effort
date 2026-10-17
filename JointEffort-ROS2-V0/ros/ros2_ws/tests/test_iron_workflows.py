"""Meaningful regressions: spline overshoot and independent encoder fits."""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from iron_trajectory import audit, coefficients, retime
from iron_calibration import fit
from iron_motion import Motion
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import TransformStamped


def point(q, t, velocity=0., acceleration=0.):
    return NS(positions=[q]*5, velocities=[velocity]*5, accelerations=[acceleration]*5,
              time_from_start=NS(sec=int(t), nanosec=int(t%1*1e9)))


class TrajectoryTests(unittest.TestCase):
    def test_actual_quintic_midpoint_and_peak(self):
        trajectory = NS(joint_names=list('abcde'), points=[point(0, 0), point(.1, 2)])
        samples, report = audit(trajectory, list('abcde'), np.array([[-.5, .5]]*5), .15, .3)
        self.assertAlmostEqual(report['max_speed_rad_s'], .09375)
        self.assertAlmostEqual(report['max_acceleration_rad_s2'], 5.7735026919*.1/4)
        self.assertTrue(any(abs(t-1)<1e-8 and np.max(abs(q-.05))<1e-8 for t, q in samples))

    def test_interior_overshoot_rejected_despite_clear_endpoints(self):
        trajectory = NS(joint_names=list('abcde'), points=[point(0, 0, 3), point(0, 1, -3)])
        with self.assertRaisesRegex(ValueError, 'travel'):
            audit(trajectory, list('abcde'), np.array([[-.5, .5]]*5), 10, 50)

    def test_nonincreasing_nan_and_dynamics_rejected(self):
        with self.assertRaises(ValueError):
            coefficients(point(0, 0), point(.1, 0), 0)
        for endpoint in [point(float('nan'), 1), point(.1, .01)]:
            with self.assertRaises(ValueError):
                audit(NS(joint_names=list('abcde'), points=[point(0, 0), endpoint]), list('abcde'), np.array([[-.5, .5]]*5), .15, .3)

    def test_retiming_bounds_dynamics_and_preserves_spline_geometry(self):
        trajectory = NS(joint_names=list('abcde'), points=[point(0, 0, .2, .3), point(.1, .2)])
        before = coefficients(*trajectory.points, .2)
        timing = retime(trajectory, list('abcde'), np.array([[-.5, .5]]*5), .075, .15)
        self.assertGreater(timing['time_stretch_factor'], 1)
        self.assertLessEqual(timing['max_speed_rad_s'], .075)
        self.assertLessEqual(timing['max_acceleration_rad_s2'], .15)
        after = coefficients(*trajectory.points, timing['duration_s'])
        np.testing.assert_allclose(before, after, atol=1e-8)


class CalibrationTests(unittest.TestCase):
    def rows(self, sign=1, ratio=1, zero=2075):
        return [{'joint': 'joint_1', 'angle_deg': angle,
                 'encoder_counts': round(zero+sign*ratio*4096*math.radians(angle)/(2*math.pi))}
                for angle in [-10, -5, 0, 5, 10]]

    def test_known_zero_reverse_sign_and_external_ratio(self):
        report = fit(self.rows(sign=-1, ratio=2))
        joint = report['joints']['joint_1']
        self.assertAlmostEqual(joint['fitted_zero_counts'], 2075)
        self.assertEqual(joint['fitted_sign'], -1)
        self.assertAlmostEqual(joint['fitted_external_ratio'], 2, places=2)
        self.assertFalse(report['calibration_confirmed']); self.assertFalse(report['hardware_enabled'])
        self.assertEqual(len(report['missing_joints']), 4)

    def test_missing_noisy_wrapped_nonfinite_measurements_rejected(self):
        for rows in [[], self.rows()[:3]]:
            with self.assertRaises(ValueError): fit(rows)
        for bad in [float('nan'), 4096, 2400, 2048.5]:
            rows = self.rows(); rows[2]['encoder_counts'] = bad
            with self.assertRaises(ValueError): fit(rows)
        with self.assertRaisesRegex(ValueError, 'wrap'):
            fit(self.rows(ratio=10))


class SupervisorTests(unittest.TestCase):
    def test_state_only_scene_diff_does_not_stop_but_geometry_does(self):
        monitor = NS(scene_parts={}, stop_reason=None)
        Motion.scene(monitor, PlanningScene())
        monitor.scene_parts['ready'] = True
        state_only = PlanningScene(is_diff=True)
        state_only.robot_state.joint_state.name = ['joint_1']
        state_only.robot_state.joint_state.position = [.01]
        Motion.scene(monitor, state_only)
        self.assertIsNone(monitor.stop_reason)
        changed = PlanningScene(is_diff=True)
        changed.world.collision_objects = [CollisionObject(id='new_obstacle')]
        Motion.scene(monitor, changed)
        self.assertIn('changed', monitor.stop_reason)

    def test_stop_request_latches_and_never_auto_resets(self):
        monitor = NS(stop_reason=None)
        response = NS(success=False, message='')
        Motion.stop(monitor, None, response)
        self.assertTrue(response.success)
        with self.assertRaisesRegex(RuntimeError, 'User requested'):
            Motion.guard(monitor)

    def test_transform_timestamps_and_order_ignored_but_geometry_change_stops(self):
        monitor = NS(scene_parts={}, stop_reason=None)
        scene = PlanningScene()
        first = TransformStamped(child_frame_id='fixture'); first.header.frame_id = 'world'
        first.transform.rotation.w = 1.
        second = TransformStamped(child_frame_id='table'); second.header.frame_id = 'world'
        second.transform.rotation.w = 1.
        scene.fixed_frame_transforms = [first, second]
        Motion.scene(monitor, scene); monitor.scene_parts['ready'] = True
        first.header.stamp.sec = 30; scene.fixed_frame_transforms = [second, first]
        Motion.scene(monitor, scene); self.assertIsNone(monitor.stop_reason)
        first.transform.translation.x = .1; scene.fixed_frame_transforms = [first, second]
        Motion.scene(monitor, scene); self.assertIn('transforms changed', monitor.stop_reason)

    def test_already_at_target_is_checked_no_motion(self):
        monitor = NS(guard=lambda: None)
        target = np.zeros(5)
        self.assertIsNone(Motion.plan(monitor, target, target))
        np.testing.assert_array_equal(monitor.noop_target, target)
        monitor.q = lambda: target; monitor.valid = lambda values: True
        self.assertTrue(Motion.check(monitor, None)['no_motion'])
        monitor.valid = lambda values: False
        with self.assertRaisesRegex(RuntimeError, 'collision'):
            Motion.check(monitor, None)


if __name__ == '__main__':
    unittest.main()
