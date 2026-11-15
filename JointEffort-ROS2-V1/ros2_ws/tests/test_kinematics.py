"""Arm kinematics and the tip-velocity -> joint-velocity step used by the task bridge. No ROS needed."""
import unittest

import numpy as np
import yaml

import _paths
from amw_bringup.arm import home_pose, load, robot_description
from amw_teleop.kinematics import Robot, differential_step, task_jacobian

SRC = _paths.WS / 'src'
ARM = load(SRC / 'amw_bringup/config/arm.yaml')
URDF = (SRC / 'amw_description/urdf/amw.urdf').read_text()
SRDF = (SRC / 'amw_description/srdf/amw.srdf').read_text()
# Same robot the bridge gets at runtime: amw.urdf with arm.yaml's limits.
R = Robot(robot_description(URDF, SRDF, ARM, 'mock'))
_T = yaml.safe_load((SRC / 'amw_bringup/config/teleop.yaml').read_text())['amw_teleop']['ros__parameters']
M = {k: v for k, v in _T.items() if k.startswith('task_') or k == 'joint_limit_margin_rad'}
M['max_joint_velocity_rad_s'] = ARM.max_speed
HOME = np.array([home_pose(SRDF)[n] for n in R.names('iron')])


class KinematicsTests(unittest.TestCase):
    def test_robot_reads_the_launch_description_limits(self):
        np.testing.assert_allclose(R.limits('iron'), [[j.lower, j.upper] for j in ARM])

    def test_jacobian_matches_finite_difference(self):
        rng = np.random.default_rng(41)
        lim = R.limits('iron')
        for _ in range(25):
            q = rng.uniform(lim[:, 0] + 0.05, lim[:, 1] - 0.05)
            t, j = R.fk('iron', q, True)
            for k in range(5):
                h = 1e-6
                dq = np.eye(5)[k] * h
                a, b = R.fk('iron', q + dq), R.fk('iron', q - dq)
                np.testing.assert_allclose((a[:3, 3] - b[:3, 3]) / (2 * h), j[:3, k], atol=1e-8)
                dr = (a[:3, :3] - b[:3, :3]) / (2 * h) @ t[:3, :3].T
                np.testing.assert_allclose([dr[2, 1], dr[0, 2], dr[1, 0]], j[3:, k], atol=1e-8)

    def test_home_pose_is_well_conditioned(self):
        t, j = R.fk('iron', HOME, True)
        self.assertGreater(np.linalg.svd(task_jacobian(t, j)[0], compute_uv=False)[-1], M['task_min_singular_value_m'])

    def test_small_move_from_home_is_ok(self):
        v, info = differential_step(R, 'iron', HOME, [0.005, 0, 0], [0, 0, 0], M)
        self.assertEqual(info['status'], 'ok')
        self.assertTrue(np.any(v != 0))

    def test_bad_inputs(self):
        for dt in [0, -1, float('nan')]:
            with self.assertRaises(ValueError):
                differential_step(R, 'iron', HOME, [0, 0, 0], [0, 0, 0], M, dt)
        with self.assertRaises(ValueError):
            differential_step(R, 'iron', HOME, [float('nan'), 0, 0], [0, 0, 0], M)

    def pushing_command(self, q, i):
        """A tip velocity whose unconstrained solution turns joint i positive (towards its upper limit)."""
        free = dict(M, joint_limit_margin_rad=-10.0)
        best = None
        for axis in range(3):
            for sign in (1, -1):
                cmd = np.zeros(3)
                cmd[axis] = 0.005 * sign
                v, _ = differential_step(R, 'iron', q, cmd, [0, 0, 0], free)
                if best is None or v[i] > best[0]:
                    best = (v[i], cmd)
        self.assertGreater(best[0], 0.01, 'no tip direction drives this joint')
        return best[1]

    def test_joint_at_limit_blocks_only_that_direction(self):
        lim = R.limits('iron')
        for i in (1, 2):
            q = HOME.copy()
            q[i] = lim[i, 1] - 0.01                      # 0.01 rad from the upper limit
            push = self.pushing_command(q, i)
            v, info = differential_step(R, 'iron', q, push, [0, 0, 0], M)
            self.assertEqual(info['status'], 'joint_limit')
            self.assertIn(i, info['blocked'])
            self.assertEqual(v[i], 0.0)
            self.assertTrue(np.any(v != 0), 'the other joints still help')
            v, info = differential_step(R, 'iron', q, -push, [0, 0, 0], M)
            self.assertNotIn(i, info['blocked'])
            self.assertLess(v[i], 0, 'moving away from the limit is allowed')

    def test_joint_past_its_limit_can_come_back(self):
        lim = R.limits('iron')
        q = HOME.copy()
        q[2] = lim[2, 1] - 0.01
        push = self.pushing_command(q, 2)               # found just inside, used just outside
        q[2] = lim[2, 1] + 0.02
        v, _ = differential_step(R, 'iron', q, push, [0, 0, 0], M)
        self.assertEqual(v[2], 0.0)
        v, _ = differential_step(R, 'iron', q, -push, [0, 0, 0], M)
        self.assertLess(v[2], 0)

    def test_singularity_slows_but_does_not_freeze(self):
        q = HOME.copy()
        best = None
        for elbow in np.linspace(R.limits('iron')[2, 0], R.limits('iron')[2, 1], 200):
            q[2] = elbow
            t, j = R.fk('iron', q, True)
            smin = np.linalg.svd(task_jacobian(t, j)[0], compute_uv=False)[-1]
            if best is None or smin < best[0]:
                best = (smin, elbow)
        q[2] = best[1]
        v, info = differential_step(R, 'iron', q, [0, 0, 0.005], [0, 0, 0], M)
        self.assertTrue(np.all(np.isfinite(v)))
        self.assertLessEqual(np.max(np.abs(v)), ARM.max_speed + 1e-12)
        if best[0] < M['task_min_singular_value_m']:
            self.assertEqual(info['status'], 'singularity')

    def test_joint_speed_cap_and_inputs_unchanged(self):
        linear, angular = np.array([1., 0, 0]), np.array([0., 1, 0])
        v, _ = differential_step(R, 'iron', HOME, linear, angular, M)
        self.assertLessEqual(np.max(abs(v)), ARM.max_speed + 1e-12)
        np.testing.assert_array_equal(linear, [1, 0, 0])
        np.testing.assert_array_equal(angular, [0, 1, 0])

    def test_roll_about_the_iron_is_left_free(self):
        t = R.fk('iron', HOME)
        v, _ = differential_step(R, 'iron', HOME, [0, 0, 0], t[:3, 0] * .01, M)
        np.testing.assert_allclose(v, 0, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
