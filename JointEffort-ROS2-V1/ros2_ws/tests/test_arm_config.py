"""arm.yaml loading, checks, in-place edits and the generated robot description. No ROS needed."""
import math
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import _paths
from amw_bringup import arm as A

SRC = _paths.WS / 'src'
ARM_YAML = SRC / 'amw_bringup/config/arm.yaml'
URDF = (SRC / 'amw_description/urdf/amw.urdf').read_text()
SRDF = (SRC / 'amw_description/srdf/amw.srdf').read_text()
K = 4096 / (2 * math.pi)


class ArmYamlTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / 'arm.yaml'
        shutil.copy(ARM_YAML, self.path)

    def tearDown(self):
        self.dir.cleanup()

    def edit(self, joint, **values):
        A.update_values(self.path, joint, values)

    def test_active_file_loads(self):
        arm = A.load(ARM_YAML)
        self.assertEqual([j.name for j in arm], A.JOINTS)
        self.assertEqual(sorted(j.motor_id for j in arm), [1, 2, 3, 4, 5])

    def test_migration_reports_the_same_angles_as_the_old_driver(self):
        """Old: angle = (center_steps - raw)/K. New: dir*(raw - zero)/K. Must agree everywhere."""
        old = yaml.safe_load((_paths.WS / 'archive/sts_calibration_v1_before_cleanup.yaml').read_text())['joints']
        for j in A.load(ARM_YAML):
            for raw in range(0, 4096, 13):
                self.assertAlmostEqual(j.angle(raw), (old[j.name]['center_steps'] - raw) / K, places=12)
            self.assertAlmostEqual(j.reference, old[j.name]['datum_offset_rad'], places=4)

    def test_flipping_dir_is_the_only_change_needed(self):
        arm = A.load(self.path)
        raw = 2300
        before = arm.joints['joint_2'].angle(raw)
        self.edit('joint_2', dir=-arm.joints['joint_2'].dir)
        after = A.load(self.path).joints['joint_2'].angle(raw)
        self.assertAlmostEqual(after, -before)

    def test_zero_for_round_trips(self):
        j = A.load(ARM_YAML).joints['joint_3']
        z = j.zero_for(raw=3100, angle=j.reference)
        j.zero = z
        self.assertLess(abs(j.angle(3100) - j.reference), 0.5 / K)

    def test_update_keeps_comments_and_other_joints(self):
        text_before = self.path.read_text()
        self.edit('joint_3', zero=2900, lower_deg=-80.5)
        text = self.path.read_text()
        self.assertIn('# elbow', text)
        self.assertIn('Joint turns the wrong way in RViz?', text)
        arm = A.load(self.path)
        self.assertEqual(arm.joints['joint_3'].zero, 2900)
        self.assertAlmostEqual(math.degrees(arm.joints['joint_3'].lower), -80.5)
        self.assertEqual(arm.joints['joint_2'].zero, A.load(ARM_YAML).joints['joint_2'].zero)
        self.assertEqual(len(text.splitlines()), len(text_before.splitlines()))

    def test_errors_name_the_joint(self):
        cases = [('joint_4', {'dir': 2}, 'joint_4.dir'),
                 ('joint_1', {'motor_id': 2}, 'motor_id values must all be different'),
                 ('joint_5', {'lower_deg': 140.0}, 'joint_5: lower_deg must be less'),
                 ('joint_3', {'zero': 4000}, 'joint_3: its travel'),
                 ('joint_2', {'acceleration': 0}, 'joint_2.acceleration')]
        for joint, change, message in cases:
            shutil.copy(ARM_YAML, self.path)
            self.edit(joint, **change)
            with self.assertRaises(A.ArmConfigError) as ctx:
                A.load(self.path)
            self.assertIn(message, str(ctx.exception))


class DescriptionTest(unittest.TestCase):
    arm = A.load(ARM_YAML)

    def control(self, hardware, readonly=False):
        root = ET.fromstring(A.robot_description(URDF, SRDF, self.arm, hardware, readonly))
        return root, root.find('ros2_control')

    def test_limits_come_from_arm_yaml(self):
        root, _ = self.control('mock')
        for j in self.arm:
            limit = root.find(f"joint[@name='{j.name}']/limit")
            self.assertAlmostEqual(float(limit.get('lower')), j.lower)
            self.assertAlmostEqual(float(limit.get('upper')), j.upper)
            self.assertAlmostEqual(float(limit.get('velocity')), self.arm.max_speed)

    def test_mock_starts_at_srdf_home(self):
        _, rc = self.control('mock')
        home = A.home_pose(SRDF)
        self.assertEqual(rc.find('hardware/plugin').text, A.MOCK_PLUGIN)
        for j in rc.findall('joint'):
            self.assertAlmostEqual(float(j.find("state_interface[@name='position']/param").text), home[j.get('name')])

    def test_real_has_direction_zero_range_and_acceleration(self):
        _, rc = self.control('real')
        self.assertEqual(rc.find('hardware/plugin').text, A.STS_PLUGIN)
        for j in self.arm:
            params = {p.get('name'): p.text for p in rc.find(f"joint[@name='{j.name}']").findall('param')}
            self.assertEqual(int(params['position_center_steps']), j.zero)
            self.assertEqual(int(params['encoder_direction']), j.dir)
            self.assertEqual(int(params['acceleration']), j.acceleration)
            lo, hi = self.arm.driver_range(j)
            self.assertAlmostEqual(float(params['min_position']), lo)
            self.assertAlmostEqual(float(params['max_position']), hi)

    def test_readonly_has_no_command_interface(self):
        _, rc = self.control('real', readonly=True)
        self.assertEqual(rc.findall('.//command_interface'), [])
        self.assertEqual([p for p in rc.iter('param') if p.get('name') in ('min_position', 'max_position')], [])

    def test_home_outside_limits_is_rejected(self):
        tight = A.load(ARM_YAML)
        tight.joints['joint_3'].upper = -1.3      # home has joint_3 at -0.61 (-35 deg)
        with self.assertRaises(A.ArmConfigError):
            A.robot_description(URDF, SRDF, tight)

    def test_planning_limits(self):
        limits = A.planning_limits(self.arm)['joint_limits']
        self.assertEqual(sorted(limits), A.JOINTS)
        self.assertAlmostEqual(limits['joint_1']['max_velocity'], 0.15, places=5)


if __name__ == '__main__':
    unittest.main()
