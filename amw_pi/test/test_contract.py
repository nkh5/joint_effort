import tempfile
import math
from pathlib import Path
import unittest
from amw_pi_bridge.contract import (validate_config, radians_to_counts, counts_to_radians,
                                    preview_trajectory, sample_all, NAMES, read_config)
from amw_pi_bridge.demo import fixture, FakeBus


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.config = fixture()
        self.row = self.config['joints'][0]

    def test_blank_calibration_refused(self):
        path = Path(__file__).parents[1]/'config/iron_calibration.json'
        with self.assertRaises(ValueError):
            read_config(path, Path(__file__).parents[2]/'joint_effort_iron/urdf/iron.urdf')

    def test_duplicate_ids_refused(self):
        self.config['joints'][1]['motor_id'] = 1
        with self.assertRaises(ValueError): validate_config(self.config)

    def test_missing_direction_refused(self):
        self.row['direction'] = None
        with self.assertRaises(ValueError): validate_config(self.config)

    def test_bool_id_refused(self):
        self.row['motor_id'] = True
        with self.assertRaises(ValueError): validate_config(self.config)

    def test_count_direction_and_ratio(self):
        self.row.update(direction=1, gear_ratio=2.)
        self.assertAlmostEqual(counts_to_radians(self.row, 2560), math.pi/8)
        self.assertEqual(radians_to_counts(self.row, math.pi/8), 2560)
        self.row['direction'] = -1
        self.assertAlmostEqual(counts_to_radians(self.row, 2560), -math.pi/8)

    def test_zero_is_joint_zero_not_absolute_encoder_degrees(self):
        self.assertEqual(counts_to_radians(self.row, 2048), 0)

    def test_round_trip_quantization(self):
        for direction in [-1, 1]:
            self.row['direction'] = direction
            for q in [-.8, -.01, 0., .01, .8]:
                raw = radians_to_counts(self.row, q)
                self.assertLessEqual(abs(q-counts_to_radians(self.row, raw)), math.pi/4096)

    def test_wrap_rejected(self):
        self.row.update(zero_counts=100, min_rad=-1, max_rad=1)
        with self.assertRaises(ValueError): validate_config(self.config)

    def test_bad_feedback_rejected_without_clamping(self):
        for q in [-1, 0, 4096]:
            with self.assertRaises(ValueError): counts_to_radians(self.row, q)

    def test_measured_joint_limit_rejected(self):
        with self.assertRaises(ValueError): counts_to_radians(self.row, 3000)

    def test_nonfinite_goal_rejected(self):
        for q in [float('nan'), float('inf')]:
            with self.assertRaises(ValueError): radians_to_counts(self.row, q)

    def test_reordered_names_map_to_correct_ids(self):
        names = list(reversed(NAMES))
        result = preview_trajectory(self.config, names, [{'positions': [.01, .02, .03, .04, .05], 'time_s': 1}])
        self.assertEqual([r['motor_id'] for r in result[0]['targets']], [5, 4, 3, 2, 1])

    def test_partial_and_duplicate_joint_names_rejected(self):
        for names in [NAMES[:4], ('joint_1',)*5]:
            with self.assertRaises(ValueError):
                preview_trajectory(self.config, names, [{'positions': [0]*5, 'time_s': 1}])

    def test_nonincreasing_time_rejected(self):
        points = [{'positions': [0]*5, 'time_s': 1}]*2
        with self.assertRaises(ValueError): preview_trajectory(self.config, NAMES, points)

    def test_segment_velocity_rejected(self):
        points = [{'positions': [0]*5, 'time_s': .1}, {'positions': [.1]*5, 'time_s': .2}]
        with self.assertRaises(ValueError): preview_trajectory(self.config, NAMES, points)

    def test_explicit_velocity_limit_rejected(self):
        points = [{'positions': [0]*5, 'velocities': [.2]*5, 'time_s': 1}]
        with self.assertRaises(ValueError): preview_trajectory(self.config, NAMES, points)

    def test_missing_motor_drops_batch(self):
        class Missing(FakeBus):
            def sample(self, ident):
                if ident == 3: raise OSError('No response')
                return super().sample(ident)
        with self.assertRaises(OSError): sample_all(Missing(), self.config)

    def test_wrong_model_and_mode_rejected(self):
        for model, mode in [(778, 0), (777, 1)]:
            class Wrong(FakeBus):
                def sample(self, ident): return model, mode, 0, 2048, None
            with self.assertRaises(ValueError): sample_all(Wrong(), self.config)

    def test_complete_read_only_batch(self):
        result = sample_all(FakeBus(), self.config)
        self.assertEqual(len(result), 5)
        self.assertEqual([v['position'] for v in result], [0]*5)

    def test_urdf_limits_enforced(self):
        # Local fixture keeps Nick's tests independent of Kory's package checkout.
        joints = ''.join(
            f'<joint name="{name}" type="revolute">'
            '<limit lower="-0.5" upper="0.5" velocity="0.15"/>'
            '</joint>' for name in NAMES)
        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory)/'fixture.urdf'
            urdf.write_text('<robot name="test">'+joints+'</robot>')
            for row in self.config['joints']:
                row.update(min_rad=-.4, max_rad=.4)
            validate_config(self.config, urdf)
            self.row['min_rad'] = -.51
            with self.assertRaises(ValueError): validate_config(self.config, urdf)


if __name__ == '__main__': unittest.main()
