"""Tests for the real-arm calibration tools (no ROS, no hardware: uses tests/sts_emulator.py)."""
import contextlib
import importlib.util
import io
import math
import shutil
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import yaml

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W / 'tests'))
import sts_emulator  # noqa: E402


def load(name):
    spec = importlib.util.spec_from_file_location(name, W / 'tools' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fit_calibration = load('fit_calibration')
sts_preflight = load('sts_preflight')
sts_set_center = load('sts_set_center')
K = 4096 / (2 * math.pi)
URDF = W / 'tests/fixtures/iron_original.urdf'          # frozen: tests do not follow live edits
CSV = W / 'tests/fixtures/Calibration.csv'


class Workdir(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cal = self.tmp / 'sts_calibration.yaml'
        shutil.copy(W / 'tests/fixtures/sts_calibration_blank.yaml', self.cal)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def run_fit(self, *extra):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = fit_calibration.main(['--csv', str(CSV), '--urdf', str(URDF), '--cal', str(self.cal), *extra])
        return code, out.getvalue(), yaml.safe_load(self.cal.read_text())


class FitTests(Workdir):
    def test_csv_fit_matches_hand_numbers(self):
        fits = {n: fit_calibration.fit(p) for n, p in fit_calibration.read_csv(CSV).items()}
        self.assertEqual(sorted(fits), fit_calibration.JOINTS)       # side table in columns G-M is ignored
        self.assertEqual({n: f['s'] for n, f in fits.items()},
                         {'joint_1': 1, 'joint_2': -1, 'joint_3': -1, 'joint_4': 1, 'joint_5': 1})
        self.assertEqual({n: round(f['zero']) for n, f in fits.items()},
                         {'joint_1': 2064, 'joint_2': 2035, 'joint_3': 3338, 'joint_4': 2004, 'joint_5': 2041})

    def test_urdf_axes_agree_with_the_driver_convention(self):
        urdf = fit_calibration.read_urdf(URDF)
        fits = {n: fit_calibration.fit(p) for n, p in fit_calibration.read_csv(CSV).items()}
        for n in fit_calibration.JOINTS:
            self.assertEqual(urdf[n]['sigma'], -fits[n]['s'], n)

    def test_incomplete_without_datum(self):
        code, out, cal = self.run_fit('--write')
        self.assertEqual(code, 1)
        self.assertIn('INCOMPLETE', out)
        self.assertTrue(all(cal['joints'][n]['center_steps'] is None for n in fit_calibration.JOINTS))
        self.assertEqual(cal['joints']['joint_3']['min_rad'], 0.65)
        self.assertEqual(cal['joints']['joint_3']['max_rad'], 2.35)

    def test_center_and_driver_model_round_trip(self):
        datum = {'joint_1': 0.01, 'joint_2': -0.02, 'joint_3': 0.3, 'joint_4': 0.0, 'joint_5': 0.05}
        code, out, cal = self.run_fit('--write', '--datum', *[f'{n}={v}' for n, v in datum.items()])
        self.assertEqual(code, 0, out)
        urdf = fit_calibration.read_urdf(URDF)
        for n, pts in fit_calibration.read_csv(CSV).items():
            j = cal['joints'][n]
            f = fit_calibration.fit(pts)
            for angle_deg, _ in pts:
                counts = j['csv_zero_counts'] + f['s'] * angle_deg * 4096 / 360     # ratio-1 model
                q_driver = (j['center_steps'] - counts) / K                          # sts_hardware_interface
                q_urdf = urdf[n]['sigma'] * math.radians(angle_deg) + datum[n]
                self.assertAlmostEqual(q_driver, q_urdf, delta=1.0 / K, msg=n)

    def test_wrap_is_detected_with_shift_advice(self):
        code, out, _ = self.run_fit('--datum', 'joint_1=0', 'joint_2=0', 'joint_3=1.5', 'joint_4=0', 'joint_5=0')
        self.assertEqual(code, 1)
        self.assertIn('joint_3: center_steps 4316', out)
        self.assertIn('sts_set_center.py 3 --shift -1290', out)
        # applying that shift makes it valid
        cal = yaml.safe_load(self.cal.read_text())
        cal['joints']['joint_3']['encoder_shift_counts'] = -1290
        self.cal.write_text(yaml.safe_dump(cal))
        code, out, cal = self.run_fit('--write', '--datum', 'joint_1=0', 'joint_2=0', 'joint_3=1.5',
                                      'joint_4=0', 'joint_5=0')
        self.assertEqual(code, 0, out)
        self.assertEqual(cal['joints']['joint_3']['center_steps'], 3026)

    def test_direction_mismatch_is_blocking(self):
        flipped = self.tmp / 'iron.urdf'
        text = URDF.read_text()
        start = text.index('<joint name="joint_2"')
        text = text[:start] + text[start:].replace('<axis xyz="0 0 1" />', '<axis xyz="0 0 -1" />', 1)
        flipped.write_text(text)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = fit_calibration.main(['--csv', str(CSV), '--urdf', str(flipped), '--cal', str(self.cal),
                                         '--datum', 'joint_1=0', 'joint_2=0', 'joint_3=0.3', 'joint_4=0',
                                         'joint_5=0'])
        self.assertEqual(code, 1)
        self.assertIn('joint_2: direction mismatch', out.getvalue())
        self.assertIn('set csv_angle_sign: -1 for joint_2', out.getvalue())
        cal = yaml.safe_load(self.cal.read_text())
        cal['joints']['joint_2']['csv_angle_sign'] = -1
        self.cal.write_text(yaml.safe_dump(cal))
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = fit_calibration.main(['--csv', str(CSV), '--urdf', str(flipped), '--cal', str(self.cal),
                                         '--datum', 'joint_1=0', 'joint_2=0', 'joint_3=0.3', 'joint_4=0',
                                         'joint_5=0'])
        self.assertEqual(code, 0, out.getvalue())


class EmulatedBus(Workdir):
    RAW = {1: 2064, 2: 2035, 3: 3338, 4: 2004, 5: 2041}

    def setUp(self):
        super().setUp()
        self.bus = sts_emulator.Bus([sts_emulator.Servo(i, r) for i, r in self.RAW.items()])
        self.stop = threading.Event()
        self.port, self.thread = sts_emulator.serve(self.bus, str(self.tmp / 'fake_sts'), self.stop)

    def tearDown(self):
        self.stop.set()
        self.thread.join(timeout=1)
        super().tearDown()

    def preflight(self, *extra):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = sts_preflight.main(['--cal', str(self.cal), '--urdf', str(URDF), '--port', self.port, *extra])
        return code, out.getvalue()

    def calibrate(self, datum3=0.3):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            fit_calibration.main(['--csv', str(CSV), '--urdf', str(URDF), '--cal', str(self.cal), '--write',
                                  '--datum', 'joint_1=0', 'joint_2=0', f'joint_3={datum3}', 'joint_4=0',
                                  'joint_5=0'])

    def test_readonly_report_with_provisional_centers(self):
        code, out = self.preflight('--readonly')
        self.assertEqual(code, 0, out)
        self.assertIn('provisional', out)
        self.assertFalse(any(w for s in self.bus.servos.values() for w in s.writes), 'preflight must not write')

    def test_powered_preflight_refuses_without_calibration(self):
        code, out = self.preflight()
        self.assertEqual(code, 1)
        self.assertIn('calibration incomplete', out)

    def test_powered_preflight_out_of_limit_elbow(self):
        # elbow resting at its 0 deg mark = datum 0.3 rad, below its 0.7 limit
        self.calibrate(0.3)
        code, out = self.preflight()
        self.assertEqual(code, 1)
        self.assertIn('joint_3: at +0.30', out)
        # move it by hand into range (0.3 -> 1.5 rad: counts fall for the elbow)
        with self.bus.lock:
            self.bus.servos[3].encoder -= round(1.2 * K)
        code, out = self.preflight()
        self.assertEqual(code, 0, out)
        self.assertIn('PREFLIGHT OK', out)

    def test_missing_motor_is_fatal(self):
        with self.bus.lock:
            del self.bus.servos[4]
        code, out = self.preflight('--readonly')
        self.assertEqual(code, 2)
        self.assertIn('motor ID 4 does not answer', out)

    def test_set_center_shift(self):
        sts_set_center.PORT = self.port
        before = self.bus.servos[3].present()
        old_argv = sys.argv
        sys.argv = ['sts_set_center.py', '3', '--shift', '-1290']
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                sts_set_center.main()
        finally:
            sys.argv = old_argv
        self.assertEqual(self.bus.servos[3].present(), before - 1290)
        self.assertIn('change -1290', out.getvalue())
        self.assertEqual(self.bus.servos[3].encoder, float(before))          # did not move


if __name__ == '__main__':
    unittest.main()
