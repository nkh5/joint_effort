"""Motor check, calibration and servo tools against the software servo emulator. No ROS or hardware."""
import builtins
import contextlib
import io
import math
import tempfile
import threading
import unittest
from pathlib import Path

import _paths
import sts_emulator as emu
from amw_bringup import arm as A
from amw_tools import calibrate, common, motors, preflight

ARM_YAML = _paths.WS / 'src/amw_bringup/config/arm.yaml'
K = 4096 / (2 * math.pi)


class EmulatedArm(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.link = str(Path(self.tmp.name) / 'fake_sts')
        self.yaml = Path(self.tmp.name) / 'arm.yaml'
        text = ARM_YAML.read_text()
        old_port = A.load(ARM_YAML).serial_port
        self.yaml.write_text(text.replace(old_port, self.link))
        self.arm = A.load(self.yaml)
        # Arm at the SRDF home pose (inside all limits). Note: the elbow's 0-degree mark (reference pose)
        # is outside its travel limits, so the reference pose is only used with torque off.
        home = A.home_pose((_paths.WS / 'src/amw_description/srdf/amw.srdf').read_text())
        self.servos = [emu.Servo(j.motor_id, round(j.raw(home[j.name]))) for j in self.arm]
        self.bus = emu.Bus(self.servos)
        self.stop = threading.Event()
        _, self.thread = emu.serve(self.bus, self.link, self.stop)
        self.patches = [(common, 'ARM_YAML', self.yaml), (calibrate, 'ARM_YAML', self.yaml)]
        self.saved = [(m, n, getattr(m, n)) for m, n, _ in self.patches]
        for m, n, v in self.patches:
            setattr(m, n, v)

    def tearDown(self):
        for m, n, v in self.saved:
            setattr(m, n, v)
        self.stop.set()
        self.thread.join(timeout=1)
        self.tmp.cleanup()

    def servo(self, joint):
        return self.bus.by_id(self.arm.joints[joint].motor_id)

    def hand(self, joint, raw):
        s = self.servo(joint)
        with self.bus.lock:
            s.encoder = float(raw) + s.offset()

    def quiet(self, fn, *args, **kw):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = fn(*args, **kw)
        return result, out.getvalue()


class PreflightTest(EmulatedArm):
    def test_all_inside_limits_passes(self):
        code, out = self.quiet(preflight.run, self.arm)
        self.assertEqual(code, 0, out)
        self.assertIn('MOTOR CHECK OK', out)

    def test_joint_outside_limit_blocks_powered_start_but_not_readonly(self):
        j = self.arm.joints['joint_3']
        self.hand('joint_3', round(j.raw(j.upper + math.radians(10))))
        code, out = self.quiet(preflight.run, self.arm)
        self.assertEqual(code, 1)
        self.assertIn('joint_3: at', out)
        self.assertEqual(self.quiet(preflight.run, self.arm, readonly=True)[0], 0)

    def test_missing_motor_is_named(self):
        del self.bus.servos[5]
        code, out = self.quiet(preflight.run, self.arm, readonly=True)
        self.assertEqual(code, 2)
        self.assertIn('joint_5: motor ID 5 does not answer', out)


class CalibrateTest(EmulatedArm):
    def test_zero_writes_new_zero_from_reference_pose(self):
        for k in self.arm:
            self.hand(k.name, round(k.raw(k.reference)))
        j = self.arm.joints['joint_2']
        self.hand('joint_2', round(j.raw(j.reference)) + 25)       # motor now reads 25 counts higher
        code, out = self.quiet(calibrate.zero, ['joint_2'], yes=True)
        self.assertEqual(code, 0, out)
        new = A.load(self.yaml).joints['joint_2']
        self.assertEqual(new.zero, j.zero + 25)
        self.assertEqual(A.load(self.yaml).joints['joint_3'].zero, self.arm.joints['joint_3'].zero)
        self.assertAlmostEqual(new.angle(round(j.raw(j.reference)) + 25), j.reference, delta=0.5 / K)

    def run_check(self, joint, degrees):
        j = self.arm.joints[joint]
        start = self.servo(joint).present()
        calls = []

        def fake_input(prompt=''):
            calls.append(prompt)
            # move by `degrees` in joint angle (raw changes by dir * angle * K)
            self.hand(joint, start + j.dir * math.radians(degrees) * K)
            return ''
        saved_input, saved_ask = builtins.input, calibrate.ask
        builtins.input, calibrate.ask = fake_input, lambda q: False
        try:
            return self.quiet(calibrate.check, [joint])
        finally:
            builtins.input, calibrate.ask = saved_input, saved_ask

    def test_check_direction_ok(self):
        code, out = self.run_check('joint_2', +12)
        self.assertEqual(code, 0, out)
        self.assertIn('->  OK', out)
        self.assertIn('the iron tip moves', out)

    def test_check_direction_reversed_names_the_fix(self):
        code, out = self.run_check('joint_4', -12)
        self.assertEqual(code, 1)
        self.assertIn(f'REVERSED - set dir: {-self.arm.joints["joint_4"].dir} for joint_4', out)

    def test_check_refuses_with_torque_on(self):
        self.servo('joint_1').mem[emu.TORQUE] = 1
        with self.assertRaises(SystemExit) as ctx:
            self.quiet(calibrate.check, ['joint_1'])
        self.assertIn('Torque is ON', str(ctx.exception))


class MotorsTest(EmulatedArm):
    def test_scan_finds_all(self):
        code, out = self.quiet(motors.scan, 6)
        self.assertEqual(code, 0, out)
        self.assertIn('5 servo(s) found.', out)

    def test_recenter_makes_it_read_2048(self):
        self.hand('joint_4', 2500)
        code, out = self.quiet(motors.recenter, 'joint_4', yes=True)
        self.assertEqual(code, 0, out)
        self.assertEqual(self.servo('joint_4').present(), 2048)

    def test_tune_writes_gains_yaml(self):
        code, out = self.quiet(motors.tune, yes=True)
        self.assertEqual(code, 0, out)
        s = self.servo('joint_2')
        self.assertEqual((s.mem[21], s.mem[23], s.mem[22]), (100, 10, 70))
        code, out = self.quiet(motors.tune, yes=True)
        self.assertIn('already match', out)


if __name__ == '__main__':
    unittest.main()
