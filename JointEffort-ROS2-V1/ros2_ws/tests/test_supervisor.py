"""ON/OFF and mode state machine (amw_teleop.supervisor) and the joystick maths. No ROS needed."""
import math
import unittest

import _paths  # noqa: F401  (adds src packages to sys.path)
from amw_teleop.core import multiplier, stick, velocity
from amw_teleop.faults import FAULTS, KINDS, markdown
from amw_teleop.supervisor import Health, Supervisor

LIMITS = {f'joint_{i}': (-1.0, 1.0) for i in range(1, 6)}
L, ZL, UP, DOWN, ON = 0, 1, 2, 3, 4


def healthy(**changes):
    h = Health(joints={n: 0.0 for n in LIMITS}, joints_age=0.01, limits=LIMITS, controller='active',
               servo_ready=True, servo_code=0, imu_age=0.01)
    for k, v in changes.items():
        setattr(h, k, v)
    return h


class Driver:
    """Feeds the supervisor Joy-Con reports at 50 Hz, like the node does."""

    def __init__(self, health=None):
        self.s = Supervisor()
        self.t = 0.0
        self.health = health or healthy()
        self.buttons = [0] * 11
        self.axes = [0.0, 0.0]                    # stick x, y

    def run(self, seconds=0.0, report=True):
        end = self.t + seconds
        while True:
            if report:
                self.s.joy(self.axes, self.buttons, self.t, self.health)
            moving = self.s.step(self.health, self.t)
            if self.t >= end:
                return moving
            self.t = round(self.t + 0.02, 6)

    def press(self, button):
        self.buttons[button] = 1
        self.run()
        self.buttons[button] = 0
        self.t += 0.02
        return self.run()

    def turn_on(self):
        self.run(0.1)
        self.press(ON)
        return self.run(0.3)


class SupervisorTests(unittest.TestCase):
    def test_starts_off_and_on_needs_a_press(self):
        d = Driver()
        self.assertFalse(d.run(1.0))
        self.assertEqual(d.s.stop['code'], 'OFF')
        self.assertTrue(d.turn_on())
        self.assertEqual(d.s.state, 'active')
        self.assertIsNone(d.s.stop)

    def test_held_button_at_connect_is_not_a_press(self):
        d = Driver()
        d.buttons[ON] = 1
        self.assertFalse(d.run(0.5))
        self.assertEqual(d.s.state, 'off')

    def test_on_with_stick_deflected_waits_for_centre(self):
        d = Driver()
        d.axes[0] = 0.7
        d.run(0.1)
        d.press(ON)
        self.assertFalse(d.run(0.5))
        self.assertEqual(d.s.stop['code'], 'CENTRE_STICK')
        d.axes[0] = 0.0
        self.assertFalse(d.run(0.1))
        self.assertTrue(d.run(0.2))

    def test_second_press_turns_off(self):
        d = Driver()
        d.turn_on()
        self.assertFalse(d.press(ON))
        self.assertEqual(d.s.stop['code'], 'TURNED_OFF')

    def test_held_buttons_select_the_mode(self):
        d = Driver()
        d.turn_on()
        for held, mode in [((L,), 'z'), ((ZL,), 'rotation'), ((L, ZL), 'rotation'), ((), 'translation')]:
            d.buttons = [0] * 11
            for b in held:
                d.buttons[b] = 1
            self.assertTrue(d.run(0.1), mode)
            self.assertEqual(d.s.mode, mode)

    def test_mode_change_with_stick_pushed_pauses_until_centred(self):
        d = Driver()
        d.turn_on()
        d.axes[1] = 0.5
        d.run(0.1)
        d.buttons[L] = 1                          # L while pushing: z would jump, so wait for centre
        d.run()
        self.assertEqual((d.s.mode, d.s.stop['code']), ('z', 'MODE_CHANGED'))
        self.assertFalse(d.run(0.5))
        d.axes[1] = 0.0
        self.assertTrue(d.run(0.3))

    def test_rotation_ignores_the_stick_and_does_not_pause(self):
        d = Driver()
        d.turn_on()
        d.axes[0] = 0.8
        d.run(0.1)
        d.buttons[ZL] = 1
        self.assertTrue(d.run(0.1))
        self.assertEqual(d.s.mode, 'rotation')

    def test_rotation_needs_imu_data(self):
        d = Driver()
        d.turn_on()
        d.buttons[ZL] = 1
        d.health.imu_age = 0.01
        self.assertTrue(d.run(0.1))
        d.health.imu_age = 0.5
        self.assertFalse(d.run())
        self.assertEqual(d.s.stop['code'], 'IMU_LOST')

    def test_translation_does_not_need_imu_data(self):
        d = Driver(healthy(imu_age=math.inf))    # no Joy-Con motion data at all
        self.assertTrue(d.turn_on())
        self.assertTrue(d.run(0.5))

    def test_speed_buttons_step_and_clamp(self):
        d = Driver()
        d.run(0.1)
        self.assertEqual(d.s.level, 10)
        d.press(UP)
        self.assertEqual(d.s.level, 12)
        for _ in range(10):
            d.press(UP)
        self.assertEqual(d.s.level, 20)
        for _ in range(15):
            d.press(DOWN)
        self.assertEqual(d.s.level, 1)

    def test_recenter_counts_presses_in_any_state(self):
        d = Driver()
        d.run(0.1)
        d.press(5)
        d.turn_on()
        d.press(5)
        self.assertEqual(d.s.recenters, 2)
        self.assertEqual(d.s.state, 'active')

    def test_mode_while_off_only_changes_mode(self):
        d = Driver()
        d.run(0.1)
        d.buttons[ZL] = 1
        d.run()
        self.assertEqual((d.s.mode, d.s.state), ('rotation', 'off'))

    def test_joystick_loss_turns_off_and_never_resumes_by_itself(self):
        d = Driver()
        d.turn_on()
        self.assertFalse(d.run(0.3, report=False))
        self.assertEqual(d.s.stop['code'], 'JOYSTICK_LOST')
        self.assertFalse(d.run(1.0))
        self.assertEqual(d.s.state, 'off')

    def test_bad_report_turns_off(self):
        d = Driver()
        d.turn_on()
        d.s.joy([math.nan, 0], [0] * 11, d.t)
        self.assertEqual(d.s.stop['code'], 'JOYSTICK_BAD')

    def test_on_refused_with_reason_when_not_ready(self):
        cases = [(dict(joints=None), 'NOT_READY', 'no joint states'),
                 (dict(controller='inactive'), 'NOT_READY', 'iron_arm_controller is inactive'),
                 (dict(servo_ready=False), 'NOT_READY', 'MoveIt Servo'),
                 (dict(joints_age=2.0), 'FEEDBACK_STALE', '2.0'),
                 (dict(joints={**{n: 0.0 for n in LIMITS}, 'joint_3': 1.2}), 'JOINT_OUTSIDE_LIMIT', 'joint_3')]
        for change, code, text in cases:
            d = Driver(healthy(**change))
            self.assertFalse(d.turn_on(), code)
            self.assertEqual(d.s.stop['code'], code)
            self.assertIn(text, d.s.stop['text'])

    def test_small_sag_past_limit_is_tolerated(self):
        d = Driver(healthy(joints={**{n: 0.0 for n in LIMITS}, 'joint_3': 1.0 + math.radians(1)}))
        self.assertTrue(d.turn_on())

    def test_faults_while_moving(self):
        cases = [(dict(joints_age=0.5), 'FEEDBACK_STALE'), (dict(controller='inactive'), 'CONTROLLER_INACTIVE'),
                 (dict(servo_code=5, servo_message='Very close to collision'), 'SERVO_COLLISION'),
                 (dict(servo_code=-1), 'SERVO_ERROR'),
                 (dict(motors={'joint_2': {'temperature': 70.0, 'voltage': 12.4}}), 'MOTOR_HOT'),
                 (dict(motors={'joint_2': {'temperature': 30.0, 'voltage': 7.0}}), 'MOTOR_VOLTAGE')]
        for change, code in cases:
            d = Driver()
            d.turn_on()
            for k, v in change.items():
                setattr(d.health, k, v)
            self.assertFalse(d.run(), code)
            self.assertEqual(d.s.stop['code'], code)
            self.assertEqual(d.s.stop['kind'], 'stop')

    def test_warnings_keep_moving(self):
        for change, code in [(dict(servo_code=4, servo_message='Close to a collision'), 'COLLISION_NEAR'),
                             (dict(motors={'joint_1': {'temperature': 58.0}}), 'MOTOR_WARM'),
                             (dict(servo_code=6), 'JOINT_LIMIT')]:
            d = Driver()
            d.turn_on()
            for k, v in change.items():
                setattr(d.health, k, v)
            self.assertTrue(d.run(), code)
            self.assertEqual(d.s.warning['code'], code)

    def test_stop_request(self):
        d = Driver()
        d.turn_on()
        d.s.request_stop(d.t)
        self.assertEqual(d.s.stop['code'], 'STOP_REQUESTED')
        self.assertFalse(d.run())

    def test_kinematics_warning_names_joint_and_side(self):
        d = Driver()
        d.turn_on()
        names = list(LIMITS)
        d.s.kinematics_warning({'blocked': [2], 'status': 'joint_limit'}, names, list(LIMITS.values()),
                               [0, 0, 0.98, 0, 0], d.t)
        self.assertEqual(d.s.warning['code'], 'JOINT_LIMIT')
        self.assertIn('joint_3 is at its upper limit', d.s.warning['text'])

    def test_status_and_events(self):
        d = Driver()
        d.turn_on()
        d.press(ON)
        st = d.s.status(d.t)
        self.assertEqual(st['state'], 'off')
        self.assertEqual(st['joystick'], 'ok')
        texts = [t for _, t in st['events']]
        self.assertIn('ON', texts)
        self.assertIn('moving', texts)
        self.assertTrue(any(t.startswith('OFF: Turned off') for t in texts))


class CoreAndFaultTests(unittest.TestCase):
    def test_multiplier(self):
        self.assertAlmostEqual(multiplier(1), .1)
        self.assertEqual(multiplier(10), 1.)
        self.assertEqual(multiplier(20), 2.)

    def test_round_deadzone_and_curve(self):
        self.assertEqual(stick(0.08, 0.08, 0.12), (0.0, 0.0))            # inside the circle
        x, y = stick(0.6, 0.6, 0.12)
        self.assertAlmostEqual(x, y)                                       # diagonals stay diagonal
        self.assertAlmostEqual(math.hypot(*stick(1.0, 0.0, 0.12, 0.6)), 1.0)
        fine = stick(0.3, 0.0, 0.12, 0.6)[0]
        self.assertLess(fine, stick(0.3, 0.0, 0.12, 0.0)[0])               # expo: slower near the centre

    def test_velocity_modes_and_cap(self):
        self.assertAlmostEqual(math.hypot(*velocity([1., 1.], 'translation', 2.)), 0.02)
        self.assertEqual(velocity([1., 1.], 'translation')[2], 0.)
        self.assertEqual(velocity([1., 0.5], 'z'), [0., 0., 0.5 * 0.01 * 0.5])
        self.assertEqual(velocity([1., 1.], 'rotation'), [0., 0., 0.])

    def test_fault_table_is_complete(self):
        for f in FAULTS.values():
            self.assertIn(f.kind, KINDS)
            self.assertTrue(f.text and f.fix)
        doc = (_paths.WS / 'docs/troubleshooting.md').read_text()
        self.assertIn(markdown(), doc, 'docs/troubleshooting.md is out of date: regenerate the table with '
                                       '`python3 -m amw_teleop.faults`')


if __name__ == '__main__':
    unittest.main()
