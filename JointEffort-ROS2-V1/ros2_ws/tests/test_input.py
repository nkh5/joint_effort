import math
import tempfile
import unittest
from pathlib import Path
from joint_effort_input.core import InputState, speed_level, multiplier, velocity
from joint_effort_input.lease import Lease


class InputTests(unittest.TestCase):
    def update(self, state, now, axes=None, hold=0, mode=0):
        return state.update(axes or [0., 0., 0., 0.], [hold, mode, 0, 0, 0, 0, 0],
                            int((now+100)*1e9), int((now+100)*1e9), now)

    def prepare(self, state, now=0.):
        self.update(state, now); self.update(state, now+.1); self.update(state, now+.21)

    def test_hid_slider_quantization_all_levels_both_signs(self):
        for level in range(1, 21):
            hid = round((level-1)*255/19)
            axis = 1-2*hid/255
            self.assertEqual(speed_level(axis), level)
            self.assertEqual(speed_level(-axis, -1.), level)
        self.assertAlmostEqual(multiplier(1), .1); self.assertEqual(multiplier(10), 1.)
        self.assertEqual(multiplier(20), 2.)
        self.assertTrue(all(multiplier(i) < multiplier(i+1) for i in range(1, 20)))

    def test_start_held_and_off_center_cannot_move(self):
        s = InputState(); self.update(s, 0., [1., 0., 0., 0.], hold=1)
        self.assertFalse(s.allowed(0.)); self.update(s, .1, hold=1)
        self.assertFalse(s.allowed(.1)); self.prepare(s, .2); self.update(s, .42, hold=1)
        self.assertTrue(s.allowed(.42))

    def test_release_requires_center_before_new_hold(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .23, [1., 0., 0., 0.]); self.update(s, .24, [1., 0., 0., 0.], hold=1)
        self.assertFalse(s.allowed(.24))

    def test_enable_must_begin_with_centered_stick(self):
        s = InputState(); self.prepare(s)
        self.update(s, .22, [1., 0., 0., 0.], hold=1)
        self.assertFalse(s.allowed(.22))

    def test_mode_toggles_on_edge_and_requires_release(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1, mode=1)
        self.assertEqual(s.mode, 'rotation'); self.assertFalse(s.allowed(.22))
        self.update(s, .23, hold=1, mode=1); self.assertEqual(s.mode, 'rotation')
        self.prepare(s, .3); self.update(s, .52, hold=1)
        self.assertTrue(s.allowed(.52)); self.update(s, .53, hold=1, mode=1)
        self.assertEqual(s.mode, 'translation'); self.assertFalse(s.allowed(.53))

    def test_mode_change_while_enabled_rearms_by_centering_only(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1); self.assertTrue(s.allowed(.22))
        self.update(s, .25, hold=1, mode=1); self.assertEqual(s.mode, 'rotation')
        self.assertFalse(s.allowed(.25))
        self.update(s, .30, hold=1); self.update(s, .40, hold=1); self.assertFalse(s.allowed(.40))
        self.update(s, .51, hold=1); self.assertTrue(s.allowed(.51))      # 0.21 s centred, enable never released

    def test_mode_rearm_waits_for_centered_stick(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .25, hold=1, mode=1)
        for t in (.3, .4, .5, .6): self.update(s, t, [.5, 0., 0., 0.], hold=1)
        self.assertFalse(s.allowed(.6))
        self.update(s, .7, hold=1); self.update(s, .8, hold=1); self.assertFalse(s.allowed(.8))
        self.update(s, .91, hold=1); self.assertTrue(s.allowed(.91))

    def test_double_mode_press_still_rearms(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .25, hold=1, mode=1); self.update(s, .28, hold=1)
        self.update(s, .31, hold=1, mode=1); self.assertEqual(s.mode, 'translation')
        self.update(s, .34, hold=1); self.update(s, .45, hold=1); self.update(s, .56, hold=1)
        self.assertTrue(s.allowed(.56))

    def test_mode_press_after_other_fault_still_needs_release(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        s.fault('Servo halted; center and release required')            # e.g. forwarder.fail()
        self.update(s, .25, hold=1, mode=1)
        for t in (.3, .4, .5, .6, .7): self.update(s, t, hold=1)
        self.assertFalse(s.allowed(.7))
        self.prepare(s, .8); self.update(s, 1.02, hold=1); self.assertTrue(s.allowed(1.02))

    def test_fault_during_mode_rearm_cancels_it(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .25, hold=1, mode=1); s.fault('Software stop requested')
        for t in (.3, .4, .5, .6): self.update(s, t, hold=1)
        self.assertFalse(s.allowed(.6))

    def test_input_loss_during_mode_rearm_cancels_it(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .25, hold=1, mode=1); self.update(s, .30, hold=1)
        for t in (.6, .7, .8, .9): self.update(s, t, hold=1)               # 0.3 s gap > 0.15 s timeout
        self.assertFalse(s.allowed(.9))

    def test_old_pausing_firmware_still_resumes_with_release(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.update(s, .25, hold=0, mode=1)                                 # old Pico drops bit 7 on mode press
        self.update(s, .30); self.update(s, .40); self.update(s, .51)
        self.update(s, .52, hold=1); self.assertTrue(s.allowed(.52))

    def test_input_loss_never_auto_resumes_held(self):
        s = InputState(); self.prepare(s); self.update(s, .22, hold=1)
        self.assertFalse(s.allowed(.5)); self.update(s, .51, hold=1)
        self.assertFalse(s.allowed(.51)); self.prepare(s, .6); self.update(s, .82, hold=1)
        self.assertTrue(s.allowed(.82))

    def test_invalid_and_replayed_reports_disarm(self):
        for axes, buttons in [([math.nan, 0., 0., 0.], [0]*7), ([2., 0., 0., 0.], [0]*7),
                              ([0.]*3, [0]*7), ([0.]*4, [0]), ([0.]*4, [2]*7)]:
            s = InputState(); self.prepare(s)
            self.assertFalse(s.update(axes, buttons, 100220000000, 100220000000, .22))
            self.assertFalse(s.prepared)
        s = InputState(); self.prepare(s)
        self.assertFalse(s.update([0.]*4, [0]*7, 100210000000, 100220000000, .22))
        self.assertFalse(s.prepared)

    def test_deadzone_excludes_slider_and_vector_caps(self):
        s = InputState(); self.update(s, 0., [.04, -.04, .04, 1.])
        self.assertEqual(s.axes, [0., 0., 0.]); self.assertEqual(s.level, 1)
        self.assertAlmostEqual(math.sqrt(sum(v*v for v in velocity([1., 1., 1.], 'translation', 2.)[:3])), 0.02)
        angular = velocity([1., 1., 1.], 'rotation', 2.)[3:]
        self.assertAlmostEqual(math.sqrt(sum(v*v for v in angular)), 0.2)
        self.assertEqual(angular[2], 0.)

    def test_shared_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            a, b = Lease(Path(directory)/'owner'), Lease(Path(directory)/'owner')
            self.assertTrue(a.acquire()); self.assertFalse(b.acquire())
            a.release(); self.assertTrue(b.acquire()); b.release()


if __name__ == '__main__': unittest.main()
