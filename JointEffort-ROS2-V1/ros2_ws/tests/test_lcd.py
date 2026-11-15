"""Status screen: /amw/status -> what is shown, and every screen renders at 480x320 and 800x480."""
import unittest

import _paths  # noqa: F401  (adds src packages to sys.path)

try:
    import pygame
    from amw_teleop import lcd
except ImportError:            # pygame not installed (only needed on the Pi / for the LCD)
    pygame = None


@unittest.skipIf(pygame is None, 'pygame not installed')
class LcdTest(unittest.TestCase):
    def test_view_mapping(self):
        stop = {'code': 'JOINT_OUTSIDE_LIMIT', 'kind': 'stop', 'text': 'joint_3 is out', 'fix': 'move it'}
        cases = [({'state': 'off', 'stop': {'code': 'OFF'}, 'joystick': 'ok'}, 'off'),
                 ({'state': 'off', 'stop': {'code': 'TURNED_OFF'}, 'joystick': 'ok'}, 'off'),
                 ({'state': 'off', 'stop': stop, 'joystick': 'ok'}, 'stopped'),
                 ({'state': 'off', 'stop': stop, 'joystick': 'lost'}, 'noctrl'),
                 ({'state': 'centre', 'stop': {'code': 'MODE_CHANGED', 'text': 'Switched', 'fix': 'Centre'}}, 'centre'),
                 ({'state': 'active', 'warning': {'text': 'joint_3 at limit'}}, 'on')]
        for status, shown in cases:
            self.assertEqual(lcd.view(status)['status'], shown, status)
        v = lcd.view({'state': 'off', 'stop': stop, 'joystick': 'ok', 'mode': 'rotation', 'level': 7})
        self.assertEqual((v['text'], v['fix'], v['mode'], v['level']), ('joint_3 is out', 'move it', 'rotate', 7))
        self.assertEqual(lcd.view({'state': 'active', 'warning': {'text': 'w'}})['warning'], 'w')

    def test_every_demo_screen_renders(self):
        pygame.font.init()
        for size in [(480, 320), (800, 480)]:
            painter = lcd.Painter(size)
            surface = pygame.Surface(size)
            for keys in ['', 'o', 'c', 's', 'w', 'j', 'mw', 'n']:
                src = lcd.DemoSource(keys)
                v, stale = src.poll(0)
                painter.draw(surface, v, stale)
            painter.draw(surface, None, 'Arm software stopped')


if __name__ == '__main__':
    unittest.main()
