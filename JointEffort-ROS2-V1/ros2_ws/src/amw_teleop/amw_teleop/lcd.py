#!/usr/bin/env python3
"""amw_lcd - AMW status screen. Shows /amw/status from the amw_teleop node.

  amw lcd --fb            Pi without a desktop: draw on the LCD framebuffer (3.5" SPI LCD = 480x320)
  amw lcd                 desktop window (480x320; --size 800x480 for other panels)
  amw lcd --demo          no ROS: keyboard-driven states (keys below)
  amw lcd --demo --snap a.png [--state KEY]   save one frame and exit

What it shows, top to bottom: ON / OFF / CENTRE STICK / STOPPED with the reason and what to do,
a warning strip while moving with a blocked direction, the mode (TRANSLATE / ROTATE) and the speed
level. NO DATA covers the screen within 0.6 s if amw_teleop stops publishing.

The layout scales to the screen size. In framebuffer mode the size comes from the framebuffer and
--rotate (default 0; use 90/270 for a portrait panel used sideways). Repaints only on change: no flashing.

Demo keys: o on/off, c centre, s stop fault, w warning, m mode, + / - speed, j joystick lost,
n no data, q quit.
"""
import argparse
import glob
import json
import os
import select
import sys
import time

import pygame

TOPIC = '/amw/status'
STALE_S = 0.6
LEVEL_MAX = 20
FONT_FILE = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_PLAIN = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

GREEN, AMBER, RED = (30, 150, 80), (215, 150, 20), (190, 40, 40)
OFF_BG, OFF_FG = (55, 55, 58), (190, 190, 195)
BLUE, ORANGE = (35, 105, 205), (230, 135, 15)     # translate / rotate (colour-blind safe pair)
BG, PANEL, WHITE, GREY, DIM, CYAN = (14, 14, 16), (28, 28, 32), (240, 240, 240), (140, 140, 145), (70, 70, 75), (90, 200, 235)


# ------------------------------------------------------------------ status -> what to show (pure, tested)

def view(d):
    """One /amw/status message (dict) -> what the screen shows."""
    stop, warn = d.get('stop') or {}, d.get('warning') or {}
    state = d.get('state')
    v = {'mode': 'rotate' if d.get('mode') == 'rotation' else 'translate', 'level': d.get('level'),
         'mult': d.get('multiplier'), 'warning': None, 'text': '', 'fix': ''}
    if state == 'active':
        v['status'] = 'on'
        if warn:
            v['warning'] = warn.get('text', '')
    elif state == 'centre':
        v['status'] = 'centre'
        v['text'], v['fix'] = stop.get('text', ''), stop.get('fix', '')
    elif d.get('joystick') == 'lost':
        v['status'] = 'noctrl'
        v['text'], v['fix'] = 'Controller not connected or not sending.', 'Check the Joy-Con is connected.'
    elif stop.get('code') in (None, 'OFF', 'TURNED_OFF'):
        v['status'] = 'off'
        v['text'], v['fix'] = '', 'Centre the stick and press ON (D-pad right).'
    else:
        v['status'] = 'stopped'
        v['text'], v['fix'] = stop.get('text', ''), stop.get('fix', '')
    return v


# ------------------------------------------------------------------ drawing

class Painter:
    """All sizes are fractions of the screen, so 480x320 and 800x480 both work."""

    def __init__(self, size):
        self.W, self.H = size
        s = self.H / 320

        def font(px, bold=True):
            try:
                return pygame.font.Font(FONT_FILE if bold else FONT_PLAIN, max(8, int(px * s)))
            except (FileNotFoundError, OSError):
                return pygame.font.Font(None, max(10, int(px * s * 1.35)))
        self.f = {'state': font(46), 'state_s': font(34), 'text': font(17), 'fix': font(15, False),
                  'band': font(30), 'level': font(52), 'small': font(16), 'warn': font(17)}
        self.s = s

    def blit(self, screen, text, f, color, **pos):
        surf = f.render(text, True, color)
        rect = surf.get_rect(**pos)
        screen.blit(surf, rect)
        return rect

    def wrap(self, text, f, width, lines=2):
        """Up to `lines` lines that fit in width (last one shortened with an ellipsis)."""
        words, out = text.split(), []
        while words and len(out) < lines:
            line = words.pop(0)
            while words and f.size(line + ' ' + words[0])[0] <= width:
                line += ' ' + words.pop(0)
            out.append(line)
        if words:
            last = out[-1]
            while last and f.size(last + '…')[0] > width:
                last = last[:-1]
            out[-1] = last + '…'
        return out

    def draw(self, screen, v, stale='', heartbeat=True):
        W, H, s = self.W, self.H, self.s
        screen.fill(BG)
        if v is None:
            v = {'status': 'off', 'mode': 'translate', 'level': None, 'mult': None, 'warning': None,
                 'text': '', 'fix': ''}
        speed_h, mode_h = int(H * 0.25), int(H * 0.18)
        warn_lines = self.wrap(v['warning'], self.f['warn'], W - 20, 2) if v['warning'] else []
        warn_h = (len(warn_lines) * self.f['warn'].get_linesize() + int(10 * s)) if warn_lines else 0
        top = pygame.Rect(0, 0, W, H - speed_h - mode_h - warn_h)
        self.draw_state(screen, v, top)
        if warn_h:
            r = pygame.Rect(0, top.bottom, W, warn_h)
            pygame.draw.rect(screen, AMBER, r)
            y = r.y + int(5 * s)
            for line in warn_lines:
                self.blit(screen, line, self.f['warn'], BG, midtop=(r.centerx, y))
                y += self.f['warn'].get_linesize()
        self.draw_mode(screen, v, pygame.Rect(0, H - speed_h - mode_h, W, mode_h))
        self.draw_speed(screen, v, pygame.Rect(0, H - speed_h, W, speed_h))
        pygame.draw.circle(screen, CYAN if heartbeat else DIM, (W - int(10 * s), H - int(10 * s)), max(3, int(5 * s)))
        if stale:
            self.cover(screen, 'NO DATA', stale)

    def draw_state(self, screen, v, r):
        st = v['status']
        bg, fg, label = {'on': (GREEN, WHITE, 'ON'), 'centre': (OFF_BG, AMBER, 'CENTRE STICK'),
                         'noctrl': (OFF_BG, OFF_FG, 'NO CONTROLLER'), 'stopped': (RED, WHITE, 'STOPPED'),
                         'off': (OFF_BG, OFF_FG, 'OFF')}[st]
        pygame.draw.rect(screen, bg, r)
        lines = (self.wrap(v['text'], self.f['text'], r.width - 24, 2) if v['text'] else []) + \
                (self.wrap(v['fix'], self.f['fix'], r.width - 24, 2) if v['fix'] else [])
        big = self.f['state'] if len(label) <= 8 else self.f['state_s']
        line_h = self.f['text'].get_linesize()
        block = big.get_linesize() + len(lines) * line_h
        y = r.y + max(4, (r.height - block) // 2)
        self.blit(screen, label, big, fg, midtop=(r.centerx, y))
        y += big.get_linesize()
        n_text = len(self.wrap(v['text'], self.f['text'], r.width - 24, 2)) if v['text'] else 0
        for i, line in enumerate(lines):
            f = self.f['text'] if i < n_text else self.f['fix']
            self.blit(screen, line, f, fg if i < n_text else (fg if st != 'off' else GREY), midtop=(r.centerx, y))
            y += line_h

    def draw_mode(self, screen, v, r):
        rot = v['mode'] == 'rotate'
        pygame.draw.rect(screen, ORANGE if rot else BLUE, r)
        self.blit(screen, 'ROTATE' if rot else 'TRANSLATE', self.f['band'], WHITE, center=r.center)

    def draw_speed(self, screen, v, r):
        s = self.s
        lv = v['level'] if isinstance(v['level'], int) and 1 <= v['level'] <= LEVEL_MAX else None
        self.blit(screen, 'SPEED', self.f['small'], GREY, topleft=(r.x + int(12 * s), r.y + int(6 * s)))
        num = self.blit(screen, '--' if lv is None else str(lv), self.f['level'], WHITE,
                        bottomleft=(r.x + int(10 * s), r.bottom))
        if isinstance(v['mult'], (int, float)):
            self.blit(screen, f'x{v["mult"]:.2f}', self.f['small'], GREY, bottomleft=(num.right + int(6 * s),
                                                                                     r.bottom - int(8 * s)))
        x0, x1 = r.x + int(r.width * 0.42), r.right - int(12 * s)
        seg = (x1 - x0) / LEVEL_MAX
        top, h = r.y + int(8 * s), r.height - int(16 * s)
        for i in range(LEVEL_MAX):
            sh = int(h * (0.3 + 0.7 * (i + 1) / LEVEL_MAX))
            on = lv is not None and i < lv
            pygame.draw.rect(screen, CYAN if on else DIM, (int(x0 + i * seg), top + h - sh, max(1, int(seg) - 2), sh))

    def cover(self, screen, title, sub):
        W, H = self.W, self.H
        shade = pygame.Surface((W, H), pygame.SRCALPHA)
        shade.fill((10, 10, 12, 225))
        screen.blit(shade, (0, 0))
        box = pygame.Rect(0, 0, int(W * 0.86), int(H * 0.42))
        box.center = (W // 2, H // 2)
        pygame.draw.rect(screen, PANEL, box, border_radius=10)
        pygame.draw.rect(screen, GREY, box, 3, border_radius=10)
        self.blit(screen, title, self.f['state_s'], WHITE, center=(box.centerx, box.y + box.height * 0.36))
        line = self.wrap(sub, self.f['fix'], box.width - 20, 1)[0]
        self.blit(screen, line, self.f['fix'], GREY, center=(box.centerx, box.y + box.height * 0.74))


# ------------------------------------------------------------------ data sources

class RosSource:
    """Latest /amw/status, judged fresh by arrival time on this machine."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('amw_lcd')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, TOPIC, self.receive, qos)
        self.data, self.last, self.error = None, None, ''
        self.waiting = f'Arm not running (ROS_DOMAIN_ID {os.environ.get("ROS_DOMAIN_ID", "0")})'

    def receive(self, msg):
        try:
            d = json.loads(msg.data)
            if not isinstance(d, dict):
                raise ValueError
            self.data, self.last, self.error = d, time.monotonic(), ''
        except ValueError:
            self.error = 'Unreadable status message'

    def poll(self, now):
        self.rclpy.spin_once(self.node, timeout_sec=0)
        if self.last is None:
            return None, self.waiting
        if self.error:
            return None, self.error
        if now - self.last > STALE_S:
            return view(self.data), 'Arm software stopped'
        return view(self.data), ''

    def key(self, ch):
        pass

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


DEMO_STOP = {'code': 'JOINT_OUTSIDE_LIMIT', 'kind': 'stop', 'text': 'joint_3 is at +3.1 deg, outside its limits '
             '-84.8..+0.0 deg.', 'fix': 'Turn off (Ctrl-C amw start), move it inside by hand, restart.'}


class DemoSource:
    def __init__(self, state=None):
        self.d = {'state': 'active', 'mode': 'translation', 'level': 10, 'multiplier': 1.0, 'joystick': 'ok',
                  'stop': None, 'warning': None}
        self.stale = ''
        for k in state or '':
            self.key(k)

    def key(self, ch):
        d = self.d
        if ch == 'o':
            on = d['state'] != 'active'
            d['state'] = 'active' if on else 'off'
            d['stop'] = None if on else {'code': 'TURNED_OFF', 'kind': 'stop', 'text': 'Turned off with the D-pad right button.',
                                         'fix': 'Press ON (D-pad right) to continue.'}
        elif ch == 'c':
            d['state'] = 'centre'
            d['stop'] = {'code': 'MODE_CHANGED', 'kind': 'pause', 'text': 'Switched to rotation mode.',
                         'fix': 'Centre the stick to continue.'}
        elif ch == 's':
            d['state'], d['stop'] = 'off', DEMO_STOP
        elif ch == 'w':
            d['warning'] = None if d['warning'] else {'text': 'joint_3 is at its upper limit; motion that way is blocked.'}
        elif ch == 'm':
            d['mode'] = 'rotation' if d['mode'] == 'translation' else 'translation'
        elif ch in '+=':
            d['level'] = min(LEVEL_MAX, d['level'] + 1)
        elif ch in '-_':
            d['level'] = max(1, d['level'] - 1)
        elif ch == 'j':
            d['joystick'] = 'lost' if d['joystick'] == 'ok' else 'ok'
            if d['joystick'] == 'lost':
                d['state'] = 'off'
        elif ch == 'n':
            self.stale = '' if self.stale else 'Arm software stopped'
        lv = d['level']
        d['multiplier'] = 10 ** ((lv - 10) / 9) if lv <= 10 else 2 ** ((lv - 10) / 10)

    def poll(self, now):
        return view(self.d), self.stale

    def close(self):
        pass


# ------------------------------------------------------------------ outputs

def find_fb():
    """The LCD framebuffer: an SPI panel (fbtft) if present, else a DRM/HDMI one, else fb0."""
    found = []
    for d in sorted(glob.glob('/sys/class/graphics/fb[0-9]*')):
        try:
            found.append(('/dev/' + os.path.basename(d), open(f'{d}/name').read().strip().lower()))
        except OSError:
            continue
    for want in ('fb_', 'ili', 'drm'):
        for dev, name in found:
            if want in name:
                return dev
    return found[0][0] if found else None


class Framebuffer:
    def __init__(self, dev, rotate):
        def sysfs(attr):
            return open(f'/sys/class/graphics/{os.path.basename(dev)}/{attr}').read().strip()
        fw, fh = map(int, sysfs('virtual_size').split(','))
        bpp = int(sysfs('bits_per_pixel'))
        try:
            self.stride = int(sysfs('stride'))
        except OSError:
            self.stride = fw * bpp // 8
        masks = {16: (0xF800, 0x07E0, 0x001F, 0), 32: (0xFF0000, 0x00FF00, 0x0000FF, 0)}.get(bpp)
        if masks is None:
            sys.exit(f'{dev}: unsupported {bpp} bits per pixel')
        self.buf = pygame.Surface((fw, fh), 0, bpp, masks)
        self.rotate = rotate
        self.size = (fw, fh) if rotate in (0, 180) else (fh, fw)
        print(f'amw_lcd: {dev} {fw}x{fh}, {bpp} bpp, rotate {rotate} -> {self.size[0]}x{self.size[1]}', flush=True)
        self.f = open(dev, 'r+b', buffering=0)

    def show(self, surf):
        if self.rotate:
            surf = pygame.transform.rotate(surf, self.rotate)
        self.buf.blit(surf, (0, 0))
        proxy = self.buf.get_buffer()
        data = proxy.raw
        del proxy
        pitch = self.buf.get_pitch()
        if pitch != self.stride:
            row = self.buf.get_width() * self.buf.get_bytesize()
            pad = b'\0' * (self.stride - row)
            data = b''.join(data[y * pitch:y * pitch + row] + pad for y in range(self.buf.get_height()))
        self.f.seek(0)
        self.f.write(data)

    def close(self):
        self.f.close()


class TerminalKeys:
    """Single-key reads from the SSH terminal (q quits); no-op without a tty."""

    def __init__(self):
        self.fd = None
        if sys.stdin.isatty():
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def read(self):
        keys = []
        while self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            keys.append(os.read(self.fd, 1).decode(errors='ignore'))
        return keys

    def close(self):
        if self.fd is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def run_fb(src, dev, rotate):
    pygame.font.init()                      # fonts and surfaces only: no X, Wayland or SDL video needed
    dev = dev or find_fb()
    if dev is None:
        sys.exit('No framebuffer found. Is the LCD driver loaded? (see docs/lcd.md)')
    fb, keys = Framebuffer(dev, rotate), TerminalKeys()
    canvas, painter, last = pygame.Surface(fb.size), Painter(fb.size), None
    try:
        while 'q' not in (ks := keys.read()):
            for k in ks:
                src.key(k)
            now = time.monotonic()
            v, stale = src.poll(now)
            hb = int(now) % 2 == 0
            frame = (json.dumps(v, sort_keys=True, default=str), stale, hb)
            if frame != last:                # repaint only on change (and once a second for the heartbeat)
                painter.draw(canvas, v, stale, hb)
                fb.show(canvas)
                last = frame
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        painter.draw(canvas, None, '', False)
        painter.cover(canvas, 'OFFLINE', 'Status screen stopped')
        fb.show(canvas)
        keys.close()
        fb.close()
        src.close()


def run_window(src, size, fullscreen):
    pygame.init()
    screen = pygame.display.set_mode(size, pygame.FULLSCREEN if fullscreen else 0)
    pygame.display.set_caption('AMW status')
    painter, clock = Painter(screen.get_size()), pygame.time.Clock()
    try:
        running = True
        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    ch = {pygame.K_UP: '+', pygame.K_DOWN: '-', pygame.K_ESCAPE: 'q'}.get(e.key, e.unicode)
                    if ch == 'q':
                        running = False
                    else:
                        src.key(ch)
            now = time.monotonic()
            v, stale = src.poll(now)
            painter.draw(screen, v, stale, int(now) % 2 == 0)
            pygame.display.flip()
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        pygame.quit()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--fb', action='store_true', help='draw on the LCD framebuffer (Pi without a desktop)')
    p.add_argument('--fb-dev', help='framebuffer device (default: auto-detect)')
    p.add_argument('--rotate', type=int, default=0, choices=(0, 90, 180, 270), help='framebuffer rotation')
    p.add_argument('--size', default='480x320', help='window/snapshot size, e.g. 800x480')
    p.add_argument('--fullscreen', action='store_true', help='window mode: fill the screen')
    p.add_argument('--demo', action='store_true', help='no ROS; keyboard-driven states')
    p.add_argument('--state', default='', help='with --demo: keys to apply first, e.g. "sw"')
    p.add_argument('--snap', metavar='PNG', help='with --demo: save one frame and exit')
    args, _ = p.parse_known_args()           # ignore --ros-args added by ros2 run / launch
    size = tuple(int(x) for x in args.size.lower().split('x'))
    if args.snap:
        pygame.font.init()
        src, surf = DemoSource(args.state), pygame.Surface(size)
        v, stale = src.poll(0)
        Painter(size).draw(surf, v, stale)
        pygame.image.save(surf, args.snap)
        return
    src = DemoSource(args.state) if args.demo else RosSource()
    if args.fb:
        run_fb(src, args.fb_dev, args.rotate)
    else:
        run_window(src, size, args.fullscreen)


if __name__ == '__main__':
    main()
