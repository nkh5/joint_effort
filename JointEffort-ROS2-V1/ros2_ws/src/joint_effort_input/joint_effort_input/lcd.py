#!/usr/bin/env python3
"""amw_lcd - AMW status screen for the Waveshare 4inch HDMI LCD (480x800 panel, used as 800x480).

Reads /amw/display_state, which the input forwarder (pico_forwarder) publishes. Shows the forwarder's
own state, so the screen can never claim ENABLED when the forwarder is not driving the arm.

Self-contained: needs only rclpy, std_msgs and pygame. You can run it from the built workspace
(ros2 run joint_effort_input amw_lcd) or copy this one file to the Pi and run it with python3.

  python3 lcd.py --fb                 Pi, no desktop: draw straight to the HDMI framebuffer
  python3 lcd.py                      desktop window (preview on a PC)
  python3 lcd.py --demo               no ROS: keyboard-driven states, for layout work
  python3 lcd.py --demo --snap a.png  save one frame and exit

  --rotate 0|90|180|270   the panel's framebuffer is portrait (480x800); the default 90 turns the
                          landscape screen to fit it. Use 270 if the picture is upside down.
  --fb-dev /dev/fbN       framebuffer to use (default: auto)

Needs ROS_DOMAIN_ID=88 (the input stack's domain) and the same network as the PC running the
input launch. Staleness is judged by arrival time on this machine, so the Pi and PC clocks do not
have to agree.

Demo keys: space enabled, m mode, c centre-stick wait, +/- speed, f servo halt, w servo warning,
j no controller, u unconfirmed mapping, n no data, q quit.
"""

import argparse
import glob
import json
import math
import os
import select
import sys
import time

import pygame

W, H = 800, 480
TOPIC = '/amw/display_state'
STALE_S = 0.6            # forwarder publishes at ~20 Hz; no message for this long = NO DATA
LEVEL_MAX = 20
# MoveIt Servo status codes (moveit_msgs/ServoStatus). The forwarder keeps driving on 0, 1, 3, 4.
SERVO_OK = {0}
SERVO_SLOWING = {1, 3, 4}            # decelerating near a singularity / collision
SERVO_NAMES = {-1: 'Servo status invalid', 2: 'Halted: singularity', 5: 'Halted: collision',
               6: 'Halted: joint limit'}

# ------------------------------------------------------------------ style
GREEN = (30, 150, 80)
AMBER = (215, 150, 20)
DIS_BG = (55, 55, 58)
DIS_FG = (170, 170, 175)
RED = (205, 45, 45)
BLUE = (35, 105, 205)        # translate
ORANGE = (230, 135, 15)      # rotate (blue/orange = colour-blind safe pair)
BG = (14, 14, 16)
PANEL = (28, 28, 32)
WHITE = (240, 240, 240)
GREY = (140, 140, 145)
DIM = (70, 70, 75)
CYAN = (90, 200, 235)
FONT_FILE = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

BANNER_H = 56
MODE_H = 132
SPEED_H = 150


def font(size):
    try:
        return pygame.font.Font(FONT_FILE, size)
    except (FileNotFoundError, OSError):
        return pygame.font.Font(None, int(size * 1.35))


def make_fonts():
    return {'band': font(72), 'band_s': font(60), 'msg': font(24), 'banner': font(28),
            'label': font(24), 'level': font(96), 'of': font(40), 'mult': font(30),
            'big': font(60), 'hint': font(24)}


def blit(screen, text, f, color, **pos):
    surf = f.render(text, True, color)
    rect = surf.get_rect(**pos)
    screen.blit(surf, rect)
    return rect


def fit(text, f, width):
    """Shorten text with an ellipsis so it fits in width pixels."""
    if f.size(text)[0] <= width:
        return text
    while text and f.size(text + '…')[0] > width:
        text = text[:-1]
    return text + '…'


def arrow_head(screen, color, tip, angle, size):
    a1, a2 = angle + math.radians(150), angle - math.radians(150)
    pygame.draw.polygon(screen, color, [tip, (tip[0] + size * math.cos(a1), tip[1] + size * math.sin(a1)),
                                        (tip[0] + size * math.cos(a2), tip[1] + size * math.sin(a2))])


# ------------------------------------------------------------------ icons (r = radius in px)

def icon_enabled(screen, c, on, color, bg, r=28):
    if on:
        pygame.draw.circle(screen, color, c, r)
        k = r / 17
        pygame.draw.lines(screen, bg, False, [(c[0] - 8 * k, c[1]), (c[0] - 2 * k, c[1] + 7 * k),
                                              (c[0] + 9 * k, c[1] - 7 * k)], 7)
    else:
        pygame.draw.circle(screen, color, c, r, 6)
        pygame.draw.line(screen, color, (c[0] - r * .45, c[1]), (c[0] + r * .45, c[1]), 6)


def icon_centre(screen, c, color, r=28):
    pygame.draw.circle(screen, color, c, r, 5)
    pygame.draw.circle(screen, color, c, 7)
    for ang in (0, 90, 180, 270):
        a = math.radians(ang)
        arrow_head(screen, color, (c[0] + (r - 10) * math.cos(a), c[1] + (r - 10) * math.sin(a)), a + math.pi, 11)


def icon_translate(screen, c, color, r=30):
    x, y = c
    pygame.draw.line(screen, color, (x - r + 6, y), (x + r - 6, y), 6)
    pygame.draw.line(screen, color, (x, y - r + 6), (x, y + r - 6), 6)
    for ang in (0, 90, 180, 270):
        a = math.radians(ang)
        arrow_head(screen, color, (x + r * math.cos(a), y + r * math.sin(a)), a, 15)


def icon_rotate(screen, c, color, r=26):
    rect = pygame.Rect(0, 0, 2 * r, 2 * r)
    rect.center = c
    pygame.draw.arc(screen, color, rect, math.radians(-30), math.radians(240), 6)
    a = math.radians(-30)
    tip = (c[0] + r * math.cos(a), c[1] - r * math.sin(a))
    arrow_head(screen, color, (tip[0] - 1, tip[1] + 6), math.radians(100), 16)


def icon_plug(screen, c, color):
    x, y = c
    pygame.draw.rect(screen, color, (x - 15, y - 10, 30, 23), border_radius=4)
    pygame.draw.line(screen, color, (x - 7, y - 10), (x - 7, y - 23), 5)
    pygame.draw.line(screen, color, (x + 7, y - 10), (x + 7, y - 23), 5)
    pygame.draw.line(screen, color, (x, y + 13), (x, y + 26), 5)
    pygame.draw.line(screen, color, (x - 25, y + 25), (x + 25, y - 25), 5)


def icon_warning(screen, c, color, fg):
    x, y = c
    pygame.draw.polygon(screen, color, [(x, y - 18), (x + 20, y + 15), (x - 20, y + 15)])
    pygame.draw.line(screen, fg, (x, y - 6), (x, y + 5), 4)
    pygame.draw.circle(screen, fg, (x, y + 10), 3)


# ------------------------------------------------------------------ state -> view

def view(d):
    """Turn one /amw/display_state message into what the screen shows. Pure; tested."""
    msg = str(d.get('message', ''))
    code = d.get('servo_code')
    v = {'mode': 'rotate' if d.get('mode') == 'rotation' else 'translate',
         'level': d.get('level'), 'mult': d.get('multiplier'), 'banner': None, 'hint': msg}
    if code is not None and code not in SERVO_OK:
        text = d.get('servo_message') or SERVO_NAMES.get(code, f'Servo status {code}')
        v['banner'] = ('warn' if code in SERVO_SLOWING else 'fault', text)
    elif d.get('collision_clear') is False and d.get('input_fresh'):
        v['banner'] = ('fault', 'Collision check not clear')
    if not d.get('input_fresh', False):
        v['status'] = 'noctrl'
        v['hint'] = 'Pico not connected or not sending'
    elif d.get('enabled'):
        v['status'] = 'enabled'
        v['hint'] = ''
    elif msg.startswith('Mode changed; center the stick'):
        v['status'] = 'centre'
        v['hint'] = 'Let go of the stick to continue in the new mode'
    else:
        v['status'] = 'disabled'
        if not d.get('mapping_confirmed', True):
            v['hint'] = 'Joystick mapping unconfirmed - motion inhibited'
        elif msg.startswith('Ready: hold enable'):
            v['hint'] = 'Ready - press ON (panel 1)'
        elif 'center and release' in msg or msg.startswith('Enable released'):
            v['hint'] = 'Centre the stick, then press ON (panel 1 OFF then ON)'
    return v


# ------------------------------------------------------------------ drawing

def draw_banner(screen, fonts, kind, text, y):
    bg = RED if kind == 'fault' else AMBER
    pygame.draw.rect(screen, bg, (0, y, W, BANNER_H))
    icon_warning(screen, (34, y + BANNER_H // 2), WHITE, bg)
    blit(screen, fit(text, fonts['banner'], W - 90), fonts['banner'], WHITE, midleft=(68, y + BANNER_H // 2 + 1))


def draw_status(screen, fonts, v, rect):
    s = v['status']
    if s == 'enabled':
        bg, fg, label = GREEN, WHITE, 'ENABLED'
    elif s == 'centre':
        bg, fg, label = DIS_BG, AMBER, 'CENTRE STICK'   # grey, not orange: sits above the ROTATE band
    elif s == 'noctrl':
        bg, fg, label = DIS_BG, DIS_FG, 'NO CONTROLLER'
    else:
        bg, fg, label = DIS_BG, DIS_FG, 'DISABLED'
    pygame.draw.rect(screen, bg, rect)
    big = fonts['band'] if s in ('enabled', 'disabled') else fonts['band_s']
    hint = v['hint'] if s != 'enabled' else ''
    cy = rect.centery - (16 if hint else 0)
    tw = big.size(label)[0]
    x0 = rect.centerx - (tw + 80) // 2
    icon_c = (x0 + 28, cy)
    if s == 'noctrl':
        icon_plug(screen, icon_c, fg)
    elif s == 'centre':
        icon_centre(screen, icon_c, fg)
    else:
        icon_enabled(screen, icon_c, s == 'enabled', fg, bg)
    blit(screen, label, big, fg, midleft=(x0 + 80, cy + 2))
    if hint:
        blit(screen, fit(hint, fonts['msg'], W - 40), fonts['msg'], fg, center=(rect.centerx, rect.bottom - 26))


def draw_mode(screen, fonts, v, rect):
    rot = v['mode'] == 'rotate'
    pygame.draw.rect(screen, ORANGE if rot else BLUE, rect)
    label = 'ROTATE' if rot else 'TRANSLATE'
    tw = fonts['band'].size(label)[0]
    x0 = rect.centerx - (tw + 84) // 2
    (icon_rotate if rot else icon_translate)(screen, (x0 + 30, rect.centery), WHITE)
    blit(screen, label, fonts['band'], WHITE, midleft=(x0 + 84, rect.centery + 2))


def draw_speed(screen, fonts, v, rect):
    pygame.draw.rect(screen, BG, rect)
    lv = v['level'] if isinstance(v['level'], int) and 1 <= v['level'] <= LEVEL_MAX else None
    blit(screen, 'SPEED', fonts['label'], GREY, topleft=(rect.x + 24, rect.y + 14))
    r = blit(screen, '--' if lv is None else str(lv), fonts['level'], WHITE,
             bottomleft=(rect.x + 20, rect.bottom - 4))
    blit(screen, f'/{LEVEL_MAX}', fonts['of'], GREY, bottomleft=(r.right + 6, rect.bottom - 22))
    if isinstance(v['mult'], (int, float)):
        blit(screen, f'×{v["mult"]:.2f}', fonts['mult'], GREY, topright=(rect.right - 24, rect.y + 12))
    x0, x1 = rect.x + 290, rect.right - 24          # 20 segments, one per encoder click
    seg_w = (x1 - x0) / LEVEL_MAX
    top, h = rect.y + 52, rect.height - 72
    for i in range(LEVEL_MAX):
        sh = int(h * (0.35 + 0.65 * (i + 1) / LEVEL_MAX))
        on = lv is not None and i < lv
        pygame.draw.rect(screen, CYAN if on else DIM, (int(x0 + i * seg_w), top + h - sh, int(seg_w) - 4, sh),
                         border_radius=3)


def draw_cover(screen, fonts, title, sub):
    shade = pygame.Surface((W, H), pygame.SRCALPHA)
    shade.fill((10, 10, 12, 220))
    screen.blit(shade, (0, 0))
    box = pygame.Rect(0, 0, 560, 170)
    box.center = (W // 2, H // 2)
    pygame.draw.rect(screen, PANEL, box, border_radius=14)
    pygame.draw.rect(screen, GREY, box, 4, border_radius=14)
    blit(screen, title, fonts['big'], WHITE, center=(box.centerx, box.y + 62))
    blit(screen, fit(sub, fonts['hint'], box.width - 40), fonts['hint'], GREY, center=(box.centerx, box.bottom - 40))


def draw(screen, fonts, v, stale='', hb=True):
    """v: output of view() (or None before any data); stale: non-empty = cover with this reason."""
    screen.fill(BG)
    if v is None:
        v = {'mode': 'translate', 'level': None, 'mult': None, 'banner': None, 'status': 'disabled', 'hint': ''}
    y = 0
    if v['banner']:
        draw_banner(screen, fonts, *v['banner'], y)
        y += BANNER_H
    draw_status(screen, fonts, v, pygame.Rect(0, y, W, H - SPEED_H - MODE_H - y))
    draw_mode(screen, fonts, v, pygame.Rect(0, H - SPEED_H - MODE_H, W, MODE_H))
    draw_speed(screen, fonts, v, pygame.Rect(0, H - SPEED_H, W, SPEED_H))
    # heartbeat: alternates every second so a frozen screen is obvious
    pygame.draw.circle(screen, CYAN if hb else DIM, (W - 16, H - 16), 7)
    if stale:
        draw_cover(screen, fonts, 'NO DATA', stale)


# ------------------------------------------------------------------ data sources

class RosSource:
    """Latest /amw/display_state, judged fresh by arrival time on this machine."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('amw_lcd')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)   # matches the forwarder's STATUS
        self.node.create_subscription(String, TOPIC, self.receive, qos)
        self.data, self.last, self.error = None, None, ''
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        self.waiting = f'Waiting for {TOPIC} (ROS_DOMAIN_ID {domain}; input stack uses 88)'

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
            return view(self.data), 'Input node stopped or network lost'
        return view(self.data), ''

    def key(self, ch):
        pass

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


class DemoSource:
    def __init__(self):
        self.d = {'mode': 'translation', 'level': 10, 'multiplier': 1.0, 'enabled': True, 'input_fresh': True,
                  'mapping_confirmed': True, 'message': 'Moving arm while enabled', 'servo_code': 0,
                  'servo_message': 'No warnings', 'collision_clear': True}
        self.stale = ''

    def key(self, ch):
        d = self.d
        if ch == ' ':
            d['enabled'] = not d['enabled']
            d['message'] = 'Moving arm while enabled' if d['enabled'] else 'Ready: hold enable button to move'
        elif ch == 'm':
            d['mode'] = 'rotation' if d['mode'] == 'translation' else 'translation'
        elif ch == 'c':
            d['enabled'] = False; d['message'] = 'Mode changed; center the stick to continue'
        elif ch in '+=':
            d['level'] = min(20, d['level'] + 1)
        elif ch in '-_':
            d['level'] = max(1, d['level'] - 1)
        elif ch == 'f':
            d['servo_code'] = 0 if d['servo_code'] == 5 else 5
            d['servo_message'] = 'Very close to collision. Halting.'
        elif ch == 'w':
            d['servo_code'] = 0 if d['servo_code'] == 4 else 4
            d['servo_message'] = 'Close to a collision, decelerating.'
        elif ch == 'j':
            d['input_fresh'] = not d['input_fresh']
        elif ch == 'u':
            d['mapping_confirmed'] = not d['mapping_confirmed']; d['enabled'] = False
            d['message'] = 'Joystick mapping unconfirmed; motion inhibited'
        elif ch == 'n':
            self.stale = '' if self.stale else 'Input node stopped or network lost'
        if d['level'] is not None:
            lv = d['level']
            d['multiplier'] = 10 ** ((lv - 10) / 9) if lv <= 10 else 2 ** ((lv - 10) / 10)

    def poll(self, now):
        return view(self.d), self.stale

    def close(self):
        pass


# ------------------------------------------------------------------ outputs

def find_fb():
    """The Pi's HDMI framebuffer (KMS fbdev emulation). Prefers a DRM-backed fb, else fb0."""
    found = []
    for d in sorted(glob.glob('/sys/class/graphics/fb[0-9]*')):
        try:
            name = open(f'{d}/name').read().strip()
        except OSError:
            continue
        found.append(('/dev/' + os.path.basename(d), name))
    for dev, name in found:
        if 'drm' in name.lower():
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
        out = (W, H) if rotate in (0, 180) else (H, W)
        if out != (fw, fh):
            sys.exit(f'{dev} is {fw}x{fh}, but the screen rotated {rotate} deg is {out[0]}x{out[1]}.\n'
                     'Set the HDMI mode to 480x800 (see LCD-SETUP.md) or pick another --rotate.')
        print(f'amw_lcd: {dev} {fw}x{fh}, {bpp} bpp, rotate {rotate}', flush=True)
        self.f = open(dev, 'r+b', buffering=0)

    def show(self, surf):
        if self.rotate:
            surf = pygame.transform.rotate(surf, self.rotate)
        self.buf.blit(surf, (0, 0))
        proxy = self.buf.get_buffer()
        data = proxy.raw
        del proxy                            # unlock the surface for the next blit
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
        sys.exit('No framebuffer found. Is the HDMI LCD connected and KMS enabled? (see LCD-SETUP.md)')
    fb, keys, canvas, fonts, last = Framebuffer(dev, rotate), TerminalKeys(), pygame.Surface((W, H)), make_fonts(), None
    try:
        while 'q' not in (ks := keys.read()):
            for k in ks:
                src.key(k)
            now = time.monotonic()
            v, stale = src.poll(now)
            hb = int(now) % 2 == 0
            frame = (json.dumps(v, sort_keys=True, default=str), stale, hb)
            if frame != last:                # repaint only on change (and once a second for the heartbeat)
                draw(canvas, fonts, v, stale, hb)
                fb.show(canvas)
                last = frame
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        draw(canvas, fonts, None, 'Status screen stopped', False)
        draw_cover(canvas, fonts, 'OFFLINE', 'Status screen stopped')
        fb.show(canvas)
        keys.close(); fb.close(); src.close()


def run_window(src, fullscreen):
    pygame.init()
    screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN if fullscreen else 0)
    pygame.display.set_caption('AMW status')
    fonts, clock = make_fonts(), pygame.time.Clock()
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
            draw(screen, fonts, v, stale, int(now) % 2 == 0)
            pygame.display.flip()
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        pygame.quit()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--fb', action='store_true', help='draw to the HDMI framebuffer (Pi without a desktop)')
    p.add_argument('--fb-dev', help='framebuffer device (default: auto-detect)')
    p.add_argument('--rotate', type=int, default=90, choices=(0, 90, 180, 270),
                   help='framebuffer rotation; panel is portrait, so 90 (default) or 270')
    p.add_argument('--fullscreen', action='store_true', help='window mode: fill the screen')
    p.add_argument('--demo', action='store_true', help='no ROS; keyboard-driven states')
    p.add_argument('--snap', metavar='PNG', help='with --demo: save one frame and exit')
    args, _ = p.parse_known_args()           # ignore --ros-args added by ros2 run / launch
    if args.snap:
        pygame.font.init()
        src = DemoSource(); surf = pygame.Surface((W, H))
        v, stale = src.poll(0)
        draw(surf, make_fonts(), v, stale)
        pygame.image.save(surf, args.snap)
        return
    src = DemoSource() if args.demo else RosSource()
    if args.fb:
        run_fb(src, args.fb_dev, args.rotate)
    else:
        run_window(src, args.fullscreen)


if __name__ == '__main__':
    main()
