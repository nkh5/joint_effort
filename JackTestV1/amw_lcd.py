#!/usr/bin/env python3
"""amw_lcd.py - AMW status screen (Waveshare 3.5" RPi LCD (B), 480x320) as a ROS 2 node.

Reads:
  /joy               mode + enabled (buttons 7/8/9 from the Pico), controller link
  /amw/speed_level   encoder level 1-20 (latched, from amw_speed.py)
  ROS node graph     ARM dot = joy_teleop and amw_speed are running

Run:
  python3 amw_lcd.py           -> 480x320 window (PC)
  python3 amw_lcd.py --lcd     -> straight to the LCD framebuffer (Pi, no desktop)
  python3 amw_lcd.py --fake    -> no ROS; fake values from the keyboard (design work)
  python3 amw_lcd.py --snap x.png --fake   -> save one frame and exit

Fake-mode keys: space enable, m mode, +/- speed, 1 2 3 CTRL/SERVO/ARM dots,
f fault, n no data, q quit.  In ROS modes only q quits.
"""

import argparse
import glob
import math
import os
import select
import sys
import time

import pygame

W, H = 480, 320

# ------------------------------------------------------------------ config
BTN_TRANSLATE, BTN_ROTATE, BTN_ROTATE_SEL = 7, 8, 9   # must match the Pico
ARM_NODES = ("joy_teleop", "amw_speed")               # all must be running for ARM
JOY_NODE = "joy_node"
CTRL_TIMEOUT_S = 0.5      # no /joy for this long -> controller lost (joy publishes at 50 Hz)
GRAPH_PERIOD_S = 1.0      # how often to check which nodes are running
LEVEL_MAX = 20
FAKE_FAULT = "SERVO 3 OVERTEMP"

# ------------------------------------------------------------------ style
# Red is reserved for faults; disabled is a neutral grey, not red.
GREEN = (30, 150, 80)
DIS_BG = (55, 55, 58)
DIS_FG = (165, 165, 170)
RED = (205, 45, 45)
BLUE = (35, 105, 205)       # translate
ORANGE = (230, 135, 15)     # rotate (blue/orange = colour-blind safe pair)
BG = (14, 14, 16)
PANEL = (28, 28, 32)
WHITE = (240, 240, 240)
GREY = (140, 140, 145)
DIM = (70, 70, 75)
OK = (60, 200, 110)
CYAN = (90, 200, 235)

FONT_FILE = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FLASH_S = 0.3               # highlight time when a value changes

TOP_H = 26                  # connection strip
FAULT_H = 34
MODE_H = 92
SPEED_H = 96


def new_state():
    return {
        "enabled": False,
        "mode": "translate",
        "level": None,                  # None = unknown (amw_speed not seen yet)
        "links": {"CTRL": False, "SERVO": None, "ARM": False},  # None = not monitored
        "fault": "",
        "stale": "",                    # non-empty = cover the screen with this reason
    }


# ---------------------------------------------------------------- helpers

def font(size):
    try:
        return pygame.font.Font(FONT_FILE, size)
    except (FileNotFoundError, OSError):
        return pygame.font.Font(None, int(size * 1.35))


def make_fonts():
    return {
        "band": font(46),
        "band_s": font(38),
        "hint": font(15),
        "label": font(15),
        "level": font(64),
        "of": font(26),
        "fault": font(19),
        "big": font(40),
    }


def blit(screen, s, f, color, **pos):
    surf = f.render(s, True, color)
    rect = surf.get_rect(**pos)
    screen.blit(surf, rect)
    return rect


def arrow_head(screen, color, tip, angle, size):
    a1, a2 = angle + math.radians(150), angle - math.radians(150)
    pts = [tip,
           (tip[0] + size * math.cos(a1), tip[1] + size * math.sin(a1)),
           (tip[0] + size * math.cos(a2), tip[1] + size * math.sin(a2))]
    pygame.draw.polygon(screen, color, pts)


# ------------------------------------------------------------------ icons

def icon_enabled(screen, c, on, color):
    if on:
        pygame.draw.circle(screen, color, c, 17)
        pygame.draw.lines(screen, GREEN, False,
                          [(c[0] - 8, c[1]), (c[0] - 2, c[1] + 7), (c[0] + 9, c[1] - 7)], 4)
    else:
        pygame.draw.circle(screen, color, c, 17, 4)
        pygame.draw.line(screen, color, (c[0] - 7, c[1]), (c[0] + 7, c[1]), 4)


def icon_translate(screen, c, color, r=17):
    x, y = c
    pygame.draw.line(screen, color, (x - r + 4, y), (x + r - 4, y), 4)
    pygame.draw.line(screen, color, (x, y - r + 4), (x, y + r - 4), 4)
    for ang in (0, 90, 180, 270):
        a = math.radians(ang)
        arrow_head(screen, color, (x + r * math.cos(a), y + r * math.sin(a)), a, 9)


def icon_rotate(screen, c, color, r=15):
    rect = pygame.Rect(0, 0, 2 * r, 2 * r)
    rect.center = c
    pygame.draw.arc(screen, color, rect, math.radians(-30), math.radians(240), 4)
    a = math.radians(-30)
    tip = (c[0] + r * math.cos(a), c[1] - r * math.sin(a))
    arrow_head(screen, color, (tip[0] - 1, tip[1] + 4), math.radians(100), 10)


def icon_plug(screen, c, color):
    x, y = c
    pygame.draw.rect(screen, color, (x - 9, y - 6, 18, 14), border_radius=3)
    pygame.draw.line(screen, color, (x - 4, y - 6), (x - 4, y - 14), 3)
    pygame.draw.line(screen, color, (x + 4, y - 6), (x + 4, y - 14), 3)
    pygame.draw.line(screen, color, (x, y + 8), (x, y + 16), 3)
    pygame.draw.line(screen, color, (x - 15, y + 15), (x + 15, y - 15), 3)  # strike-through


def icon_warning(screen, c, color, fg):
    x, y = c
    pygame.draw.polygon(screen, color, [(x, y - 12), (x + 13, y + 10), (x - 13, y + 10)])
    pygame.draw.line(screen, fg, (x, y - 4), (x, y + 3), 3)
    pygame.draw.circle(screen, fg, (x, y + 7), 2)


# ------------------------------------------------------------------ parts

def draw_top(screen, fonts, s, hb):
    pygame.draw.rect(screen, PANEL, (0, 0, W, TOP_H))
    x = 10
    for name, ok in s["links"].items():
        c = (x + 6, TOP_H // 2)
        if ok:
            pygame.draw.circle(screen, OK, c, 6)
        elif ok is None:                       # not monitored yet
            pygame.draw.circle(screen, DIM, c, 6, 2)
        else:
            pygame.draw.circle(screen, GREY, c, 6, 2)
            pygame.draw.line(screen, GREY, (c[0] - 4, c[1] - 4), (c[0] + 4, c[1] + 4), 2)
        r = blit(screen, name, fonts["label"], WHITE if ok else (DIM if ok is None else GREY),
                 midleft=(x + 16, TOP_H // 2))
        x = r.right + 16
    blit(screen, "AMW", fonts["label"], GREY, midright=(W - 26, TOP_H // 2))
    # Heartbeat: alternates every second so a frozen screen is obvious
    pygame.draw.circle(screen, CYAN if hb else DIM, (W - 13, TOP_H // 2), 5)


def draw_fault(screen, fonts, s, y):
    pygame.draw.rect(screen, RED, (0, y, W, FAULT_H))
    icon_warning(screen, (22, y + FAULT_H // 2), WHITE, RED)
    blit(screen, s["fault"], fonts["fault"], WHITE, midleft=(44, y + FAULT_H // 2))


def draw_enable(screen, fonts, s, rect, flash):
    on = s["enabled"]
    bg, fg = (GREEN, WHITE) if on else (DIS_BG, DIS_FG)
    if flash:
        bg, fg = fg, bg
    pygame.draw.rect(screen, bg, rect)
    label = "ENABLED" if on else "DISABLED"
    tw = fonts["band"].size(label)[0]
    x0 = rect.centerx - (tw + 50) // 2
    icon_enabled(screen, (x0 + 17, rect.centery), on, fg)
    blit(screen, label, fonts["band"], fg, midleft=(x0 + 50, rect.centery + 1))


def draw_no_ctrl(screen, fonts, rect):
    pygame.draw.rect(screen, DIS_BG, rect)
    label = "NO CONTROLLER"
    tw = fonts["band_s"].size(label)[0]
    x0 = rect.centerx - (tw + 46) // 2
    icon_plug(screen, (x0 + 16, rect.centery), DIS_FG)
    blit(screen, label, fonts["band_s"], DIS_FG, midleft=(x0 + 46, rect.centery + 1))


def draw_mode(screen, fonts, s, rect, flash):
    rot = s["mode"] == "rotate"
    bg, fg = (ORANGE if rot else BLUE), WHITE
    if flash:
        bg, fg = fg, bg
    pygame.draw.rect(screen, bg, rect)
    label = "ROTATE" if rot else "TRANSLATE"
    tw = fonts["band"].size(label)[0]
    x0 = rect.centerx - (tw + 50) // 2
    (icon_rotate if rot else icon_translate)(screen, (x0 + 17, rect.centery), fg)
    blit(screen, label, fonts["band"], fg, midleft=(x0 + 50, rect.centery + 1))


def draw_speed(screen, fonts, s, rect, flash):
    pygame.draw.rect(screen, WHITE if flash else BG, rect)
    fg = BG if flash else WHITE
    lv = s["level"]
    blit(screen, "SPEED", fonts["label"], GREY if not flash else BG,
         topleft=(rect.x + 16, rect.y + 10))
    r = blit(screen, "--" if lv is None else str(lv), fonts["level"], fg,
             bottomleft=(rect.x + 14, rect.bottom - 6))
    blit(screen, f"/{LEVEL_MAX}", fonts["of"], GREY if not flash else BG,
         bottomleft=(r.right + 4, rect.bottom - 18))
    # 20-segment bar, one segment per encoder click
    x0, x1 = rect.x + 172, rect.right - 16
    seg_w = (x1 - x0) / LEVEL_MAX
    top, h = rect.y + 30, rect.height - 48
    for i in range(LEVEL_MAX):
        sx = int(x0 + i * seg_w)
        sh = int(h * (0.35 + 0.65 * (i + 1) / LEVEL_MAX))  # rising steps
        on = lv is not None and i < lv
        col = (fg if flash else CYAN) if on else ((200, 200, 200) if flash else DIM)
        pygame.draw.rect(screen, col, (sx, top + h - sh, int(seg_w) - 3, sh), border_radius=2)


def draw_cover(screen, fonts, title, sub):
    shade = pygame.Surface((W, H), pygame.SRCALPHA)
    shade.fill((10, 10, 12, 215))
    screen.blit(shade, (0, 0))
    box = pygame.Rect(0, 0, 340, 110)
    box.center = (W // 2, H // 2)
    pygame.draw.rect(screen, PANEL, box, border_radius=10)
    pygame.draw.rect(screen, GREY, box, 3, border_radius=10)
    blit(screen, title, fonts["big"], WHITE, center=(box.centerx, box.y + 40))
    blit(screen, sub, fonts["hint"], GREY, center=(box.centerx, box.bottom - 26))


def draw(screen, fonts, s, flashes=(), hb=True):
    screen.fill(BG)
    draw_top(screen, fonts, s, hb)
    y = TOP_H
    if s["fault"]:
        draw_fault(screen, fonts, s, y)
        y += FAULT_H
    en_rect = pygame.Rect(0, y, W, H - SPEED_H - MODE_H - y)
    if s["links"]["CTRL"] is False:
        draw_no_ctrl(screen, fonts, en_rect)
    else:
        draw_enable(screen, fonts, s, en_rect, "enabled" in flashes)
    y = en_rect.bottom
    draw_mode(screen, fonts, s, pygame.Rect(0, y, W, MODE_H), "mode" in flashes)
    y += MODE_H
    draw_speed(screen, fonts, s, pygame.Rect(0, y, W, SPEED_H), "level" in flashes)
    if s["stale"]:
        draw_cover(screen, fonts, "NO DATA", s["stale"])


class Flasher:
    """Tracks which regions changed recently so they can be highlighted."""

    def __init__(self, s):
        self.prev = {k: s[k] for k in ("enabled", "mode", "level")}
        self.until = {}

    def update(self, s, now):
        for k, v in self.prev.items():
            if s[k] != v:
                if v is not None:          # no flash on the first value we ever get
                    self.until[k] = now + FLASH_S
                self.prev[k] = s[k]
        return frozenset(k for k, t in self.until.items() if now < t)


def snapshot(s):
    return (s["enabled"], s["mode"], s["level"], tuple(s["links"].items()), s["fault"], s["stale"])


# ------------------------------------------------------------ data sources

class FakeSource:
    """Keyboard-driven values for design work. No ROS needed."""

    def __init__(self, s):
        s.update(enabled=True, level=10, links={"CTRL": True, "SERVO": True, "ARM": True})

    def apply_key(self, ch, s):
        if ch == " ":
            s["enabled"] = not s["enabled"]
        elif ch == "m":
            s["mode"] = "rotate" if s["mode"] == "translate" else "translate"
        elif ch in ("+", "="):
            s["level"] = min(LEVEL_MAX, s["level"] + 1)
        elif ch in ("-", "_"):
            s["level"] = max(1, s["level"] - 1)
        elif ch in ("1", "2", "3"):
            name = list(s["links"])[int(ch) - 1]
            s["links"][name] = not s["links"][name]
        elif ch == "f":
            s["fault"] = "" if s["fault"] else FAKE_FAULT
        elif ch == "n":
            s["stale"] = "" if s["stale"] else "Status not updating"

    def poll(self, s, now):
        pass

    def close(self):
        pass


class RosSource:
    """Fills the state from /joy, /amw/speed_level and the node graph."""

    def __init__(self, s):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import Joy
        from std_msgs.msg import Int32

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("amw_lcd")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.node.create_subscription(Int32, "/amw/speed_level", self.on_level, latched)
        self.s = s
        self.last_joy = None
        self.last_graph = -GRAPH_PERIOD_S
        self.joy_node_up = False

    def apply_key(self, ch, s):
        pass

    def on_joy(self, msg):
        b = msg.buttons
        if len(b) <= BTN_ROTATE_SEL:      # wrong device or old firmware
            return
        self.last_joy = time.monotonic()
        self.s["enabled"] = bool(b[BTN_TRANSLATE] or b[BTN_ROTATE])
        self.s["mode"] = "rotate" if b[BTN_ROTATE_SEL] else "translate"

    def on_level(self, msg):
        self.s["level"] = int(msg.data)

    def poll(self, s, now):
        self.rclpy.spin_once(self.node, timeout_sec=0)
        if now - self.last_graph >= GRAPH_PERIOD_S:
            self.last_graph = now
            names = set(self.node.get_node_names())
            self.joy_node_up = JOY_NODE in names
            s["links"]["ARM"] = all(n in names for n in ARM_NODES)
            if "amw_speed" not in names:
                s["level"] = None
        s["links"]["CTRL"] = self.last_joy is not None and now - self.last_joy < CTRL_TIMEOUT_S
        s["stale"] = "" if self.joy_node_up else "joy_node not running"

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# ------------------------------------------------------- framebuffer (Pi)

def find_lcd_fb():
    """The waveshare35b-v2 overlay uses the fbtft ili9486 driver ('fb_ili9486')."""
    for d in sorted(glob.glob("/sys/class/graphics/fb*")):
        try:
            name = open(f"{d}/name").read().strip()
        except OSError:
            continue
        if "ili9486" in name or name.startswith("fb_"):
            return "/dev/" + os.path.basename(d)
    return None


def read_sys(dev, attr):
    return open(f"/sys/class/graphics/{os.path.basename(dev)}/{attr}").read().strip()


class Framebuffer:
    def __init__(self, dev, rotate=None):
        self.dev = dev
        fw, fh = map(int, read_sys(dev, "virtual_size").split(","))
        bpp = int(read_sys(dev, "bits_per_pixel"))
        try:
            self.stride = int(read_sys(dev, "stride"))
        except OSError:
            self.stride = fw * bpp // 8
        if bpp == 16:
            masks = (0xF800, 0x07E0, 0x001F, 0)          # RGB565
        elif bpp == 32:
            masks = (0xFF0000, 0x00FF00, 0x0000FF, 0)    # XRGB8888
        else:
            sys.exit(f"{dev}: unsupported {bpp} bpp")
        self.buf = pygame.Surface((fw, fh), 0, bpp, masks)
        if rotate is None:                                # portrait fb -> rotate to fit
            rotate = 0 if (fw, fh) == (W, H) else 90
        self.rotate = rotate
        print(f"Using {dev}: {fw}x{fh}, {bpp} bpp, rotate {rotate}")
        self.f = open(dev, "r+b", buffering=0)

    def show(self, surf):
        if self.rotate:
            surf = pygame.transform.rotate(surf, self.rotate)
        if surf.get_size() != self.buf.get_size():
            sys.exit(f"Image {surf.get_size()} doesn't fit framebuffer "
                     f"{self.buf.get_size()}; try a different --rotate")
        self.buf.blit(surf, (0, 0))
        proxy = self.buf.get_buffer()
        data = proxy.raw
        del proxy  # unlock the surface for the next blit
        pitch = self.buf.get_pitch()
        if pitch != self.stride:
            row = self.buf.get_width() * self.buf.get_bytesize()
            pad = b"\0" * (self.stride - row)
            data = b"".join(data[y * pitch:y * pitch + row] + pad
                            for y in range(self.buf.get_height()))
        self.f.seek(0)
        self.f.write(data)

    def close(self):
        self.f.close()


class TerminalKeys:
    """Non-blocking single-key reads from the SSH terminal (no-op if not a tty)."""

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
            keys.append(os.read(self.fd, 1).decode(errors="ignore"))
        return keys

    def close(self):
        if self.fd is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


# ------------------------------------------------------------------ main

def run_window(src, state):
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("AMW LCD")
    fonts = make_fonts()
    clock = pygame.time.Clock()
    flasher = Flasher(state)
    running = True
    try:
        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    ch = {pygame.K_UP: "+", pygame.K_DOWN: "-",
                          pygame.K_ESCAPE: "q"}.get(e.key, e.unicode)
                    if ch == "q":
                        running = False
                    else:
                        src.apply_key(ch, state)
            now = time.monotonic()
            src.poll(state, now)
            draw(screen, fonts, state, flasher.update(state, now), int(now) % 2 == 0)
            pygame.display.flip()
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        pygame.quit()


def run_lcd(src, state, dev, rotate):
    # Only fonts + surfaces are needed; no SDL video, so no X or KMS required
    pygame.font.init()
    dev = dev or find_lcd_fb()
    if dev is None:
        sys.exit("No LCD framebuffer found. Check `cat /sys/class/graphics/fb*/name` "
                 "and that dtoverlay=waveshare35b-v2 is in config.txt, or pass --fb.")
    fb = Framebuffer(dev, rotate)
    keys = TerminalKeys()
    canvas = pygame.Surface((W, H))
    fonts = make_fonts()
    flasher = Flasher(state)
    last = None
    try:
        while True:
            ks = keys.read()
            if "q" in ks:
                break
            for k in ks:
                src.apply_key(k, state)
            now = time.monotonic()
            src.poll(state, now)
            flashes = flasher.update(state, now)
            hb = int(now) % 2 == 0
            # Push only when something visible changes (SPI is slow); the heartbeat
            # flips every second, so it also repaints once a second.
            frame = (snapshot(state), flashes, hb)
            if frame != last:
                draw(canvas, fonts, state, flashes, hb)
                fb.show(canvas)
                last = frame
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave a clear "stopped" screen instead of a frozen last frame
        draw(canvas, fonts, state, (), False)
        draw_cover(canvas, fonts, "OFFLINE", "LCD script stopped")
        fb.show(canvas)
        keys.close()
        fb.close()
        src.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--lcd", action="store_true", help="draw to the Pi's LCD framebuffer")
    p.add_argument("--fake", action="store_true", help="no ROS; keyboard-driven values")
    p.add_argument("--fb", help="framebuffer device (default: auto-detect)")
    p.add_argument("--rotate", type=int, choices=(0, 90, 180, 270),
                   help="override rotation if the image is sideways/upside down")
    p.add_argument("--snap", metavar="PNG", help="save one frame to a PNG and exit")
    args, _ = p.parse_known_args()   # ignore --ros-args etc. added by ros2 launch

    state = new_state()
    if args.snap:
        pygame.font.init()
        if args.fake:
            FakeSource(state)
        surf = pygame.Surface((W, H))
        draw(surf, make_fonts(), state)
        pygame.image.save(surf, args.snap)
        return
    src = FakeSource(state) if args.fake else RosSource(state)
    if args.lcd:
        run_lcd(src, state, args.fb, args.rotate)
    else:
        run_window(src, state)


if __name__ == "__main__":
    main()
