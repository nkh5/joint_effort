"""Pure input contract: no ROS, GPIO, motor writes or device assumptions hidden here."""
import math


def speed_level(axis, low_axis=1.0):
    if not math.isfinite(axis) or abs(axis) > 1.001 or low_axis not in (-1., 1.):
        raise ValueError('Invalid encoder slider')
    return min(20, max(1, round((1.0 - max(-1., min(1., axis))*low_axis)*9.5)+1))


def multiplier(level):
    if not 1 <= level <= 20:
        raise ValueError('Speed level must be 1–20')
    return 10**((level-10)/9) if level <= 10 else 2**((level-10)/10)


def deadzone(value, width):
    return 0.0 if abs(value) <= width else math.copysign((abs(value)-width)/(1-width), value)


def capped(vector, limit):
    norm = math.sqrt(sum(v*v for v in vector))
    scale = min(1.0, limit/norm) if norm else 1.0
    return [v*scale for v in vector]


def velocity(axes, mode, factor=1.0):
    # Canonical order is X,Y,twist. Twist is Z translation in translation mode.
    # Rotation uses two world pointing components; the five-joint bridge frees shaft roll.
    if mode == 'translation':
        return capped([v*.0015*factor for v in axes], .003)+[0., 0., 0.]
    return [0., 0., 0.]+capped([0., axes[1]*.015*factor, axes[0]*.015*factor], .03)


class InputState:
    def __init__(self, axes=(0, 1, 2), signs=(1., 1., 1.), slider=3,
                 low_axis=1., hold=0, toggle=1, width=.08, timeout=.15):
        if (len(axes) != 3 or len(signs) != 3 or len(set(axes)) != 3 or slider in axes
                or any(type(i) is not int or i < 0 for i in (*axes, slider, hold, toggle))
                or hold == toggle or any(v not in (-1., 1.) for v in signs)
                or low_axis not in (-1., 1.) or not .01 <= width <= .25
                or not .05 <= timeout <= .3):
            raise ValueError('Invalid joystick mapping')
        self.indices, self.signs, self.slider = axes, signs, slider
        self.low_axis, self.hold_index, self.toggle_index = low_axis, hold, toggle
        self.width, self.timeout = width, timeout
        self.mode = 'translation'; self.axes = [0., 0., 0.]; self.level = 10
        self.held = self.previous_hold = self.previous_toggle = self.prepared = False
        self.receipt = -math.inf; self.last_stamp = -1; self.neutral_since = None
        self.reason = 'Waiting for joystick; center and release to prepare'

    def fault(self, reason):
        self.prepared = False; self.neutral_since = None; self.reason = reason

    def fresh(self, now):
        return 0 <= now-self.receipt <= self.timeout

    def update(self, raw_axes, buttons, stamp, clock, now):
        if (not 4 <= len(raw_axes) <= 16 or not 7 <= len(buttons) <= 32
                or max(*self.indices, self.slider) >= len(raw_axes)
                or max(self.hold_index, self.toggle_index) >= len(buttons)
                or any(not math.isfinite(v) or abs(v) > 1.001 for v in raw_axes)
                or any(v not in (0, 1) for v in buttons)
                or not 0 <= (clock-stamp)*1e-9 <= self.timeout or stamp <= self.last_stamp):
            self.fault('Invalid, stale or replayed joystick report'); return False
        if not self.fresh(now):
            self.fault('Joystick reconnected; center and release required')
        self.receipt, self.last_stamp = now, stamp
        held, toggle = bool(buttons[self.hold_index]), bool(buttons[self.toggle_index])
        if toggle and not self.previous_toggle:
            self.mode = 'rotation' if self.mode == 'translation' else 'translation'
            self.fault('Mode changed; center and release required')
        self.axes = [deadzone(max(-1., min(1., raw_axes[i]))*s, self.width)
                     for i, s in zip(self.indices, self.signs)]
        self.level = speed_level(raw_axes[self.slider], self.low_axis)
        self.held = held
        if held and not self.previous_hold and any(self.axes):
            self.fault('Hold enable while centered, then move the stick')
        if self.previous_hold and not held:
            self.fault('Enable released; center and release to prepare again')
        if not held and not toggle and not any(self.axes):
            if self.neutral_since is None: self.neutral_since = now
            if now-self.neutral_since >= .2:
                self.prepared = True; self.reason = 'Ready: hold enable button to move'
        else:
            self.neutral_since = None
        if held and not self.previous_hold and not self.prepared:
            self.reason = 'Center and release before holding enable'
        self.previous_hold, self.previous_toggle = held, toggle
        return True

    def allowed(self, now):
        if not self.fresh(now): self.fault('Joystick input lost; center and release required')
        return self.fresh(now) and self.prepared and self.held
