"""The ON/OFF state machine. Pure Python (no ROS), so every rule is unit-tested.

States
  off     arm does not move. `stop` says why (OFF at start, TURNED_OFF, or a fault).
  centre  ON was pressed (or the mode changed with the stick pushed); waits until the stick has
          been centred for `centre_time` seconds, then moves on to active. `stop` is CENTRE_STICK
          or MODE_CHANGED.
  active  the controller moves the arm. `warning` may say why a direction is blocked or slowed.

Joy-Con (Joy message from amw_joycon; indices in teleop.yaml)
  on_button          D-pad right: toggles ON/OFF. ON is refused (with the reason) if the arm is not ready.
  speed_up/down      D-pad up/down: speed level +/- speed_step (1..20).
  z_button  (held)   L: stick up/down moves the tip up/down instead of x/y      -> mode 'z'
  rotate_button (held) ZL: the iron points the way the Joy-Con points (stick ignored) -> mode 'rotation'
  neither held       stick x/y moves the tip in x/y                              -> mode 'translation'
  recenter_button    D-pad left: the Joy-Con's current pose becomes "iron straight down" (counted in
                     `recenters`; the node reads it). Works in any state.
A mode change pauses only if the stick is pushed and the new mode uses the stick; centre it to go on.

Anything in the stop list of faults.py turns the arm OFF; ON must be pressed again. Input loss
always turns it OFF, so a controller or Pi restart can never resume motion by itself.
"""
import math
from collections import deque
from dataclasses import dataclass, field

from .core import LEVEL_MAX, multiplier, stick
from .faults import report

SERVO_OK, SERVO_SLOWING_SINGULAR, SERVO_HALT_SINGULAR = 0, 1, 2
SERVO_SLOWING_COLLISION, SERVO_SLOWING_COLLISION_2, SERVO_HALT_COLLISION, SERVO_JOINT_BOUND = 3, 4, 5, 6


@dataclass
class Config:
    stick_axes: tuple = (0, 1)             # Joy axis for stick x, y
    stick_signs: tuple = (1.0, 1.0)
    on_button: int = 4                     # D-pad right
    speed_up_button: int = 2               # D-pad up
    speed_down_button: int = 3             # D-pad down
    z_button: int = 0                      # L (held)
    rotate_button: int = 1                 # ZL (held)
    recenter_button: int = 5               # D-pad left
    speed_step: int = 2
    start_level: int = 10
    deadzone: float = 0.12
    expo: float = 0.6
    centre_time: float = 0.2               # s centred before moving
    input_timeout: float = 0.15            # s without a valid Joy report
    imu_timeout: float = 0.15              # s without hand IMU data while rotating
    feedback_timeout: float = 0.3          # s without joint states
    limit_tolerance: float = math.radians(2.0)   # rad a joint may sit beyond its limit (sag, overshoot)
    motor_warm_c: float = 55.0
    motor_hot_c: float = 65.0
    motor_volt_range: tuple = (9.5, 14.0)

    @property
    def buttons(self):
        return (self.on_button, self.speed_up_button, self.speed_down_button, self.z_button, self.rotate_button,
                self.recenter_button)

    def __post_init__(self):
        if (len(self.stick_axes) != 2 or len(set(self.stick_axes)) != 2 or len(set(self.buttons)) != 6
                or min(*self.stick_axes, *self.buttons) < 0
                or any(s not in (-1.0, 1.0) for s in self.stick_signs) or len(self.stick_signs) != 2
                or not 1 <= self.speed_step <= 10 or not 1 <= self.start_level <= LEVEL_MAX
                or not 0.01 <= self.deadzone <= 0.4 or not 0.0 <= self.expo <= 1.0
                or not 0.05 <= self.input_timeout <= 0.5 or not 0.05 <= self.imu_timeout <= 0.5):
            raise ValueError('invalid controller configuration (check teleop.yaml)')


@dataclass
class Health:
    """What the node knows about the arm this tick."""
    joints: dict = None                    # name -> angle (rad), from the last /joint_states
    joints_age: float = math.inf           # s since that message
    limits: dict = field(default_factory=dict)   # name -> (lower, upper) rad
    controller: str = None                 # iron_arm_controller state, None = not known yet
    servo_ready: bool = False              # Servo started and switched to joint-jog input
    servo_code: int = None
    servo_message: str = ''
    motors: dict = field(default_factory=dict)   # name -> {'temperature': C, 'voltage': V}
    imu_age: float = math.inf              # s since the last hand IMU sample


class Supervisor:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.state = 'off'
        self.mode = 'translation'
        self.level = self.cfg.start_level
        self.axes = [0.0, 0.0]
        self.stop = report('OFF')
        self.warning = None
        self.last_joy = -math.inf
        self.centred_since = None
        self.prev = None                         # buttons of the last report: no edge on connect
        self.recenters = 0                       # D-pad left presses
        self.events = deque(maxlen=12)
        self._last_event = None

    # ------------------------------------------------------------------ helpers
    @property
    def active(self):
        return self.state == 'active'

    @property
    def speed(self):
        return multiplier(self.level)

    def log(self, now, text):
        if text != self._last_event:
            self._last_event = text
            self.events.append((now, text))

    def turn_off(self, now, fault):
        self.state = 'off'
        self.stop = fault
        self.warning = None
        self.centred_since = None
        self.log(now, f'OFF: {fault["text"]}')

    def pause(self, now, fault):
        self.state = 'centre'
        self.stop = fault
        self.warning = None
        self.centred_since = None
        self.log(now, fault['text'])

    def joystick_fresh(self, now):
        return now - self.last_joy <= self.cfg.input_timeout

    # ------------------------------------------------------------------ inputs
    def joy(self, axes, buttons, now, health=None):
        """One Joy report. Returns False if it was unusable."""
        c = self.cfg
        if (len(axes) <= max(c.stick_axes) or len(buttons) <= max(c.buttons)
                or any(not math.isfinite(v) or abs(v) > 1.001 for v in axes) or any(b not in (0, 1) for b in buttons)):
            if self.state != 'off':
                self.turn_off(now, report('JOYSTICK_BAD', detail=f'{len(axes)} axes, {len(buttons)} buttons'))
            return False
        self.last_joy = now
        x, y = (max(-1.0, min(1.0, axes[i])) * s for i, s in zip(c.stick_axes, c.stick_signs))
        self.axes = list(stick(x, y, c.deadzone, c.expo))
        held = {b: bool(buttons[b]) for b in c.buttons}
        pressed = {b: self.prev is not None and held[b] and not self.prev[b] for b in c.buttons}
        self.prev = held

        mode = 'rotation' if held[c.rotate_button] else ('z' if held[c.z_button] else 'translation')
        if mode != self.mode:
            self.mode = mode
            self.log(now, f'mode {mode}')
            if self.state != 'off' and mode != 'rotation' and any(self.axes):
                self.pause(now, report('MODE_CHANGED', detail=mode))
        for button, step in ((c.speed_up_button, c.speed_step), (c.speed_down_button, -c.speed_step)):
            if pressed[button]:
                level = max(1, min(LEVEL_MAX, self.level + step))
                if level != self.level:
                    self.level = level
                    self.log(now, f'speed level {level}')
        if pressed[c.recenter_button]:
            self.recenters += 1
            self.log(now, 'Joy-Con neutral set')
        if pressed[c.on_button]:
            if self.state == 'off':
                self.try_on(now, health)
            else:
                self.turn_off(now, report('TURNED_OFF'))
        return True

    def request_stop(self, now):
        if self.state != 'off':
            self.turn_off(now, report('STOP_REQUESTED'))

    def not_ready(self, health, now):
        """A stop-fault report if the arm cannot be enabled now, else None."""
        if not self.joystick_fresh(now):
            return report('JOYSTICK_LOST', value=f'{now - self.last_joy:.1f}' if math.isfinite(self.last_joy) else 'all')
        if health is None or health.joints is None:
            return report('NOT_READY', detail='no joint states yet')
        if health.joints_age > self.cfg.feedback_timeout:
            return report('FEEDBACK_STALE', value=f'{health.joints_age:.1f}')
        outside = self.outside_limits(health)
        if outside:
            return outside
        if health.controller != 'active':
            return report('NOT_READY', detail=f'iron_arm_controller is {health.controller or "not loaded yet"}')
        if not health.servo_ready:
            return report('NOT_READY', detail='MoveIt Servo has not started yet')
        return None

    def try_on(self, now, health):
        problem = self.not_ready(health, now)
        if problem:
            self.turn_off(now, problem)
            return
        self.pause(now, report('CENTRE_STICK'))
        self.log(now, 'ON')

    def outside_limits(self, health):
        for name, q in (health.joints or {}).items():
            lo, hi = health.limits.get(name, (-math.inf, math.inf))
            if q < lo - self.cfg.limit_tolerance or q > hi + self.cfg.limit_tolerance:
                return report('JOINT_OUTSIDE_LIMIT', joint=name, value=f'{math.degrees(q):+.1f}',
                              detail=f'{math.degrees(lo):+.1f}..{math.degrees(hi):+.1f} deg')
        return None

    # ------------------------------------------------------------------ every tick
    def step(self, health, now):
        """Check everything that can stop the arm. Returns True if the arm may move this tick."""
        c = self.cfg
        warning = None
        if self.state != 'off':
            problem = None
            if not self.joystick_fresh(now):
                problem = report('JOYSTICK_LOST', value=f'{now - self.last_joy:.1f}')
            elif health.joints is None or health.joints_age > c.feedback_timeout:
                problem = report('FEEDBACK_STALE', value=f'{health.joints_age:.1f}')
            elif health.controller is not None and health.controller != 'active':
                problem = report('CONTROLLER_INACTIVE', detail=health.controller)
            elif self.mode == 'rotation' and health.imu_age > c.imu_timeout:
                problem = report('IMU_LOST', value=f'{min(health.imu_age, 999.0):.1f}')
            else:
                problem = self.outside_limits(health)
            if problem is None and health.servo_code == SERVO_HALT_COLLISION:
                problem = report('SERVO_COLLISION', detail=health.servo_message or 'see the model')
            elif problem is None and health.servo_code == -1:
                problem = report('SERVO_ERROR', detail=health.servo_message or 'invalid state')
            if problem is None:
                for name, m in sorted(health.motors.items()):
                    temp, volt = m.get('temperature'), m.get('voltage')
                    if temp is not None and temp >= c.motor_hot_c:
                        problem = report('MOTOR_HOT', joint=name, value=f'{temp:.0f}')
                    elif volt is not None and volt > 0 and not c.motor_volt_range[0] <= volt <= c.motor_volt_range[1]:
                        problem = report('MOTOR_VOLTAGE', joint=name, value=f'{volt:.1f}')
                    elif temp is not None and temp >= c.motor_warm_c and warning is None:
                        warning = report('MOTOR_WARM', joint=name, value=f'{temp:.0f}')
                    if problem:
                        break
            if problem:
                self.turn_off(now, problem)
                return False
            if health.servo_code in (SERVO_SLOWING_COLLISION, SERVO_SLOWING_COLLISION_2):
                warning = report('COLLISION_NEAR', detail=health.servo_message or 'Servo')
            elif health.servo_code in (SERVO_SLOWING_SINGULAR, SERVO_HALT_SINGULAR):
                warning = report('SINGULARITY')
            elif health.servo_code == SERVO_JOINT_BOUND:
                warning = report('JOINT_LIMIT', joint='a joint', detail=health.servo_message or 'Servo')

        if self.state == 'centre':
            if any(self.axes):
                self.centred_since = None
            elif self.centred_since is None:
                self.centred_since = now
            elif now - self.centred_since >= c.centre_time:
                self.state = 'active'
                self.stop = None
                self.log(now, 'moving')
        self.set_warning(warning, now)
        return self.state == 'active'

    def set_warning(self, warning, now):
        if warning and (self.warning or {}).get('text') != warning['text']:
            self.log(now, 'warning: ' + warning['text'])
        self.warning = warning

    def kinematics_warning(self, info, names, limits, q, now):
        """Merge the task solver's result (blocked joints, singularity) into the warning."""
        if self.warning:                 # a hardware/Servo warning is more important
            return
        if info.get('blocked'):
            i = info['blocked'][0]
            lo, hi = limits[i]
            side = 'lower' if abs(q[i] - lo) < abs(q[i] - hi) else 'upper'
            self.set_warning(report('JOINT_LIMIT', joint=names[i], detail=side), now)
        elif info.get('status') == 'singularity':
            self.set_warning(report('SINGULARITY'), now)

    # ------------------------------------------------------------------ output
    def status(self, now):
        return {'state': self.state, 'enabled': self.state == 'active', 'mode': self.mode, 'level': self.level,
                'multiplier': round(self.speed, 4), 'joystick': 'ok' if self.joystick_fresh(now) else 'lost',
                'stop': self.stop, 'warning': self.warning,
                'events': [[round(now - t, 1), text] for t, text in self.events]}
