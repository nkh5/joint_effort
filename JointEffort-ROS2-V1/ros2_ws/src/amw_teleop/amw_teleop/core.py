"""Joystick maths: speed level, deadzone, stick -> tip velocity. Pure Python, no ROS."""
import math

LEVEL_MAX = 20


def speed_level(axis, low_axis=1.0):
    """Encoder slider axis (-1..1) -> speed level 1..20."""
    if not math.isfinite(axis) or abs(axis) > 1.001 or low_axis not in (-1., 1.):
        raise ValueError('Invalid encoder slider')
    return min(LEVEL_MAX, max(1, round((1.0 - max(-1., min(1., axis)) * low_axis) * 9.5) + 1))


def multiplier(level):
    """Speed factor: 0.1x at level 1, 1x at level 10, 2x at level 20."""
    if not 1 <= level <= LEVEL_MAX:
        raise ValueError('Speed level must be 1-20')
    return 10 ** ((level - 10) / 9) if level <= 10 else 2 ** ((level - 10) / 10)


def deadzone(value, width):
    return 0.0 if abs(value) <= width else math.copysign((abs(value) - width) / (1 - width), value)


def capped(vector, limit):
    norm = math.sqrt(sum(v * v for v in vector))
    scale = min(1.0, limit / norm) if norm else 1.0
    return [v * scale for v in vector]


def velocity(axes, mode, factor=1.0, linear=0.01, angular=0.1, linear_cap=0.02, angular_cap=0.2):
    """Stick (X, Y, twist; each -1..1) -> [vx, vy, vz, wx, wy, wz] in the world frame.

    translation: X/Y/twist -> tip X/Y/Z at `linear` m/s per unit stick (times the speed factor)
    rotation:    X/Y -> tilt about world X/Y at `angular` rad/s. World Z would mostly spin the iron
                 about its own shaft (it points down), which the five-joint arm leaves free.
    """
    if mode == 'translation':
        return capped([v * linear * factor for v in axes], linear_cap) + [0., 0., 0.]
    return [0., 0., 0.] + capped([axes[0] * angular * factor, axes[1] * angular * factor, 0.], angular_cap)
