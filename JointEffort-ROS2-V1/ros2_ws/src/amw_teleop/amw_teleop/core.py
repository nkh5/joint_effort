"""Controller maths: speed level, stick deadzone and curve, stick -> tip velocity. Pure Python, no ROS."""
import math

LEVEL_MAX = 20


def multiplier(level):
    """Speed factor: 0.1x at level 1, 1x at level 10, 2x at level 20."""
    if not 1 <= level <= LEVEL_MAX:
        raise ValueError('Speed level must be 1-20')
    return 10 ** ((level - 10) / 9) if level <= 10 else 2 ** ((level - 10) / 10)


def stick(x, y, deadzone, expo=0.0):
    """Joy-Con stick (x, y; each -1..1, rest offset already removed) -> (x, y) after a round deadzone
    and a response curve. Round (radial) so diagonals do not snap to the axes. expo 0 = linear;
    expo 0.6 = gentle near the centre for fine moves, full speed at full deflection."""
    r = math.hypot(x, y)
    if r <= deadzone:
        return 0.0, 0.0
    u = min(1.0, (r - deadzone) / (1.0 - deadzone))
    f = (1.0 - expo) * u + expo * u ** 3
    return x / r * f, y / r * f


def capped(vector, limit):
    norm = math.sqrt(sum(v * v for v in vector))
    scale = min(1.0, limit / norm) if norm else 1.0
    return [v * scale for v in vector]


def velocity(axes, mode, factor=1.0, linear=0.01, linear_cap=0.02, z_ratio=0.5):
    """Stick (x, y after `stick`) -> [vx, vy, vz] tip velocity in the world frame (m/s).

    translation: stick x/y -> tip X/Y at `linear` m/s at full stick (times the speed factor)
    z:           stick up/down -> tip Z at z_ratio * that speed (L held; a negative z_ratio reverses it)
    rotation:    no translation (the iron follows the hand; see node.py)
    """
    x, y = axes
    if mode == 'translation':
        return capped([x * linear * factor, y * linear * factor, 0.0], linear_cap)
    if mode == 'z':
        return capped([0.0, 0.0, y * linear * factor * z_ratio], linear_cap)
    return [0.0, 0.0, 0.0]
