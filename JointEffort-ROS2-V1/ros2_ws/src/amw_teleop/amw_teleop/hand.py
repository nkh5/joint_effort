"""Joy-Con pointing: while ZL is held, the iron points the way the Joy-Con points. Pure Python + numpy.

  Neutral: the Joy-Con held upright (long side vertical, stick facing you) = the iron pointing straight
  down. D-pad left stores the Joy-Con's current pose as that neutral ("recentre").

  1. Orientation tracks the Joy-Con all the time: gyro integration (bias already removed by amw_joycon),
     with the accelerometer's gravity direction slowly pulling tilt back so it does not drift
     (a Mahony-style complementary filter). Heading (turning about vertical) still drifts slowly;
     recentring resets it.
  2. hand_tilt: how far the Joy-Con's long axis has tilted away from where it was at the neutral pose,
     as a rotation vector in the Joy-Con's own axes at the neutral pose. Twisting about the long axis
     does not count (the iron cannot roll about itself either).
  3. iron_target: two of those components (forward/back and sideways tilt, chosen in teleop.yaml) tilt
     a straight-down iron about world X and Y, scaled by hand_scale and limited to hand_max_angle_deg.
  4. point_velocity: angular velocity that swings the iron's current direction toward the target,
     always perpendicular to the iron (no roll).
"""
import math

import numpy as np

from .kinematics import rotation

G = 9.80665
DOWN = np.array([0.0, 0.0, -1.0])


def exp_rotation(v):
    """Rotation vector -> rotation matrix."""
    v = np.asarray(v, dtype=float)
    angle = float(np.linalg.norm(v))
    return np.eye(3) if angle < 1e-12 else rotation(v / angle, angle)


def swing(a, b):
    """Rotation vector of the smallest rotation taking unit vector a to unit vector b."""
    c = np.cross(a, b)
    s = float(np.linalg.norm(c))
    angle = math.atan2(s, float(np.dot(a, b)))
    if s < 1e-12:
        return np.zeros(3)                       # parallel (or exactly opposite: leave it)
    return c / s * angle


def orthonormal(r):
    u, _, vt = np.linalg.svd(r)
    return u @ vt


class Orientation:
    """Joy-Con orientation (Joy-Con axes -> gravity-aligned frame) from gyro (rad/s) + accel (m/s^2)."""

    MAX_GAP_S = 0.1

    def __init__(self, gravity_gain=1.0):
        self.gain = gravity_gain                 # rad/s per unit tilt error; 0 = gyro only
        self.r = None
        self.last_stamp = None

    def add(self, gyro, accel, stamp):
        a = np.asarray(accel, dtype=float)
        n = float(np.linalg.norm(a))
        if self.r is None:                       # first sample: tilt from gravity, heading arbitrary
            if n > 0.5 * G:
                self.r = exp_rotation(swing(a / n, np.array([0.0, 0.0, 1.0])))
                self.last_stamp = stamp
            return
        dt = stamp - (self.last_stamp if self.last_stamp is not None else stamp)
        self.last_stamp = stamp
        if not 0.0 < dt <= self.MAX_GAP_S:
            return
        w = np.asarray(gyro, dtype=float)
        if self.gain > 0 and abs(n - G) < 0.15 * G:              # only trust gravity when not shaken
            measured_up = a / n                                   # accelerometer reads +1 g upward at rest
            predicted_up = self.r.T @ np.array([0.0, 0.0, 1.0])
            w = w + self.gain * np.cross(measured_up, predicted_up)
        self.r = orthonormal(self.r @ exp_rotation(w * dt))


def hand_tilt(r_ref, r_now, long_axis):
    """Tilt of the Joy-Con's long axis since the neutral pose (rotation vector, neutral-pose axes)."""
    a = np.eye(3)[long_axis]
    return swing(a, (r_ref.T @ r_now) @ a)


def iron_target(tilt, axes=(0, 2), signs=(1.0, 1.0), scale=1.0, max_angle=math.radians(60)):
    """Direction the iron should point: straight down, tilted about world X by tilt[axes[0]] and about
    world Y by tilt[axes[1]] (times signs and scale), at most max_angle from vertical."""
    v = np.array([signs[0] * tilt[axes[0]], signs[1] * tilt[axes[1]], 0.0]) * scale
    n = float(np.linalg.norm(v))
    if n > max_angle:
        v *= max_angle / n
    return exp_rotation(v) @ DOWN


def point_velocity(direction, target, gain=3.0):
    """World angular velocity (rad/s) swinging the iron direction toward the target direction."""
    return gain * swing(np.asarray(direction, dtype=float), np.asarray(target, dtype=float))
