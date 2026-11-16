"""Hand-rotation clutch: while ZL is held, the iron turns the way the Joy-Con has turned since the
moment the rotation started (scaled down). Pure Python + numpy, no ROS.

  1. HandRotation integrates the Joy-Con gyro (bias already removed by amw_joycon) from the moment
     rotation starts: R_rel = R_rel * Exp(w dt), in the Joy-Con's own axes at that moment.
  2. target_rotation picks two of its rotation-vector components (Joy-Con forward tilt and side tilt,
     set in teleop.yaml) and maps them to tilts about world X and Y, scaled by hand_scale:
         R_target = Exp(scale * r_world) * R_tool0
  3. follow_velocity turns the remaining error into an angular velocity for the five-joint solve:
         w = gain * Log(R_target * R_tool^T)
     The part along the iron is dropped by the solve (the arm leaves roll about the iron free).
"""
import math

import numpy as np

from .kinematics import rotation


def log_rotation(r):
    """Rotation matrix -> rotation vector (axis * angle, rad)."""
    c = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:                      # 180 deg: axis from the largest diagonal entry
        i = int(np.argmax(np.diag(r)))
        axis = r[:, i] + np.eye(3)[i]
        return axis / np.linalg.norm(axis) * angle
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2.0 * math.sin(angle)) * angle


def exp_rotation(v):
    """Rotation vector -> rotation matrix."""
    v = np.asarray(v, dtype=float)
    angle = float(np.linalg.norm(v))
    return np.eye(3) if angle < 1e-12 else rotation(v / angle, angle)


class HandRotation:
    """Relative Joy-Con rotation since reset(), from gyro samples (rad/s, Joy-Con axes)."""

    MAX_GAP_S = 0.1          # a longer gap between samples is not integrated (lost data)

    def __init__(self):
        self.reset()

    def reset(self):
        self.r = np.eye(3)
        self.last_stamp = None

    def add(self, gyro, stamp):
        """One sample: gyro (3, rad/s) measured at `stamp` (s)."""
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if 0.0 < dt <= self.MAX_GAP_S:
                self.r = self.r @ exp_rotation(np.asarray(gyro, dtype=float) * dt)
                u, _, vt = np.linalg.svd(self.r)       # keep it a clean rotation matrix
                self.r = u @ vt
        self.last_stamp = stamp

    def vector(self):
        return log_rotation(self.r)


def target_rotation(r_tool0, hand_vector, axes=(0, 1), signs=(1.0, 1.0), scale=0.5, max_angle=math.radians(60)):
    """Iron orientation the hand asks for. hand_vector[axes[0]] -> tilt about world X,
    hand_vector[axes[1]] -> tilt about world Y (each times its sign), scaled and limited."""
    v = np.array([signs[0] * hand_vector[axes[0]], signs[1] * hand_vector[axes[1]], 0.0]) * scale
    n = np.linalg.norm(v)
    if n > max_angle:
        v *= max_angle / n
    return exp_rotation(v) @ r_tool0


def follow_velocity(r_target, r_tool, gain=3.0):
    """World-frame angular velocity (rad/s) that turns r_tool toward r_target."""
    return gain * log_rotation(r_target @ r_tool.T)
