"""Arm movement math. Distances are in meters and angles are in radians."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    r, p, y = rpy
    t[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    t[:3, 3] = xyz
    return t


def origin(element):
    if element is None:
        return np.eye(4)
    return transform([float(v) for v in element.get('xyz', '0 0 0').split()],
                     [float(v) for v in element.get('rpy', '0 0 0').split()])


class Robot:
    def __init__(self, urdf_path):
        self.root = ET.parse(Path(urdf_path)).getroot()
        self.joints = list(self.root.findall('joint'))
        self.links = {link.get('name'): link for link in self.root.findall('link')}
        self.by_child = {j.find('child').get('link'): j for j in self.joints}
        self.active = {j.get('name'): j for j in self.joints if j.get('type') == 'revolute'}

    def chain(self, tip):
        result = []
        while tip in self.by_child:
            joint = self.by_child[tip]
            result.append(joint)
            tip = joint.find('parent').get('link')
        return result[::-1]

    def names(self, arm):
        return [j.get('name') for j in self.chain(f'{arm}_tcp') if j.get('type') == 'revolute']

    def limits(self, arm):
        return np.array([[float(self.active[n].find('limit').get(k)) for k in ['lower', 'upper']]
                         for n in self.names(arm)])

    def fk(self, arm, q, with_jacobian=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError('Five finite joint angles required')
        t = np.eye(4)
        axes, points = [], []
        index = 0
        for joint in self.chain(f'{arm}_tcp'):
            # Follow the URDF chain: place each joint, then apply its rotation.
            t = t @ origin(joint.find('origin'))
            if joint.get('type') == 'revolute':
                axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                axes.append(t[:3, :3] @ axis)
                points.append(t[:3, 3].copy())
                moving = np.eye(4)
                moving[:3, :3] = rotation(axis, q[index])
                t = t @ moving
                index += 1
        if not with_jacobian:
            return t
        jac = np.zeros((6, 5))
        for i, (axis, point) in enumerate(zip(axes, points)):
            # Each column gives one joint's effect on tip position and rotation.
            jac[:3, i] = np.cross(axis, t[:3, 3] - point)
            jac[3:, i] = axis
        return t, jac

    def frames(self, positions):
        frames = {'world': np.eye(4)}
        pending = self.joints.copy()
        while pending:
            before = len(pending)
            for joint in pending.copy():
                parent = joint.find('parent').get('link')
                if parent not in frames:
                    continue
                t = frames[parent] @ origin(joint.find('origin'))
                if joint.get('type') == 'revolute':
                    moving = np.eye(4)
                    axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                    moving[:3, :3] = rotation(axis, positions.get(joint.get('name'), 0.0))
                    t = t @ moving
                frames[joint.find('child').get('link')] = t
                pending.remove(joint)
            if len(pending) == before:
                raise ValueError('Disconnected or cyclic URDF')
        return frames


def tangent_basis(direction):
    """Two world-frame axes perpendicular to the tool's local +X axis."""
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    reference = np.eye(3)[np.argmin(np.abs(d))]
    first = np.cross(d, reference)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(d, first)))


def task_jacobian(t, jac, orientation_length=0.12):
    basis = tangent_basis(t[:3, 0])
    # Keep XYZ and two pointing directions. Shaft roll is free.
    # The length factor balances position and angle units in the same solve.
    return np.vstack((jac[:3], orientation_length * basis.T @ jac[3:])), basis


def differential_step(robot, arm, q, linear, angular, model, dt=0.02):
    """XYZ + tool-pointing velocity, free roll. Returns bounded joint velocity.

    angular is about the current TCP in world axes. The component parallel to
    the tool direction is intentionally omitted. Limits preserve direction by
    uniformly scaling velocities, rather than clipping individual axes.
    """
    linear, angular = np.array(linear, dtype=float, copy=True), np.array(angular, dtype=float, copy=True)
    if linear.shape != (3,) or angular.shape != (3,) or not np.isfinite(np.r_[linear, angular]).all():
        raise ValueError('Finite 3-vector commands required')
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('Positive timestep required')
    t, jac = robot.fk(arm, q, True)
    length = model['task_orientation_length_m']
    reduced, basis = task_jacobian(t, jac, length)
    u, singular, vh = np.linalg.svd(reduced, full_matrices=False)
    # Stop when the arm cannot produce every requested task direction reliably.
    if singular[-1] < model['task_min_singular_value_m']:
        return np.zeros(5), {'status': 'singularity', 'min_singular_value_m': float(singular[-1])}
    for vector, cap in [(linear, model['task_max_linear_speed_m_s']),
                        (angular, model['task_max_angular_speed_rad_s'])]:
        norm = np.linalg.norm(vector)
        if norm > cap:
            vector *= cap / norm
    request = np.r_[linear, length * basis.T @ angular]
    damping = model['task_damping_m']
    # A damped inverse limits large joint speeds near difficult arm positions.
    qdot = vh.T @ ((singular / (singular**2 + damping**2)) * (u.T @ request))
    speed = model['mock_max_joint_velocity_rad_s']
    # Scale all joints together to keep their coordinated motion.
    qdot *= min(1.0, speed / max(np.max(np.abs(qdot)), 1e-12))
    limits = robot.limits(arm)
    q = np.asarray(q)
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        return np.zeros(5), {'status': 'out_of_bounds', 'min_singular_value_m': float(singular[-1])}
    # Allow motion away from a nearby limit, but stop motion farther into it.
    margin = 0.03
    if np.any((q < limits[:, 0] + margin) & (qdot < 0)) or np.any((q > limits[:, 1] - margin) & (qdot > 0)):
        return np.zeros(5), {'status': 'joint_margin', 'min_singular_value_m': float(singular[-1])}
    # Also check where the next command step would put the joints.
    if np.any(q + qdot * dt < limits[:, 0]) or np.any(q + qdot * dt > limits[:, 1]):
        return np.zeros(5), {'status': 'predicted_limit', 'min_singular_value_m': float(singular[-1])}
    return qdot, {'status': 'ok', 'min_singular_value_m': float(singular[-1]),
                  'scaled_task_residual': float(np.linalg.norm(reduced @ qdot - request))}
