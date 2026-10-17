"""JointTrajectory quintic interpolation, matching position/velocity/acceleration endpoints.

No ROS imports: path sampling and analytic extrema can be tested offline.
"""
import math
import numpy as np


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def coefficients(first, second, duration):
    """Coefficients in normalized time u, one row per polynomial power."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('Trajectory times must strictly increase')
    vectors = [np.asarray(v, dtype=float) for p in (first, second)
               for v in (p.positions, p.velocities, p.accelerations)]
    if any(v.shape != (5,) or not np.isfinite(v).all() for v in vectors):
        raise ValueError('Five finite positions, velocities and accelerations required')
    q0, v0, a0, q1, v1, a1 = vectors
    c = np.zeros((6, 5)); c[0] = q0; c[1] = v0 * duration; c[2] = a0 * duration**2 / 2
    c[3:] = np.linalg.solve(np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]]),
                            np.array([q1-c[:3].sum(axis=0),
                                      v1*duration-c[1]-2*c[2], a1*duration**2-2*c[2]]))
    return c


def extrema(c):
    """Evaluate each joint at endpoints and every real stationary point."""
    low, high = [], []
    for column in c.T:
        roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(column))
        u = [0., 1.] + [float(z.real) for z in roots if abs(z.imag) < 1e-8 and 0 < z.real < 1]
        values = np.polynomial.polynomial.polyval(u, column)
        low.append(min(values)); high.append(max(values))
    return np.array(low), np.array(high)


def audit(trajectory, names, limits, speed, acceleration, max_dt=.05):
    """Analytic joint bounds/dynamics; finite collision samples of the actual spline.

    Samples at <=50 ms and a conservative <=0.005 rad travel bound per interval.
    Finite samples are NOT continuous swept-volume collision certification.
    """
    if list(trajectory.joint_names) != list(names) or len(trajectory.points) < 2:
        raise ValueError('Expected ordered five-joint trajectory with at least two points')
    points = trajectory.points; times = [seconds(p.time_from_start) for p in points]
    if not all(math.isfinite(t) and t >= 0 for t in times) or abs(times[0]) > 1e-8:
        raise ValueError('Trajectory must start at zero time')
    samples = [(0., np.array(points[0].positions))]; peak_speed = peak_accel = 0.
    for first, second, start, end in zip(points, points[1:], times, times[1:]):
        dt = end-start; c = coefficients(first, second, dt)
        lo, hi = extrema(c)
        if np.any(lo < limits[:, 0]-1e-7) or np.any(hi > limits[:, 1]+1e-7):
            raise ValueError('Interpolated trajectory exceeds joint travel')
        dc = np.polynomial.polynomial.polyder(c, axis=0)/dt
        ddc = np.polynomial.polynomial.polyder(dc, axis=0)/dt
        velocity = max(np.max(np.abs(v)) for v in extrema(dc))
        accel = max(np.max(np.abs(v)) for v in extrema(ddc))
        peak_speed = max(peak_speed, float(velocity)); peak_accel = max(peak_accel, float(accel))
        if velocity > speed+1e-5 or accel > acceleration+1e-5:
            raise ValueError('Interpolated speed/acceleration exceeds configured limits')
        count = max(1, math.ceil(dt/max_dt), math.ceil(velocity*dt/.005))
        for u in np.linspace(0, 1, count+1)[1:]:
            samples.append((start+u*dt, np.polynomial.polynomial.polyval(u, c)))
    return samples, {'duration_s': times[-1], 'collision_samples': len(samples),
                     'max_speed_rad_s': peak_speed, 'max_acceleration_rad_s2': peak_accel,
                     'interpolation': 'quintic position/velocity/acceleration',
                     'max_sample_dt_s': max_dt, 'max_joint_travel_per_sample_rad': .005}


def retime(trajectory, names, limits, speed, acceleration):
    """Uniform time stretch preserves the entire quintic joint-space curve."""
    _, before = audit(trajectory, names, limits, math.inf, math.inf)
    ratio = max(1., before['max_speed_rad_s']/speed,
                math.sqrt(before['max_acceleration_rad_s2']/acceleration))
    factor = ratio*1.02 if ratio > 1 else 1.
    if factor > 1:
        for point in trajectory.points:
            nanoseconds = round(seconds(point.time_from_start)*factor*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
            point.velocities = [float(v/factor) for v in point.velocities]
            point.accelerations = [float(v/factor**2) for v in point.accelerations]
    _, after = audit(trajectory, names, limits, speed, acceleration)
    return {'time_stretch_factor': factor, 'original_duration_s': before['duration_s'],
            'duration_s': after['duration_s'], 'original_max_speed_rad_s': before['max_speed_rad_s'],
            'original_max_acceleration_rad_s2': before['max_acceleration_rad_s2'],
            'max_speed_rad_s': after['max_speed_rad_s'], 'max_acceleration_rad_s2': after['max_acceleration_rad_s2']}
