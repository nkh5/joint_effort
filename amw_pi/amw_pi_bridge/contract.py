"""Calibrated single-turn STS contract. No ROS or serial imports."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

TAU = 2 * math.pi
NAMES = tuple(f'joint_{i}' for i in range(1, 6))


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def read_config(path, urdf_path):
    config = json.loads(Path(path).read_text())
    validate_config(config, urdf_path)
    return config


def validate_config(config, urdf_path=None):
    if config.get('schema') != 1 or config.get('calibration_confirmed') is not True:
        raise ValueError('Complete and verify calibration before setting calibration_confirmed=true')
    rows = config.get('joints', [])
    if len(rows) != 5 or {r.get('name') for r in rows} != set(NAMES):
        raise ValueError('Exactly joint_1 through joint_5 are required')
    ids = [r.get('motor_id') for r in rows]
    if any(type(i) is not int or not 1 <= i <= 253 for i in ids) or len(set(ids)) != 5:
        raise ValueError('Five distinct motor IDs in 1..253 are required')
    if not isinstance(config.get('serial_port'), str) or not config['serial_port'].startswith('/dev/'):
        raise ValueError('Set the actual /dev/serial/by-id/... adapter path')
    if type(config.get('baud_rate')) is not int or not 9600 <= config['baud_rate'] <= 1000000:
        raise ValueError('Invalid baud_rate')
    bounds = {}
    if urdf_path:
        root = ET.parse(urdf_path).getroot()
        bounds = {j.get('name'): j.find('limit').attrib for j in root.findall('joint')
                  if j.get('type') == 'revolute'}
    for row in rows:
        name = row['name']
        for key in ['model_number', 'zero_counts', 'min_counts', 'max_counts']:
            if type(row.get(key)) is not int:
                raise ValueError(f'{name}: integer {key} is required')
        if not 0 <= row['model_number'] <= 65535:
            raise ValueError(f'{name}: invalid model number')
        if not 0 <= row['min_counts'] <= row['zero_counts'] <= row['max_counts'] <= 4095:
            raise ValueError(f'{name}: invalid single-turn count limits or zero')
        if row['min_counts'] == row['max_counts']:
            raise ValueError(f'{name}: empty count range')
        if type(row.get('direction')) is not int or row['direction'] not in (-1, 1):
            raise ValueError(f'{name}: direction must be +1 or -1')
        if not number(row.get('gear_ratio')) or row['gear_ratio'] <= 0:
            raise ValueError(f'{name}: positive gear_ratio required')
        if any(not number(row.get(k)) for k in ['min_rad', 'max_rad', 'max_velocity_rad_s']):
            raise ValueError(f'{name}: finite joint limits and velocity required')
        if not row['min_rad'] < row['max_rad'] or not 0 < row['max_velocity_rad_s'] <= .15:
            raise ValueError(f'{name}: invalid limits or velocity above current model cap 0.15 rad/s')
        if bounds:
            b = bounds[name]
            if row['min_rad'] < float(b['lower']) or row['max_rad'] > float(b['upper']):
                raise ValueError(f'{name}: calibrated travel exceeds the supplied URDF')
            if row['max_velocity_rad_s'] > float(b['velocity']):
                raise ValueError(f'{name}: velocity exceeds URDF')
        for q in [row['min_rad'], row['max_rad']]:
            radians_to_counts(row, q)


def counts_to_radians(row, counts):
    if type(counts) is not int or not row['min_counts'] <= counts <= row['max_counts']:
        raise ValueError(f"{row['name']}: measured counts outside calibrated range")
    q = row['direction'] * (counts - row['zero_counts']) * TAU / (4096 * row['gear_ratio'])
    if not row['min_rad'] <= q <= row['max_rad']:
        raise ValueError(f"{row['name']}: measured angle outside calibrated limits")
    return q


def radians_to_counts(row, q):
    if not number(q) or not row['min_rad'] <= q <= row['max_rad']:
        raise ValueError(f"{row['name']}: requested angle outside calibrated limits")
    raw = row['zero_counts'] + row['direction'] * q * 4096 * row['gear_ratio'] / TAU
    if not row['min_counts'] <= raw <= row['max_counts']:
        raise ValueError(f"{row['name']}: target would cross raw count limits; wrapping forbidden")
    # Preview quantization only. These values are never sent to motors.
    return int(math.floor(raw + .5))


def sample_all(bus, config):
    """Read all five using Nick's STSBus; discard the entire batch on any fault."""
    result = []
    for row in config['joints']:
        model, mode, torque, counts, _ = bus.sample(row['motor_id'])
        if model != row['model_number'] or mode != 0:
            raise ValueError(f"{row['name']}: wrong model or not position mode")
        result.append({'name': row['name'], 'motor_id': row['motor_id'],
                       'position': counts_to_radians(row, counts),
                       'counts': counts, 'torque': torque})
    return result


def preview_trajectory(config, names, points):
    """Validate waypoints and map by NAME, never message array order.

    Does not interpolate, execute, or certify acceleration/collision feasibility.
    points are dicts with positions, velocities, accelerations, effort, time_s.
    """
    if len(names) != 5 or set(names) != set(NAMES):
        raise ValueError('Trajectory must contain every joint exactly once')
    if not 1 <= len(points) <= 500:
        raise ValueError('Expected 1..500 waypoints')
    by_name = {r['name']: r for r in config['joints']}
    output, previous_t, previous_q = [], -1., None
    for point in points:
        t, positions = point['time_s'], point['positions']
        if not number(t) or t < 0 or t <= previous_t or len(positions) != 5:
            raise ValueError('Invalid point size or non-increasing time_from_start')
        for key in ['velocities', 'accelerations', 'effort']:
            values = point.get(key, [])
            if len(values) not in (0, 5) or any(not number(v) for v in values):
                raise ValueError(f'Invalid {key}')
        if point.get('effort'):
            raise ValueError('Effort commands are outside this position-only interface')
        targets = []
        for i, name in enumerate(names):
            row, q = by_name[name], positions[i]
            raw = radians_to_counts(row, q)
            cap = row['max_velocity_rad_s']
            if point.get('velocities') and abs(point['velocities'][i]) > cap + 1e-9:
                raise ValueError(f'{name}: waypoint velocity exceeds calibrated limit')
            if previous_q is not None and abs(q-previous_q[i]) / (t-previous_t) > cap + 1e-9:
                raise ValueError(f'{name}: average segment velocity exceeds calibrated limit')
            targets.append({'joint': name, 'motor_id': row['motor_id'], 'radians': q, 'counts': raw})
        output.append({'time_from_start_s': t, 'targets': targets})
        previous_t, previous_q = t, positions
    return output
