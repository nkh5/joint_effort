"""Check that a motor map belongs to the model being run. No hardware access."""
import hashlib
import math
import xml.etree.ElementTree as ET

JOINTS = [f'joint_{i}' for i in range(1, 6)]
K = 4096 / (2 * math.pi)


def problems(data, urdf_text, require_tcp=True):
    errors = []
    if data.get('model_revision') != 'R02':
        errors.append('R02 requires a new assembled-arm calibration; legacy V1 offsets cannot be reused')
    if data.get('urdf_sha256') != hashlib.sha256(urdf_text.encode()).hexdigest():
        errors.append('calibration does not match iron.urdf; refit the R02 measurements after model changes')
    flags = data.get('physical_checks') or {}
    for key in ['directions_verified', 'travel_limits_verified'] + (['tcp_verified'] if require_tcp else []):
        if flags.get(key) is not True:
            errors.append(f'physical_checks.{key} has not been confirmed on the assembled arm')
    root = ET.fromstring(urdf_text)
    ids = []
    for name in JOINTS:
        joint = (data.get('joints') or {}).get(name) or {}
        ids.append(joint.get('motor_id'))
        center, direction = joint.get('center_steps'), joint.get('encoder_direction')
        lo, hi = joint.get('min_rad'), joint.get('max_rad')
        if type(center) is not int or not 0 <= center <= 4095:
            errors.append(f'{name}: center_steps must be an integer 0..4095')
        if type(direction) is not int or direction not in (-1, 1):
            errors.append(f'{name}: encoder_direction must be -1 or 1, measured in R02 positive direction')
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (lo, hi)) or not -math.pi < lo < hi < math.pi:
            errors.append(f'{name}: finite min_rad < max_rad within one turn required')
            continue
        limit = root.find(f"joint[@name='{name}']/limit")
        a, b = float(limit.get('lower')), float(limit.get('upper'))
        if not lo < a < b < hi:
            errors.append(f'{name}: driver range must enclose the URDF limits')
        if type(center) is int and type(direction) is int and direction in (-1, 1):
            raw = [center + direction * q * K for q in (lo, hi)]
            if min(raw) < 0 or max(raw) > 4095:
                errors.append(f'{name}: configured travel crosses encoder wrap; do not power this map')
    if any(type(i) is not int or not 1 <= i <= 253 for i in ids) or len(set(ids)) != 5:
        errors.append('motor IDs must be five distinct integers 1..253')
    return errors
