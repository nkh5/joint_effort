#!/usr/bin/env python3
"""Fit raw counts against measured R02 joint angles. Offline only; never moves motors."""
import argparse
import csv
import hashlib
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W / 'src/joint_effort_iron'))
from joint_effort_iron.calibration import JOINTS, K


def fit(rows, max_error_deg=.5):
    if len(rows) < 3:
        raise ValueError('need at least three measurements, including zero and both directions')
    rows = sorted(rows)
    angles, raw = np.array(rows, dtype=float).T
    if not np.isfinite(angles).all() or not np.isfinite(raw).all():
        raise ValueError('measurements must be finite')
    if np.any(raw < 0) or np.any(raw > 4095) or np.any(raw != np.round(raw)):
        raise ValueError('raw counts must be integers 0..4095')
    if angles.min() >= 0 or angles.max() <= 0 or not np.any(angles == 0) or np.ptp(angles) < 10:
        raise ValueError('include zero and negative/positive angles spanning at least 10 degrees')
    # Unwrap nearby samples for the fit only. Powered travel may not cross raw 0/4095.
    counts = np.unwrap(raw * 2*math.pi/4096) * 4096/(2*math.pi)
    slope, _ = np.polyfit(angles, counts, 1)
    direction = 1 if slope > 0 else -1
    ratio = abs(slope) / (4096/360)
    if not .95 <= ratio <= 1.05:
        raise ValueError(f'slope implies ratio {ratio:.3f}; check directions, gearing and measurements')
    center = float(np.mean(counts-direction*angles*4096/360))
    residual = counts - (center + direction*angles*4096/360)
    error = float(max(abs(residual))*360/4096)
    if error > max_error_deg:
        raise ValueError(f'max fit error {error:.3f} degrees exceeds {max_error_deg}; remeasure')
    return dict(center_steps=int(round(center)) % 4096, encoder_direction=direction,
                max_error_deg=error, fitted_ratio=ratio, samples=len(rows))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv', default=str(W/'calibration/r02-measurements.csv'))
    p.add_argument('--cal', default=str(W/'src/joint_effort_iron/config/sts_calibration.yaml'))
    p.add_argument('--urdf', default=str(W/'src/joint_effort_iron/urdf/iron.urdf'))
    p.add_argument('--write', action='store_true')
    p.add_argument('--max-error-deg', type=float, default=.5)
    a = p.parse_args(argv)
    if not math.isfinite(a.max_error_deg) or not 0 < a.max_error_deg <= 1:
        p.error('use a fit tolerance greater than zero and at most 1 degree')
    rows = {n: [] for n in JOINTS}
    with open(a.csv, newline='') as f:
        for r in csv.DictReader(f):
            name = r.get('joint', '').strip()
            if name in rows and r.get('angle_deg') and r.get('encoder_counts'):
                rows[name].append((float(r['angle_deg']), float(r['encoder_counts'])))
    cal = yaml.safe_load(Path(a.cal).read_text())
    text = Path(a.urdf).read_text()
    root = ET.fromstring(text)
    errors = []
    for name in JOINTS:
        try:
            result = fit(rows[name], a.max_error_deg)
            limit = root.find(f"joint[@name='{name}']/limit")
            lo, hi = float(limit.get('lower'))-.05, float(limit.get('upper'))+.05
            raw = [result['center_steps'] + result['encoder_direction']*q*K for q in (lo, hi)]
            if min(raw) < 0 or max(raw) > 4095:
                raise ValueError('travel crosses raw 0/4095; encoder offset/range needs a separate review')
            cal['joints'][name].update(result, min_rad=lo, max_rad=hi)
            print(f'{name}: zero {result["center_steps"]}, direction {result["encoder_direction"]:+d}, '
                  f'max error {result["max_error_deg"]:.3f} degrees')
        except ValueError as e:
            errors.append(f'{name}: {e}')
    if errors:
        print('\n'.join(errors)); print('No calibration file written.'); return 1
    cal.update(model_revision='R02', urdf_sha256=hashlib.sha256(text.encode()).hexdigest())
    # A fit cannot prove the wiring, travel clearance, or physical tip location.
    cal['physical_checks'] = dict(directions_verified=False, travel_limits_verified=False, tcp_verified=False)
    if a.write:
        Path(a.cal).write_text(yaml.safe_dump(cal, sort_keys=False))
        print(f'Wrote {a.cal}. Physical checks remain unconfirmed.')
    else:
        print('Fit passed. Nothing written; add --write to save it.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
