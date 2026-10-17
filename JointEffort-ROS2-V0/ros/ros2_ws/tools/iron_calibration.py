#!/usr/bin/env python3
"""Fit measured joint angles to raw encoder counts offline. Never controls a motor."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def fit(rows):
    report = {'scope': 'Offline measurement fit; no hardware authorization',
              'calibration_confirmed': False, 'hardware_enabled': False, 'joints': {}}
    if not rows:
        raise ValueError('No measurement rows; fill a COPY of the template with actual observations')
    for row in rows:
        if row['joint'] not in [f'joint_{i}' for i in range(1, 6)]:
            raise ValueError('Unknown joint name')
    for joint in sorted({r['joint'] for r in rows}):
        values = np.array([[float(r['angle_deg']), float(r['encoder_counts'])] for r in rows if r['joint'] == joint])
        angle, counts = values.T
        if not np.isfinite(values).all() or np.any(np.abs(angle) > 30) or np.any((counts < 0) | (counts > 4095)) or np.any(counts != np.round(counts)):
            raise ValueError(joint+': finite CAD-relative +/-30 deg angles and integer raw 0..4095 counts required')
        if len(np.unique(angle)) < 5 or np.ptp(angle) < 10:
            raise ValueError(joint+': need at least five distinct measured angles spanning at least 10 degrees')
        if np.ptp(counts) > 2048:
            raise ValueError(joint+': possible encoder wrap; this single-turn fit refuses to guess unwrapping')
        q = np.deg2rad(angle); design = np.column_stack([np.ones(len(q)), q])
        zero, slope = np.linalg.lstsq(design, counts, rcond=None)[0]
        residual = counts-design@np.array([zero, slope]); ratio = abs(slope)*2*math.pi/4096
        if not .1 <= ratio <= 20 or not 0 <= zero <= 4095:
            raise ValueError(joint+': implausible zero/ratio; check units and measurements')
        if np.max(np.abs(residual)) > 4:
            raise ValueError(joint+': residual exceeds four counts; inspect backlash, angle measurements and data')
        endpoints = zero+slope*np.deg2rad([-30, 30])
        if np.min(endpoints) < 0 or np.max(endpoints) > 4095:
            raise ValueError(joint+': proposed travel crosses encoder wrap; requires a separate reviewed mapping')
        report['joints'][joint] = {'sample_count': len(q), 'measured_angle_span_deg': float(np.ptp(angle)),
                                  'fitted_zero_counts': float(zero), 'fitted_sign': 1 if slope > 0 else -1,
                                  'fitted_external_ratio': float(ratio), 'max_residual_counts': float(np.max(np.abs(residual))),
                                  'rms_residual_counts': float(np.sqrt(np.mean(residual**2))),
                                  'rms_residual_equivalent_deg': float(np.rad2deg(np.sqrt(np.mean(residual**2))/abs(slope))),
                                  'proposed_software_end_counts': endpoints.tolist()}
    report['missing_joints'] = [f'joint_{i}' for i in range(1, 6) if f'joint_{i}' not in report['joints']]
    report['review_required'] = ['Physical zero datum and motor ID labels', 'Independent sign/ratio/travel confirmation',
                                 'Matching motor voltage datasheet', 'Tip offset and backlash/load measurements',
                                 'Independent hardware stop and driver validation']
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('measurements', type=Path); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.measurements.resolve() == a.output.resolve():
        p.error('Output must differ from input measurements')
    try:
        with a.measurements.open(newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames != ['joint', 'angle_deg', 'encoder_counts']:
                raise ValueError('CSV header must be joint,angle_deg,encoder_counts')
            rows = list(reader)
        report = fit(rows); report['measurement_source'] = str(a.measurements.resolve())
        report['measurement_sha256'] = hashlib.sha256(a.measurements.read_bytes()).hexdigest()
        a.output.write_text(json.dumps(report, indent=2)+'\n')
        print('Offline fit saved:', a.output.resolve())
        print('Measured joints:', ', '.join(report['joints']), '; missing:', ', '.join(report['missing_joints']) or 'none')
        print('Review required; calibration remains unconfirmed. No model settings or motor registers changed.')
    except (ValueError, KeyError, OSError, TypeError) as error:
        p.error(str(error))


if __name__ == '__main__':
    main()
