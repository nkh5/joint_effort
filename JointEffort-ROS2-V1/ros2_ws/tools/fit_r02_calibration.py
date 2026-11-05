#!/usr/bin/env python3
"""Offline R02 calibration. Fit joint encoders and cross-check motor encoders.

Supports the original joint,angle_deg,encoder_counts columns plus the
side-by-side Motor table in r02-measurements.csv. Never moves a motor.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W / 'src/joint_effort_iron'))
try:
    from joint_effort_iron.calibration import JOINTS, K
except ImportError as exc:
    # The project module is required to validate URDF travel limits correctly.
    raise SystemExit(f'Cannot load the workspace calibration module: {exc}')

COUNTS_PER_DEG = 4096.0 / 360.0


def read_measurements(csv_path):
    """Read normal joint rows and the additional Motor angle / encoder value table."""
    joint_data = {name: [] for name in JOINTS}
    motor_data = {name: [] for name in JOINTS}
    with open(csv_path, newline='') as file:
        table = list(csv.reader(file))
    if not table or table[0][:3] != ['joint', 'angle_deg', 'encoder_counts']:
        raise ValueError('CSV must start with joint,angle_deg,encoder_counts')

    angle_column = None
    motor_columns = {}
    for line_number, row in enumerate(table[1:], start=2):
        joint = row[0].strip() if row else ''
        if joint in joint_data:
            if len(row) < 3 or not row[1].strip() or not row[2].strip():
                raise ValueError(f'CSV line {line_number}: missing joint angle/counts')
            joint_data[joint].append((float(row[1]), float(row[2])))

        # Locate the motor table header anywhere on the right side of the CSV.
        for col, cell in enumerate(row):
            if cell.strip().lower() == 'angle / encoder value':
                angle_column = col
                motor_columns = {
                    f'joint_{value.strip()}': pos
                    for pos, value in enumerate(row[col + 1:], start=col + 1)
                    if value.strip().isdigit()
                    and f'joint_{value.strip()}' in motor_data
                }
                break
        else:
            if angle_column is None or len(row) <= angle_column:
                continue
            angle_text = row[angle_column].strip()
            try:
                angle = float(angle_text)
            except ValueError:
                continue  # e.g., blank, "Motor", or "crossed"
            for name, col in motor_columns.items():
                count_text = row[col].strip() if col < len(row) else ''
                if count_text and count_text.lower() not in {'crossed', 'n/a'}:
                    motor_data[name].append((angle, float(count_text)))
    return joint_data, motor_data


def analyse(rows):
    """Return a diagnostic fit without accepting poor measurements for motor use."""
    if len(rows) < 3:
        raise ValueError('need at least 3 angle/count measurements')
    angles, raw = np.array(sorted(rows), dtype=float).T
    if not np.isfinite(angles).all() or not np.isfinite(raw).all():
        raise ValueError('measurements must be finite')
    if np.any((raw < 0) | (raw > 4095) | (raw != np.round(raw))):
        raise ValueError('encoder counts must be integers from 0 to 4095')
    if len(set(angles)) != len(angles):
        raise ValueError('duplicate angles: average repeated measurements first')
    if angles.min() >= 0 or angles.max() <= 0 or np.ptp(angles) < 10:
        raise ValueError('need negative and positive angles spanning at least 10 degrees')
    counts = np.unwrap(raw * 2 * math.pi / 4096) * 4096 / (2 * math.pi)
    slope, intercept = np.polyfit(angles, counts, 1)
    direction = 1 if slope > 0 else -1
    # The driver uses nominal encoder resolution, not a fitted resolution.
    center = float(np.mean(counts - direction * angles * COUNTS_PER_DEG))
    errors = (counts-center)/(direction*COUNTS_PER_DEG)-angles
    return dict(center_steps=int(round(center)) % 4096, encoder_direction=direction,
                fitted_zero_count=center % 4096, fitted_ratio=float(abs(slope)/COUNTS_PER_DEG),
                max_error_deg=float(max(abs(errors))), rms_error_deg=float(np.sqrt(np.mean(errors**2))),
                free_slope_max_error_deg=float(max(abs(counts-(slope*angles+intercept)))/abs(slope)) if slope else None,
                samples=len(rows), zero_sample_present=bool(np.any(angles==0)),
                readings=[dict(angle_deg=float(a),encoder_counts=int(c),
                               model_angle_deg=float(a+e),error_deg=float(e))
                          for a,c,e in zip(angles,raw,errors)])


def quality_problems(result, max_error_deg):
    errors = []
    if not .85 <= result['fitted_ratio'] <= 1.15:
        errors.append(f'encoder ratio {result["fitted_ratio"]:.3f} is outside 0.85..1.15; check gearing/data')
    if result['max_error_deg'] > max_error_deg:
        errors.append(f'max angle error {result["max_error_deg"]:.3f} degrees exceeds {max_error_deg:.3f}; remeasure')
    return errors


def fit(rows, max_error_deg=.5):
    """Fit accepted data. Keep diagnostics separate from approval to write a motor map."""
    result = analyse(rows)
    errors = quality_problems(result, max_error_deg)
    if errors:
        raise ValueError('; '.join(errors))
    return {k:result[k] for k in ('center_steps','encoder_direction','max_error_deg','fitted_ratio','samples')}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv', default=str(W/'calibration/r02-measurements.csv'))
    p.add_argument('--cal', default=str(W/'src/joint_effort_iron/config/sts_calibration.yaml'))
    p.add_argument('--urdf', default=str(W/'src/joint_effort_iron/urdf/iron.urdf'))
    p.add_argument('--write', action='store_true')
    p.add_argument('--report', type=Path, help='save diagnostic results as JSON, including rejected fits')
    p.add_argument('--max-error-deg', type=float, default=.5)
    a = p.parse_args(argv)
    if not math.isfinite(a.max_error_deg) or not 0<a.max_error_deg<=1:
        p.error('fit tolerance must be greater than zero and at most 1 degree')
    report = dict(result='FAIL',max_error_allowed_deg=a.max_error_deg,joints={},motor_reference={},
                  problems=[],calibration_written=False,physical_checks_confirmed=False)
    try:
        rows, motor_rows = read_measurements(a.csv)
        cal = yaml.safe_load(Path(a.cal).read_text())
        text = Path(a.urdf).read_text();root = ET.fromstring(text)
        report['measurements_sha256'] = hashlib.sha256(Path(a.csv).read_bytes()).hexdigest()
        report['urdf_sha256'] = hashlib.sha256(text.encode()).hexdigest()
        report['fit_tool_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        ids = [(cal.get('joints') or {}).get(n,{}).get('motor_id') for n in JOINTS]
        if any(type(i) is not int or not 1<=i<=253 for i in ids) or len(set(ids))!=5:
            report['problems'].append('motor IDs must be five distinct integers 1..253')
        print('R02 joint fit (nominal 4096 counts/turn; motor reference table is not used for these zeros)')
        for name in JOINTS:
            try:
                result = analyse(rows[name]);issues = quality_problems(result,a.max_error_deg)
                limit = root.find(f"joint[@name='{name}']/limit")
                lo,hi = float(limit.get('lower'))-.05,float(limit.get('upper'))+.05
                raw = [result['center_steps']+result['encoder_direction']*q*K for q in (lo,hi)]
                result.update(min_rad=lo,max_rad=hi,raw_travel_counts=[min(raw),max(raw)])
                if min(raw)<0 or max(raw)>4095:
                    issues.append('configured travel crosses encoder wrap; review offset/range separately')
                result['accepted']=not issues;result['problems']=issues
                report['joints'][name]=result
                report['problems'] += [f'{name}: {e}' for e in issues]
                if not result['zero_sample_present']:
                    print(f'{name}: no measured zero point; the zero below is extrapolated from the supplied angles')
                print(f'{name}: zero={result["fitted_zero_count"]:.2f}, direction={result["encoder_direction"]:+d}, '
                      f'ratio={result["fitted_ratio"]:.4f}, max_error={result["max_error_deg"]:.3f} deg '
                      f'[{"PASS" if not issues else "REJECTED"}]')
                if not issues:
                    cal['joints'][name].update({k:result[k] for k in
                        ('center_steps','encoder_direction','max_error_deg','fitted_ratio','samples','min_rad','max_rad')})
            except (ValueError,TypeError,AttributeError) as e:
                report['problems'].append(f'{name}: {e}')
        # The separate Motor table has older references. Diagnose it, never merge it into R02 data.
        for name,data in motor_rows.items():
            if data:
                try:report['motor_reference'][name]=analyse(data)
                except ValueError as e:report['motor_reference'][name]={'problem':str(e)}
        if report['problems']:
            print('\n'.join(report['problems']))
            print('Fit rejected. No calibration file written; physical checks remain unconfirmed.')
        else:
            report['result']='PASS'
            cal.update(model_revision='R02',urdf_sha256=hashlib.sha256(text.encode()).hexdigest())
            cal['physical_checks']=dict(directions_verified=False,travel_limits_verified=False,tcp_verified=False)
            if a.write:
                Path(a.cal).write_text(yaml.safe_dump(cal,sort_keys=False));report['calibration_written']=True
                print(f'Wrote {a.cal}. Physical checks remain unconfirmed.')
            else:print('Fit passed. Nothing written; add --write to save it.')
    except (ValueError,OSError,TypeError) as e:
        report['problems'].append(str(e));print('Fit failed:',e)
    if a.report:
        a.report.parent.mkdir(parents=True,exist_ok=True)
        a.report.write_text(json.dumps(report,indent=2)+'\n');print(f'Report: {a.report}')
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
