#!/usr/bin/env python3
"""Exercise the offline CSV command with synthetic readings, never physical data."""
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

W = Path(__file__).resolve().parents[1]
WORK = W/'work/calibration-test'; WORK.mkdir(parents=True, exist_ok=True)
measurements = WORK/'synthetic-readings.csv'; output = WORK/'synthetic-fit.json'
rows = []
for i in range(1, 6):
    sign = -1 if i%2 == 0 else 1
    for angle in [-10, -5, 0, 5, 10]:
        rows.append({'joint': f'joint_{i}', 'angle_deg': angle,
                     'encoder_counts': round(2048+sign*4096*math.radians(angle)/(2*math.pi))})


def write(data):
    with measurements.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['joint', 'angle_deg', 'encoder_counts'])
        writer.writeheader(); writer.writerows(data)


def run(path):
    return subprocess.run([sys.executable, str(W/'tools/iron_calibration.py'), str(path), '--output', str(output)],
                          capture_output=True, text=True, timeout=10)


write(rows); result = run(measurements); assert result.returncode == 0, result.stderr
fitted = json.loads(output.read_text())
assert not fitted['calibration_confirmed'] and not fitted['hardware_enabled'] and not fitted['missing_joints']
for i in range(1, 6):
    joint = fitted['joints'][f'joint_{i}']
    assert joint['fitted_sign'] == (-1 if i%2 == 0 else 1)
    assert abs(joint['fitted_zero_counts']-2048) < 1e-8
    assert abs(joint['fitted_external_ratio']-1) < .005
template = W.parent/'iron_prototype/calibration/measurements-template.csv'
before = hashlib.sha256(template.read_bytes()).hexdigest()
empty = run(template); assert empty.returncode == 2 and 'No measurement rows' in empty.stderr
noisy = [dict(row) for row in rows]; noisy[2]['encoder_counts'] += 100
write(noisy); rejected = run(measurements)
assert rejected.returncode == 2 and 'residual exceeds four counts' in rejected.stderr
assert hashlib.sha256(template.read_bytes()).hexdigest() == before
report = {'result': 'PASS', 'input_kind': 'Synthetic software test only; no physical measurements',
          'checks': {'five_joint_fits': True, 'both_motor_directions': True,
                     'calibration_remains_unconfirmed': True, 'hardware_remains_disabled': True,
                     'empty_template_rejected_and_preserved': True, 'noisy_data_rejected': True},
          'scope': 'No serial ports, motor commands, model-map changes or measured workbook entries'}
(W/'reports/iron/calibration-check.json').write_text(json.dumps(report, indent=2)+'\n')
print('PASS: synthetic CSV fit, sign/zero recovery, blank/noisy rejection, hardware-disabled output.')
