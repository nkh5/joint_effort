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
    raise SystemExit(f'Run this script from the repository calibration/scripts folder: {exc}')

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


def fit(rows):
    """Fit all samples with nominal 4096-count/revolution scaling."""
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

    # Unwrapping is only for analyzing the data. It does not change driver behavior.
    unwrapped = np.unwrap(raw * 2 * math.pi / 4096) * 4096 / (2 * math.pi)
    slope, _ = np.polyfit(angles, unwrapped, 1)
    direction = 1 if slope > 0 else -1
    fitted_ratio = abs(slope) / COUNTS_PER_DEG
    if not 0.85 <= fitted_ratio <= 1.15:
        raise ValueError(f'unexpected encoder ratio {fitted_ratio:.3f}; check gearing/data')

    # Keep the same nominal counts/radian model used in the original controller.