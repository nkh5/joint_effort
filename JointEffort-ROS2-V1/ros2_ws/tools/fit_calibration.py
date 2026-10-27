#!/usr/bin/env python3
"""Turn the hand measurements into the STS driver settings in sts_calibration.yaml.

No ROS, no serial port.  Run from ros2_ws:

  python3 tools/fit_calibration.py            # print the fit and every check, change nothing
  python3 tools/fit_calibration.py --write    # also write center_steps/min_rad/max_rad
  python3 tools/fit_calibration.py --write --datum joint_1=+0.0000 joint_3=+1.5200 ...
                                              # store datum offsets (printed by run_datum.sh) first

Inputs
  calibration/Calibration.csv   rows "joint_N,angle_deg,encoder_counts" (other columns ignored).
                                angle_deg is measured from your 0 deg mark on each joint (link
                                parallel to the motor's long side), + = the direction the ORIGINAL
                                V0 URDF (all axes 0 0 1) called positive.
  urdf/iron.urdf                joint axes (0 0 1 or 0 0 -1) and <limit> lower/upper.
  sts_calibration.yaml          motor_id, datum_offset_rad (measured with run_datum.sh),
                                encoder_shift_counts (only if you re-offset a servo) and
                                csv_angle_sign (-1 only if the read-only check showed that joint's
                                CSV angles were measured with + the other way round).

Model (direct drive, ratio 1, K = 4096/2pi counts per rad):
  counts  = csv_zero_counts + s * K * angle          (s = +1 or -1, fitted per joint)
  driver  q = (center_steps - counts) / K            (sts_hardware_interface, fixed)
  URDF    q = sigma * angle + datum_offset_rad       (sigma = sign of the URDF axis)
  => sigma must equal -s, and center_steps = csv_zero_counts + encoder_shift_counts + K * datum_offset_rad
"""
import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parent.parent
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
ROLES = {'joint_1': 'base', 'joint_2': 'shoulder', 'joint_3': 'elbow',
         'joint_4': 'wrist 1', 'joint_5': 'wrist 2'}
K = 4096 / (2 * math.pi)          # counts per radian
CPD = 4096 / 360                  # counts per degree
SERVO_EEPROM_KEYS = ('p_coefficient', 'i_coefficient', 'd_coefficient', 'deadband')   # kept as written


def read_csv(path):
    rows = {}
    with open(path, newline='') as f:
        for r in csv.reader(f):
            if len(r) >= 3 and r[0].strip() in JOINTS and r[1].strip() and r[2].strip():
                rows.setdefault(r[0].strip(), []).append((float(r[1]), float(r[2])))
    return rows


def fit(points):
    """Least-squares fit of counts vs angle: free slope, then the ratio-1 zero."""
    n = len(points)
    a = [p[0] for p in points]
    c = [p[1] for p in points]
    ma, mc = sum(a) / n, sum(c) / n
    sxx = sum((x - ma) ** 2 for x in a)
    if n < 2 or sxx == 0:
        raise ValueError('need at least two different angles')
    slope = sum((x - ma) * (y - mc) for x, y in points) / sxx
    s = 1 if slope > 0 else -1
    zero = sum(y - s * CPD * x for x, y in points) / n
    res = [y - (zero + s * CPD * x) for x, y in points]
    return {'s': s, 'zero': zero, 'ratio': abs(slope) / CPD,
            'max_err_deg': max(abs(r) for r in res) / CPD,
            'rms_err_deg': math.sqrt(sum(r * r for r in res) / n) / CPD,
            'span_deg': (min(a), max(a)), 'n': n}


def read_urdf(path):
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall('joint'):
        name = j.get('name')
        if name not in JOINTS:
            continue
        axis = [float(v) for v in j.find('axis').get('xyz').split()]
        if axis[:2] != [0.0, 0.0] or abs(abs(axis[2]) - 1) > 1e-9:
            raise ValueError(f'{name}: axis {axis} is not 0 0 +-1')
        lim = j.find('limit')
        out[name] = {'sigma': 1 if axis[2] > 0 else -1,
                     'lower': float(lim.get('lower')), 'upper': float(lim.get('upper'))}
    missing = [n for n in JOINTS if n not in out]
    if missing:
        raise ValueError(f'URDF is missing {missing}')
    return out


def num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def analyse(fits, urdf, cal, margin):
    """Per-joint results plus a list of blocking problems."""
    results, problems = {}, []
    for name in JOINTS:
        j = (cal.get('joints') or {}).get(name) or {}
        f = fits.get(name)
        u = urdf[name]
        r = {'motor_id': j.get('motor_id'), 'fit': f, 'sigma': u['sigma'],
             'lower': u['lower'], 'upper': u['upper'],
             'datum_offset_rad': num(j.get('datum_offset_rad')),
             'encoder_shift_counts': int(j.get('encoder_shift_counts') or 0),
             'csv_angle_sign': -1 if j.get('csv_angle_sign') == -1 else 1,
             'min_rad': round(u['lower'] - margin, 4), 'max_rad': round(u['upper'] + margin, 4),
             'center_steps': None, 'raw_range': None, 'notes': [],
             'servo': {k: j[k] for k in SERVO_EEPROM_KEYS if j.get(k) is not None}}
        if f is None:
            problems.append(f'{name}: no rows in the CSV')
            results[name] = r
            continue
        s = f['s'] * r['csv_angle_sign']          # counts vs +angle in the V0 URDF sense
        r['s'] = s
        if u['sigma'] != -s:
            problems.append(f'{name}: direction mismatch - counts {"rise" if s > 0 else "fall"} with +angle, '
                            f'so the URDF axis must be 0 0 {"-1" if s > 0 else "1"} (it is 0 0 {u["sigma"]}). '
                            'If the read-only RViz check proved the URDF axis right, set csv_angle_sign: '
                            f'{-r["csv_angle_sign"]} for {name} and rerun')
        provisional = f['zero'] + r['encoder_shift_counts']
        r['provisional_center'] = int(round(provisional))
        if r['datum_offset_rad'] is None:
            r['notes'].append('datum_offset_rad not measured yet (run_datum.sh); center_steps left empty')
        else:
            c = int(round(provisional + K * r['datum_offset_rad']))
            r['center_steps'] = c
            lo_raw = c - r['max_rad'] * K
            hi_raw = c - r['min_rad'] * K
            r['raw_range'] = (int(math.floor(lo_raw)), int(math.ceil(hi_raw)))
            if not 0 <= c <= 4095 or lo_raw < 0 or hi_raw > 4095:
                mid_raw = c - 0.5 * (r['min_rad'] + r['max_rad']) * K
                shift = int(round(2048 - mid_raw))
                problems.append(
                    f'{name}: center_steps {c} / raw travel {r["raw_range"][0]}..{r["raw_range"][1]} leaves 0..4095. '
                    f'Re-offset motor {r["motor_id"]} by {shift:+d} counts '
                    f'(tools/sts_set_center.py {r["motor_id"]} --shift {shift}), then set encoder_shift_counts to '
                    f'{r["encoder_shift_counts"] + shift} and rerun')
        lo_seen, hi_seen = f['span_deg']
        for lim_rad in (u['lower'], u['upper']):
            if r['datum_offset_rad'] is not None:
                angle = math.degrees((lim_rad - r['datum_offset_rad']) * u['sigma']) * r['csv_angle_sign']
                if not lo_seen - 5 <= angle <= hi_seen + 5:
                    r['notes'].append(f'URDF limit {lim_rad:+.3f} rad is {angle:+.0f} deg from your 0 mark, '
                                      f'outside the measured {lo_seen:+.0f}..{hi_seen:+.0f} deg (extrapolated)')
        if f['max_err_deg'] > 3:
            r['notes'].append(f'fit error up to {f["max_err_deg"]:.1f} deg - worth re-measuring')
        results[name] = r
    ids = [results[n]['motor_id'] for n in JOINTS]
    if len(set(ids)) != 5 or not all(isinstance(i, int) and 1 <= i <= 253 for i in ids):
        problems.append(f'motor_id values must be 5 distinct integers 1-253 (are {ids})')
    return results, problems


HEADER = """\
# sts_calibration.yaml - real-arm settings for sts_hardware_interface (read by launch/real.launch.py,
# tools/sts_preflight.py and tools/sts_calibration_monitor.py).
#
# WRITTEN BY tools/fit_calibration.py --write.  Edit only these by hand:
#   serial_port, baud_rate, motor_id, datum_offset_rad, encoder_shift_counts, csv_angle_sign
# then rerun the tool. Everything else is recomputed from calibration/Calibration.csv and urdf/iron.urdf.
#
#   csv_zero_counts       raw count at your 0 deg mark (ratio-1 fit of Calibration.csv)
#   datum_offset_rad      URDF joint angle at which the link sits on your 0 deg mark (run_datum.sh)
#   encoder_shift_counts  counts added by tools/sts_set_center.py --shift after the CSV was taken (normally 0)
#   csv_angle_sign        1 normally; -1 if that joint's CSV angles turned out to use the opposite + direction
#   center_steps          raw count at URDF angle 0 = csv_zero + shift + datum_offset * 4096/2pi
#   min_rad / max_rad     driver hard limits = URDF <limit> -/+ {margin} rad (clamps commands AND reported angle)
#
# Driver convention (fixed): angle = (center_steps - raw) * 2pi/4096, i.e. + angle = raw decreasing.
# Direction is set by the URDF <axis> (0 0 1 or 0 0 -1); the tool checks it against the CSV.
# Optional per-joint servo settings (kept by the tool, written to servo EEPROM by real.launch.py at
# every start, and they stay in the servo after you remove them here - set the old value back instead):
#   p_coefficient, i_coefficient, d_coefficient (position PID 0-255), deadband (0-255 counts).
#   Read the current values first: ./scripts/run_preflight.sh --readonly --gains
# Never run sts_set_center.py or /one_key_calibration without updating encoder_shift_counts:
# they move every number in this file.
"""


def render(cal, results, margin):
    lines = [HEADER.format(margin=margin),
             f'serial_port: {cal.get("serial_port")}',
             f'baud_rate: {int(cal.get("baud_rate", 1000000))}',
             '',
             'joints:']

    def y(v):
        return 'null' if v is None else repr(v)
    for name in JOINTS:
        r = results[name]
        f = r['fit']
        lines += [f'  {name}:   # {ROLES[name]}',
                  f'    motor_id: {y(r["motor_id"])}',
                  f'    csv_zero_counts: {y(None if f is None else int(round(f["zero"])))}',
                  f'    datum_offset_rad: {y(r["datum_offset_rad"])}',
                  f'    encoder_shift_counts: {r["encoder_shift_counts"]}',
                  f'    csv_angle_sign: {r["csv_angle_sign"]}',
                  f'    center_steps: {y(r["center_steps"])}',
                  f'    min_rad: {y(r["min_rad"])}',
                  f'    max_rad: {y(r["max_rad"])}']
        lines += [f'    {k}: {v}' for k, v in r['servo'].items()]
    return '\n'.join(lines) + '\n'


def report(results, problems):
    print(f'{"joint":8} {"id":>3} {"n":>2} {"csv zero":>8} {"+angle =>":>10} {"ratio":>6} {"max err":>8} '
          f'{"axis":>5} {"datum rad":>9} {"center":>6} {"raw travel":>11}  URDF limits')
    for name in JOINTS:
        r = results[name]
        f = r['fit']
        if f is None:
            print(f'{name:8} {r["motor_id"]!s:>3}  (no CSV data)')
            continue
        datum = '-' if r['datum_offset_rad'] is None else f'{r["datum_offset_rad"]:+.4f}'
        center = '-' if r['center_steps'] is None else str(r['center_steps'])
        travel = '-' if r['raw_range'] is None else f'{r["raw_range"][0]}..{r["raw_range"][1]}'
        print(f'{name:8} {r["motor_id"]!s:>3} {f["n"]:2d} {f["zero"]:8.0f} '
              f'{"counts up" if r["s"] > 0 else "counts dn":>10} {f["ratio"]:6.3f} {f["max_err_deg"]:6.1f} deg '
              f'{r["sigma"]:+5d} {datum:>9} {center:>6} {travel:>11}  {r["lower"]:+.3f}..{r["upper"]:+.3f}')
        for note in r['notes']:
            print(f'           note: {note}')
    print()
    if problems:
        print('PROBLEMS (must fix before driving the motors):')
        for p in problems:
            print('  - ' + p)
    else:
        print('No blocking problems.')
    complete = all(results[n]['center_steps'] is not None for n in JOINTS) and not problems
    print('Calibration COMPLETE: powered launch allowed.' if complete else
          'Calibration INCOMPLETE: only `run_real.sh readonly:=true` will start.')
    return complete


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default=str(WS / 'calibration/Calibration.csv'))
    ap.add_argument('--urdf', default=str(WS / 'src/joint_effort_iron/urdf/iron.urdf'))
    ap.add_argument('--cal', default=str(WS / 'src/joint_effort_iron/config/sts_calibration.yaml'))
    ap.add_argument('--margin', type=float, default=0.05, help='driver limit beyond the URDF limit, rad')
    ap.add_argument('--write', action='store_true', help='write the results into --cal')
    ap.add_argument('--datum', nargs='+', default=[], metavar='JOINT=RAD',
                    help='set datum_offset_rad for these joints before fitting (needs --write to keep)')
    args = ap.parse_args(argv)

    fits = {name: fit(pts) for name, pts in read_csv(args.csv).items()}
    urdf = read_urdf(args.urdf)
    cal = yaml.safe_load(Path(args.cal).read_text()) or {}
    for item in args.datum:
        name, _, value = item.partition('=')
        if name not in JOINTS:
            raise SystemExit(f'--datum: unknown joint {name!r}')
        cal.setdefault('joints', {}).setdefault(name, {})['datum_offset_rad'] = round(float(value), 4)
    results, problems = analyse(fits, urdf, cal, args.margin)
    complete = report(results, problems)
    if args.write:
        Path(args.cal).write_text(render(cal, results, args.margin))
        print(f'\nWrote {args.cal}')
    return 0 if complete else 1


if __name__ == '__main__':
    sys.exit(main())
