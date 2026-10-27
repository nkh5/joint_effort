#!/usr/bin/env python3
"""Read-only check of the five servos before ROS turns torque on.  No ROS needed.

  python3 tools/sts_preflight.py              # powered check: every joint must be inside its URDF limit
  python3 tools/sts_preflight.py --readonly   # report only (used before readonly runs and at the bench)
  python3 tools/sts_preflight.py --port /dev/serial/by-id/...   # override the calibration file's port
  python3 tools/sts_preflight.py --readonly --gains            # also print each servo's P/I/D/deadband

Sends READ packets only (no writes, no torque change). For each motor it prints ID response,
raw count, the joint angle the driver will report, torque state, voltage and temperature.
Exit code 0 = OK to launch, 1 = do not launch (reason printed), 2 = cannot talk to the bus.

Why: sts_hardware_interface clamps its first goal to min_rad..max_rad. A joint resting outside
its range would jump to the limit the moment torque comes on.
"""
import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parent.parent
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
K = 4096 / (2 * math.pi)
ADDR_MODEL, ADDR_MODE, ADDR_TORQUE, ADDR_PRESENT, ADDR_VOLT, ADDR_TEMP = 3, 33, 40, 56, 62, 63
VOLT_RANGE = (10.5, 13.0)   # 12 V supply measured at 12.6 V
TEMP_MAX = 55


def pkt(i, ins, *p):
    b = [i, len(p) + 2, ins, *p]
    return bytes([0xFF, 0xFF, *b, (~sum(b)) & 0xFF])


class Bus:
    def __init__(self, port, baud):
        import serial                      # imported here so --help works without pyserial
        try:
            self.s = serial.Serial(port, baud, timeout=0.05, exclusive=True)
        except (serial.SerialException, OSError) as e:
            raise SystemExit(f'Cannot open {port}: {e}\n'
                             'Is ROS or another tool using the adapter? Is the USB cable in? '
                             'Are you in the dialout group?') from None

    def read(self, i, addr, n, tries=3):
        for _ in range(tries):
            self.s.reset_input_buffer()
            self.s.write(pkt(i, 2, addr, n))
            r = self.s.read(6 + n)
            if len(r) == 6 + n and r[:3] == bytes([0xFF, 0xFF, i]) and (~sum(r[2:-1])) & 0xFF == r[-1]:
                return r[5] if n == 1 else r[5] | r[6] << 8
            time.sleep(0.005)
        return None


def signed15(v):                           # STS present position: bit 15 = sign
    return -(v & 0x7FFF) if v & 0x8000 else v


def urdf_limits(path):
    root = ET.parse(path).getroot()
    return {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
            for j in root.findall('joint') if j.get('name') in JOINTS}


def center_for(j):
    """center_steps, or the provisional readonly center (csv zero + shift), or None."""
    c = j.get('center_steps')
    if isinstance(c, int):
        return c, 'calibrated'
    z = j.get('csv_zero_counts')
    if isinstance(z, int):
        return z + int(j.get('encoder_shift_counts') or 0), 'provisional (csv zero, angle from your 0 mark)'
    return None, 'none'


def check(bus, cal, limits, readonly):
    rows, problems = [], []
    for name in JOINTS:
        j = cal['joints'][name]
        mid = j['motor_id']
        model = bus.read(mid, ADDR_MODEL, 2)
        if model is None:
            problems.append(f'{name}: motor ID {mid} does not answer (wiring, power, or wrong ID)')
            rows.append((name, mid, None))
            continue
        raw = bus.read(mid, ADDR_PRESENT, 2)
        torque, mode = bus.read(mid, ADDR_TORQUE, 1), bus.read(mid, ADDR_MODE, 1)
        volt, temp = bus.read(mid, ADDR_VOLT, 1), bus.read(mid, ADDR_TEMP, 1)
        if raw is None:
            problems.append(f'{name}: ID {mid} answered once, then stopped answering')
            rows.append((name, mid, None))
            continue
        raw = signed15(raw)
        center, kind = center_for(j)
        q = None if center is None else (center - raw) / K
        lo, hi = limits[name]
        rows.append((name, mid, dict(model=model, raw=raw, q=q, kind=kind, torque=torque, mode=mode,
                                     volt=None if volt is None else volt / 10, temp=temp, lo=lo, hi=hi)))
        if not 0 <= raw <= 4095:
            problems.append(f'{name}: raw count {raw} outside 0..4095')
        if mode not in (0, None):
            problems.append(f'{name}: servo is in mode {mode}, expected 0 (position)')
        if volt is not None and not VOLT_RANGE[0] <= volt / 10 <= VOLT_RANGE[1]:
            problems.append(f'{name}: supply {volt / 10:.1f} V outside {VOLT_RANGE[0]}-{VOLT_RANGE[1]} V')
        if temp is not None and temp > TEMP_MAX:
            problems.append(f'{name}: {temp} C is above {TEMP_MAX} C - let it cool')
        if not readonly:
            if kind != 'calibrated':
                problems.append(f'{name}: no center_steps yet - calibration incomplete')
            elif not lo <= q <= hi:
                problems.append(f'{name}: at {q:+.3f} rad ({math.degrees(q):+.1f} deg), outside its URDF limit '
                                f'{lo:+.3f}..{hi:+.3f}. Move it inside by hand (torque is off) and rerun')
    return rows, problems


def show(rows):
    print(f'{"joint":8} {"id":>3} {"raw":>5} {"angle rad":>10} {"angle deg":>10} {"URDF limit rad":>17} '
          f'{"in":>3} {"torque":>6} {"volt":>5} {"temp":>4}  center')
    for name, mid, d in rows:
        if d is None:
            print(f'{name:8} {mid:>3}   --- no reply ---')
            continue
        q = d['q']
        qs = '         -' if q is None else f'{q:+10.4f}'
        ds = '         -' if q is None else f'{math.degrees(q):+10.2f}'
        inside = '-' if q is None or d['kind'] != 'calibrated' else ('yes' if d['lo'] <= q <= d['hi'] else 'NO')
        torque = {0: 'off', 1: 'ON'}.get(d['torque'], '?')
        volt = '?' if d['volt'] is None else f'{d["volt"]:.1f}'
        print(f'{name:8} {mid:>3} {d["raw"]:5d} {qs} {ds} {d["lo"]:+8.3f}..{d["hi"]:+7.3f} {inside:>3} '
              f'{torque:>6} {volt:>5} {d["temp"]!s:>4}  {d["kind"]}')


def show_gains(bus, cal):
    print(f'{"joint":8} {"id":>3} {"P":>4} {"I":>4} {"D":>4} {"deadband cw/ccw":>16} {"ACC":>4}')
    for name in JOINTS:
        mid = cal['joints'][name]['motor_id']
        v = [bus.read(mid, a, 1) for a in (21, 23, 22, 26, 27, 41)]
        print(f'{name:8} {mid:>3} ' + ' '.join(f'{"?" if x is None else x:>4}' for x in v[:3])
              + f' {v[3]!s:>8}/{v[4]!s:<7} {v[5]!s:>4}')
    print()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cal', default=str(WS / 'src/joint_effort_iron/config/sts_calibration.yaml'))
    ap.add_argument('--urdf', default=str(WS / 'src/joint_effort_iron/urdf/iron.urdf'))
    ap.add_argument('--port', default='')
    ap.add_argument('--readonly', action='store_true', help='report only; do not require calibrated, in-limit joints')
    ap.add_argument('--gains', action='store_true', help='also print each servo P/I/D, deadband and ACC registers')
    args = ap.parse_args(argv)
    cal = yaml.safe_load(Path(args.cal).read_text())
    bus = Bus(args.port or cal['serial_port'], int(cal.get('baud_rate', 1000000)))
    if args.gains:
        show_gains(bus, cal)
    rows, problems = check(bus, cal, urdf_limits(args.urdf), args.readonly)
    show(rows)
    print()
    if all(d is None for _, _, d in rows):
        print('NO MOTOR ANSWERS: check 12 V power, the adapter jumper (B), the cable, and the port.')
        return 2
    if problems:
        print('DO NOT LAUNCH:' if not args.readonly else 'Warnings:')
        for p in problems:
            print('  - ' + p)
        if any(d is None for _, _, d in rows):
            return 2          # a silent motor also stops the driver from starting
        return 0 if args.readonly else 1
    print('PREFLIGHT OK' + (' (read-only report)' if args.readonly else ': all joints answer and are inside their limits.'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
