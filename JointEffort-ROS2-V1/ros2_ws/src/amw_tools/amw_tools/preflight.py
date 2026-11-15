"""Read-only motor check, run by the launch file before it starts the real arm. No ROS needed.

  amw motors read              table of all five motors (report only)
  python3 -m amw_tools.preflight --arm arm.yaml [--readonly] [--port /dev/...]

Sends READ packets only: no writes, no torque change. Exit code 0 = OK to start, 1 = do not start
(the reason is printed), 2 = no motor answers at all.

Why: with torque on, the driver holds each joint where it is. A joint resting outside its limits
would be refused (or jump), and a motor that does not answer stops the driver from starting.
"""
import argparse
import math
import sys

from .common import load_arm, open_bus

VOLT_RANGE = (10.5, 13.0)   # 12 V supply, measured 12.6 V
TEMP_MAX = 55               # C


def check(bus, arm, readonly):
    """Returns (rows, problems, notes). rows: [(joint, status dict or None, angle or None)].
    problems block a powered start; notes are printed only."""
    rows, problems, notes = [], [], []
    for j in arm:
        st = bus.status(j.motor_id)
        if st is None:
            problems.append(f'{j.name}: motor ID {j.motor_id} does not answer (power, cable, adapter jumper, or wrong ID)')
            rows.append((j, None, None))
            continue
        q = j.angle(st['raw'])
        rows.append((j, st, q))
        if not 0 <= st['raw'] <= 4095:
            problems.append(f'{j.name}: raw count {st["raw"]} outside 0..4095')
        if st['mode'] not in (0, None):
            problems.append(f'{j.name}: motor is in mode {st["mode"]}, expected 0 (position)')
        if not VOLT_RANGE[0] <= st['volt'] <= VOLT_RANGE[1]:
            problems.append(f'{j.name}: supply {st["volt"]:.1f} V outside {VOLT_RANGE[0]}-{VOLT_RANGE[1]} V')
        if st['temp'] > TEMP_MAX:
            problems.append(f'{j.name}: {st["temp"]} C is above {TEMP_MAX} C - let it cool')
        if st['error']:
            from .stsbus import error_text
            notes.append(f'{j.name}: motor error flag set: {error_text(st["error"]) or hex(st["error"])}')
        if not readonly and not j.lower <= q <= j.upper:
            problems.append(f'{j.name}: at {math.degrees(q):+.1f} deg, outside its limits '
                            f'{math.degrees(j.lower):+.1f}..{math.degrees(j.upper):+.1f}. '
                            'Move it inside by hand (torque is off) and start again')
    return rows, problems, notes


def show(rows):
    print(f'{"joint":8} {"id":>3} {"raw":>5} {"angle deg":>10} {"limits deg":>15} {"in":>3} '
          f'{"torque":>6} {"volt":>5} {"temp":>4} {"amps":>5}')
    for j, st, q in rows:
        if st is None:
            print(f'{j.name:8} {j.motor_id:>3}   --- no reply ---')
            continue
        inside = 'yes' if j.lower <= q <= j.upper else 'NO'
        torque = {0: 'off', 1: 'ON'}.get(st['torque'], '?')
        print(f'{j.name:8} {j.motor_id:>3} {st["raw"]:5d} {math.degrees(q):+10.2f} '
              f'{math.degrees(j.lower):+7.1f}..{math.degrees(j.upper):+6.1f} {inside:>3} {torque:>6} '
              f'{st["volt"]:5.1f} {st["temp"]:4d} {st["current_a"]:5.2f}')


def run(arm, port=None, readonly=False):
    with open_bus(arm, port) as bus:
        rows, problems, notes = check(bus, arm, readonly)
    show(rows)
    print()
    for n in notes:
        print('Note: ' + n)
    if all(st is None for _, st, _ in rows):
        print('NO MOTOR ANSWERS: check 12 V power, the adapter jumper (B), the cable and the port.')
        return 2
    if problems:
        print('Warnings:' if readonly else 'DO NOT START:')
        for p in problems:
            print('  - ' + p)
        if any(st is None for _, st, _ in rows):
            return 2
        return 0 if readonly else 1
    print('MOTOR CHECK OK' + (' (read-only)' if readonly else ': all joints answer and are inside their limits.'))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', help='arm.yaml (default: the one in the source tree)')
    ap.add_argument('--port', default='')
    ap.add_argument('--readonly', action='store_true', help='report only; joints may be outside their limits')
    args = ap.parse_args(argv)
    return run(load_arm(args.arm), args.port or None, args.readonly)


if __name__ == '__main__':
    sys.exit(main())
