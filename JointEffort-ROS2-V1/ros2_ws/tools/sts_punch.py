#!/usr/bin/env python3
"""Read or set each servo's minimum start force ("punch", STS register 24) - ROS must be stopped.

  python3 tools/sts_punch.py                 show all five motors (factory value: 16)
  python3 tools/sts_punch.py 3 24            set motor 3 to 24
  python3 tools/sts_punch.py all 24          set all five to 24
  python3 tools/sts_punch.py all 16          back to the factory value

The servo applies at least this much drive as soon as it decides to move, so a higher value breaks
through static friction at a smaller position error (smaller stick-slip steps); too high and the joint
buzzes or twitches while holding. Change in small steps (16 -> 20 -> 24 -> 32) and test each with
tools/jog_track.py. The value is stored in the servo's EEPROM: it survives power-off and is NOT
written by run_real.sh, so note what you set (sts_calibration.yaml does not track it).
Other option: --port /dev/serial/by-id/...  (default: serial_port in sts_calibration.yaml)
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / 'bench'))
from stsbus import Bus  # noqa: E402

ADDR_PUNCH, ADDR_LOCK, ADDR_TORQUE = 24, 55, 40
IDS = [1, 2, 3, 4, 5]


def default_port():
    try:
        import yaml
        return yaml.safe_load((WS / 'src/joint_effort_iron/config/sts_calibration.yaml').read_text())['serial_port']
    except Exception:
        return None


def read_punch(bus, sid):
    d = bus.read(sid, ADDR_PUNCH, 1)
    if d is None:
        raise SystemExit(f'Motor {sid}: no reply')
    return d[0]


def write_punch(bus, sid, value):
    for _ in range(5):
        bus.write(sid, ADDR_LOCK, [0])
        time.sleep(0.005)
        bus.write(sid, ADDR_PUNCH, [value])
        time.sleep(0.005)
        bus.write(sid, ADDR_LOCK, [1])
        time.sleep(0.03)
        if read_punch(bus, sid) == value:
            return
    raise SystemExit(f'Motor {sid}: value did not stick (reads {read_punch(bus, sid)})')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('motor', nargs='?', help="motor id 1-5 or 'all'")
    ap.add_argument('value', nargs='?', type=int, help='new minimum start force (factory 16)')
    ap.add_argument('--port', default=default_port())
    args = ap.parse_args()
    if subprocess.run(['pgrep', '-f', '^[^ ]*ros2_control_node'], capture_output=True).returncode == 0:
        raise SystemExit('Stop run_real.sh first (only one program may use the adapter).')
    if not args.port:
        raise SystemExit('No serial port: pass --port /dev/serial/by-id/...')
    bus = Bus(args.port)
    try:
        if args.motor is None:
            print('motor  min start force')
            for sid in IDS:
                print(f'{sid:>5}  {read_punch(bus, sid):>15}')
            return 0
        if args.value is None or not 0 <= args.value <= 100:
            raise SystemExit('Give a value 0-100 (factory 16; try 20, 24, 32)')
        ids = IDS if args.motor == 'all' else [int(args.motor)]
        for sid in ids:
            old = read_punch(bus, sid)
            write_punch(bus, sid, args.value)
            print(f'motor {sid}: min start force {old} -> {args.value}')
        print('Stored in the servos (survives power-off). Start run_real.sh and test with jog_track.py.')
        return 0
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
