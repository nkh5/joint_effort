#!/usr/bin/env python3
"""Slowly move STS servos to target raw steps, with a runaway guard.

  python3 sts_move.py 1=3081 3=3489 4=105 5=3118        # move these motors
  python3 sts_move.py --release 1 2 3 4 5                # torque off (support the arm first)

Each motor: goal is first set to where it IS, then torque on, then the goal is
walked toward the target in small steps. If any motor stops following its goal
(error > GUARD steps) - e.g. a runaway or a jam - that motor's torque is cut at once
and the move stops; the other motors keep holding. Run with ROS stopped.
"""
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00'
STEP = 6          # raw steps per cycle (~0.5 deg)
PERIOD = 0.03     # s per cycle  -> ~200 steps/s, ~17 deg/s
GUARD = 60        # max allowed |present - goal| in raw steps (~5 deg)
ADDR_TORQUE, ADDR_GOAL, ADDR_PRESENT = 40, 42, 56


def pkt(i, ins, *p):
    b = [i, len(p) + 2, ins, *p]
    return bytes([0xFF, 0xFF, *b, (~sum(b)) & 0xFF])


class Bus:
    def __init__(self, port):
        self.s = serial.Serial(port, 1000000, timeout=0.05)

    def read_pos(self, i):
        self.s.reset_input_buffer()
        self.s.write(pkt(i, 2, ADDR_PRESENT, 2))
        r = self.s.read(8)
        if len(r) < 8 or r[:3] != bytes([0xFF, 0xFF, i]) or (~sum(r[2:7])) & 0xFF != r[7]:
            raise IOError(f'ID {i}: bad/no reply ({r.hex()})')
        v = r[5] | r[6] << 8
        return -(v & 0x7FFF) if v & 0x8000 else v

    def write(self, i, addr, *data):
        self.s.write(pkt(i, 3, addr, *data))
        self.s.read(6)                       # consume the status reply so it can't collide

    def goal(self, i, g):
        self.write(i, ADDR_GOAL, g & 0xFF, g >> 8)

    def torque(self, i, on):
        self.write(i, ADDR_TORQUE, 1 if on else 0)


def move(bus, targets):
    for i, t in targets.items():
        if not 0 <= t <= 4095:
            raise SystemExit(f'ID {i}: target {t} outside 0-4095')
    cur = {i: bus.read_pos(i) for i in targets}
    print('start :', cur)
    try:
        _drive(bus, targets, cur)
    except BaseException as e:               # read error, runaway, Ctrl-C: never leave torque on
        for _ in range(3):                   # repeat: a garbled packet must not leave a motor powered
            for i in targets:
                try:
                    bus.torque(i, False)
                except Exception:
                    pass
        raise SystemExit(f'ABORTED ({e}) - torque OFF on {sorted(targets)}. Support the arm.')


def _drive(bus, targets, cur):
    for i in targets:                        # hold where it is first: no jump at torque-on
        bus.goal(i, cur[i])
        bus.torque(i, True)
    goal = dict(cur)
    while True:
        done = True
        for i, t in targets.items():
            d = t - goal[i]
            if d:
                goal[i] += max(-STEP, min(STEP, d))
                bus.goal(i, goal[i])
                done = False
        time.sleep(PERIOD)
        for i in targets:
            p = bus.read_pos(i)
            if abs(p - goal[i]) > GUARD:
                raise RuntimeError(f'ID {i} at {p} but goal {goal[i]} - not following')
        if done:
            break
    time.sleep(0.3)
    print('final :', {i: bus.read_pos(i) for i in targets}, '(holding; torque stays on)')


def main(argv):
    bus = Bus(PORT)
    if argv[:1] == ['--release']:
        for i in map(int, argv[1:]):
            bus.torque(i, False)
        print('torque off:', argv[1:])
        return
    targets = {int(k): int(v) for k, v in (a.split('=') for a in argv)}
    if not targets:
        raise SystemExit(__doc__)
    move(bus, targets)


if __name__ == '__main__':
    main(sys.argv[1:])
