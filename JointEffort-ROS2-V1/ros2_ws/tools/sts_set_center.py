#!/usr/bin/env python3
"""Make a servo report X at its current physical position (changes its stored offset).

  python3 tools/sts_set_center.py ID X          e.g.  python3 tools/sts_set_center.py 4 2048
  python3 tools/sts_set_center.py ID --shift N  add N counts to whatever it reads now
                                                (what fit_calibration.py asks for; then add N to
                                                that joint's encoder_shift_counts)

WARNING: this moves every calibrated number for that motor. After calibration only use --shift,
and only when tools/fit_calibration.py tells you to.

Torque off and ROS stopped. The servo does not move: only the number it reports changes,
by adjusting its position offset (registers 31-32, EEPROM, survives power-off).
If the result is not X (+/-3), the old offset is written back.
"""
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00'
ADDR_OFS, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 31, 40, 55, 56


def pkt(i, ins, *p):
    b = [i, len(p) + 2, ins, *p]
    return bytes([0xFF, 0xFF, *b, (~sum(b)) & 0xFF])


class Bus:
    def __init__(self):
        self.s = serial.Serial(PORT, 1000000, timeout=0.05)

    def read(self, i, addr, n):
        self.s.reset_input_buffer()
        self.s.write(pkt(i, 2, addr, n))
        r = self.s.read(6 + n)
        if len(r) != 6 + n or r[:3] != bytes([0xFF, 0xFF, i]) or (~sum(r[2:-1])) & 0xFF != r[-1]:
            raise SystemExit(f'ID {i}: bad/no reply ({r.hex()}) - nothing changed')
        return r[5] if n == 1 else r[5] | r[6] << 8

    def write(self, i, addr, *data):
        self.s.reset_input_buffer()
        self.s.write(pkt(i, 3, addr, *data))
        self.s.read(6)
        time.sleep(0.02)                      # EEPROM write time


def ofs_decode(v):                            # 12-bit sign-magnitude: bit 11 = negative
    return -(v & 0x7FF) if v & 0x800 else v & 0x7FF


def ofs_encode(o):
    return (abs(o) & 0x7FF) | (0x800 if o < 0 else 0)


def wrap(o):                                  # same physical offset, into -2047..2047
    o %= 4096
    return o - 4096 if o > 2047 else o


def set_offset(bus, i, o):
    v = ofs_encode(o)
    bus.write(i, ADDR_LOCK, 0)                # unlock EEPROM
    bus.write(i, ADDR_OFS, v & 0xFF, v >> 8)
    bus.write(i, ADDR_LOCK, 1)                # lock again


def main():
    args = sys.argv[1:]
    shift = None
    if len(args) == 3 and args[1] == '--shift':
        i, shift = int(args[0]), int(args[2])
        if not -2047 <= shift <= 2047:
            raise SystemExit('--shift must be -2047..2047')
    elif len(args) == 2:
        i, x = int(args[0]), int(args[1])
        if not 0 <= x <= 4095:
            raise SystemExit('X must be 0-4095')
    else:
        raise SystemExit(__doc__)
    bus = Bus()
    if bus.read(i, ADDR_TORQUE, 1):
        raise SystemExit(f'ID {i}: torque is ON - release it first (support the arm)')
    y = bus.read(i, ADDR_PRESENT, 2)
    if shift is not None:
        x = y + shift
        if not 0 <= x <= 4095:
            raise SystemExit(f'ID {i}: reads {y}; {y}{shift:+d} = {x} is outside 0-4095. Move the joint '
                             f'closer to the middle of its travel and try again - nothing changed')
    old = ofs_decode(bus.read(i, ADDR_OFS, 2))
    new = wrap(old + (y - x))                 # reading = encoder - offset
    print(f'ID {i}: reads {y}, offset {old} -> {new} so it should read {x}')
    set_offset(bus, i, new)
    got = bus.read(i, ADDR_PRESENT, 2)
    if min(abs(got - x), 4096 - abs(got - x)) > 3:
        set_offset(bus, i, old)
        raise SystemExit(f'ID {i}: read {got}, not {x} - restored old offset {old} '
                         f'(now reads {bus.read(i, ADDR_PRESENT, 2)})')
    print(f'ID {i}: now reads {got} (was {y}, change {got - y:+d}). Saved in the servo (survives power-off).')
    if shift is not None:
        print(f'Add {got - y:+d} to encoder_shift_counts of motor {i} in sts_calibration.yaml, '
              f'then rerun tools/fit_calibration.py --write')


if __name__ == '__main__':
    main()
