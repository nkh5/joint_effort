#!/usr/bin/env python3
"""One-file STS3215 bench tool for Pi 5 + Waveshare Bus Servo Adapter (A).

Install: python -m pip install pyserial==3.5
Examples (replace the port with the observed adapter port):
  python amw_pi.py --port /dev/ttyUSB0 scan
  python amw_pi.py --port /dev/ttyUSB0 read --id 1
  python amw_pi.py --port /dev/ttyUSB0 id --id 1 --new-id 3 --single --execute
  python amw_pi.py --port /dev/ttyUSB0 move --id 3 --delta 32 --bench --execute

ID and movement commands preview unless --execute is present. Use one secured,
unloaded motor for writes. This is position control, not continuous rotation.
Motor power is external. USB jumper is B. Pico JSON and ROS are not included.
Protocol reference: https://github.com/ftservo/FTServo_Python/tree/
cbcfa64674d592f7e5028ae72af42580f60500b4/scservo_sdk
"""
import argparse
import math
import sys
import time


class MotorError(RuntimeError):
    pass


def packet(ident, instruction, data=b""):
    if not 1 <= ident <= 253:
        raise MotorError("Motor ID must be 1 through 253.")
    body = bytes([ident, len(data) + 2, instruction]) + data
    return b"\xff\xff" + body + bytes([(~sum(body)) & 255])


class Bus:
    def __init__(self, port):
        self.port = port

    def request(self, ident, instruction, data=b"", reply=True):
        frame = packet(ident, instruction, data)
        self.port.reset_input_buffer()
        if self.port.write(frame) != len(frame):
            raise MotorError("Incomplete serial write.")
        if not reply:
            return b""
        deadline = time.monotonic() + 0.10
        received = bytearray()
        saw_bytes = False
        while time.monotonic() < deadline:
            chunk = self.port.read(1)
            saw_bytes = saw_bytes or bool(chunk)
            received.extend(chunk)
            while len(received) >= 2 and received[:2] != b"\xff\xff":
                del received[0]
            if len(received) < 4:
                continue
            length = received[3]
            if not 2 <= length <= 64:
                raise MotorError("Invalid packet length; check wiring and duplicate IDs.")
            if len(received) < length + 4:
                continue
            if sum(received[2:]) & 255 != 255:
                raise MotorError("Bad reply checksum; check wiring and duplicate IDs.")
            if received[2] != ident:
                raise MotorError("Reply ID mismatch; close other bus software.")
            if received[4]:
                raise MotorError(f"Motor {ident} reported fault bits 0x{received[4]:02x}.")
            return bytes(received[5:-1])
        if saw_bytes:
            raise MotorError("Incomplete or noisy reply.")
        raise TimeoutError(f"No reply from motor {ident}.")

    def ping(self, ident):
        try:
            self.request(ident, 1)
            return True
        except TimeoutError:
            return False

    def read(self, ident, address, count=1):
        data = self.request(ident, 2, bytes([address, count]))
        if len(data) != count:
            raise MotorError("Reply contains the wrong number of bytes.")
        return int.from_bytes(data, "little")

    def write(self, ident, address, data, reply=True):
        self.request(ident, 3, bytes([address]) + bytes(data), reply)

    def position(self, ident):
        raw = self.read(ident, 56, 2)
        return -(raw & 0x7fff) if raw & 0x8000 else raw

    def off(self, ident):
        self.write(ident, 40, [0])
        if self.read(ident, 40) != 0:
            raise MotorError("Torque disable did not verify.")

    def goal(self, ident, position):
        # Registers 41..47: acceleration, goal position, time, speed.
        # Fixed conservative bench defaults: acceleration 10, speed 100.
        self.write(ident, 41, bytes([10]) + position.to_bytes(2, "little")
                   + b"\x00\x00" + (100).to_bytes(2, "little"))


def scan(bus):
    return [ident for ident in range(1, 254) if bus.ping(ident)]


def set_id(bus, old, new):
    found = scan(bus)
    if found != [old]:
        raise MotorError(f"Expected only ID {old}; found {found}.")
    if new == old:
        print("ID already correct; no EEPROM write.")
        return
    bus.off(old)
    try:
        bus.write(old, 55, [0])  # Unlock EEPROM.
        # Send once. The reply address may change with the ID register.
        bus.write(old, 5, [new], reply=False)
        time.sleep(0.1)
        if bus.read(new, 5) != new:
            raise MotorError("New ID did not verify.")
    finally:
        locked = False
        for ident in (new, old):
            try:
                if bus.ping(ident):
                    bus.write(ident, 55, [1])
                    locked = bus.read(ident, 55) == 1
                    if locked:
                        break
            except (MotorError, OSError) as exc:
                print(f"Relock attempt for ID {ident}: {exc}", file=sys.stderr)
        if not locked:
            raise MotorError("EEPROM relock unverified. Keep motor isolated and scan.")
    print(f"ID {old} changed to {new}. Label it, power cycle, then read again.")


def move(bus, ident, delta, seconds, execute):
    if not 1 <= abs(delta) <= 64:
        raise MotorError("Use a nonzero offset of at most 64 counts.")
    if bus.read(ident, 33) != 0:
        raise MotorError("Motor must already be in position mode (mode 0).")
    if bus.read(ident, 40) != 0:
        raise MotorError("Torque is on. Use 'off --bench' on an unloaded motor first.")
    current = bus.position(ident)
    target = current + delta
    if not (100 <= current <= 3995 and 100 <= target <= 3995):
        raise MotorError("Position outside bench range 100..3995; wrapping is disabled.")
    print(f"Motor {ident}: {current} -> {target} counts")
    if not execute:
        print("Preview only. Add --bench --execute for unloaded bench movement.")
        return
    try:
        bus.goal(ident, current)  # Replace a stale goal before enabling torque.
        bus.write(ident, 40, [1])
        bus.goal(ident, target)
        deadline = time.monotonic() + seconds
        measured = current
        while time.monotonic() < deadline:
            measured = bus.position(ident)
            print(f"Target {target}, measured {measured}", flush=True)
            time.sleep(0.1)
        if abs(target - measured) > 10:
            raise MotorError("Motor did not reach the target within 10 counts.")
    finally:
        try:
            bus.off(ident)
            print("Bench torque disabled and verified.")
        except Exception as exc:
            raise MotorError(f"Torque disable UNVERIFIED; cut motor power: {exc}") from exc


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["scan", "read", "id", "move", "off"])
    p.add_argument("--port", required=True, help="Observed Waveshare serial port")
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--id", type=int, help="Current motor ID")
    p.add_argument("--new-id", type=int)
    p.add_argument("--delta", type=int, default=32, help="Relative position in encoder counts")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--execute", action="store_true", help="Allow ID or movement writes")
    p.add_argument("--single", action="store_true", help="Confirm ONE physical unloaded motor is connected")
    p.add_argument("--bench", action="store_true", help="Confirm motor is secured and unloaded")
    a = p.parse_args()
    if a.action != "scan" and (a.id is None or not 1 <= a.id <= 253):
        p.error("Supply --id from 1 through 253.")
    if a.action == "id" and (a.new_id is None or not 1 <= a.new_id <= 253):
        p.error("Supply --new-id from 1 through 253.")
    if a.action == "id" and a.execute and not a.single:
        p.error("ID writes require --single with one physical unloaded motor connected.")
    if (a.action == "move" and a.execute or a.action == "off") and not a.bench:
        p.error("This operation requires --bench with a secured unloaded motor.")
    if not math.isfinite(a.seconds) or not 0.1 <= a.seconds <= 30:
        p.error("--seconds must be between 0.1 and 30.")
    try:
        import serial
        with serial.Serial(a.port, a.baud, timeout=0.01, write_timeout=0.2, exclusive=True) as port:
            bus = Bus(port)
            if a.action == "scan":
                print("Scanning IDs 1..253; allow about 25 seconds.", flush=True)
                found = scan(bus)
                print("Responding IDs:", found)
                if not found:
                    raise MotorError("No motors found. Check power, USB jumper, port and baud.")
            elif a.action == "id":
                if a.execute:
                    set_id(bus, a.id, a.new_id)
                else:
                    if not bus.ping(a.id):
                        raise MotorError("Current ID did not respond.")
                    print(f"Preview: ID {a.id} -> {a.new_id}. Add --single --execute to write.")
            elif a.action == "move":
                move(bus, a.id, a.delta, a.seconds, a.execute)
            elif a.action == "off":
                bus.off(a.id)
                print("Bench torque disabled and verified.")
            else:
                deadline = time.monotonic() + a.seconds
                while time.monotonic() < deadline:
                    print(f"Motor {a.id}: {bus.position(a.id)} counts", flush=True)
                    time.sleep(0.1)
        return 0
    except ImportError:
        print("Install pyserial: python -m pip install pyserial==3.5", file=sys.stderr)
    except KeyboardInterrupt:
        print("Stopped.", file=sys.stderr)
        return 130
    except (MotorError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
