#!/usr/bin/env python3
"""AMW STS servo commissioning on Raspberry Pi 5.

Requires pyserial and the official FTServo_Python scservo_sdk (see README.md).
scan/watch only read. assign/provision change IDs on ONE isolated STS servo.
No motion commands, broadcasts, mode changes, or encoder calibration are sent.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime
import math
from pathlib import Path
import sys
import time

ID_REGISTER = 5
MODEL_REGISTER = 3
MODE_REGISTER = 33
TORQUE_REGISTER = 40
LOCK_REGISTER = 55
COUNTS_PER_TURN = 4096


class BusError(RuntimeError):
    pass


def degrees(counts):
    """Absolute encoder angle for a valid single revolution STS reading."""
    if not 0 <= counts < COUNTS_PER_TURN:
        raise ValueError(f"{counts} is outside 0..4095; check model/mode/feedback")
    return counts * 360.0 / COUNTS_PER_TURN


def load_sdk(sdk_path):
    if sdk_path:
        root = Path(sdk_path).expanduser().resolve()
        if not (root / "scservo_sdk" / "__init__.py").is_file():
            raise BusError("--sdk must point to the FTServo_Python repository folder")
        sys.path.insert(0, str(root))
    try:
        from scservo_sdk import PortHandler, sms_sts
        from scservo_sdk.scservo_def import COMM_SUCCESS, COMM_RX_TIMEOUT
    except ImportError as exc:
        raise BusError("Install pyserial and use --sdk ~/amw/FTServo_Python. See README.md.") from exc
    return PortHandler, sms_sts, COMM_SUCCESS, COMM_RX_TIMEOUT


class STSBus:
    def __init__(self, packet, success=0, timeout=-6, retries=2):
        self.packet = packet
        self.success = success
        self.timeout = timeout
        self.retries = retries

    def read(self, method, *args, strict=True):
        """Retry reads only. Never turn a failed read into a position of zero."""
        for attempt in range(self.retries):
            value, result, error = getattr(self.packet, method)(*args)
            if result == self.success:
                if strict and error:
                    raise BusError(f"ID {args[0]}: servo status 0x{error:02x}: "
                                   f"{self.packet.getRxPacketError(error)}")
                return value, error
            if attempt + 1 < self.retries:
                time.sleep(0.005)
        raise BusError(f"ID {args[0]}: {self.packet.getTxRxResult(result)}")

    def byte(self, motor_id, register):
        return self.read("read1ByteTxRx", motor_id, register)[0]

    def discover(self, first=0, last=253, progress=False):
        """Return responding addresses and noisy addresses, NOT physical count."""
        found, noisy = {}, []
        for motor_id in range(first, last + 1):
            outcomes = []
            for _ in range(self.retries):
                value, result, error = self.packet.read1ByteTxRx(motor_id, ID_REGISTER)
                outcomes.append(result)
                if result == self.success:
                    if value == motor_id:
                        found[motor_id] = error
                    else:
                        noisy.append(motor_id)
                    break
            if motor_id not in found and any(r != self.timeout for r in outcomes):
                if motor_id not in noisy:
                    noisy.append(motor_id)
            if progress and (motor_id % 25 == 0 or motor_id == last):
                print(f"\rScanning ID {motor_id:3d}/{last}...", end="", file=sys.stderr, flush=True)
        if progress:
            print(file=sys.stderr)
        return found, noisy

    def sample(self, motor_id):
        model = self.read("read2ByteTxRx", motor_id, MODEL_REGISTER)[0]
        mode = self.byte(motor_id, MODE_REGISTER)
        torque = self.byte(motor_id, TORQUE_REGISTER)
        counts = self.read("ReadPos", motor_id)[0]
        angle = degrees(counts) if mode == 0 and 0 <= counts <= 4095 else None
        return model, mode, torque, counts, angle

    def write_verified(self, motor_id, register, value):
        # Some servos suppress write acknowledgments. Readback is authoritative.
        result, error = self.packet.write1ByteTxRx(motor_id, register, value)
        if result == self.success and error:
            raise BusError(f"ID {motor_id}: rejected write, status 0x{error:02x}")
        time.sleep(0.03)
        actual = self.byte(motor_id, register)
        if actual != value:
            raise BusError(f"ID {motor_id}: register {register} read {actual}, expected {value}")

    def assign_isolated(self, old_id, new_id, disable_torque=False):
        """Caller MUST physically isolate one compatible STS servo first."""
        if not 0 <= old_id <= 253 or not 1 <= new_id <= 253:
            raise BusError("Old ID must be 0..253; new ID must be 1..253")
        found, noisy = self.discover()
        if noisy or set(found) != {old_id}:
            raise BusError(f"Expected only ID {old_id}; got {list(found)}, noisy={noisy}. No writes sent.")
        if found[old_id]:
            raise BusError(f"Servo reports fault 0x{found[old_id]:02x}. No writes sent.")
        model = self.read("read2ByteTxRx", old_id, MODEL_REGISTER)[0]
        if old_id == new_id:
            return model  # Already correct; no EEPROM write needed.
        if self.byte(old_id, TORQUE_REGISTER) != 0:
            if not disable_torque:
                raise BusError("Torque is enabled. Support/unload the servo, then rerun with --disable-torque.")
            self.write_verified(old_id, TORQUE_REGISTER, 0)
        active_id = old_id
        unlocked = False
        try:
            # Mark first so even an interrupted unlock gets a relock attempt.
            unlocked = True
            self.write_verified(old_id, LOCK_REGISTER, 0)
            # ID changes can make the write acknowledgment appear at either ID.
            # Send exactly once and verify by reading the new address.
            result, error = self.packet.write1ByteTxRx(old_id, ID_REGISTER, new_id)
            time.sleep(0.1)
            if result == self.success and error:
                raise BusError(f"ID write rejected, servo status 0x{error:02x}")
            if self.byte(new_id, ID_REGISTER) != new_id:
                raise BusError("New ID readback did not match")
            active_id = new_id
            if self.read("read2ByteTxRx", new_id, MODEL_REGISTER)[0] != model:
                raise BusError("Model changed after ID write; stop and inspect the bus")
            self.write_verified(new_id, LOCK_REGISTER, 1)
            unlocked = False
            return model
        finally:
            if unlocked:
                # Best effort only. No broadcast and no automatic rollback.
                locked = False
                for candidate in dict.fromkeys([active_id, new_id, old_id]):
                    try:
                        if self.byte(candidate, ID_REGISTER) == candidate:
                            self.write_verified(candidate, LOCK_REGISTER, 1)
                            locked = True
                            break
                    except (BusError, OSError, ValueError, IndexError):
                        continue
                if not locked:
                    print("WARNING: could not verify EEPROM relock. Keep this servo isolated; "
                          "check old/new IDs before proceeding.", file=sys.stderr)


@contextmanager
def open_bus(args):
    PortHandler, Packet, success, timeout = load_sdk(args.sdk)
    # Monotonic timeout avoids system clock changes affecting transactions.
    class MonotonicPort(PortHandler):
        def getCurrentTime(self):
            return time.monotonic() * 1000.0

        def readPort(self, length):
            data = super().readPort(length)
            if not data:
                time.sleep(0.0005)  # Avoid the SDK's busy spin on absent IDs.
            return data

    port = MonotonicPort(args.port)
    try:
        if not port.setBaudRate(args.baud):
            raise BusError(f"Unsupported baud rate: {args.baud}")
        port.ser.write_timeout = 1.0
        # Advisory exclusivity: also stop ROS/controller processes explicitly.
        port.ser.exclusive = True
        yield STSBus(Packet(port), success, timeout, args.retries)
    finally:
        if port.is_open:
            port.closePort()


def show_samples(bus, ids):
    good = 0
    print(" ID  MODEL  MODE TORQUE   COUNTS   ENCODER_DEG")
    for motor_id in ids:
        try:
            model, mode, torque, counts, angle = bus.sample(motor_id)
            label = f"{angle:10.2f}" if angle is not None else "n/a (mode/range)"
            print(f"{motor_id:3d} {model:6d} {mode:5d} {torque:6d} {counts:8d} {label}")
            good += 1
        except BusError as exc:
            print(f"{motor_id:3d} ERROR: {exc}")
    print(f"Valid feedback now: {good}/{len(ids)} discovered IDs", flush=True)
    return good


def isolated_prompt(new_id):
    print(f"\nNext target ID: {new_id}")
    print("Turn servo power OFF. Connect ONLY the servo being assigned.")
    print("Disconnect all others, including any second adapter socket.")
    print("Support/unload the mechanism, then restore servo power.")
    print("Confirm this is an STS servo with the SMS_STS register map, not SCS/PWM.")
    print("A scan cannot prove physical isolation when multiple servos share an ID.")
    if input('Type ONE STS SERVO to continue, or anything else to cancel: ').strip() != "ONE STS SERVO":
        raise BusError("Cancelled; no ID assignment attempted")


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--sdk", help="Path to cloned FTServo_Python repository")
    p.add_argument("--retries", type=int, choices=range(1, 6), default=2)
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("ports", help="List serial ports without opening them")
    for name in ("scan", "watch"):
        s = sub.add_parser(name, help="Read IDs and current encoder angles")
        s.add_argument("--first", type=int, default=0)
        s.add_argument("--last", type=int, default=253)
        if name == "watch":
            s.add_argument("--interval", type=float, default=1.0)
    a = sub.add_parser("assign", help="Change one physically isolated servo's ID")
    a.add_argument("--old-id", type=int, required=True)
    a.add_argument("--new-id", type=int, required=True)
    a.add_argument("--disable-torque", action="store_true")
    a = sub.add_parser("provision", help="Guided sequential IDs; connect one servo at a time")
    a.add_argument("--count", type=int, required=True)
    a.add_argument("--start-id", type=int, default=1)
    a.add_argument("--disable-torque", action="store_true")
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.command == "ports":
        try:
            from serial.tools import list_ports
        except ImportError:
            raise BusError("Install pyserial first: python -m pip install pyserial==3.5")
        for port in list_ports.comports():
            print(f"{port.device}: {port.description} [{port.hwid}]")
        return 0
    if args.command in ("scan", "watch"):
        if not 0 <= args.first <= args.last <= 253:
            p.error("Scan bounds must satisfy 0 <= first <= last <= 253")
        if args.command == "watch" and (not math.isfinite(args.interval) or args.interval <= 0):
            p.error("--interval must be finite and positive")
        with open_bus(args) as bus:
            found, noisy = bus.discover(args.first, args.last, progress=True)
            ids = sorted(found)
            print(f"Responding motor IDs: {len(ids)} ({', '.join(map(str, ids)) or 'none'})")
            print("Physical motor count is only reliable after unique IDs are confirmed.")
            if noisy:
                print(f"Unreliable responses at IDs {noisy}; check duplicate IDs, wiring, and baud.")
            if not ids:
                print("Check servo power, USB adapter mode, correct port, and servo baud rate.")
                return 2
            while True:
                print(f"\n{datetime.now().isoformat(timespec='seconds')}")
                good = show_samples(bus, ids)
                if args.command == "scan":
                    return 0 if good == len(ids) and not noisy else 2
                time.sleep(args.interval)
    elif args.command == "assign":
        if not 0 <= args.old_id <= 253 or not 1 <= args.new_id <= 253:
            p.error("Old ID must be 0..253; new ID must be 1..253")
        isolated_prompt(args.new_id)
        with open_bus(args) as bus:
            model = bus.assign_isolated(args.old_id, args.new_id, args.disable_torque)
            print(f"Verified ID {args.old_id} -> {args.new_id}; model {model}. Label this motor.")
            show_samples(bus, [args.new_id])
        print("Power cycle this isolated servo, then scan to confirm ID persistence.")
    else:
        if args.count < 1 or args.start_id < 1 or args.start_id + args.count - 1 > 253:
            p.error("Requested IDs must fit within 1..253")
        for target in range(args.start_id, args.start_id + args.count):
            isolated_prompt(target)
            # Reopen for each servo so the adapter can be unplugged between steps.
            with open_bus(args) as bus:
                found, noisy = bus.discover(progress=True)
                if noisy or len(found) != 1:
                    raise BusError(f"Expected one responding ID; found {list(found)}, noisy={noisy}")
                old_id = next(iter(found))
                bus.assign_isolated(old_id, target, args.disable_torque)
                print(f"Verified ID {target}. Label the motor before disconnecting.")
                show_samples(bus, [target])
        print("Power OFF, assemble the chain, restore power, then run scan to verify all IDs.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nStopped. No motion command sent; existing torque state may remain enabled.", file=sys.stderr)
        sys.exit(130)
    except (BusError, OSError, ValueError, IndexError, EOFError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
