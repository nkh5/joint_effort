"""Minimal Feetech STS serial-bus client (same packets as the SCServo SDK the ROS driver uses).

Every AMW tool that talks to the motors directly uses this module. ROS must not be running on the
same adapter: the port is opened exclusively, so a second user gets a clear error instead of
corrupted packets. Positions are raw encoder counts (0..4095).
"""
import time

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE

# STS3215 register map (EEPROM below 40 needs the lock at 55 cleared to write)
ADDR_MODEL = 3
ADDR_ID = 5
ADDR_P, ADDR_D, ADDR_I, ADDR_PUNCH = 21, 22, 23, 24
ADDR_DEADBAND_CW, ADDR_DEADBAND_CCW = 26, 27
ADDR_OFFSET = 31            # 2 bytes, sign-magnitude, bit 11 = negative
ADDR_MODE = 33
ADDR_TORQUE = 40
ADDR_ACC = 41
ADDR_GOAL = 42
ADDR_LOCK = 55
ADDR_PRESENT = 56           # position(2) speed(2) load(2) voltage(1) temperature(1) ... current at 69
ADDR_VOLT, ADDR_TEMP, ADDR_MOVING, ADDR_CURRENT = 62, 63, 66, 69
FEEDBACK_LEN = 15           # 56..70

ERROR_BITS = {0x01: 'voltage out of range', 0x02: 'position sensor', 0x04: 'overheat',
              0x08: 'overcurrent', 0x20: 'overload'}


class BusError(RuntimeError):
    pass


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, ins, params=()):
    body = [sid, len(params) + 2, ins, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def s16(v, bit=15):
    """STS sign-magnitude decode (sign in `bit`)."""
    return -(v & ~(1 << bit) & 0xFFFF) if v & (1 << bit) else v


def enc16(v, bit=15):
    return (abs(int(v)) | (1 << bit)) if v < 0 else int(v)


def offset_decode(v):
    return -(v & 0x7FF) if v & 0x800 else v & 0x7FF


def offset_encode(o):
    return (abs(o) & 0x7FF) | (0x800 if o < 0 else 0)


def error_text(err):
    return ', '.join(t for b, t in ERROR_BITS.items() if err & b) or ''


class Bus:
    def __init__(self, port, baud=1000000, timeout=0.02):
        import serial                      # imported here so --help works without pyserial
        try:
            self.s = serial.Serial(port, baud, timeout=timeout, exclusive=True)
        except (serial.SerialException, OSError) as e:
            raise BusError(f'Cannot open {port}: {e}\n'
                           '  - Is ROS (amw start) or another tool using the adapter? Stop it first.\n'
                           '  - Is the USB cable in? Are you in the dialout group?') from None
        self.errors = 0
        self.last_error = {}               # id -> servo error byte from its last reply

    def close(self):
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- low level -------------------------------------------------------------------------
    def _status(self, sid):
        """Read one status packet; returns its parameter bytes, or None."""
        head = self.s.read(4)
        if len(head) < 4 or head[0] != 0xFF or head[1] != 0xFF:
            return None
        if head[2] == 0xFF:                # tolerate one extra 0xFF
            head = head[1:] + self.s.read(1)
            if len(head) < 4:
                return None
        rid, length = head[2], head[3]
        rest = self.s.read(length)
        if len(rest) != length or rid != sid or length < 2:
            return None
        if checksum([rid, length, *rest[:-1]]) != rest[-1]:
            return None
        self.last_error[sid] = rest[0]
        return bytes(rest[1:-1])

    def read(self, sid, addr, n, tries=3):
        for _ in range(tries):
            self.s.reset_input_buffer()
            self.s.write(packet(sid, READ, (addr, n)))
            data = self._status(sid)
            if data is not None and len(data) == n:
                return data
            self.errors += 1
            time.sleep(0.005)
        return None

    def read_u8(self, sid, addr):
        d = self.read(sid, addr, 1)
        return None if d is None else d[0]

    def read_u16(self, sid, addr, signed=False):
        d = self.read(sid, addr, 2)
        if d is None:
            return None
        v = d[0] | d[1] << 8
        return s16(v) if signed else v

    def write(self, sid, addr, data, wait=True):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, WRITE, (addr, *data)))
        if wait and sid != BROADCAST:
            return self._status(sid) is not None
        return True

    def ping(self, sid):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, PING))
        return self._status(sid) is not None

    # --- registers -------------------------------------------------------------------------
    def position(self, sid):
        return self.read_u16(sid, ADDR_PRESENT, signed=True)

    def torque(self, sid, on):
        return self.write(sid, ADDR_TORQUE, [1 if on else 0])

    def write_eeprom(self, sid, addr, data):
        """Write EEPROM register(s): unlock, write, lock. Survives power-off."""
        ok = self.write(sid, ADDR_LOCK, [0])
        ok = self.write(sid, addr, data) and ok
        ok = self.write(sid, ADDR_LOCK, [1]) and ok
        time.sleep(0.02)
        if not ok:
            raise BusError(f'ID {sid}: no reply while writing register {addr}')

    def status(self, sid):
        """One read of everything a health check needs, or None if the servo does not answer."""
        if self.read(sid, ADDR_MODEL, 2) is None:
            return None
        fb = self.read(sid, ADDR_PRESENT, FEEDBACK_LEN)
        torque, mode = self.read_u8(sid, ADDR_TORQUE), self.read_u8(sid, ADDR_MODE)
        if fb is None:
            return None
        return {'raw': s16(fb[0] | fb[1] << 8), 'speed': s16(fb[2] | fb[3] << 8),
                'load': s16(fb[4] | fb[5] << 8, 10), 'volt': fb[6] / 10, 'temp': fb[7],
                'current_a': (fb[13] | fb[14] << 8) * 0.0065, 'torque': torque, 'mode': mode,
                'error': self.last_error.get(sid, 0)}

    # --- same packets as SMS_STS::SyncWritePosEx / syncReadPacketTx ------------------------
    def sync_write_pos(self, ids, pos, speed=None, acc=None):
        n = 7
        params = [ADDR_ACC, n]
        for k, sid in enumerate(ids):
            p = enc16(pos[k])
            v = int(speed[k]) if speed is not None else 0
            a = int(acc[k]) if acc is not None else 0
            params += [sid, a, p & 0xFF, p >> 8, 0, 0, v & 0xFF, v >> 8]
        self.s.write(packet(BROADCAST, SYNC_WRITE, params))

    def sync_read(self, ids, addr=ADDR_PRESENT, n=FEEDBACK_LEN):
        """Returns {id: bytes or None}."""
        self.s.reset_input_buffer()
        self.s.write(packet(BROADCAST, SYNC_READ, (addr, n, *ids)))
        out = {}
        for sid in ids:
            d = self._status(sid)
            out[sid] = d if d is not None and len(d) == n else None
            if out[sid] is None:
                self.errors += 1
        return out
