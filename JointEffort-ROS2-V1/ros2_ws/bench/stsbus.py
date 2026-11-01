"""Minimal Feetech STS serial-bus client (same packets as sts_hardware_interface's SCServo SDK).

Used by the bench tests only. Positions are raw encoder counts (0-4095).
"""
import time

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE
ADDR_ACC, ADDR_GOAL, ADDR_TORQUE, ADDR_LOCK, ADDR_PRESENT = 41, 42, 40, 55, 56
FEEDBACK_LEN = 15            # 56..70: position, speed, load, voltage, temperature, ..., current


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, ins, params=()):
    body = [sid, len(params) + 2, ins, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def s16(v, bit=15):
    """STS sign-magnitude decode (sign in bit 15)."""
    return -(v & ~(1 << bit) & 0xFFFF) if v & (1 << bit) else v


def enc16(v, bit=15):
    return (abs(int(v)) | (1 << bit)) if v < 0 else int(v)


class Bus:
    def __init__(self, port, baud=1000000, timeout=0.02):
        import serial
        try:
            self.s = serial.Serial(port, baud, timeout=timeout, exclusive=True)
        except (serial.SerialException, OSError) as e:
            raise SystemExit(f'Cannot open {port}: {e}\nStop ROS / other tools using the adapter, check the cable '
                             'and the dialout group.') from None
        self.errors = 0

    def close(self):
        self.s.close()

    # --- low level -------------------------------------------------------------------------
    def _status(self, sid):
        """Read one status packet; returns params bytes or None."""
        head = self.s.read(4)
        if len(head) < 4 or head[0] != 0xFF or head[1] != 0xFF:
            return None
        if head[2] == 0xFF:            # tolerate one extra 0xFF
            head = head[1:] + self.s.read(1)
            if len(head) < 4:
                return None
        rid, length = head[2], head[3]
        rest = self.s.read(length)
        if len(rest) != length or rid != sid:
            return None
        if checksum([rid, length, *rest[:-1]]) != rest[-1]:
            return None
        return bytes(rest[1:-1])       # skip error byte and checksum

    def read(self, sid, addr, n, tries=3):
        for _ in range(tries):
            self.s.reset_input_buffer()
            self.s.write(packet(sid, READ, (addr, n)))
            data = self._status(sid)
            if data is not None and len(data) == n:
                return data
            self.errors += 1
        return None

    def read_u16(self, sid, addr, signed=False):
        d = self.read(sid, addr, 2)
        if d is None:
            return None
        v = d[0] | d[1] << 8
        return s16(v) if signed else v

    def position(self, sid):
        return self.read_u16(sid, ADDR_PRESENT, signed=True)

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

    @staticmethod
    def decode_feedback(d):
        return {'pos': s16(d[0] | d[1] << 8), 'speed': s16(d[2] | d[3] << 8),
                'load': s16(d[4] | d[5] << 8, 10), 'volt': d[6] / 10, 'temp': d[7],
                'current_a': (d[13] | d[14] << 8) * 0.0065 if len(d) >= 15 else None}

    # --- convenience ---------------------------------------------------------------------
    def hold_here(self, sid, acc=100):
        """Goal := present position, then torque on (no jump)."""
        p = self.position(sid)
        if p is None:
            raise RuntimeError(f'ID {sid}: no position reply')
        pe = enc16(p)
        self.write(sid, ADDR_ACC, [acc, pe & 0xFF, pe >> 8, 0, 0, 0, 0])
        self.write(sid, ADDR_TORQUE, [1])
        return p

    def torque(self, sid, on):
        self.write(sid, ADDR_TORQUE, [1 if on else 0])

    def goto(self, sid, pos, speed=0, acc=100):
        pe = enc16(pos)
        self.write(sid, ADDR_ACC, [acc, pe & 0xFF, pe >> 8, 0, 0, speed & 0xFF, speed >> 8], wait=False)
        self.s.read(6)


def now():
    return time.perf_counter()
