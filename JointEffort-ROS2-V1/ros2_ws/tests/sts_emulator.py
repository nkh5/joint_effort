#!/usr/bin/env python3
"""Software stand-in for a chain of Feetech STS3215 servos on a pseudo-terminal.

Used by the tests and for dry runs without hardware:

  python3 tests/sts_emulator.py --link /tmp/fake_sts 1=2064 2=2035 3=3338 4=2004 5=2041
  # then point a tool at it, e.g.
  python3 tools/sts_preflight.py --readonly --port /tmp/fake_sts

Each ID=RAW argument creates a servo whose present position starts at RAW. While running,
type "hand ID RAW" + Enter to move a servo as if by hand (only while its torque is off),
or "stats" to print how many packets were seen.

Implements PING, READ, WRITE, SYNC_READ and SYNC_WRITE with the STS register map
(little-endian 16-bit, position bit 15 = sign, offset at 31-32 in sign-magnitude bit 11).
With torque on, the present position walks toward the goal at the commanded speed
(0 = 3400 steps/s) - good enough to check directions and limits, not dynamics.
"""
import argparse
import os
import select
import sys
import threading
import time
import tty

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE
MODEL, ID_ADDR, OFS, MODE, TORQUE, ACC, GOAL, SPEED, LOCK, PRESENT = 3, 5, 31, 33, 40, 41, 42, 46, 55, 56
PRESENT_SPEED, LOAD, VOLT, TEMP, MOVING, CURRENT = 58, 60, 62, 63, 66, 69


def checksum(body):
    return (~sum(body)) & 0xFF


def ofs_decode(v):
    return -(v & 0x7FF) if v & 0x800 else v & 0x7FF


class Servo:
    def __init__(self, sid, raw, volt=126, temp=31, sag=0):
        self.sag = sag                     # counts the servo settles away from its goal (gravity load)
        self.mem = bytearray(256)
        self.encoder = float(raw)          # physical shaft position, counts (offset 0)
        self.mem[MODEL:MODEL + 2] = (777).to_bytes(2, 'little')
        self.mem[ID_ADDR] = sid
        self.mem[MODE] = 0
        self.mem[LOCK] = 1
        self.mem[VOLT] = volt
        self.mem[TEMP] = temp
        self.writes = []                   # (addr, bytes) log for tests
        self.set_goal_to_present()

    @property
    def sid(self):
        return self.mem[ID_ADDR]

    def offset(self):
        return ofs_decode(int.from_bytes(self.mem[OFS:OFS + 2], 'little'))

    def present(self):
        return int(round(self.encoder - self.offset())) % 4096

    def set_goal_to_present(self):
        self.mem[GOAL:GOAL + 2] = self.present().to_bytes(2, 'little')

    def refresh(self):
        p = self.present()
        self.mem[PRESENT:PRESENT + 2] = p.to_bytes(2, 'little')

    def step(self, dt):
        if not self.mem[TORQUE]:
            self.mem[MOVING] = 0
            return
        g = int.from_bytes(self.mem[GOAL:GOAL + 2], 'little')
        g = -(g & 0x7FFF) if g & 0x8000 else g
        speed = int.from_bytes(self.mem[SPEED:SPEED + 2], 'little') or 3400
        err = g + self.sag - (self.encoder - self.offset())
        move = max(-speed * dt, min(speed * dt, err))
        self.encoder += move
        self.mem[MOVING] = 1 if abs(err) > 1 else 0

    def write(self, addr, data):
        self.writes.append((addr, bytes(data)))
        for k, b in enumerate(data):
            self.mem[addr + k] = b

    def read(self, addr, n):
        self.refresh()
        return bytes(self.mem[addr:addr + n])


class Bus:
    def __init__(self, servos):
        self.servos = {s.sid: s for s in servos}
        self.lock = threading.Lock()
        self.packets = 0

    def by_id(self, sid):
        for s in self.servos.values():
            if s.sid == sid:
                return s
        return None

    def status(self, sid, payload=b''):
        body = bytes([sid, len(payload) + 2, 0]) + payload
        return b'\xff\xff' + body + bytes([checksum(body)])

    def handle(self, sid, ins, params):
        """Return reply bytes for one instruction packet."""
        self.packets += 1
        out = b''
        with self.lock:
            if ins == PING:
                s = self.by_id(sid)
                if s:
                    out = self.status(sid)
            elif ins == READ:
                s = self.by_id(sid)
                if s and len(params) == 2:
                    out = self.status(sid, s.read(params[0], params[1]))
            elif ins == WRITE:
                targets = list(self.servos.values()) if sid == BROADCAST else [self.by_id(sid)]
                for s in targets:
                    if s:
                        s.write(params[0], params[1:])
                if sid != BROADCAST and targets[0]:
                    out = self.status(sid)
            elif ins == SYNC_READ and sid == BROADCAST:
                addr, n, ids = params[0], params[1], params[2:]
                for i in ids:
                    s = self.by_id(i)
                    if s:
                        out += self.status(i, s.read(addr, n))
            elif ins == SYNC_WRITE and sid == BROADCAST:
                addr, n = params[0], params[1]
                rest = params[2:]
                for k in range(0, len(rest) - n, n + 1):
                    s = self.by_id(rest[k])
                    if s:
                        s.write(addr, rest[k + 1:k + 1 + n])
        return out

    def physics(self, dt):
        with self.lock:
            for s in self.servos.values():
                s.step(dt)


class Parser:
    """Byte-stream -> instruction packets."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        packets = []
        while True:
            start = self.buf.find(b'\xff\xff')
            if start < 0:
                del self.buf[:-1]
                return packets
            del self.buf[:start]
            while len(self.buf) > 2 and self.buf[2] == 0xFF:      # extra 0xFF padding
                del self.buf[0]
            if len(self.buf) < 4:
                return packets
            length = self.buf[3]
            total = 4 + length
            if len(self.buf) < total:
                return packets
            pkt = bytes(self.buf[:total])
            del self.buf[:total]
            sid, ins, params, chk = pkt[2], pkt[4], pkt[5:-1], pkt[-1]
            if checksum(pkt[2:-1]) == chk:
                packets.append((sid, ins, params))


def serve(bus, link, stop, period=0.002):
    master, slave = os.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    path = os.ttyname(slave)
    if link:
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(path, link)
    parser = Parser()
    last = time.monotonic()
    ready = threading.Event()

    def loop():
        nonlocal last
        ready.set()
        try:
            while not stop.is_set():
                r, _, _ = select.select([master], [], [], period)
                if r:
                    try:
                        data = os.read(master, 4096)
                    except OSError:
                        data = b''
                    for sid, ins, params in parser.feed(data):
                        reply = bus.handle(sid, ins, params)
                        if reply:
                            os.write(master, reply)
                now = time.monotonic()
                bus.physics(now - last)
                last = now
        finally:
            os.close(master)
            os.close(slave)
            if link and os.path.islink(link):
                os.unlink(link)
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    ready.wait()
    return path, t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--link', default='/tmp/fake_sts', help='symlink to the pseudo-terminal')
    ap.add_argument('--sag', action='append', default=[], metavar='ID=COUNTS',
                    help='servo ID settles COUNTS away from its goal under torque (simulated gravity load)')
    ap.add_argument('servos', nargs='+', metavar='ID=RAW')
    args = ap.parse_args()
    sag = {int(a.split('=')[0]): int(a.split('=')[1]) for a in args.sag}
    servos = [Servo(int(a.split('=')[0]), int(a.split('=')[1]), sag=sag.get(int(a.split('=')[0]), 0))
              for a in args.servos]
    bus = Bus(servos)
    stop = threading.Event()
    path, thread = serve(bus, args.link, stop)
    print(f'STS emulator on {path} (link {args.link}) with IDs {sorted(bus.servos)}', flush=True)
    try:
        for line in sys.stdin:
            words = line.split()
            if words[:1] == ['hand'] and len(words) == 3:
                s = bus.by_id(int(words[1]))
                with bus.lock:
                    if s and not s.mem[TORQUE]:
                        s.encoder = float(words[2]) + s.offset()
                        print(f'ID {s.sid} now reads {s.present()}', flush=True)
                    else:
                        print('no such ID, or its torque is on', flush=True)
            elif words[:1] == ['stats']:
                with bus.lock:
                    print(f'{bus.packets} packets; ' + ', '.join(
                        f'ID {s.sid}: raw {s.present()} torque {s.mem[TORQUE]} acc {s.mem[ACC]} '
                        f'speed {int.from_bytes(s.mem[SPEED:SPEED + 2], "little")}' for s in bus.servos.values()),
                        flush=True)
        stop.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1)


if __name__ == '__main__':
    main()
