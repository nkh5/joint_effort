"""Explicit EEPROM ID assignment for one physically isolated, supported STS motor.

Protocol reference: ftservo/FTServo_Python scservo_sdk/sms_sts.py:
ID register 5 is EEPROM; register 55 unlocks/relocks EEPROM writes.
No position goal, broadcast, baud, mode, or torque-enable write is permitted.
"""
import time
from .commission import identity, require


class IDBus:
    def __init__(self, port):
        self.port = port
        self.writes = []

    def request(self, ident, opcode, args, count, *, missing=False, response_ids=None):
        require(type(ident) is int and 0 <= ident <= 253, 'Invalid motor ID')
        require(not self.port.in_waiting, 'Unexpected serial bytes; stop and inspect the isolated motor')
        body = bytes([ident, len(args)+2, opcode, *args])
        packet = b'\xff\xff'+body+bytes([~sum(body) & 255])
        require(self.port.write(packet) == len(packet), 'Incomplete serial write; do not retry ID writes')
        self.port.flush()
        raw = bytearray()
        deadline = time.monotonic()+.012
        while len(raw) < count+6 and time.monotonic() < deadline:
            raw.extend(self.port.read(count+6-len(raw)))
        if not raw and missing:
            return None
        require(len(raw) == count+6, 'Missing or incomplete ID-operation reply')
        require(raw[:2] == b'\xff\xff' and raw[2] in (response_ids or {ident}) and raw[3] == count+2,
                'Invalid ID-operation response header/ID/length')
        require(sum(raw[2:]) & 255 == 255, 'ID-operation checksum failed')
        require(raw[4] == 0, f'Motor reported fault bits {raw[4]}')
        require(not self.port.in_waiting, 'Extra replies; check that only one motor is connected')
        return list(raw[5:-1])

    def read(self, ident, address, count=1, *, missing=False):
        require((address, count) in {(2, 7), (9, 10), (30, 1), (33, 38),
                                     (5, 1), (40, 1), (55, 1)}, 'Read outside ID-assignment scope')
        return self.request(ident, 2, [address, count], count, missing=missing)

    def scan(self):
        found = []
        for ident in range(254):
            value = self.read(ident, 5, missing=True)
            if value is not None:
                require(value == [ident], 'Reported ID differs from address')
                found.append(ident)
        return found

    def write(self, ident, address, value, *, changing_to=None):
        require((address == 55 and value in (0, 1)) or (address == 40 and value == 0)
                or (address == 5 and type(value) is int and value in range(1, 7)),
                'Only ID, EEPROM lock, or explicit torque-off writes are allowed')
        if address == 5:
            require(changing_to == value, 'ID change must name the expected new reply address')
        self.writes.append({'id': ident, 'address': address, 'value': value})
        # Firmware may ACK the ID change at the old ID, at the new ID, or not at all.
        # The one-shot write is authoritative only after new-address readback.
        self.request(ident, 3, [address, value], 0, missing=address == 5,
                     response_ids={ident, changing_to} if address == 5 else None)
        time.sleep(.03)
        target = changing_to if address == 5 else ident
        require(self.read(target, address) == [value], 'ID/lock/torque write readback failed')


def checked_motor(bus, ident):
    info = identity(bus, ident)
    state = bus.read(ident, 33, 38)
    require(state[0] == 0 and state[7] in (0, 1) and state[32] == 0,
            'Motor mode, torque or status is invalid')
    require(10 <= state[29]/10 <= 12.9 and state[30] <= 50,
            'Motor voltage or temperature is outside the verified profile')
    return info, state[7]


def assign_isolated(bus, old_id, new_id, *, single_motor_connected, disable_torque=False):
    require(single_motor_connected is True, 'Disconnect all but one supported motor first')
    require(type(old_id) is int and 0 <= old_id <= 253 and type(new_id) is int and new_id in range(1, 7),
            'Old ID must be 0..253; new ID must be 1..6')
    require(bus.scan() == [old_id], 'Expected only the stated old ID; no EEPROM writes sent')
    info, torque = checked_motor(bus, old_id)
    if old_id == new_id:
        return {'result': 'PASS', 'old_id': old_id, 'id': new_id, 'id_changed': False,
                'eeprom_locked': True, 'power_cycle_verification_required': True}
    if torque:
        require(disable_torque, 'Support/unload the motor and use --disable-torque, or turn torque off before assigning')
        bus.write(old_id, 40, 0)
    unlocked = False
    relocked = False
    try:
        unlocked = True  # Even an interrupted unlock needs a best-effort relock.
        bus.write(old_id, 55, 0)
        bus.write(old_id, 5, new_id, changing_to=new_id)  # Never automatically repeat this write.
        new_info = identity_without_lock(bus, new_id)
        require(new_info == info['model_code'], 'Motor model changed after ID assignment')
        bus.write(new_id, 55, 1)
        relocked = True
        require(bus.read(old_id, 5, missing=True) is None, 'Old ID still responds; keep motors isolated')
        return {'result': 'PASS', 'old_id': old_id, 'id': new_id, 'id_changed': True,
                'eeprom_locked': True, 'power_cycle_verification_required': True}
    finally:
        if unlocked and not relocked:
            # Address may already have changed. Never roll back or repeat the ID write.
            for candidate in dict.fromkeys([new_id, old_id]):
                try:
                    if bus.read(candidate, 5, missing=True) == [candidate]:
                        bus.write(candidate, 55, 1)
                        relocked = True
                        break
                except Exception:
                    continue
            if not relocked:
                print('EEPROM RELOCK UNVERIFIED. Keep this motor isolated; inspect old/new IDs before proceeding.', flush=True)


def identity_without_lock(bus, ident):
    # The lock is intentionally open between the ID write and the verified relock.
    info = bus.read(ident, 2, 7)
    require(info[0] == 0 and info[1:3] == [9, 3] and info[3] == ident and info[4] == 0 and info[6] == 1,
            'New-address identity readback failed')
    return info[1] | info[2] << 8


def verify_isolated(bus, ident, *, single_motor_connected):
    require(single_motor_connected is True, 'Disconnect all but the motor being verified')
    require(bus.scan() == [ident], 'Expected only the assigned motor ID; check after the power cycle')
    checked_motor(bus, ident)
    return {'result': 'PASS', 'id': ident, 'eeprom_locked': True, 'writes_sent': 0,
            'scope': 'Readback only; operator must power-cycle before running this persistence check'}
