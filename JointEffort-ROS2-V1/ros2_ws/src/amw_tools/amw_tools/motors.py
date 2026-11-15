"""amw motors: direct servo-bus tools. ROS must be stopped (the adapter can only have one user).

  amw motors scan [--max N]        which IDs answer
  amw motors read                  angle, limits, torque, voltage, temperature, current per joint
  amw motors gains                 P/I/D, deadband, punch and ACC stored in each servo
  amw motors tune                  write gains.yaml into the servos (EEPROM)
  amw motors punch JOINT|all VAL   set the minimum start force (factory 16)
  amw motors set-id OLD NEW        give a servo a new ID (connect only that servo)
  amw motors recenter JOINT        make the servo read 2048 where it is now (then recalibrate its zero)
"""
import yaml

from . import stsbus as sts
from .common import GAINS_YAML, ask, load_arm, open_bus


def scan(max_id=20, port=None):
    arm = load_arm()
    expected = {j.motor_id: j.name for j in arm}
    found = []
    with open_bus(arm, port) as bus:
        for sid in range(1, max_id + 1):
            if bus.ping(sid):
                found.append(sid)
                raw = bus.position(sid)
                print(f'ID {sid:3d}: answers, raw {raw}   {expected.get(sid, "(not in arm.yaml)")}')
    missing = [f'{n} (ID {i})' for i, n in expected.items() if i not in found]
    print(f'\n{len(found)} servo(s) found.' + (f' Missing: {", ".join(missing)}' if missing else ''))
    return 0 if not missing else 1


def read(port=None):
    from .preflight import run
    return run(load_arm(), port, readonly=True)


def _gains(bus, sid):
    v = {k: bus.read_u8(sid, a) for k, a in [('p', sts.ADDR_P), ('i', sts.ADDR_I), ('d', sts.ADDR_D),
                                            ('db_cw', sts.ADDR_DEADBAND_CW), ('db_ccw', sts.ADDR_DEADBAND_CCW),
                                            ('punch', sts.ADDR_PUNCH), ('acc', sts.ADDR_ACC)]}
    return v


def gains(port=None):
    arm = load_arm()
    print(f'{"joint":8} {"id":>3} {"P":>4} {"I":>4} {"D":>4} {"deadband":>9} {"punch":>6} {"ACC":>4}')
    with open_bus(arm, port) as bus:
        for j in arm:
            g = _gains(bus, j.motor_id)
            f = {k: '?' if x is None else x for k, x in g.items()}
            print(f'{j.name:8} {j.motor_id:>3} {f["p"]:>4} {f["i"]:>4} {f["d"]:>4} {f["db_cw"]!s:>4}/{f["db_ccw"]!s:<4} '
                  f'{f["punch"]:>6} {f["acc"]:>4}')
    return 0


def tune(port=None, yes=False):
    arm = load_arm()
    wanted = (yaml.safe_load(GAINS_YAML.read_text()) or {}).get('joints') or {}
    plan = []
    with open_bus(arm, port) as bus:
        for j in arm:
            want = wanted.get(j.name) or {}
            have = _gains(bus, j.motor_id)
            for key, addrs in [('p', [sts.ADDR_P]), ('i', [sts.ADDR_I]), ('d', [sts.ADDR_D]),
                               ('deadband', [sts.ADDR_DEADBAND_CW, sts.ADDR_DEADBAND_CCW]), ('punch', [sts.ADDR_PUNCH])]:
                if key not in want:
                    continue
                value = want[key]
                if type(value) is not int or not 0 <= value <= 255:
                    raise SystemExit(f'gains.yaml {j.name}.{key} must be a whole number 0..255 (is {value!r})')
                current = have['db_cw'] if key == 'deadband' else have[key]
                if current != value or (key == 'deadband' and have['db_ccw'] != value):
                    plan.append((j, key, addrs, current, value))
        if not plan:
            print('All servos already match gains.yaml. Nothing to write.')
            return 0
        for j, key, _, current, value in plan:
            print(f'{j.name} (ID {j.motor_id}) {key}: {current} -> {value}')
        if not yes and not ask('\nWrite these to the servos (stored permanently)?'):
            print('Nothing written.')
            return 1
        for j, key, addrs, _, value in plan:
            for a in addrs:
                bus.write_eeprom(j.motor_id, a, [value])
    print('Written. Run `amw motors gains` to confirm.')
    return 0


def punch(target, value, port=None):
    if not 0 <= value <= 255:
        raise SystemExit('punch must be 0..255 (factory value 16)')
    arm = load_arm()
    joints = list(arm) if target == 'all' else [arm.joints.get(target)]
    if None in joints:
        raise SystemExit(f'unknown joint {target!r}; use joint_1 .. joint_5 or all')
    with open_bus(arm, port) as bus:
        for j in joints:
            bus.write_eeprom(j.motor_id, sts.ADDR_PUNCH, [value])
            print(f'{j.name} (ID {j.motor_id}): punch now {bus.read_u8(j.motor_id, sts.ADDR_PUNCH)}')
    print('Stored in the servo. Add "punch: %d" to gains.yaml for the joints you changed so it is tracked.' % value)
    return 0


def set_id(old, new, port=None):
    if not 1 <= new <= 253 or not 1 <= old <= 253:
        raise SystemExit('IDs must be 1..253')
    arm = load_arm()
    with open_bus(arm, port) as bus:
        if not bus.ping(old):
            raise SystemExit(f'ID {old} does not answer')
        if bus.ping(new):
            raise SystemExit(f'ID {new} is already taken by another servo on the bus')
        bus.write_eeprom(old, sts.ADDR_ID, [new])
        ok = bus.ping(new)
    print(f'ID {old} -> {new}: ' + ('done, it answers as ' + str(new) if ok else 'FAILED (no answer on the new ID)'))
    return 0 if ok else 1


def recenter(joint, port=None, yes=False):
    """Change the servo's stored offset so it reads 2048 at its current position."""
    arm = load_arm()
    j = arm.joints.get(joint)
    if j is None:
        raise SystemExit(f'unknown joint {joint!r}')
    with open_bus(arm, port) as bus:
        if bus.read_u8(j.motor_id, sts.ADDR_TORQUE):
            raise SystemExit(f'{joint}: torque is ON. Stop the arm and support it first.')
        before = bus.position(j.motor_id)
        old = sts.offset_decode(bus.read_u16(j.motor_id, sts.ADDR_OFFSET))
        new = old + (before - 2048)
        new = (new + 2048) % 4096 - 2048
        print(f'{joint} (ID {j.motor_id}) reads {before}; offset {old} -> {new} makes it read 2048 here.')
        print('This changes the number the motor reports, so its zero in arm.yaml becomes wrong.')
        if not yes and not ask('Continue?'):
            return 1
        bus.write_eeprom(j.motor_id, sts.ADDR_OFFSET, list(sts.offset_encode(new).to_bytes(2, 'little')))
        after = bus.position(j.motor_id)
        if after is None or min(abs(after - 2048), 4096 - abs(after - 2048)) > 3:
            bus.write_eeprom(j.motor_id, sts.ADDR_OFFSET, list(sts.offset_encode(old).to_bytes(2, 'little')))
            raise SystemExit(f'{joint}: read {after}, not 2048 - old offset {old} restored')
    print(f'{joint}: now reads {after}. Next: put it on its 0-degree mark and run `amw calibrate zero {joint}`.')
    return 0
