"""amw calibrate: set each joint's zero, check its direction and measure its travel. Talks to the
motors directly (ROS must be stopped) and never turns torque on.

  amw calibrate zero  [joint ...]   arm on its 0-degree marks -> write `zero` for those joints
  amw calibrate check [joint ...]   move each joint by hand -> checks direction and motor ID
  amw calibrate limits [joint ...]  move joints through their safe travel -> propose limits

Everything is written to arm.yaml (comments kept). See docs/calibration.md.
"""
import math
import time

from .common import ARM_YAML, SRDF, URDF, ArmConfigError, ask, deg, load_arm, open_bus

DIRECTION_WORDS = (('forward', 'back'), ('left', 'right'), ('up', 'down'))


def read_raw(bus, j, samples=5):
    values = []
    for _ in range(samples):
        v = bus.position(j.motor_id)
        if v is None:
            raise SystemExit(f'{j.name}: motor ID {j.motor_id} does not answer')
        values.append(v)
        time.sleep(0.01)
    return round(sum(values) / len(values))


def require_torque_off(bus, joints):
    from .stsbus import ADDR_TORQUE
    on = [j.name for j in joints if bus.read_u8(j.motor_id, ADDR_TORQUE)]
    if on:
        raise SystemExit(f'Torque is ON for {on}. Stop the arm (Ctrl-C `amw start`) and support it first.')


def pick(arm, names):
    if not names:
        return list(arm)
    bad = [n for n in names if n not in arm.joints]
    if bad:
        raise SystemExit(f'unknown joint(s) {bad}; use joint_1 .. joint_5')
    return [arm.joints[n] for n in names]


def update(joint_name, values):
    try:
        from amw_bringup.arm import update_values
        update_values(ARM_YAML, joint_name, values)
    except ArmConfigError as e:
        raise SystemExit(str(e))


def validate_after_write():
    try:
        from amw_bringup.arm import home_pose, load
        arm = load(ARM_YAML)
        home = home_pose(SRDF.read_text())
    except ArmConfigError as e:
        print(f'\nWARNING: arm.yaml now has a problem: {e}\nFix it before starting the arm.')
        return
    outside = [j.name for j in arm if not j.lower <= home[j.name] <= j.upper]
    if outside:
        print(f'\nWARNING: the mock start pose (SRDF "home") is outside the new limits for {outside}. '
              'Edit the home pose in src/amw_description/srdf/amw.srdf.')
    else:
        print('\narm.yaml checked OK. Restart the arm to use it (no rebuild needed).')


# ---------------------------------------------------------------- zero

def zero(names, yes=False):
    arm = load_arm()
    joints = pick(arm, names)
    with open_bus(arm) as bus:
        require_torque_off(bus, joints)
        print('Put each of these joints on its 0-degree mark (link parallel to the motor\'s long side):')
        for j in joints:
            print(f'  {j.name:8} -> the arm will then read {deg(j.reference):+.2f} deg')
        if not yes:
            input('Press Enter when the arm is in place (Ctrl-C to cancel)... ')
        rows = []
        for j in joints:
            raw = read_raw(bus, j)
            new = j.zero_for(raw, j.reference)
            rows.append((j, raw, new))
    print(f'\n{"joint":8} {"raw":>5} {"old zero":>9} {"new zero":>9} {"change":>14}')
    for j, raw, new in rows:
        print(f'{j.name:8} {raw:5d} {j.zero:9d} {new:9d} {new - j.zero:+6d} ({(new - j.zero) * 360 / 4096:+5.2f} deg)')
    big = [j.name for j, _, new in rows if abs(new - j.zero) > 60]
    if big:
        print(f'\nNote: {big} changed by more than ~5 deg. Expected after a motor swap or re-centre; otherwise '
              'check the joint really is on its mark.')
    if not yes and not ask('\nWrite the new zeros to arm.yaml?'):
        print('Nothing written.')
        return 1
    for j, _, new in rows:
        update(j.name, {'zero': new})
    print(f'Wrote {ARM_YAML}')
    validate_after_write()
    return 0


# ---------------------------------------------------------------- check (direction + wiring)

def describe_plus(arm, joint, angles):
    """Which way the iron tip should move for a small + turn of `joint`, from the arm model."""
    try:
        import numpy as np
        from amw_teleop.kinematics import Robot
    except ImportError:
        return 'see the RViz model (numpy not available here)'
    robot = Robot(URDF)
    names = robot.names('iron')
    q = np.array([angles[n] for n in names])
    step = np.zeros(5)
    step[names.index(joint)] = math.radians(10)
    move = (robot.fk('iron', q + step)[:3, 3] - robot.fk('iron', q)[:3, 3]) * 1000
    parts = [f'{abs(v):.0f} mm {words[0] if v > 0 else words[1]}'
             for v, words in sorted(zip(move, DIRECTION_WORDS), key=lambda p: -abs(p[0])) if abs(v) >= 3]
    return 'the iron tip moves ' + (', '.join(parts) if parts else 'very little (watch the link itself)')


def check(names, threshold_deg=3.0):
    arm = load_arm()
    joints = pick(arm, names)
    print('For each joint: move ONLY that joint by hand, about 10-20 degrees in its + direction, then press '
          'Enter.\n(+X = forward, away from the base; +Y = left; +Z = up.  Ctrl-C to stop.)\n')
    results = []
    with open_bus(arm) as bus:
        require_torque_off(bus, list(arm))
        for j in joints:
            before = {k.name: k.angle(read_raw(bus, k, 3)) for k in arm}
            print(f'{j.name} ({j.motor_id}): + direction = {describe_plus(arm, j.name, before)}')
            input('  move it now, then press Enter... ')
            after = {k.name: k.angle(read_raw(bus, k, 3)) for k in arm}
            change = {k: deg(after[k] - before[k]) for k in after}
            moved_others = [k for k in change if k != j.name and abs(change[k]) > threshold_deg]
            d = change[j.name]
            if abs(d) < threshold_deg:
                verdict = 'NO MOVE SEEN' + (f' - but {moved_others} moved: motor IDs swapped?' if moved_others else
                                            ' - move it further and run again')
            elif d > 0:
                verdict = 'OK'
            else:
                verdict = f'REVERSED - set dir: {-j.dir} for {j.name}'
            if moved_others and abs(d) >= threshold_deg:
                verdict += f' (also saw {moved_others} move - did you bump them?)'
            print(f'  {j.name} changed {d:+.1f} deg  ->  {verdict}\n')
            results.append((j, d, verdict))
    reversed_ = [j for j, _, v in results if v.startswith('REVERSED')]
    print('Summary: ' + ', '.join(f'{j.name} {v.split(" -")[0]}' for j, _, v in results))
    if reversed_ and ask(f'Flip dir for {[j.name for j in reversed_]} in arm.yaml?'):
        for j in reversed_:
            update(j.name, {'dir': -j.dir})
        print('Flipped. Their zero was worked out with the old direction, so put those joints on their marks '
              f'and run `amw calibrate zero {" ".join(j.name for j in reversed_)}` next.')
        validate_after_write()
    return 0 if all(v == 'OK' for _, _, v in results) else 1


# ---------------------------------------------------------------- limits

def limits(names, margin_deg=2.0):
    arm = load_arm()
    joints = pick(arm, names)
    seen = {j.name: [math.inf, -math.inf] for j in joints}
    print('Move each joint SLOWLY to both ends of its safe travel (stop before collisions, cable strain or '
          'hard stops). Ctrl-C when done.\n')
    with open_bus(arm) as bus:
        require_torque_off(bus, joints)
        try:
            while True:
                line = []
                for j in joints:
                    raw = bus.position(j.motor_id)
                    if raw is None:
                        line.append(f'{j.name[-1]}: ---')
                        continue
                    a = deg(j.angle(raw))
                    s = seen[j.name]
                    s[0], s[1] = min(s[0], a), max(s[1], a)
                    line.append(f'{j.name[-1]}:{a:+6.1f} [{s[0]:+6.1f},{s[1]:+6.1f}]')
                print('\r' + '  '.join(line), end='', flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print('\n')
    print(f'{"joint":8} {"seen min..max deg":>20} {"current limits":>18} {"proposed":>18}')
    proposals = {}
    for j in joints:
        lo, hi = seen[j.name]
        if not math.isfinite(lo) or hi - lo < 2 * margin_deg + 1:
            print(f'{j.name:8} {"(not enough travel seen)":>20}')
            continue
        new = (round(lo + margin_deg, 1), round(hi - margin_deg, 1))
        proposals[j.name] = new
        print(f'{j.name:8} {lo:+9.1f}..{hi:+7.1f} {deg(j.lower):+8.1f}..{deg(j.upper):+7.1f} '
              f'{new[0]:+8.1f}..{new[1]:+7.1f}')
    if not proposals:
        return 1
    print(f'\nProposed limits stay {margin_deg} deg inside what you reached.')
    if not ask('Write the proposed limits to arm.yaml?'):
        print('Nothing written. You can also edit lower_deg/upper_deg in arm.yaml by hand.')
        return 1
    for name, (lo, hi) in proposals.items():
        update(name, {'lower_deg': float(lo), 'upper_deg': float(hi)})
    print(f'Wrote {ARM_YAML}')
    validate_after_write()
    return 0
