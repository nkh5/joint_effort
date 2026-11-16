"""Every reason the arm can stop, pause or slow down - one table.

The teleop node reports exactly one of these as `stop` (why it is off or paused) and at most one
as `warning` (moving, but limited). The terminal, `amw status`, the LCD and docs/troubleshooting.md
all show the same text, so there is one place to look.

  kind   stop   arm turned OFF; press ON (D-pad right) to try again
         pause  arm stays ON but waits; do what `fix` says (centre the stick)
         warn   arm keeps moving, but one direction is blocked or slowed

`python3 -m amw_teleop.faults` prints the table as Markdown (docs/troubleshooting.md is generated from it).
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Fault:
    code: str
    kind: str          # stop | pause | warn
    text: str          # what happened; may use {joint} {value} {detail}
    fix: str           # what to do


FAULTS = {f.code: f for f in [
    # ---- off (not a fault)
    Fault('OFF', 'stop', 'Arm is off.', 'Centre the stick and press ON (D-pad right).'),
    Fault('TURNED_OFF', 'stop', 'Turned off with the D-pad right button.', 'Press ON (D-pad right) to continue.'),
    Fault('STOP_REQUESTED', 'stop', 'Stopped by `amw stop`.', 'Press ON (D-pad right) to continue.'),
    # ---- not ready when ON was pressed
    Fault('NOT_READY', 'stop', 'Arm not ready: {detail}.',
          'Wait until `amw start` has finished starting (Servo initialized), then press ON.'),
    # ---- input
    Fault('JOYSTICK_LOST', 'stop', 'No controller input for {value} s.',
          'Check the Joy-Con is connected (Bluetooth) and the amw_joycon node is running; `amw joystick` shows '
          'raw input. Press ON when it is back.'),
    Fault('JOYSTICK_BAD', 'stop', 'Unusable controller report: {detail}.',
          'Check that only one Joy-Con is connected and teleop.yaml matches the amw_joycon button order.'),
    Fault('IMU_LOST', 'stop', 'No Joy-Con motion data for {value} s while rotating.',
          'Check the amw_joycon terminal (it waits for the Joy-Con to be held still at start). Press ON when '
          'it is back.'),
    # ---- arm feedback and hardware
    Fault('FEEDBACK_STALE', 'stop', 'Joint states stopped arriving ({value} s old).',
          'Motor bus or driver problem: read the `amw start` terminal (e.g. "SyncRead failed"), check 12 V and '
          'the adapter cable, restart.'),
    Fault('JOINT_OUTSIDE_LIMIT', 'stop', '{joint} is at {value} deg, outside its limits {detail}.',
          'Turn off (Ctrl-C `amw start`), move it inside by hand, restart. Or widen its limits in arm.yaml if '
          'that travel is safe.'),
    Fault('CONTROLLER_INACTIVE', 'stop', 'iron_arm_controller is not active ({detail}).',
          'Read the `amw start` terminal for a controller or driver error, then restart.'),
    Fault('MOTOR_HOT', 'stop', '{joint} motor is at {value} C.',
          'Turn off and let it cool. Check for binding or the joint pushing against a stop.'),
    Fault('MOTOR_VOLTAGE', 'stop', '{joint} motor supply is {value} V.',
          'Check the 12 V supply, the fuse and the wiring to the servo adapter.'),
    # ---- MoveIt Servo
    Fault('SERVO_COLLISION', 'stop', 'MoveIt Servo stopped the arm: about to collide ({detail}).',
          'Turn off and move the arm away by hand. If nothing is really there, the table/PCB/fixture positions '
          'in amw.urdf do not match the bench.'),
    Fault('SERVO_ERROR', 'stop', 'MoveIt Servo reports a problem: {detail}.', 'Restart `amw start`.'),
    # ---- pauses (arm stays ON)
    Fault('CENTRE_STICK', 'pause', 'Waiting for the stick to be centred.',
          'Let go of the stick for a moment.'),
    Fault('MODE_CHANGED', 'pause', 'Switched to {detail} mode.', 'Centre the stick to continue.'),
    # ---- warnings (arm keeps moving)
    Fault('JOINT_LIMIT', 'warn', '{joint} is at its {detail} limit; motion that way is blocked.',
          'Push the stick the other way.'),
    Fault('SINGULARITY', 'warn', 'Near a pose where the tip cannot move in every direction; motion slowed.',
          'Go back the way you came. To reset, turn off and move the arm by hand to a bent pose.'),
    Fault('COLLISION_NEAR', 'warn', 'Slowing down near a modelled obstacle ({detail}).', 'Move away from it.'),
    Fault('MOTOR_WARM', 'warn', '{joint} motor is at {value} C.', 'Give it a break soon.'),
]}

KINDS = ('stop', 'pause', 'warn')


def report(code, joint=None, value=None, detail=None):
    """A fault as the dict published in /amw/status."""
    f = FAULTS[code]
    fields = {'joint': joint or '', 'value': '' if value is None else value, 'detail': detail or ''}
    return {'code': code, 'kind': f.kind, 'joint': joint, 'text': f.text.format(**fields), 'fix': f.fix}


def markdown():
    rows = ['| code | what happens | message | what to do |', '| --- | --- | --- | --- |']
    effect = {'stop': 'arm turns OFF', 'pause': 'arm waits (stays ON)', 'warn': 'keeps moving, limited'}
    for f in FAULTS.values():
        rows.append(f'| `{f.code}` | {effect[f.kind]} | {f.text} | {f.fix} |')
    return '\n'.join(rows)


if __name__ == '__main__':
    print(markdown())
