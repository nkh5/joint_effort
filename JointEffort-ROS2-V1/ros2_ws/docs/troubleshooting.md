# Why did the arm stop?

Look in **one** place: `amw status` (second terminal), or the LCD. Both show the same thing the
`amw start` terminal prints when the state changes:

```
[ON] moving (translation)
[WAIT] Switched to rotation mode. -- Centre the stick to continue.
[STOP] joint_3 is at +3.1 deg, outside its limits -84.8..+0.0 deg. -- Turn off (Ctrl-C ...), move it inside by hand, restart.
```

`amw status` also lists the last events with how long ago they happened, so you can see what
happened after the fact.

- **stop**: the arm turns OFF. Fix the cause and press ON (panel 1). Nothing turns it back on by
  itself, not even a reconnected joystick.
- **pause**: the arm stays ON and waits (centre the stick).
- **warning**: the arm keeps moving; one direction is blocked or slowed.

## All reasons

| code | what happens | message | what to do |
| --- | --- | --- | --- |
| `OFF` | arm turns OFF | Arm is off. | Centre the stick and press ON (panel 1). |
| `TURNED_OFF` | arm turns OFF | Turned off with panel 1. | Press ON (panel 1) to continue. |
| `STOP_REQUESTED` | arm turns OFF | Stopped by `amw stop`. | Press ON (panel 1) to continue. |
| `NOT_READY` | arm turns OFF | Arm not ready: {detail}. | Wait until `amw start` has finished starting (Servo initialized), then press ON. |
| `JOYSTICK_LOST` | arm turns OFF | No joystick input for {value} s. | Check the Pico USB cable; `amw joystick` shows raw input. Press ON when it is back. |
| `JOYSTICK_BAD` | arm turns OFF | Unusable joystick report: {detail}. | Check that only one controller is connected and the Pico runs Pico_Controller.ino. |
| `FEEDBACK_STALE` | arm turns OFF | Joint states stopped arriving ({value} s old). | Motor bus or driver problem: read the `amw start` terminal (e.g. "SyncRead failed"), check 12 V and the adapter cable, restart. |
| `JOINT_OUTSIDE_LIMIT` | arm turns OFF | {joint} is at {value} deg, outside its limits {detail}. | Turn off (Ctrl-C `amw start`), move it inside by hand, restart. Or widen its limits in arm.yaml if that travel is safe. |
| `CONTROLLER_INACTIVE` | arm turns OFF | iron_arm_controller is not active ({detail}). | Read the `amw start` terminal for a controller or driver error, then restart. |
| `MOTOR_HOT` | arm turns OFF | {joint} motor is at {value} C. | Turn off and let it cool. Check for binding or the joint pushing against a stop. |
| `MOTOR_VOLTAGE` | arm turns OFF | {joint} motor supply is {value} V. | Check the 12 V supply, the fuse and the wiring to the servo adapter. |
| `SERVO_COLLISION` | arm turns OFF | MoveIt Servo stopped the arm: about to collide ({detail}). | Turn off and move the arm away by hand. If nothing is really there, the table/PCB/fixture positions in amw.urdf do not match the bench. |
| `SERVO_ERROR` | arm turns OFF | MoveIt Servo reports a problem: {detail}. | Restart `amw start`. |
| `CENTRE_STICK` | arm waits (stays ON) | Waiting for the stick to be centred. | Let go of the stick (and twist) for a moment. |
| `MODE_CHANGED` | arm waits (stays ON) | Switched to {detail} mode. | Centre the stick to continue. |
| `JOINT_LIMIT` | keeps moving, limited | {joint} is at its {detail} limit; motion that way is blocked. | Push the stick the other way. |
| `SINGULARITY` | keeps moving, limited | Near a pose where the tip cannot move in every direction; motion slowed. | Go back the way you came. To reset, turn off and move the arm by hand to a bent pose. |
| `COLLISION_NEAR` | keeps moving, limited | Slowing down near a modelled obstacle ({detail}). | Move away from it. |
| `MOTOR_WARM` | keeps moving, limited | {joint} motor is at {value} C. | Give it a break soon. |

## Still stuck?

| symptom | check |
| --- | --- |
| `amw start real` exits right away | read the motor table it prints (`DO NOT START` lines name the joint) |
| LCD says NO DATA | is `amw start` running? `systemctl status amw-lcd`; same ROS domain (88) |
| ON does nothing, `amw status` says joystick lost | `amw joystick` shows raw input; `ls /dev/input/js*` |
| motion is jerky or a joint lags | `amw track joint_N` records it; see the gains in docs/calibration.md |
| a joint moves the wrong way | `amw calibrate check` (docs/calibration.md) |

The raw ROS topics, if you need them: `amw ros2 topic echo /amw/status`, `/iron_servo/status`,
`/diagnostics` (or `rqt_robot_monitor`).
