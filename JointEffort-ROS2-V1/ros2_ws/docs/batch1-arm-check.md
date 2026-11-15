# Batch 1 arm check (about 20-30 min)

> Do steps 0-5 here (same angles, directions, hold, jogs), then docs/batch2-arm-check.md
> instead of step 6 (driving with the new teleop node).

Batch 1 changed structure, calibration format, driver patch, launch and tools, but **not** how the arm
should behave: same angles, same limits, same joystick logic. This check confirms that on the real
arm. Do the steps in order and stop at the first thing that does not match.

Your old setup stays untouched: the new code goes into a **new folder** with its own driver build, so
`~/jackbranch` keeps working as a fallback.

Safety as always: 12 V disconnect within reach, support the arm whenever torque goes off
(Ctrl-C on `amw start real` turns torque off on every servo).

## 0. Code on GitHub (laptop, once)

The cleanup is one commit on `WorkingV1`. If it is already pushed there, skip this step.

## 1. Fresh clone and build (Pi)

```bash
ssh jointeffort@<pi-address>        # the Pi's current Tailscale address (hostname jointeffort)
git clone -b WorkingV1 https://github.com/nkh5/joint_effort ~/joint_effort
cd ~/joint_effort/JointEffort-ROS2-V1/ros2_ws
echo "alias amw=$PWD/amw" >> ~/.bashrc && source ~/.bashrc
# fresh Ubuntu Server: install ROS 2 Jazzy and the packages in docs/setup.md first
amw build --workers 1
amw test
```

Check: colcon ends with `Summary: 8 packages finished`; `amw test` ends with `OK` (60 tests).

Note down: build OK / test count.

## 2. Same angles as before (Pi, arm powered, ROS stopped, torque off)

Leave the arm where it is (anywhere). Run the old check and the new one back to back without
touching the arm:

```bash
cd ~/jackbranch/ros2_ws 2>/dev/null || cd ~/jackbranch
./scripts/run_preflight.sh --readonly        # skip if ~/jackbranch is gone after the reflash
cd ~/joint_effort/JointEffort-ROS2-V1/ros2_ws
amw motors read
```

Check: for every joint, `angle deg` in the new table equals `angle deg` in the old one (within 0.1),
and `raw` is the same. Both end without "no reply".

Note down: the five angles from each table.

## 3. Directions (Pi, ROS stopped, torque off, arm supported)

```bash
amw calibrate check
```

For each joint it says which way the iron tip should move for a "+" turn. Move only that joint by
hand 10-20 degrees that way, press Enter. Answer **N** if it offers to change anything.

Check: all five say `OK`.

Note down: verdict per joint. If any says `REVERSED`, stop and tell me (the old setup would have been
reversed too).

## 4. Powered hold (Pi)

Move the arm by hand somewhere inside all limits, near the usual start pose (elbow bent, not on its
straight-arm mark). Then:

```bash
amw start real --no-teleop
```

Check, in order: the motor table and `MOTOR CHECK OK: all joints answer and are inside their limits`;
`[amw] real arm, controllers only`; the arm holds where it is with **no jump**.

Second terminal (`ssh` again):

```bash
amw status --once
```

Check: `Ctrl` line shows `iron_arm_controller active, joint_state_broadcaster active`; every joint row
has an angle, a positive margin, and temp/volt/amps numbers.

Note down: no jump yes/no, temps and volts.

## 5. Jogs (Pi, `amw start real --no-teleop` still running)

Second terminal:

```bash
amw jog joint_1 +3
amw jog joint_1 -3
amw jog joint_2 +3
amw jog joint_2 -3
amw jog joint_3 +3
amw jog joint_3 -3
amw jog joint_4 +3
amw jog joint_4 -3
amw jog joint_5 +3
amw jog joint_5 -3
```

Check: `error_code 0`; only the jogged joint moves noticeably; the real joint moves the way you
expect. `got` should be like before (e.g. the elbow's -3 reached about -1.9 deg on Oct 26: same or
better is fine). Optional comparison with the Nov 6 recordings: `amw track all`.

Note down: asked / got per jog.

Stop: support the arm, Ctrl-C the first terminal.

## 6. Joystick driving (Pi)

Pico plugged into the Pi. Arm inside its limits, nothing near the iron.

```bash
amw start real
```

(One terminal now starts everything, including the joystick. `mapping_confirmed:=true` is the
default.) Wait for `Servo initialized successfully`. Second terminal:

```bash
amw status
```

Panel 1 OFF, centre the stick until the status line says `Ready: ...`, panel 1 ON. Then, at a
moderate speed level (5-8):

| action | expected |
| --- | --- |
| stick +X / +Y / twist + | tip moves forward / left / up |
| stick button (mode) | arm stops; status shows rotation; centre the stick and it continues |
| rotation mode, stick X / Y | iron yaw / pitch, tip stays put |
| drive a joint into its limit | stops; `amw status` WHY line names the joint-limit reason and the joint row shows `<- Servo halts` |
| panel 1 OFF | stops |

Check also that the LCD still shows the state (same topics and domain as before).

Note down: each row OK / not, and what the WHY line said when it stopped.

Stop: panel 1 OFF, support the arm, Ctrl-C.

## Report back

Send me the notes from steps 1-6 (a photo of a filled-in table is fine). If everything matches,
Batch 2 starts: the merged teleop node, the ON/OFF latch on the Pi with new Pico firmware, one status
topic with stop reasons, limit/singularity escape, and the LCD.
