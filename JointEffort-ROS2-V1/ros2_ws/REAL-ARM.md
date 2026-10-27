# Real arm bring-up runbook (V1)

Every step lists the exact commands, what to check, and what to write in
[`calibration/bringup-log.md`](calibration/bringup-log.md). Do the steps in order; each one
relies on the previous one passing. Stop at the first thing that does not match and fix it
(the "If it fails" lines say how) before moving on.

Run every command **inside the distrobox** (prompt starts with `📦`), from this `ros2_ws` folder:

```bash
distrobox enter <your-jazzy-box>
cd "$HOME/Fall 2026/Senior Design/joint_effort/JointEffort-ROS2-V1/ros2_ws"
export JE_ROS_DOMAIN_ID=88
```

`./scripts/run_*.sh` always use ROS domain 88. Anything you run through `./scripts/with_ros.sh`
(echo, `ros2 control ...`) needs `JE_ROS_DOMAIN_ID=88` - the `export` line covers that terminal.
A bare `ros2 ...` command sees nothing (wrong domain).

## Current state (October 26, 2026)

| joint | datum (rad) | URDF limit (rad) | axis | notes |
| --- | --- | --- | --- | --- |
| 1 base | 0.0000 | −0.524 .. 0.524 | `0 0 1`, csv_angle_sign −1 | can reach −1.57 .. 1.35; widen if y travel runs out |
| 2 shoulder | −0.4160 (upper arm vertical at its mark) | −1.5 .. 0.6 | `0 0 1` | can reach −1.70 .. 0.65 |
| 3 elbow | −0.8409 (arm straight at its mark) | 0.0 .. 1.48 | `0 0 1` | can reach −1.93 .. 1.53 |
| 4 wrist 1 | 0.0000 | −0.524 .. 0.524 | `0 0 -1` | can reach −1.15 .. 1.03 |
| 5 wrist 2 | +1.5470 (its mark = URDF ~90°) | 0.6 .. 2.4 | `0 0 -1` | can reach −0.10 .. 2.98 |

Home pose (mock start, SRDF `park`): (0, −0.5, 1.2, 0, 1.5). Joystick speed: 10 mm/s at level 10,
full stick (1 mm/s at level 1, 20 mm/s cap); rotation 0.1 rad/s (cap 0.2).
Joints 2, 3 and 5 work well outside the ±20-30° the CSV covers, so their angles are extrapolated
there (worst for wrist 2): add CSV rows inside the working range when you can.

## How the pieces fit

```
Pico (Pico_Controller.ino) --USB HID--> joy_node --> pico_forwarder --> /iron/task_velocity
   --> iron_task_bridge (5x5 reduced Jacobian) --> MoveIt Servo --> iron_arm_controller (JTC)
   --> sts_hardware_interface --> Waveshare adapter --> STS3215 IDs 1-5
```

The mock and the real arm run the **same** stack. Only the `<ros2_control>` hardware block
differs: `launch/real.launch.py` swaps `mock_components/GenericSystem` for
`sts_hardware_interface/STSHardwareInterface` using `config/sts_calibration.yaml`.

Calibration model (direct drive, K = 4096/2π counts/rad):

| quantity | where it comes from |
| --- | --- |
| counts at your 0° mark (`csv_zero_counts`) and the counts-vs-angle sign | `calibration/Calibration.csv`, fitted by `tools/fit_calibration.py` |
| URDF angle of your 0° mark (`datum_offset_rad`) | measured in RViz in step 2 |
| `center_steps` = csv_zero + shift + K·datum | computed by the fit tool |
| direction | URDF `<axis>`. The driver reports angle = (center − counts)/K. Joints 4 and 5 are `0 0 -1` (counts rise with +angle). Joint 1 was first set the same way, but the read-only check showed it reversed, so it is back to `0 0 1` with `csv_angle_sign: -1` (its CSV angles used the opposite +) |
| driver hard limits `min_rad`/`max_rad` | URDF `<limit>` ± 0.05 rad (the URDF limit is what the software stops at) |

## Safety rules (always)

- 12 V power disconnect within reach whenever the servos are powered.
- **Ctrl-C on `run_real.sh` turns torque OFF on every servo: support the arm first.**
  `readonly:=true` also means torque off. After a crash, servos may keep holding their last goal.
- Only one program may use the servo adapter at a time. Preflight checks this (it opens the
  port exclusively) - if it says it cannot open the port, something else has it.
- Never run `tools/sts_set_center.py` or the driver's `/one_key_calibration` service except
  where step 2b tells you to: they change every calibrated number for that motor.
- Do not run `tools/generate_iron.py`: it regenerates `iron.urdf` and would undo the V1 axis
  flips, the elbow limit and the home pose.

---

## 0. One-time setup

### 0.1 Snapshot the repo before changing anything

From the repository root (`joint_effort/`):

```bash
git status
git checkout -b pre-v1-snapshot
git add JointEffort-ROS2-V0 JointEffort-Physical-Tests kory_input_changes
git commit -m "Snapshot of real-arm work before V1 rebuild"
git checkout main
```

Do **not** add `JointEffort-ROS2-V0.zip` or any `build/ install/ log/ work/` folders.

### 0.2 Put V1 in place and retire the duplicates

Copy the `JointEffort-ROS2-V1` folder to the repo root (next to `JointEffort-ROS2-V0`), then:

```bash
mkdir -p archive
git mv kory_input_changes archive/kory_input_changes
git add JointEffort-ROS2-V1 JointEffort-Physical-Tests/USE-V1-INSTEAD.md
git commit -m "Add JointEffort-ROS2-V1: one workspace for mock and real arm"
```

What stays and why:

| folder | status |
| --- | --- |
| `JointEffort-ROS2-V1/` | **the** workspace from now on (mock + real arm) |
| `JointEffort-ROS2-V0/` | reference only (the state that passed the mock tests) |
| `JointEffort-Physical-Tests/` | keep **only** `inspect_motors.sh` and `motor_test.sh assign-id/verify-id` for ID work. Do not build or launch its `ros2_ws` (custom driver, heartbeat, held-enable layout). |
| `Pico_Controller/Pico_Controller.ino` | the firmware to use (no heartbeat). Other `.ino` copies are not used. |
| `archive/kory_input_changes/` | heartbeat/kory-layout variant, kept for reference |

### 0.3 Serial and joystick access from the distrobox

On the **Fedora host** (not in the box), once:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in. Then, with the servo adapter and the Pico plugged in, **inside the box**:

```bash
chmod +x scripts/*.sh
./scripts/check_serial.sh
```

Check: `dialout` is in the groups line, the adapter
`usb-1a86_USB_Single_Serial_5B14028914-if00` is listed and `readable/writable: yes`, a
`/dev/input/js*` exists, and the last line says `python3 serial ... + yaml: ok`.

If it fails: no `dialout` → log out of the host session fully and back in. Adapter missing →
`ls /dev/serial/by-id/` on the host; if your adapter has a different id, put it in
`serial_port` in `src/joint_effort_iron/config/sts_calibration.yaml`.

### 0.4 Dependencies (inside the box)

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions git \
  python3-numpy python3-scipy python3-yaml python3-serial python3-pytest python3-matplotlib \
  python3-pyqt5 qtbase5-dev ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-moveit-servo \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joy
```

### 0.5 Driver, build, unit tests

```bash
./scripts/get_driver.sh
./scripts/with_ros.sh colcon build --symlink-install --base-paths src
./scripts/with_ros.sh python3 -m pytest -q tests
```

Check: `sts_hardware_interface at eeb0ad0 ...`; colcon `Summary: 9 packages finished`; pytest
`52 passed`. Add `--parallel-workers 1` to colcon if the laptop runs out of memory.

Log: build OK / test count.

---

## 1. Mock regression (no motors needed)

Proves the V1 changes (axis flips, elbow home pose, controller name) did not break the stack
you already tested.

### 1.1 Automated check

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/check_iron_live.py
```

It starts its own mock; do not run `run_iron.sh` at the same time. Check: three `PASS:` lines.

### 1.2 With the Pico

Terminal 1:

```bash
./scripts/run_iron.sh
```

Check: RViz shows the arm in the home pose (shoulder −0.5 rad, elbow 1.2 rad - elbow clearly
bent, iron above the PCB), not folded into the table.

Terminal 2 (Pico plugged in):

```bash
./scripts/run_input.sh mapping_confirmed:=true
```

Check the printed status lines: `Ready: hold enable button to move` once the stick is centred.
Press panel 1 (ON) with the stick centred, then:

| action | expected in RViz |
| --- | --- |
| stick +X | tip moves along world +X |
| stick +Y | tip moves along world +Y |
| twist + | tip moves up (+Z) |
| stick button (mode) | arm pauses; status shows ROTATION; panel 2 resumes |
| rotation mode, stick X / Y | iron yaw / pitch, tip stays put |
| panel 1 | OFF, arm stops |

If it fails: `Joystick input lost` and nothing else → joy_node did not find the Pico. Run
`./scripts/with_ros.sh ros2 run joy joy_enumerate_devices` and put the exact name it prints into
`device_name` in `src/joint_effort_input/config/input.yaml` (expected `AMW Controller`). An axis
reversed → flip its entry in `stick_signs` in the same file.

Log: each row OK / reversed.

---

## 2. Datum: where is your 0° mark in URDF angles? (no motors needed)

`Calibration.csv` measured angles from your physical mark (link parallel to the motor's long
side). The driver needs the URDF angle of that mark. Measure it on the model:

```bash
./scripts/run_datum.sh
```

An RViz window and a slider window open. For each joint, drag its slider until the RViz link
sits exactly on the same reference you used for the CSV (parallel to that motor's long side).
Sliders ignore the URDF limits on purpose. Close the slider window. The terminal prints the
five values and a ready-made command; run that command, e.g.:

```bash
python3 tools/fit_calibration.py --write --datum joint_1=+0.0000 joint_2=+0.0000 joint_3=+1.2000 joint_4=+0.0000 joint_5=+0.0000
```

Check the fit output:

- every joint row has a `center` and a `raw travel`;
- no `direction mismatch` lines;
- last line `Calibration COMPLETE: powered launch allowed.`;
- `note:` lines are information (joint 1 and 5 fit errors 3-4.5°, joint 2 limits extrapolated
  beyond the ±20° you measured). They do not block.

Log: the five datum values (rad and deg) and the five `center_steps`.

### 2b. Only if the fit says "Re-offset motor N by S counts"

This happens when a joint's centre would fall outside 0..4095 (most likely the elbow, if its
mark sits far from the middle of its travel). Servo power on, ROS stopped, torque off (arm
supported):

```bash
python3 tools/sts_set_center.py N --shift S
```

Check: `now reads ... (was ..., change S)`. Put that change into `encoder_shift_counts` for that
joint in `src/joint_effort_iron/config/sts_calibration.yaml`, then rerun the same
`fit_calibration.py --write --datum ...` command. Log: motor, shift, new center.

---

## 3. Bus check without ROS

Servo power on (12 V). Arm supported.

```bash
./scripts/run_preflight.sh --readonly
```

Check: five rows, no `no reply`; `torque off`; volt ≈ 12.6; temp < 55; last line
`PREFLIGHT OK (read-only report)`. The `angle rad` column is the angle ROS will report.

Now put **each joint on its 0° mark** (one at a time, or all if the arm allows) and rerun.
Check: for each joint at its mark, `angle rad` ≈ its `datum_offset_rad` (within ~0.03 rad) and
`raw` ≈ `csv_zero_counts` + shift (within ~15 counts).

If it fails: `motor ID N does not answer` → power, cable, adapter jumper in position B, or the
ID was not assigned (use `../JointEffort-Physical-Tests/inspect_motors.sh --scan --label chain`).
Raw far from csv_zero at the mark → that motor's offset changed since the CSV was taken
(someone ran set-center/one-key calibration): re-measure that joint's CSV rows.

Log: raw and angle per joint at the mark.

---

## 4. Read-only ROS: wiring and direction (torque off)

Terminal 1:

```bash
./scripts/run_real.sh readonly:=true
```

Check the startup: `PREFLIGHT OK`, then `[real.launch] READ-ONLY (torque off)` with the five
centers, five lines `initialized in mode 0, torque disabled (read-only)`, and RViz showing the
arm in (roughly) the same pose as the real one.

Terminal 2:

```bash
./scripts/run_calibration_monitor.sh
```

For **each joint, one at a time**, move it slowly by hand about ±10° and back:

| check | expected |
| --- | --- |
| which monitor row changes | only that joint's row |
| RViz | that link turns the **same way** as the real one, by about the same amount |
| at the 0° mark | `angle rad` ≈ datum_offset_rad |

If a different row moves → the motor IDs are swapped; fix `motor_id` in
`sts_calibration.yaml`. If RViz turns the **opposite** way → flip that joint's `<axis>` in
`src/joint_effort_iron/urdf/iron.urdf` (`0 0 1` ↔ `0 0 -1`); if its `<limit>` is not symmetric
also swap and negate lower/upper (only joint 3's is). For joint 2 or 3 also negate its mock
`initial_value` near the bottom of `iron.urdf`, its value in the `park` state of
`config/iron.srdf`, and its entry in `initial_positions_rad` in `config/model.json` (the home pose). Then edit `sts_calibration.yaml` as described next, and:

```bash
python3 tools/fit_calibration.py --write
./scripts/with_ros.sh colcon build --symlink-install --base-paths src --packages-select joint_effort_iron
```

and restart terminal 1. The fit tool will then report `direction mismatch` for that joint,
because its CSV angles were measured with + the other way round. Since the RViz check is the
ground truth, in `sts_calibration.yaml` for that joint set `csv_angle_sign: -1` **and negate its
`datum_offset_rad`** (the same physical pose now has the opposite URDF angle), then run
`python3 tools/fit_calibration.py --write` again; it must end with `COMPLETE`. Rerun step 4 for
that joint.

Log: per joint: row OK, direction OK.

---

## 5. Read-only ROS: travel limits (torque off)

Same two terminals as step 4. Move each joint **slowly** to each end of its safe travel (stop
before collisions, cable strain, or hard stops). Ctrl-C the monitor: it prints `seen ... rad`
per joint.

Check against the URDF limits (see "Current state" above): the
URDF range must be **inside** what the joint can safely reach. The monitor only remembers the
extremes since it started - take the widest values over all runs. If a joint cannot safely reach its
URDF limit, narrow that `<limit>` in `iron.urdf`, then `fit_calibration.py --write` and rebuild
`joint_effort_iron` as in step 4.

Log: seen min/max per joint, any limit changed.

Finish: support the arm, Ctrl-C terminal 1.

---

## 6. First powered hold (controllers only)

Put the arm by hand somewhere **inside** all limits, close to the home pose, nothing near it.
To watch the angles while you do (torque off, nothing else running):
`watch -n 0.5 ./scripts/run_preflight.sh` - **Ctrl-C it before starting run_real.sh**: two programs
on the adapter cause `Failed to read feedback ... via SyncRead` and recovery loops.

```bash
./scripts/run_real.sh teleop:=false
```

Check: `PREFLIGHT OK: all joints answer and are inside their limits.`; `DRIVING MOTORS
(controllers only, teleop:=false)`; five lines `torque enabled`; the arm **holds where it is with
no jump**. In a second terminal:

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh ros2 control list_controllers
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh ros2 topic hz /joint_states
```

Check: `iron_arm_controller` and `joint_state_broadcaster` both `active`; joint states ≈ 50 Hz.

Emergency stop test (support the arm, it goes limp):

```bash
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```

Check: torque off on all five. Do **not** release it with `data: false` (the controller still
holds the old goal and the arm would jump back): support the arm and Ctrl-C terminal 1 instead.

If preflight refuses: it names the joint that is outside its limit - move it inside by hand
(torque is off) and rerun. Log: hold OK, rate, e-stop OK.

---

## 7. Single-joint jogs (controllers only)

Terminal 1: `./scripts/run_real.sh teleop:=false` (arm inside limits). Terminal 2, for each joint:

```bash
./scripts/run_jog.sh joint_1 +3
./scripts/run_jog.sh joint_1 -3
./scripts/run_jog.sh joint_2 +3
./scripts/run_jog.sh joint_2 -3
./scripts/run_jog.sh joint_3 +3
./scripts/run_jog.sh joint_3 -3
./scripts/run_jog.sh joint_4 +3
./scripts/run_jog.sh joint_4 -3
./scripts/run_jog.sh joint_5 +3
./scripts/run_jog.sh joint_5 -3
```

Each moves one joint 3° over 3 s (`--time 1` for 1 s) and prints a table. The other joints keep
their *commanded* targets. Check: `error_code 0`; only the jogged joint has a non-zero
`moved deg`; `got` within ~0.3° of `asked`; the real joint moves the same way as in RViz.
`end - target deg` is each joint's holding error (the elbow sits ~1° off under gravity).
Steps are capped at 10° and refused outside the URDF limit.

Smoothness: add `servo_acc:=N` (1-254, driver default 100) to `run_real.sh` to make the servos
ease into each new goal - try 50, 20, 10 and keep the lowest that still lands within ~0.3°:

```bash
./scripts/run_real.sh teleop:=false servo_acc:=20
```

Read the servos' current P/I/D/deadband/ACC registers (ROS stopped):
`./scripts/run_preflight.sh --readonly --gains`.

If it fails: a joint moves the wrong way → step 4 was not passed for it. `got` much smaller
than `asked` → the joint is load-limited or binding; check mechanics before continuing.

Log: asked/got per jog, direction OK.

---

## 8. Joystick on the real arm

Arm inside limits, nothing near the iron, power disconnect in reach.

Start well away from singular poses - e.g. (0, 0.0, 1.05, 0, 1.4) rad; avoid the upper arm leaning
far back (shoulder −1.24 .. −0.98) or the elbow near its top limit with the arm leaning back.

Terminal 1:

```bash
./scripts/run_real.sh servo_acc:=20      # or without servo_acc
```

Wait until both controllers are active and Servo has started (`Servo initialized
successfully`, then at most a few `Waiting to receive robot state update` lines that stop).
Terminal 2:

```bash
./scripts/run_input.sh mapping_confirmed:=true
```

Set the encoder to a moderate speed level (5-8 of 20). Turn panel 1 OFF, centre the stick until
the status says `Ready: ...`, then press panel 1 ON (if the Pico was already ON when
`run_input.sh` started, it refuses until you do this OFF/ON). Then small moves, one axis at a time, same table as step 1.2 - but now
compare the real tip against the world axes shown in RViz. Then test mode change (pause),
panel 2 resume, and panel 1 OFF.

Stop: panel 1 OFF, support the arm, Ctrl-C terminal 1, then terminal 2.

If it fails: `Bridge refused enable (... joint outside limit?)` → the `iron_task_bridge` log
line `Ignoring joint states (cannot enable): joint_N ... outside URDF limit` names the joint.
`Servo halted: ... collision` with the arm in free space → the modelled table/PCB/fixture in
`iron.urdf` is not where the real ones are; move the arm away and tell the team.

### 8b. When it stops responding

Any fault (Servo halt, stale feedback, a joint at its limit) disables the arm and it **stays**
disabled until panel 1 OFF → centre → ON. Terminal 1 shows none of this. Watch it in three more
terminals (each with the `export JE_ROS_DOMAIN_ID=88` setup):

```bash
# 3: bridge decision - prints on change
./scripts/with_ros.sh env PYTHONUNBUFFERED=1 ros2 topic echo --csv /iron_task_bridge/status | awk '$0!=p{print strftime("%H:%M:%S"), $0; fflush(); p=$0}'
# 4: MoveIt Servo warnings
./scripts/with_ros.sh env PYTHONUNBUFFERED=1 ros2 topic echo --csv /iron_servo/status | awk '$0!=p{print strftime("%H:%M:%S"), $0; fflush(); p=$0}'
# 5: joint-state problems (joint outside limit / stale)
./scripts/with_ros.sh env PYTHONUNBUFFERED=1 ros2 topic echo --csv /iron_task_bridge/feedback_problem | awk '$0!=p{print strftime("%H:%M:%S"), $0; fflush(); p=$0}'
```

(If the filtered versions print nothing, drop everything from `|` on; they then scroll.)

| where | message | meaning / fix |
| --- | --- | --- |
| bridge | `joint_margin` / `predicted_limit` | a joint is within 0.03 rad of its limit and the stick pushes it further: whole motion stops; push the other way, or widen that limit |
| bridge | `singularity` | the arm can't make every tip direction here; the joystick can't get out - stop, move the arm by hand to a better pose (e.g. the start pose above), restart |
| bridge | `feedback_fault_reenable_required` | joint states went stale or a joint slipped past its limit: OFF/ON (move the joint inside first) |
| Servo | `6, Close to a joint bound ... halting` | a joint within 0.04 rad of its limit; Servo halts and the forwarder disables |
| Servo | `4, Close to a collision` | modelled table/PCB/fixture nearby - slows; halts if closer |

Which joint is at its limit (paste while it is stopped):

```bash
./scripts/with_ros.sh python3 -c "
import rclpy,math,xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory as g
from rclpy.node import Node
from sensor_msgs.msg import JointState
L={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in E.parse(g('joint_effort_iron')+'/urdf/iron.urdf').getroot().findall('joint') if j.get('type')=='revolute'}
rclpy.init();n=Node('limits');m=[]
n.create_subscription(JointState,'/joint_states',m.append,1)
while not m: rclpy.spin_once(n,timeout_sec=.1)
for name,q in sorted(zip(m[0].name,m[0].position)):
  lo,hi=L[name];d=min(q-lo,hi-q);print(f'{name}: {q:+.3f}  limit {lo:+.3f}..{hi:+.3f}  closest {d:+.3f} rad'+('   <-- within Servo margin 0.04' if d<0.04 else ''))
"
```

Joystick input, current and temperature:

```bash
./scripts/with_ros.sh ros2 topic echo /joy --field axes          # X, Y, twist near 0 at rest
./scripts/with_ros.sh ros2 topic echo --once /dynamic_joint_states --field interface_values
```

Changing a limit: edit `<limit>` in `src/joint_effort_iron/urdf/iron.urdf` (URDF angles, the
numbers RViz and the readout show - never your CSV mark angles), then
`python3 tools/fit_calibration.py --write`, rebuild `joint_effort_iron`, and restart terminals 1
and 2 (running programs keep the limits they started with).

### Known open issues (October 26)

- Slow motion is stair-stepped (servo deadband + friction + backlash); `servo_acc` may help.
- Elbow holds ~1° off target against gravity and loses ~0.9° on direction reversals (~4-5 mm at
  the tip) - look at P gain/deadband, gravity compensation, and mechanical play.
- `singularity` and `joint_margin` freeze all motion instead of allowing moves away from the problem.
- Any Servo halt latches the arm off until panel 1 OFF/ON.

Log: each axis OK / reversed, pause/resume/OFF OK.

---

## 9. Save the results

```bash
git add JointEffort-ROS2-V1/ros2_ws/src/joint_effort_iron/config/sts_calibration.yaml \
        JointEffort-ROS2-V1/ros2_ws/src/joint_effort_iron/urdf/iron.urdf \
        JointEffort-ROS2-V1/ros2_ws/calibration/bringup-log.md
git commit -m "Real arm: datum, calibration and bring-up results"
git push
```

## Dry run without hardware

Everything from step 3 on can be rehearsed against a software servo bus:

```bash
python3 tests/sts_emulator.py --link /tmp/fake_sts 1=2064 2=2361 3=2752 4=2004 5=2041
# other terminal:
./scripts/run_preflight.sh --readonly --port /tmp/fake_sts
./scripts/run_real.sh readonly:=true port:=/tmp/fake_sts
```

Type `hand 3 2900` + Enter in the emulator terminal to move servo 3 "by hand" while its torque
is off. The emulator answers PING/READ/WRITE/SYNC_READ/SYNC_WRITE; it checks directions and
logic, not dynamics.
