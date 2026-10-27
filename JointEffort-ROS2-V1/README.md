# Joint Effort ROS 2 - V1 (mock + real iron arm)

One workspace for both the mock arm and the real STS3215 arm. It is `JointEffort-ROS2-V0` (the
stack that passed the Pico + mock tests) plus the real-arm path on
[adityakamath/sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface),
with the fixes below. Start with **[ros2_ws/REAL-ARM.md](ros2_ws/REAL-ARM.md)**: the step-by-step
bring-up runbook.

```bash
cd ros2_ws
./scripts/get_driver.sh
./scripts/with_ros.sh colcon build --symlink-install --base-paths src
./scripts/run_iron.sh                         # mock arm
./scripts/run_real.sh readonly:=true          # real arm, torque off
```

## What was wrong before V1

| problem | effect | fix in V1 |
| --- | --- | --- |
| Two real-arm stacks: V0 `real.launch.py` (sts_hardware_interface) and Physical-Tests' own `IronSystem` driver with heartbeat/held-enable checks | the last runs used `IronSystem`; it latched `encoder outside measured count limits` on joints 2, 3 and 1 | only sts_hardware_interface; Physical-Tests ROS stack retired |
| Elbow limit changed to 0.7..2.3 rad but the mock still started every joint at 0 | mock state outside limits, so the bridge silently refused to enable | home pose (0, −0.5, 1.2, 0, 0) rad in URDF, SRDF and model.json; chosen collision-free with ±0.2 rad on every joint |
| `input.yaml` opened "Joint Effort AMW Controller" but `Pico_Controller.ino` enumerates as "AMW Controller" | joy_node would not find the Pico | `device_name: AMW Controller`, enable = bit 7, mode = button 0 |
| Driver reports angle = (center − counts)/K; joints 4, 5 have counts rising with +angle | those joints would run backwards | URDF axis `0 0 -1` on joints 4 and 5 (kinematics, mock and real all read it). Joint 1 was flipped back after the read-only check (`csv_angle_sign: -1`) |
| `sts_calibration.yaml` empty; CSV zero is your physical mark, not the URDF zero | no usable center_steps | `tools/fit_calibration.py` + RViz datum step (`scripts/run_datum.sh`) |
| MoveIt 2.12.4 Servo start-up race | on the real driver Servo sat in "Waiting to receive robot state update" forever | real launch: Servo is its own scene monitor and the joint-state broadcaster starts once Servo waits |
| No check before torque on | a joint resting outside its range jumps to the limit at activation | `tools/sts_preflight.py`, run by `run_real.sh` |

## Changes made during real-arm bring-up (October 26, 2026)

Everything below is already in this folder. See `ros2_ws/REAL-ARM.md` "Current state" for the table.

| change | files |
| --- | --- |
| Datums measured: j1 0, j2 −0.4160, j3 −0.8409, j4 0, j5 +1.5470 → `center_steps` 2064 / 1763 / 2790 / 2004 / 3049 | `sts_calibration.yaml` |
| Joint 1 direction corrected (axis `0 0 1`, `csv_angle_sign: -1`) | `iron.urdf`, `sts_calibration.yaml` |
| Limits from measured travel: j2 −1.5..0.6, j3 0.0..1.48, j5 0.6..2.4 (j1, j4 still ±0.524) | `iron.urdf`, `sts_calibration.yaml` |
| Home pose (0, −0.5, 1.2, 0, 1.5) | `iron.urdf` (mock initial values), `iron.srdf`, `model.json`, `tests/test_iron_bridge.py` |
| Joystick speeds ×~7: 10 mm/s at level 10 (cap 20 mm/s), rotation 0.1 rad/s (cap 0.2) - core.py and joy_teleop must match | `joint_effort_input/core.py`, `config/teleop.yaml`, `model.json`, `tests/test_input*.py`, `tools/check_input_live.py` |
| Jog tool holds other joints at their commanded targets (stopped the elbow creeping 0.35° per jog); shows holding error | `joint_effort_iron/jog_joint.py` |
| `servo_acc:=N` launch option (servo acceleration via a forward_command_controller) | `launch/real.launch.py`, `config/controllers.yaml`, `package.xml` |
| Optional per-joint servo gains in the calibration file (`p_coefficient`, `i_coefficient`, `d_coefficient`, `deadband`; written to EEPROM at start) + `run_preflight.sh --gains` readout | `real.launch.py`, `tools/fit_calibration.py`, `tools/sts_preflight.py` |
| Emulator: `--sag ID=COUNTS` (gravity-like holding error), `stats` shows ACC/speed | `tests/sts_emulator.py` |
| Calibration-tool tests use frozen copies of the original URDF/calibration, so they pass after you recalibrate | `tests/test_calibration_tools.py`, `tests/fixtures/` |
| Runbook: current state, `servo_acc`, step 8 monitoring/troubleshooting, open issues | `ros2_ws/REAL-ARM.md` |

## New / changed files (first V1 release)

| file | what |
| --- | --- |
| `ros2_ws/REAL-ARM.md` | bring-up runbook (commands, checks, what to log) |
| `ros2_ws/calibration/Calibration.csv` | your measurements (side table in columns G-M is ignored) |
| `ros2_ws/calibration/bringup-log.md` | fill-in log for the runbook |
| `ros2_ws/tools/fit_calibration.py` | CSV + URDF + datum → `sts_calibration.yaml`, with direction and encoder-wrap checks |
| `ros2_ws/tools/sts_preflight.py` | read-only servo check (IDs, raw, angle, limits, torque, voltage, temperature) |
| `ros2_ws/tools/sts_calibration_monitor.py` | live angles/raw counts while read-only (now uses the CSV zero until calibrated) |
| `ros2_ws/tools/sts_set_center.py` | `--shift N` option for the encoder-wrap case |
| `ros2_ws/src/joint_effort_iron/launch/real.launch.py` | provisional centers in read-only, `teleop:=false`, limit/wrap checks, Servo start-up fix |
| `ros2_ws/src/joint_effort_iron/launch/datum.launch.py`, `joint_effort_iron/datum_sliders.py` | RViz sliders to measure where your 0° marks are |
| `ros2_ws/src/joint_effort_iron/joint_effort_iron/jog_joint.py` | move one joint a few degrees through the trajectory controller |
| `ros2_ws/src/joint_effort_iron/urdf/iron.urdf`, `config/iron.srdf`, `config/model.json` | axis flips, home pose |
| `ros2_ws/src/joint_effort_iron/joint_effort_iron/bridge.py` | logs *why* it ignores joint states (e.g. joint outside limit) |
| `ros2_ws/src/joint_effort_input/config/input.yaml` | Pico device name and buttons |
| `ros2_ws/scripts/run_real.sh, run_datum.sh, run_jog.sh, run_preflight.sh, get_driver.sh, check_serial.sh` | entry points |
| `ros2_ws/tests/sts_emulator.py`, `tests/test_calibration_tools.py` | software STS servo bus + tests for the new tools |
| `ros2_ws/tests/test_iron.py`, `test_iron_bridge.py`, `tools/study_iron_workspace.py`, `reports/iron/*` | updated for the home pose / flipped axes |

## What has been tested (no physical arm)

- Clean build of all 9 packages (including the driver at `eeb0ad0`) on Jazzy, MoveIt 2.12.4;
  52 unit tests pass (also with the October 26 calibration and limits).
- `tools/check_iron_live.py` (V0's full mock integration check) passes on V1.
- Mock: stick/task +X/+Y/+Z move the tip along world +X/+Y/+Z.
- Real launch with the actual sts_hardware_interface talking to the emulated servo bus:
  read-only (torque stays off), powered hold without a jump, single-joint jogs accurate to
  0.01°, and the full Pico → forwarder → bridge → Servo → driver chain moving the tip in the
  commanded direction; mode press pauses.

Not tested: anything that depends on the physical arm - real directions, datum, loads, speed,
friction, and whether the modelled table/PCB/fixture match the bench.
