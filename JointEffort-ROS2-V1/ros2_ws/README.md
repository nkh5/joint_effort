# AMW arm workspace (ROS 2 Jazzy)

Teleoperated soldering-iron arm: Pico joystick → ROS 2 → MoveIt Servo → five Feetech STS3215 servos.
The same stack runs on simulated motors (`mock`) and on the real arm (`real`).

## Quick start

```bash
./amw build                 # first time: fetches + patches the motor driver, builds (see docs/setup.md)
./amw start mock            # simulated arm (RViz if you have a display)
./amw start real            # the real arm: motor check first, then joystick driving
./amw status                # second terminal: why it is or isn't moving, joints vs limits, motor health
```

Put `alias amw=$HOME/<path>/ros2_ws/amw` in `~/.bashrc` to type just `amw` (see docs/setup.md).
`amw <command> --help` explains every option.

| Command | What it does | ROS |
| --- | --- | --- |
| `amw build [--clean]` | fetch + patch the driver, `colcon build` | - |
| `amw start mock\|real` | everything: controllers, Servo, teleop, joystick, RViz | starts it |
| `amw start real --readonly` | joint states only, torque OFF | starts it |
| `amw start real --no-teleop` | controllers only (for `jog` / `track`) | starts it |
| `amw status [--once]` | one screen: ON/OFF, stop reason + fix, Servo, joints vs limits, motor temp/volt/amps, recent events | running |
| `amw stop [--estop]` | turn the arm OFF (`--estop`: torque off, arm goes limp) | running |
| `amw jog joint_2 +3` | move one joint 3 degrees and report | `--no-teleop` |
| `amw track joint_3` | recorded jog sequence with plots (in `results/`) | `--no-teleop` |
| `amw calibrate zero\|check\|limits` | set zeros / check directions / measure travel | **stopped** |
| `amw motors scan\|read\|gains\|tune\|punch\|set-id\|recenter` | servo bus tools | **stopped** |
| `amw joystick` | raw Pico input | running |
| `amw lcd [--fb\|--demo]` | status screen (window, or the Pi's LCD) | running |
| `amw test` | unit tests (no ROS or hardware) | - |
| `amw ros2 ...` | any `ros2` command on the arm's ROS domain (88) | - |

## Where things are

| Path | What |
| --- | --- |
| `src/amw_bringup/config/arm.yaml` | **motors, calibration and joint limits** - the only file you edit to calibrate |
| `src/amw_bringup/config/gains.yaml` | servo PID/deadband/punch (written only by `amw motors tune`) |
| `src/amw_bringup/config/teleop.yaml` | joystick mapping, speeds, safety thresholds |
| `src/amw_bringup/config/*.yaml` | controllers, MoveIt Servo, kinematics plugin |
| `src/amw_bringup/launch/amw.launch.py` | the one launch file (`amw start` runs it) |
| `src/amw_description/` | URDF (geometry, never edited for calibration), SRDF (home pose), meshes, RViz view |
| `src/amw_teleop/` | the teleop node: ON/OFF logic (`supervisor.py`), stop reasons (`faults.py`), five-joint solve (`kinematics.py`), LCD (`lcd.py`) |
| `src/amw_tools/` | the `amw` command and the servo bus library |
| `src/amw_moveit/` | MoveIt Servo runner with a clean shutdown |
| `driver/amw_driver.patch` | our one change to the pinned `sts_hardware_interface` driver |
| `tests/` | unit tests and the software servo emulator (`tests/sts_emulator.py`) |
| `archive/` | old calibration data and the pre-cleanup calibration file, for reference |
| `pi/amw-lcd.service` | starts the LCD at boot on the Pi |
| `docs/` | setup, calibration, troubleshooting (every stop reason), LCD, arm checks |

## How it fits together

```
Pico (USB gamepad, raw buttons) -> joy_node -> amw_teleop
     amw_teleop: ON/OFF + mode logic, safety checks, 5x5 solve (tip XYZ + iron pointing)
  -> MoveIt Servo (limits, collisions, smoothing) -> iron_arm_controller
  -> sts_hardware_interface -> Waveshare adapter -> STS3215 IDs 1-5
amw_teleop -> /amw/status -> amw status, LCD          (one place for every stop reason)
```

Using the controller: the arm starts **OFF**. Panel 1 turns it ON (or OFF again); it moves once the
stick is centred. The stick button switches translate/rotate; centre the stick to continue. Any
fault turns it OFF with the reason on the LCD and in `amw status` (see docs/troubleshooting.md).

Joint angle reported everywhere: `angle = dir * (raw - zero) * 360/4096` degrees, with `zero` and `dir`
per joint in `arm.yaml`. See [docs/calibration.md](docs/calibration.md).
