# Full-arm ROS hardware source

This optional path is for the calibrated five-joint arm. It is not required for
the two-motor tests in the main README. The source is included so the hardware
work can be continued from one folder. Full-arm physical acceptance is pending.

## Build on Ubuntu 24.04 and ROS 2 Jazzy

ROS 2 Jazzy and its Ubuntu package repository must already be configured.
Install the ROS dependencies on the computer that will run the driver:

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions python3-serial \
  python3-numpy python3-yaml python3-pyqt5 qtbase5-dev \
  ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-moveit-servo \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joy \
  ros-jazzy-ament-cmake-python ros-jazzy-ament-cmake-gtest
```

From this folder's `ros2_ws` directory:

```bash
chmod +x scripts/*.sh
./scripts/with_ros.sh env CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --base-paths src --parallel-workers 1
```

Build only this workspace's `src` directory. Do not add another `amw_pi_bridge`,
joy_teleop or teleop_tools_msgs copy. Do not copy another computer's install/build
folders. MoveIt lifetime fixes were developed against MoveIt 2.12.4; a different
version may need review. The wrappers load Jazzy and default to physical domain 89.

## Read native feedback after building

With only the identified base and shoulder connected:

```bash
./scripts/with_ros.sh ros2 run joint_effort_hardware sts_inspect \
  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00 1 2
```

This sends no motor-setting or movement writes. Stop other serial owners first.
The ROS entry point for the same two-motor test is `./scripts/run_motor_test.sh`.

## Calibration before full-arm control

Record each motor's ID, zero reading, direction, external ratio and safe travel
in `hardware/joint-measurements.csv`. Base and shoulder IDs are known. Their zero,
direction, ratio and full travel are not yet calibrated. The remaining three
motors still need individual identification. Do not connect unknown duplicate IDs.

Copy `src/joint_effort_hardware/config/iron_hardware.template.json` to a local
file such as `hardware/iron-measured.json`, and enter actual measurements.
Review the tool tip, base, table and obstacle geometry against the assembled arm.
The template's confirmation fields intentionally start false.

`tools/iron_calibration.py` can fit zero, direction and ratio from measured angle/
encoder pairs. Use `hardware/angle-measurements-template.csv`. Its fit is an
estimate for review; its +/-30-degree proposals are not measured travel limits.

Check the file without opening a serial port:

```bash
./scripts/with_ros.sh python3 tools/hardware_check.py /absolute/path/to/iron-measured.json
```

Verify the motor speed units and encoder settings on all five motors before
full-arm activation. The full driver assumes the count-per-second settings
already checked by the separate two-motor tool on the base and shoulder.
Account for the observed motor voltage-sensor offset when configuring monitoring;
the accepted sensor range does not increase the motor's supply rating.

## Start the measured system

Only after calibration and physical checks are complete:

```bash
./scripts/run_hardware.sh calibration:=/absolute/path/to/iron-measured.json rviz:=false
```

Hardware and trajectory controller start inactive; motor writes default off.
Changing `allow_motor_writes` does not itself activate the hardware. First
activation and full-arm acceptance still require review; no automatic activation
script is supplied. The five-joint path was tested with emulated motors, not as
a completed physical-arm system.

The hardware launch reuses model, kinematics, collision and shared launch code.
Some retained shared files have `mock` in their names because they came from the
earlier workspace. The physical launch replaces the mock hardware plugin before
starting ROS. The old mock demo shell scripts and movement tools are omitted.

## Input and stopping

`run_hardware_input.sh` needs the same calibration and `mapping_confirmed:=true`.
It also checks held enable, mode flags and a changing Pico heartbeat. The proposed
firmware is in `../ros2_ws/firmware`; read its guide before any build or flash.
It has not been compiled/flashed here and is not required for motor-only tests.

`tools/hardware_stop.py`, run through the wrapper on domain 89, requests a
software stop and hardware hold. It does not release torque and cannot replace
a physical power disconnect. The normal MoveIt Execute button stays disabled.
