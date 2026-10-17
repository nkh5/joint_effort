# Setup

Use Ubuntu 24.04 with ROS 2 Jazzy installed and its package repository configured.

## Dependencies

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-yaml python3-matplotlib python3-pyqt5 \
  qtbase5-dev ros-jazzy-desktop ros-jazzy-moveit \
  ros-jazzy-moveit-servo ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joy ros-jazzy-joint-state-publisher-gui
```

The workspace includes joy_teleop and teleop_tools_msgs source. Do not add a second source copy.

The local MoveIt correction was tested with MoveIt 2.12.4. A later package version may need a new build or review.

## Build

Extract the bundle, then open its `ros2_ws` folder in a terminal:

```bash
chmod +x scripts/*.sh
./scripts/with_ros.sh colcon build --symlink-install --base-paths src
./scripts/run_iron.sh
```

For a computer with less memory, add `--parallel-workers 1` to the build command. Close older model/input launches before starting a new one.

The wrapper loads Jazzy and the local build. It also selects the ROS domain and local log folders. Use it for the commands in the guides.

## Input permissions

For USB serial diagnostics, the user normally needs the dialout group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after adding the group. USB joystick access and USB serial access are separate checks.

## Updating the model

Edit `tools/generate_iron.py` for lasting geometry/settings changes. Rebuild after generation. The generator resets the workspace study's collision results; run the workspace study and live checks again before using saved hover targets.

Rebuild locally after source changes. Build products from another computer are not included.
