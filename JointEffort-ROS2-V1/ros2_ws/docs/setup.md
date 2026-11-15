# Setup

Ubuntu 24.04 with ROS 2 Jazzy: the AMW Pi (Ubuntu Desktop) or the laptop's `ubuntu24` distrobox.

## 1. Packages (once)

ROS 2 Jazzy itself: follow the official Ubuntu (deb packages) install. On the Pi (Ubuntu Server)
`ros-jazzy-ros-base` is enough; on a laptop `ros-jazzy-desktop` adds RViz. Then:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-numpy python3-yaml python3-serial \
  python3-pygame python3-matplotlib fonts-dejavu-core \
  ros-jazzy-moveit-servo ros-jazzy-moveit-kinematics ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joy ros-jazzy-diagnostic-msgs
```

(`python3-pygame` is for the status screen, `python3-matplotlib` only for `amw track` plots.)

## 2. Serial and joystick access (once)

```bash
sudo usermod -aG dialout "$USER"      # in a distrobox: run this on the Fedora host instead
```

Log out and back in. Check with `groups` (must list `dialout`), `ls /dev/serial/by-id/` (the adapter
`usb-1a86_USB_Single_Serial_5B14028914-if00`) and `ls /dev/input/js*` (the Pico). If your adapter has
a different id, change `serial_port` in `src/amw_bringup/config/arm.yaml`.

## 3. The `amw` command

```bash
echo "alias amw=$PWD/amw" >> ~/.bashrc     # run this inside ros2_ws
source ~/.bashrc
```

`amw` sources ROS Jazzy and this workspace by itself, and runs on ROS domain **88** (the LCD service
uses the same). Your own shell settings (e.g. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in the distrobox) are kept. To use plain `ros2` commands
on the arm, either `amw ros2 topic list` or `export ROS_DOMAIN_ID=88` in that shell.

## 4. Build and test

```bash
amw build            # add --workers 1 if the machine runs out of memory
amw test
```

`amw build` clones `adityakamath/sts_hardware_interface` into `src/` (pinned commit `eeb0ad0`),
resets it, applies `driver/amw_driver.patch` and runs
`colcon build --symlink-install`. After switching branches or renaming packages, use
`amw build --clean` once so old packages do not linger in `install/`.

Config files under `src/amw_bringup/config/` are read from the source tree: edit, then restart
`amw start`. No rebuild needed. Python changes also need no rebuild (symlink install); C++ and new
files do.

## What the driver patch changes

`driver/amw_driver.patch` against the pinned driver:

- per-joint `encoder_direction` (+1/-1): the `dir` in arm.yaml
- per-joint `acceleration` (1..254): the servo ACC register used for every goal
- the reported position is no longer clamped to the limits (a sagging or out-of-range joint is visible)
- activation is refused if a joint is outside its range, instead of the first goal jumping to the limit

## Dry run without hardware

`tests/sts_emulator.py` pretends to be the five servos on a pseudo serial port, so the real-arm path
(driver, motor check, calibration tools) can be rehearsed on a laptop:

```bash
python3 tests/sts_emulator.py --link /tmp/fake_sts 1=2097 2=1959 3=3643 4=2024 5=1073   # home pose
# second terminal:
amw motors read --port /tmp/fake_sts
amw start real --port /tmp/fake_sts
```

Type `hand 3 3500` + Enter in the emulator terminal to move servo 3 "by hand" while its torque is
off. It checks directions and logic, not dynamics.
