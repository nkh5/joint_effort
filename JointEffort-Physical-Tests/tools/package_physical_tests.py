#!/usr/bin/env python3
"""Make the source folder for physical motor tests. Does not upload or move motors."""
import hashlib
import json
from pathlib import Path
import shutil
import stat
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'ros2_ws'
NAME = 'JointEffort-Physical-Tests'
DEST = ROOT/'releases'/NAME
SKIP = {'__pycache__', 'build', 'install', 'log', 'work', '.git'}


def copy_file(source, relative):
    target = DEST/relative
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.is_symlink(): raise RuntimeError('Review source symlink: '+str(source))
    shutil.copy2(source, target)


def write(relative, content, executable=False):
    target = DEST/relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content)
    target.chmod(0o755 if executable else 0o644)


def assemble():
    if DEST.exists(): raise RuntimeError('Destination already exists; preserve it before rebuilding')
    DEST.mkdir(parents=True)
    for folder in ['src', 'firmware', 'hardware']:
        for path in sorted((SOURCE/folder).rglob('*')):
            if path.is_file() and not set(path.relative_to(SOURCE).parts)&SKIP and path.suffix not in {'.pyc', '.log'}:
                copy_file(path, Path('ros2_ws')/path.relative_to(SOURCE))
    for name in ['with_ros.sh','start_clean.sh','run_hardware.sh','run_hardware_input.sh','run_motor_test.sh']:
        copy_file(SOURCE/'scripts'/name,Path('ros2_ws/scripts')/name)
    for name in ['hardware_check.py','hardware_stop.py','identify_motors.py','iron_calibration.py']:
        copy_file(SOURCE/'tools'/name,Path('ros2_ws/tools')/name)
    for name in ['test_hardware.py','test_motor_identification.py','test_commission.py','test_id_assignment.py']:
        copy_file(SOURCE/'tests'/name,Path('ros2_ws/tests')/name)
    for name in ['run_hardware.sh','run_hardware_input.sh','run_motor_test.sh']:
        (DEST/'ros2_ws/scripts'/name).chmod(0o755)
    copy_file(ROOT/'THIRD_PARTY.md','THIRD_PARTY.md')
    angles=ROOT/'iron_prototype/calibration/measurements-template.csv'
    if not angles.exists(): angles=SOURCE/'hardware/angle-measurements-template.csv'
    copy_file(angles,'ros2_ws/hardware/angle-measurements-template.csv')
    # Shared launch code loads this filename for the physical profile. Keep it.
    # The normal entry points in this folder start physical tests only.
    for name in ['with_ros.sh','start_clean.sh']:
        p=DEST/'ros2_ws/scripts'/name
        p.write_text(p.read_text().replace('JE_ROS_DOMAIN_ID:-87','JE_ROS_DOMAIN_ID:-89'))
        p.chmod(0o755)
    write('.gitignore','''**/build/
**/install/
**/log/
**/work/
**/__pycache__/
*.pyc
*.log
.DS_Store
ros2_ws/reports/
releases/
ros2_ws/hardware/*measured*.json
''')
    write('motor_test.sh','''#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 "$bundle_dir/scripts/motor_test.py" "$@"
''',True)
    write('scripts/motor_test.py','''#!/usr/bin/env python3
"""Run the physical motor test from this folder without building ROS."""
import os
from pathlib import Path
import sys
workspace = Path(__file__).resolve().parents[1]/'ros2_ws'
sys.path.insert(0, str(workspace/'src/joint_effort_hardware'))
os.chdir(workspace)
from joint_effort_hardware.commission import main
raise SystemExit(main())
''',True)
    write('inspect_motors.sh','''#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$bundle_dir/ros2_ws"
exec /usr/bin/python3 tools/identify_motors.py "$@"
''',True)
    write('setup.sh','''#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y python3 python3-serial
printf '%s\\n' 'Installed Python dependencies. Run ./check_setup.sh next.'
''',True)
    write('check_setup.sh','''#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 "$bundle_dir/scripts/check_setup.py"
''',True)
    write('scripts/check_setup.py','''#!/usr/bin/env python3
"""Check Python and list USB serial devices. Never opens a motor port."""
import os
from pathlib import Path
import sys
if sys.platform != 'linux':
    raise SystemExit('Use Linux on the Pi or Ubuntu computer connected to the servo adapter.')
try:
    import serial
except ImportError:
    raise SystemExit('Missing pyserial. Run ./setup.sh on Ubuntu.')
print('Python:', sys.version.split()[0], '| pyserial:', serial.__version__)
ports=sorted(Path('/dev/serial/by-id').glob('*'))
if not ports:
    print('No USB serial devices found. Connect the servo adapter and check USB passthrough if using a VM.')
for port in ports:
    access=os.access(port, os.R_OK | os.W_OK)
    print(port.name, '->', port.resolve(), '| readable/writable:', access)
print('This check did not open a port or send a command. See README.md for the motor test.')
''',True)
    write('check_code.sh','''#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir="$bundle_dir/ros2_ws"
export PYTHONPATH="$workspace_dir/src/joint_effort_hardware:$workspace_dir/src/joint_effort_input:$workspace_dir/src/amw_pi_bridge${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$workspace_dir/work/test-tmp"
export TMPDIR="$workspace_dir/work/test-tmp"
exec /usr/bin/python3 -m unittest discover -s "$workspace_dir/tests" -v
''',True)
    write('README.md',README)
    write('docs/ROS-HARDWARE.md',ROS_GUIDE)
    write('docs/STATUS.md',STATUS)
    write('docs/TEST-DETAILS.md',DETAILS)
    write('ros2_ws/README.md','''# Physical ROS source

Start with the README in the parent folder. The three-motor fixed-base tests can run
without ROS using `../motor_test.sh`. See `../docs/ROS-HARDWARE.md` for the full
five-joint driver, its dependencies and the calibration still required.

This source keeps shared model, kinematics and launch files from the earlier
workspace because the physical ROS path imports them. The old mock demo scripts
and mock test commands are not included in the main test workflow.
''')
    p=DEST/'THIRD_PARTY.md'
    p.write_text(p.read_text().replace('`ros2_ws/src/amw_pi_bridge` and `ros2_ws/hardware_reference` come from', '`ros2_ws/src/amw_pi_bridge` comes from'))
    # Replace stale imported quick-start instructions with accurate paths.
    write('ros2_ws/src/amw_pi_bridge/README.md','''# Team motor feedback and trajectory preview

Imported from the team repository at commit
`7e63e7a929292e6bf87458223642f0ec8ba5b048`.
The Python source is unchanged. It supplies calibration conversions, read-only
feedback and trajectory previews. Its `amw_motors.py` uses an external FTServo
SDK for its optional commands. The primary motor test in the top-level README
uses pyserial and does not need that SDK. Do not run competing serial owners.
There is one copy of this ROS package in this workspace.
''')
    copy_file(Path(__file__),'tools/package_physical_tests.py')


README='''# Joint Effort physical motor tests

Use this folder on the Linux computer connected to the Waveshare USB servo
adapter. Connect three motors with distinct IDs at 1,000,000 baud:

| Motor | ID | Test behavior |
| --- | --- | --- |
| Base | 1 | Hold its measured starting position with torque enabled. |
| Shoulder | 2 | May be selected for a movement test. |
| Elbow | 3 | May be selected for a movement test. |

Keep motors 4, 5 and 6 disconnected for these tests. Stop other programs using the
servo adapter. The Pico and an RViz window are not needed. Motor 3 must use the
same verified code-777, phase-12, position-mode and resolution-1 setup; this tool
checks those settings. The separate `assign-id` command changes the ID in EEPROM;
movement and preparation commands never change IDs.

Motor 1 is held when a movement command starts. Preparation remains read-only,
so support the arm during preparation. This three-motor version has passed
serial-emulator tests; the updated behavior still needs a physical check.
See [current status](docs/STATUS.md).

## 1. Download and set up

Download the team GitHub repository, then open this `JointEffort-Physical-Tests`
folder in a terminal. Keep the folder structure intact. On Ubuntu 24.04:

```bash
chmod +x ./*.sh
./setup.sh
./check_setup.sh
```

`setup.sh` installs Python 3 and pyserial. These motor tests do not require ROS
or a MoveIt build. They run the same commissioning module included in the ROS
package. The optional full ROS source is in `ros2_ws`.

If a serial device shows `readable/writable: False`, add your user to dialout:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in, then run `./check_setup.sh` again. Use a normal user account
for motor tests. On a virtual machine, attach the USB servo adapter to Ubuntu.
The Pico is a separate USB device.

## 2. Check the connection

The default port is the shared team's adapter:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00
```

It remains stable when Linux changes `ttyACM` numbers. If using a different
adapter, find its path in `./check_setup.sh` and pass `--port` before the command:

```bash
./motor_test.sh --port /dev/serial/by-id/YOUR_SERVO_ADAPTER prepare
```

Do not choose the AMW Controller/Pico port. Confirm the adapter's USB jumper is
in position B. Use the intended 12 V motor version; the team's supply measured
12.6 V. Keep the arm supported and the arranged power disconnect available.

## 3. Permanently assign IDs 1 through 6 (one motor at a time)

This saves the ID in motor EEPROM. It persists when servo power is removed.
The software does not guess which physical motor is the base, shoulder or elbow:
label each motor and connect only that one while assigning it.

1. Support/unload the motor, turn servo power off, and disconnect every other
   motor from both adapter sockets. Restore power to this one motor.
2. Find its current ID with `./inspect_motors.sh --scan --label isolated`.
3. Assign its role. The examples below assume its **current** ID is 1; replace
   `--old-id 1` with the ID actually found. Run only the command for the motor
   that is physically connected:

```bash
# ONLY the base connected:
./motor_test.sh assign-id --motor base --old-id 1 --single-motor-connected --execute
# ONLY the shoulder connected:
./motor_test.sh assign-id --motor shoulder --old-id 1 --single-motor-connected --execute
# ONLY the elbow connected:
./motor_test.sh assign-id --motor elbow --old-id 1 --single-motor-connected --execute
```

For motors you want to label 4, 5 or 6, use a numeric target. For example,
with only the motor being assigned ID 6 connected:

```bash
./motor_test.sh assign-id --old-id 1 --new-id 6 --single-motor-connected --execute
```

`--new-id` accepts 1 through 6. Use either `--motor` or `--new-id`, not both.
These extra IDs are for assignment only: the current movement test still
requires just IDs 1, 2 and 3, and keeps ID 1 stationary. Keep IDs 4, 5 and 6
disconnected from the current physical tests.

The command scans for other responding addresses, checks the verified motor
profile, unlocks EEPROM, writes the ID once, verifies the new address, and relocks
EEPROM. An already-correct ID is checked without rewriting EEPROM. No position
goals or torque-enable commands are sent. If torque is enabled, it refuses the
change unless you add `--disable-torque` while the motor is supported/unloaded.
That option turns torque off and leaves it off.

4. After PASS, turn servo power off and back on **while that motor stays isolated**.
   Then check the saved ID using the corresponding read-only command:

```bash
./motor_test.sh verify-id --motor base --single-motor-connected
# For the other isolated motors, use --motor shoulder or --motor elbow.
# To verify numeric ID 6 after its power cycle:
./motor_test.sh verify-id --id 6 --single-motor-connected
```

Repeat for each motor being assigned, then turn power off before reconnecting the three-motor
chain. A scan cannot distinguish multiple motors sharing the same ID, which is
why physical isolation is required even if only one address responds. If an ID
write fails, keep the motor isolated and inspect its old/new address before
trying again; the program attempts to relock but never retries or rolls back
the ID write automatically. Run a new `prepare` after reconnecting all three.

## 4. Prepare without movement

```bash
./motor_test.sh prepare
```

This sends reads only and checks all three motors at 50 Hz for three seconds. Wait
for `"result": "PASS"`. It records a small window around the actual positions.
It does not set an arm zero or send the motors to encoder zero or 2048.

## 5. Test one motor at a time

Run each command separately. Watch the arm and check for `PASS` before continuing.
Four counts request about 0.35 degrees at the motor shaft:

```bash
./motor_test.sh step --motor shoulder --counts 4 --execute
```

To go in the opposite direction:

```bash
./motor_test.sh step --motor shoulder --counts -4 --execute
```

Then test motor 3 using `--motor elbow` instead of `--motor shoulder`.
Motor 1 cannot be selected: `--motor base` is rejected.
The prepared window stays within +/-8 counts of each starting position.
Do not keep repeating commands to seek a larger movement. Stop on `FAIL` and
save the output. Record which physical direction increasing counts produces.

## 6. Optional 10-degree shoulder or elbow test

This is a separate, larger test. Confirm the selected motor has a clear path in the chosen
direction. The examples use shoulder ID 2; use `--motor elbow` for ID 3.
Motor 1 remains held. The ordinary small-step limit is unchanged. Run preparation first:

```bash
./motor_test.sh prepare-turn --motor shoulder --direction positive
```

After it passes, start the movement:

```bash
./motor_test.sh turn --motor shoulder --execute
```

This requests 116 counts (10.20 degrees) over about 13 seconds. `PASS` requires
at least 114 measured counts (10.02 degrees). It does not return automatically.
Use `--direction negative` during a new preparation to test the opposite way.
Degrees describe the motor shaft; external gearing can change the joint angle.
This extension is included for the next physical test; it is not yet physically
validated. It cannot be run using an old small-step session.

## Stopping and results

Ctrl+C stops new steps. With fresh, healthy feedback, the program requests the
latest measured position. After a serial failure it sends no more writes; the
motor may finish its last small goal. Use the physical power disconnect if needed.

**Motor 1 and the selected moving motor keep torque enabled after a movement.**
Motor 1's measured starting goal is written and verified before its torque is
enabled; its goal never becomes a movement target. If it drifts more than two
encoder counts (about 0.18 motor-shaft degrees) or loses hold torque, the test
stops further movement commands. This is a servo hold, not a mechanical lock.
The hold target is captured at the start of each test; movement of motor 1 by
more than two counts since preparation requires a new preparation.

Support the arm before removing servo power. Closing the terminal does not
release torque. The remaining unselected motor (2 or 3) receives reads only and
must remain supported if its torque is off. Monitoring runs during the test;
after it exits, the motors retain their last goals without software monitoring.

The latest output is in `ros2_ws/reports/iron/three-motor-last-test.json`. Dated
copies are kept beside it. Preparation is stored in `ros2_ws/work/three-motor-session.json` and expires
after 15 minutes. Prepare again after a power cycle or manual repositioning.
Old two-motor sessions are not accepted. Generated sessions and logs are ignored by Git.

## What else is included

- [Test details](docs/TEST-DETAILS.md): limits, feedback checks and inspection.
- [ROS hardware guide](docs/ROS-HARDWARE.md): optional full-arm ROS build and launch.
- [Status](docs/STATUS.md): tested work and remaining measurements.
- [File list](docs/FILES.md): every source file and its purpose.
- [Third-party notices](THIRD_PARTY.md): source credits and license information.

The full five-joint ROS controller still needs actual calibration for all five
joints. These three-motor tests do not run joystick control or collision checking.
Do not mark blank calibration fields confirmed to bypass those requirements.

## Put this on GitHub

Upload this whole folder as `JointEffort-Physical-Tests/` next to the existing
project folders. Keep the old V0 snapshot for reference. Teammates should use
this folder for physical tests. It contains one copy of each ROS package.
Do not add build/install/log/work folders, SSH keys or local test sessions.
The manifest lists source file checksums; build files are generated on each computer.
After copying this folder into a clone of the team repository, run these commands
from the repository root to record and upload it:

```bash
git add JointEffort-Physical-Tests
git diff --cached --stat
git commit -m "Add physical arm motor tests"
git push
```

Review the staged file list before committing. Upload the source folder; the ZIP
is provided as a download/transfer copy.
'''

ROS_GUIDE='''# Full-arm ROS hardware source

This optional path is for the calibrated five-joint arm. It is not required for
the three-motor tests in the main README. The source is included so the hardware
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
The ROS entry point for the same three-motor test is `./scripts/run_motor_test.sh`.

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
already checked by the separate commissioning tool (now IDs 1, 2 and 3, with ID 1 held).
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
'''

STATUS='''# Test status

Prepared on October 18, 2026 from the current Ubuntu workspace.

## Observed on physical hardware

- Waveshare USB servo adapter; base and shoulder on the same serial chain.
- Base ID 1 and shoulder ID 2. Shoulder changed from duplicate ID 1 and retained
  ID 2 after a power cycle. Both reported code 777 and position mode.
- Both checked at 1,000,000 baud, phase 12 and encoder resolution 1.
- Supply checked by the team with a multimeter: 12.6 V. Purchase record describes
  a 12 V motor variant. An earlier 4--7.4 V datasheet describes a different variant.
- 150 read-only samples at about 50 Hz. Slowest pair of reads: about 5.6 ms.
- A small base test returned PASS. One saved result showed two counts of actual
  movement for a four-count target change, within the test's final tolerance.

## Three-motor update (October 18, 2026)

- The primary Python/ROS commissioning entry point now requires IDs 1, 2 and 3.
- Motor 1 holds its starting measured position; only shoulder 2 and elbow 3 can
  receive movement tests. Both small steps and the 10-degree extension enforce it.
- A held-base drift or torque loss stops subsequent movement goals. Preparation
  remains read-only. New three-motor sessions are required.
- The updated folder passed 57 focused Python checks on Linux using pseudo-terminal
  motor replies, including 30 commissioning and 13 EEPROM-ID checks. No physical motors were used.
- Permanent EEPROM ID assignment now supports numeric IDs 1..6, with read-only
  post-power-cycle verification. Motion testing remains limited to IDs 1..3.
  ID writes, missing acknowledgements and failed-write relocking are emulator-tested.
- Third-motor identity, three-motor feedback timing and hold behavior under load
  remain physically unvalidated. Earlier physical results below concern the
  original two-motor version and do not establish this update's bench results.

## Earlier release software checks (before this update)

- The current full workspace passed 75 Python tests locally.
- The 16 original commissioning checks passed on the Pi before the 10-degree
  extension was added. The current 20 commissioning checks passed locally.
- The assembled folder passed 34 focused Python checks, 12 native protocol
  checks, and a clean build of all 10 ROS packages on Ubuntu/Jazzy.
- The package's focused check includes commissioning, identification and physical
  configuration tests. They use local temporary devices and do not move motors.

## Pending

- This assembled folder has not yet been installed on the Pi. The Pi was off.
- The 10-degree extension has not run on physical motors.
- Shoulder movement has not been independently confirmed here.
- Full five-joint ROS motion, zero/direction/ratio/travel calibration, physical
  collision-model validation, tip accuracy and repeatability remain pending.
- Proposed Pico heartbeat firmware has not been compiled or flashed here.

Automated checks do not establish physical soldering accuracy or collision-free
motion. Full-arm calibration remains blank. The main README provides the limited
three-motor fixed-base test commands.
'''

DETAILS='''# Motor test details

The current primary test runs `commission.py` directly with Python and pyserial.
The ROS `sts_commission` executable imports the same module after a ROS build.
Neither path needs a Pico for these tests.

## Reads and commands

Preparation reads all three motors at 50 Hz. It checks ID, code 777, position mode,
phase 12, resolution 1, settings lock, current position, temperature, voltage and
motor status. It expects write acknowledgements. No motor writes are sent during
preparation. Each complete set of three feedback reads must finish within 15 ms. A loop delay
above 50 ms ends the test. Faulted serial reads latch; they are not retried.

Movement first latches motor 1's measured starting position, writes that exact
RAM goal, verifies the goal readback, and enables hold torque if needed. It then
preloads and enables the selected shoulder (2) or elbow (3) at its measured pose.
Motor 1's goal stays fixed. Moving it with `--motor base` is forbidden, including
for the 10-degree extension. Drift beyond two encoder counts or loss of its
hold torque stops the selected movement. No hold-goal retargeting follows drift.

The remaining unselected motor (2 or 3) receives reads only. Its position and
torque state must remain unchanged within the existing two-count tolerance.
Motor 1 and the selected motor receive RAM-only goal, acceleration, speed and
torque-enable writes. Goals increase by one count every 0.1 seconds. Native speed
is nonzero, nominally 10 counts/second; firmware may quantize low speeds.

An ordinary test stays inside +/-8 counts around the prepared positions. The
10-degree preparation permits only the selected motor/direction, with a bounded
116-count request. No encoder wrapping or EEPROM changes are part of movement tests.
Permanent assignment is available only through the separate `assign-id` command. Sessions expire after 15 minutes. The three-motor profile and all three saved
IDs are required; old two-motor sessions are rejected.

The team's meter showed 12.6 V while internal sensors reported 12.6--12.8 V.
The test accepts reported sensor values up to 12.9 V and stops if they change by
more than about 0.2 V from preparation. This accounts for the observed sensor
difference; it does not authorize a higher physical supply voltage.

## Read-only identification

To scan the adapter and save raw replies:

```bash
./inspect_motors.sh --port /dev/serial/by-id/YOUR_SERVO_ADAPTER --scan --label chain
```

This does not change motor IDs or enable torque. A scan cannot reliably separate
two motors with the same address. If IDs are unknown, support the arm, disconnect
servo/USB power, and connect one motor at a time before identifying it. The
regular test expects base 1, shoulder 2 and elbow 3; it will not silently renumber them.

## Permanent ID assignment

`motor_test.sh assign-id` uses the bundled `id_assignment.py` and pyserial; it
requires no external servo SDK. It writes only ID register 5 and the EEPROM
unlock/relock register 55, plus torque register 40 = 0 only with the explicit
`--disable-torque` option. Firmware may acknowledge the ID write at its old ID,
new ID or not at all: the new-address readback is required. The ID write is sent
once. Unexpected/malformed replies fail the operation, and the program makes a
best-effort addressed relock at the old/new ID without changing the ID again.

Permanent numeric ID targets are 1 through 6. Base 1, shoulder 2 and elbow 3
are role aliases; only these three IDs participate in the movement test. Physical single-motor
isolation is required: a scan cannot prove that duplicate IDs are absent.
`verify-id` is read-only and should be run after the operator power-cycles the
isolated motor. The software cannot establish that power was actually cycled.

Register-map reference: the official [FTServo Python STS implementation](https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py)
identifies ID 5 as EEPROM and lock 55 as the EEPROM lock control. No firmware
implementation is copied into this new module.

## Offline code checks

```bash
./check_code.sh
```

These automated checks use pseudo-terminals and made-up calibration data, not
the physical adapter. They do not run a mock arm demonstration or move motors.
The native C++ protocol tests are also retained with the ROS source.

The accelerated ten-degree emulator checks relax only their test read budget
to tolerate host scheduling. A separate deterministic regression enforces the
production 15 ms three-motor feedback budget; the actual tool retains that limit.
'''


def inventory():
    paths=[p for p in sorted(DEST.rglob('*')) if p.is_file() and not set(p.relative_to(DEST).parts)&SKIP]
    paths=[p for p in paths if p.name not in {'MANIFEST.json','FILES.md'} and p.suffix!='.pyc']
    lines=['# Files in this folder','','Paths are relative to `JointEffort-Physical-Tests`.','', '| File | Purpose |','| --- | --- |']
    for p in paths:
        name=p.relative_to(DEST).as_posix()
        lines.append('| `'+name+'` | '+purpose(name)+' |')
    lines += ['| `docs/FILES.md` | This file list. |','| `MANIFEST.json` | Source sizes and SHA-256 checksums. |','']
    write('docs/FILES.md','\n'.join(lines))


def purpose(name):
    base=Path(name).name
    special={'VERIFICATION.json':'Record packaging checks and physical validation limits.','motor_test.sh':'Run the physical motor test with Python.','inspect_motors.sh':'Read motor addresses and raw replies.','setup.sh':'Install Python dependencies on Ubuntu.','check_setup.sh':'Check Python and list adapter permissions.','check_code.sh':'Run offline code checks without opening the adapter.','motor_test.py':'Locate the bundled commissioning module.','check_setup.py':'Inspect Python and USB device paths without serial commands.','commission.py':'Hold base 1 and test shoulder 2 or elbow 3.', 'id_assignment.py':'Assign and verify one isolated motor ID in EEPROM.','configuration.py':'Validate calibration and generate the physical robot description.','sts_bus.cpp':'Encode motor packets, verify replies and convert angles/counts.','sts_bus.hpp':'Declare serial and joint-conversion interfaces.','iron_system.cpp':'Connect five physical joints to ros2_control.','sts_inspect.cpp':'Read native motor identity and status.','hardware.xml':'Register the physical hardware plugin.','hardware.launch.py':'Start the inactive physical ROS system.','run_hardware.sh':'Start physical ROS on domain 89.','run_hardware_input.sh':'Start the physical input path.','run_motor_test.sh':'Run commissioning through the ROS entry point.','hardware_check.py':'Check calibration without serial access.','hardware_stop.py':'Request a software hold without torque release.','identify_motors.py':'Read addresses and capture diagnostic replies.','iron_calibration.py':'Fit measured angle/count samples offline.','package_physical_tests.py':'Record the assembly recipe used for this folder.'}
    if base in special:return special[base]
    if name.startswith('ros2_ws/firmware/'):return 'Proposed, unflashed Pico firmware or its instructions/license.'
    if name.startswith('ros2_ws/hardware/'):return 'Physical measurement worksheet; blanks still require measurement.'
    if name.startswith('ros2_ws/tests/') or base.startswith('test_'):return 'Automated hardware-code check; does not use real motors.'
    if '/vendor/' in name:return 'Vendored joystick mapping/message source or retained upstream notice.'
    if '/src/amw_pi_bridge/' in name:return 'Team read-only feedback, conversion or trajectory-preview dependency.'
    if base=='CMakeLists.txt' or base in {'setup.py','setup.cfg','package.xml'}:return 'Build/install rules or package dependencies.'
    if base=='__init__.py' or '/resource/' in name:return 'Python/ROS package registration.'
    if base=='COLCON_IGNORE':return 'Keep this reference folder out of ROS package discovery.'
    if base.endswith('.stl'):return 'CAD visual mesh or simplified collision geometry.'
    if base.endswith('.urdf'):return 'Shared robot geometry; physical launch replaces the hardware section.'
    if '/src/joint_effort_iron/' in name:return 'Shared iron-arm model, limits, Servo bridge or launch dependency.'
    if '/src/joint_effort_input/' in name:return 'Joystick normalization, permission, heartbeat or status dependency.'
    if '/src/joint_effort_robot/' in name:return 'Shared kinematics and robot-description dependency.'
    if '/src/joint_effort_moveit/' in name:return 'MoveIt lifecycle, Servo or collision-checking dependency.'
    if '/src/joint_effort_cad_review/' in name or '/src/joint_effort_arm_draft_description/' in name:return 'Shared CAD model or visualization dependency.'
    if base in {'with_ros.sh','start_clean.sh'}:return 'Load a clean Jazzy environment and this workspace.'
    if base=='.gitignore':return 'Exclude local build files, sessions, logs and caches.'
    return 'Setup, operating instructions, status or source credits.'


def freeze():
    inventory()
    files={}
    for p in sorted(DEST.rglob('*')):
        if not p.is_file():continue
        relative=p.relative_to(DEST)
        if set(relative.parts)&SKIP or p.suffix in {'.pyc','.log'}:continue
        if relative.parts[:2]==('ros2_ws','reports'):continue
        if p.name=='MANIFEST.json':continue
        data=p.read_bytes();files[relative.as_posix()]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'executable':bool(p.stat().st_mode&stat.S_IXUSR)}
    write('MANIFEST.json',json.dumps({'name':NAME,'scope':'Three-motor fixed-base tests and optional full-arm ROS source; calibration pending','files':files},indent=2)+'\n')
    archive=DEST.with_suffix('.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for name in sorted([*files,'MANIFEST.json']):
            p=DEST/name;info=zipfile.ZipInfo(NAME+'/'+name);info.create_system=3
            info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=(0o100000 | (0o755 if p.stat().st_mode&stat.S_IXUSR else 0o644))<<16
            z.writestr(info,p.read_bytes())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        assert len(z.namelist())==len(files)+1
    archive.with_suffix('.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
    print(f'{DEST}\n{archive}\n{len(files)+1} source files; {archive.stat().st_size} ZIP bytes')


if __name__=='__main__':
    assemble()
    freeze()
