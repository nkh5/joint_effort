# Joint Effort physical motor tests

Use this folder on the Linux computer connected to the Waveshare USB servo
adapter. The current test arm uses base ID 1 and shoulder ID 2 at 1,000,000 baud.
Keep the other three motors disconnected for these tests. Stop other programs
using the servo adapter. The Pico and an RViz window are not needed.

The base and shoulder have passed communication checks together. A small base
movement passed on the Pi. The 10-degree extension has passed automated tests,
but has not yet run on the motors. See [current status](docs/STATUS.md).

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

## 3. Prepare without movement

```bash
./motor_test.sh prepare
```

This sends reads only and checks both motors at 50 Hz for three seconds. Wait
for `"result": "PASS"`. It records a small window around the actual positions.
It does not set an arm zero or send the motors to encoder zero or 2048.

## 4. Test one motor at a time

Run each command separately. Watch the arm and check for `PASS` before continuing.
Four counts request about 0.35 degrees at the motor shaft:

```bash
./motor_test.sh step --motor base --counts 4 --execute
```

To go in the opposite direction:

```bash
./motor_test.sh step --motor base --counts -4 --execute
```

Then test the shoulder using `--motor shoulder` instead of `--motor base`.
The prepared window stays within +/-8 counts of each starting position.
Do not keep repeating commands to seek a larger movement. Stop on `FAIL` and
save the output. Record which physical direction increasing counts produces.

## 5. Optional 10-degree base test

This is a separate, larger test. Confirm the base has a clear path in the chosen
direction. The ordinary small-step limit is unchanged. Run preparation first:

```bash
./motor_test.sh prepare-turn --motor base --direction positive
```

After it passes, start the movement:

```bash
./motor_test.sh turn --motor base --execute
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

**Torque stays enabled after a movement.** Support the arm before removing servo
power. Closing the terminal does not release torque. The unselected motor's
settings are not changed, and it must remain supported if its torque is off.

The latest output is in `ros2_ws/reports/iron/two-motor-last-test.json`. Dated
copies are kept beside it. Preparation is stored in `ros2_ws/work` and expires
after 15 minutes. Prepare again after a power cycle or manual repositioning.
Generated sessions and logs are ignored by Git.

## What else is included

- [Test details](docs/TEST-DETAILS.md): limits, feedback checks and inspection.
- [ROS hardware guide](docs/ROS-HARDWARE.md): optional full-arm ROS build and launch.
- [Status](docs/STATUS.md): tested work and remaining measurements.
- [File list](docs/FILES.md): every source file and its purpose.
- [Third-party notices](THIRD_PARTY.md): source credits and license information.

The full five-joint ROS controller still needs actual calibration for all five
joints. These two-motor tests do not run joystick control or collision checking.
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
