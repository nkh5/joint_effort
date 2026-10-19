# Joint Effort physical motor tests

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
