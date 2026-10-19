# Motor test details

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
