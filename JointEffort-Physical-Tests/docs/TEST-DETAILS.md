# Motor test details

The current primary test runs `commission.py` directly with Python and pyserial.
The ROS `sts_commission` executable imports the same module after a ROS build.
Neither path needs a Pico for these tests.

## Reads and commands

Preparation reads both motors at 50 Hz. It checks ID, code 777, position mode,
phase 12, resolution 1, settings lock, current position, temperature, voltage and
motor status. It expects write acknowledgements. No motor writes are sent during
preparation. Each pair of feedback reads must finish within 15 ms. A loop delay
above 50 ms ends the test. Faulted serial reads latch; they are not retried.

Movement preloads the measured position before torque enable. The selected motor
gets RAM-only goal, acceleration, speed and torque-enable writes. The other motor
receives reads only. Goals increase by one count every 0.1 seconds. Native speed
is nonzero, nominally 10 counts/second; firmware may quantize low speeds.

An ordinary test stays inside +/-8 counts around the prepared positions. The
10-degree preparation permits only the selected motor/direction, with a bounded
116-count request. No encoder wrapping, EEPROM changes or automatic ID assignment
are part of movement tests. Sessions expire after 15 minutes.

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
regular test expects base 1 and shoulder 2; it will not silently renumber them.

## Offline code checks

```bash
./check_code.sh
```

These automated checks use pseudo-terminals and made-up calibration data, not
the physical adapter. They do not run a mock arm demonstration or move motors.
The native C++ protocol tests are also retained with the ROS source.
