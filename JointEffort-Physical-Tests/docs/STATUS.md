# Test status

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
