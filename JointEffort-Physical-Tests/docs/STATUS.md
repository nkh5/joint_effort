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

## Software checks

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
base-and-shoulder tests that can be run now.
