# Physical arm status — R02

The R02 software is ready for measurement and software testing. It is not yet calibrated to
this assembled arm. The default calibration file deliberately has empty centers and directions.
Powered startup rejects it.

Previous settings are saved in `calibration/sts_calibration_legacy_v1.yaml`. Earlier startup
instructions are available in Git history. Do not reuse those offsets with R02: the joint zero
positions and some joint directions have changed.

Read [R02-INTEGRATION.md](R02-INTEGRATION.md) for the model changes and completed mock tests.
The next physical steps are:

1. Measure raw encoder counts at known R02 angles on the assembled arm. Fill in
   `calibration/r02-measurements.csv`. Use the R02 link references, not the motor case surfaces.
2. Run `python3 tools/fit_r02_calibration.py` to inspect the fit. Add `--write` when the
   measurements are correct. This tool never opens a serial port or moves a motor.
3. Verify each motor ID and positive direction against the physical joint. Check actual
   clearance over the chosen travel. Measure the actual iron tip location.
4. Enter only completed checks in `physical_checks` in
   `src/joint_effort_iron/config/sts_calibration.yaml`. A successful fit does not complete these checks.
5. Compare real feedback with RViz before enabling output. After the measurements agree, test
   a hold, a small single-joint move, and then coordinated movement.

Important behavior retained from the STS driver:

- Powered activation seeds targets from measured joint positions. It does not drive to mock home.
- `readonly:=true` turns torque OFF. Support the arm before using that launch.
- Shutdown and emergency-stop handling disable torque; they are not a load-bearing brake.
- The new calibration does not write PID or deadband values. Earlier EEPROM settings remain.
- Raw 0/4095 wrap inside the requested travel is rejected. The fitting tool does not change
  servo IDs, encoder offsets or EEPROM settings.

Use one program at a time on the servo adapter. The software tests use ROS domain 188 and
GenericSystem mock motors; normal prototype startup remains on domain 88.
