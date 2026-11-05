# R02 software checks

These results describe the R02 integration on Ubuntu 24.04 with ROS 2 Jazzy. No physical motor commands
were sent. Test traffic used isolated ROS domain 188 and GenericSystem mock hardware.

- All 10 ROS packages built, including the patched STS driver.
- 57 project Python tests passed.
- 33 upstream conversion tests and 113 upstream mock hardware tests passed.
- The added C++ direction test checked 6,002 raw-count/angle round trips, both directions,
  velocity signs and endpoint clamps.
- `live-check.json`: five-joint jogging, TF, collision sampling, obstacle halt, planned
  motion, synthetic joystick translation/pointing, input loss, feedback loss, software stop
  and clean shutdown.
- `rviz-check.json`: model and TF displays loaded and RViz shut down cleanly.
- `demo-command-check.json`: the documented status, joint, tip and joystick demo commands.
- `collision-hulls.json`: hashes and vertex-containment checks for the six R02 envelopes.
- `collision-samples.json`: the offline 201-pose FCL sample. It does not include the live
  243-pose grid. The live grid found six tool/table contacts, saved in `live-check.json`.
- `prototype-workspace.png`: zero-pose view of the collision envelopes and example work area.

A zero pose is a measurement reference. The mock starts slightly bent because exact R02 zero
is singular for the five-component tip task. Neither pose is a physical torque test.

The table, fixture, tool offset and travel bounds need physical checks. Samples and an
obstacle-stop check do not certify every trajectory or guarantee physical clearance.
Earlier reports outside this folder are for earlier models.

Raw test logs and machine-specific build output stay local and are ignored by Git.
`verification.json` records the tested revision, package/test counts and source fingerprints.

## Physical measurement fit

`calibration-fit.json` records the measurements pulled at `6616549`. All five joint fits
exceed the default 0.5-degree maximum error, so the active calibration was left unchanged.
The separate Motor table is retained as reference data and is not used for R02 zeros.
The calibration CLI repair passed 59 project tests, including accepted/rejected file-write
checks. No physical motor commands were sent.
