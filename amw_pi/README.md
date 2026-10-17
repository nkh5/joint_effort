# Nick's amw_pi_bridge package

Copy this entire directory into the existing ROS workspace's `src/` directory:
`~/ros2_ws/src/amw_pi_bridge/`. Do not replace the other packages.

See the accompanying **Nick_Pi5_Integration_Guide.pdf** for complete deployment,
calibration, Jack/Kory coordination, and the remaining physical-control work.

## What it does

- `motor_feedback`: reads actual motor positions using the included existing
  `amw_motors.py` and publishes `/nick/joint_states_preview`.
- `trajectory_preview`: observes Kory's trajectory output and previews calibrated
  motor counts on `/nick/motor_targets_preview`; it does not write motors.
- `interface_demo`: demonstrates the conversion offline with a fictitious fixture.

This package is a feedback/preview integration stage, not a motor-motion driver.
Calibration is intentionally blank. Runtime requires Kory's installed
`joint_effort_iron` package to validate against its actual URDF.

## Local offline test (from this directory)

```bash
PYTHONPATH=. python3 -m unittest discover -s test -v
PYTHONPATH=. python3 -m amw_pi_bridge.demo
```

The 32 offline tests cover the contract and the existing commissioning library.
They require neither ROS nor Kory's source. ROS and physical hardware are untested.

## Provenance

`amw_pi_bridge/amw_motors.py` is an unchanged copy of your selected
`amw_motor_tools/amw_motors.py`. Other package files are the new integration code.
No Kory source package or third-party motor SDK is included. The FTServo SDK is
installed separately as described in the PDF. The ROS package license field
remains `LicenseRef-Unlicensed`; this handoff does not assign a distribution license.
