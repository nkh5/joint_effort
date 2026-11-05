# R02 arm CAD — aligned zero pose

## Open the model

Open **JE-Arm-Calibration-Zero-R02.SLDASM** in SOLIDWORKS 2026.
Keep all six `R02-*.SLDPRT` files in this same folder. The holder and mock iron
are included. Download or clone the CAD through Git LFS to obtain the actual files.

This is the static six-link calibration reference corresponding to the R02 URDF
in `JointEffort-ROS2-V1/ros2_ws/src/joint_effort_arm_calibration_description`.
Its links have fixed placements; the matching URDF supplies the five moving joints.

## R02 zero pose

- Fixed base level: +Z up, +X forward, +Y left.
- Rotating upper bracket aligned with the fixed motor-mount bracket.
- Upper link vertical and forearm horizontal forward.
- Wrist in the reference orientation and iron pointing down.
