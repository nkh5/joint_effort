# R02 arm URDF handoff — aligned calibration reference

The completed package is
[`ros2_ws/src/joint_effort_arm_calibration_description`](ros2_ws/src/joint_effort_arm_calibration_description).
It contains the updated holder and mock iron, six SI-meter STL meshes, five
revolute joints, a fixed iron-tip frame, and an RViz display launch for ROS2 Jazzy.

## Start the model

From the repository root, inside the team's Ubuntu/Jazzy environment:

```bash
cd JointEffort-ROS2-V1/ros2_ws
./scripts/with_ros.sh colcon build --symlink-install --base-paths src --packages-select joint_effort_arm_calibration_description
./scripts/with_ros.sh ros2 launch joint_effort_arm_calibration_description display.launch.py
```

All joint sliders start at zero. This display launch has no motor driver.
The current `run_real.sh` / MoveIt setup continues to use the existing model.
Start the display independently of an existing arm launch so model and
joint-state publishers do not compete in the same ROS domain.

## R02 zero pose

- Fixed base level, with +Z up, +X forward and +Y left.
- Rotating upper bracket aligned with the fixed motor-mount bracket.
- Upper link vertical, forearm horizontal forward, wrist centered as pictured,
  and iron pointing down.

![R02 reference pose](ros2_ws/src/joint_effort_arm_calibration_description/zero-pose.png)

The base correction is 30.3400722516624 degrees from the earlier captured pose.
The URDF's base visual and collision origins preserve that alignment. Keep those
origins when adapting the file. Source part coordinates were deliberately retained.

## Connect the existing bench measurements to this model

The motor/protractor data in `ros2_ws/calibration/bench-2026-10-31.csv` remains
useful. Its reference is the motor housing. R02 defines the assembled arm's zero.
Record a reference pose after installing the motors, accounting for horn
positioning, any encoder-offset changes, and which motor occupies each joint.

The correspondence below describes chain order; verify physical motor IDs and
direction signs on the assembled arm before configuring the driver.

| Existing stack name | R02 name | Positive axis at R02 zero, in base coordinates |
| --- | --- | --- |
| `joint_1` | `base_rotation_joint` | +Z |
| `joint_2` | `shoulder_joint` | +Y |
| `joint_3` | `elbow_joint` | +Y |
| `joint_4` | `wrist_yaw_joint` | -Z |
| `joint_5` | `wrist_pitch_joint` | -Y |

Place the arm in R02 zero, record the encoder count for each joint, confirm the
positive direction with a small known move, and check several nonzero measured
poses. Fill the package's `config/calibration.template.json`. Do not copy the old
`datum_offset_rad` or `center_steps` into R02 without transforming their reference.

The next ROS integration should update the joint names, axis/sign conventions,
calibrated centers, travel limits, SRDF/MoveIt setup, and controller joint lists
as one consistent change. The original bench data and current controller
configuration are preserved by this handoff.

## Scope and checks

The package represents the saved R02 CAD snapshot, including 104 active source
instances and 119 solids. It has not incorporated the newer `main` CAD files
reviewed at `57207ad`. Compare those parts geometrically before claiming that
the URDF represents the newest team assembly.

Saved/reopened native CAD placements and all source-instance transforms were
verified. Base bracket normals align numerically. The new base orientation
passed 120 fixed-to-moving external component pair checks; all five native
intersection candidates had zero overlap. Moving-to-moving geometry and the
earlier shoulder/elbow floor reliefs retain their prior checks. Internal servo
contacts, the small pre-existing within-base case/mount overlaps, physical fit,
and dynamic collision clearance remain outside that qualification.

URDF parsing, 31 independent kinematic configurations, mesh units/winding/bounds,
and CAD-to-URDF geometry placement checks passed. Portable numerical records are
in the package's `evidence` directory. The exact URDF and meshes match the
validated local R02 export; workstation-specific native reports remain local.

The +/-30-degree travel, 0.1 Nm effort and 0.25 rad/s velocity values are display
placeholders. Measure actual usable travel, verify cable clearances, and measure
the physical iron TCP. There are no dynamics inertials. ROS2 runtime and real
motor motion have not yet been tested for R02.
