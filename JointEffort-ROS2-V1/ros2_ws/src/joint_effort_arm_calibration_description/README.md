# Joint Effort arm calibration description — R02

ROS2 Jazzy description of the five-servo arm with the revised holder and mock
soldering iron. All five joint values equal zero in the pictured reference pose:
base brackets aligned, upper link vertical, forearm horizontal forward, iron down.
+X is fixed-base forward, +Y left, +Z up. Geometry uses **meters**.

![R02 zero pose](zero-pose.png)

R02 corrects the source assembly's retained 30.340072-degree base clocking. Both
the base visual and collision origins contain the required rotation; keep them.
The reference CAD is a static six-part assembly. This URDF supplies five revolute
joints and a fixed `iron_tip` frame.

## Display in the team's Jazzy workspace

From the repository root, in the team's Ubuntu/Jazzy environment:

```bash
cd JointEffort-ROS2-V1/ros2_ws
./scripts/with_ros.sh colcon build --symlink-install --base-paths src --packages-select joint_effort_arm_calibration_description
./scripts/with_ros.sh ros2 launch joint_effort_arm_calibration_description display.launch.py
```

For a standard Jazzy installation outside the team's wrapper:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select joint_effort_arm_calibration_description
source install/setup.bash
ros2 launch joint_effort_arm_calibration_description display.launch.py
```

The launch starts robot_state_publisher, joint-state sliders at zero, and RViz.
It does not connect to motors. Keep the entire package, including its six meshes.

## Hardware calibration and integration

Read [the R02 handoff](../../../R02-URDF-HANDOFF.md) and
[CALIBRATION-ACCURACY.md](CALIBRATION-ACCURACY.md) before integrating with the
existing controller stack. This additive package does not replace `iron.urdf`,
the MoveIt configuration, or the existing calibrated driver settings.

The R02 names are `base_rotation_joint`, `shoulder_joint`, `elbow_joint`,
`wrist_yaw_joint`, and `wrist_pitch_joint`. The existing stack uses `joint_1`
through `joint_5`; joint names, axes, physical motor IDs, direction signs,
encoder zeros, SRDF and controller mappings need reconciliation together.

Use the existing bench measurements as raw motor data. Their housing-based
reference and the existing YAML's older arm datums do not automatically equal
the R02 assembled zero. Place the assembled arm in this pose, record each zero
count, confirm direction with a small controlled move, and verify additional
measured poses. Fill `config/calibration.template.json`; fields remain null until
measured. Its nominal direct-drive formula uses 4096 encoder counts/revolution.

The +/-30-degree limits, 0.1 Nm effort and 0.25 rad/s velocity in this URDF are
**display placeholders**. Establish real travel and cable clearances before
motor operation. No inertial parameters are supplied for dynamics simulation.
The nominal iron TCP uses the CAD mock's photo-derived reference and still needs
physical measurement.

## Geometry and validation scope

This handoff preserves the validated R02 CAD snapshot. The later `main` CAD file
changes reviewed at `57207ad` have not been compared geometrically or merged into
it. Do not infer that this package represents those newer files.

The CAD includes the earlier proposed shallow shoulder/elbow cradle-floor
reliefs. Confirm printed motor seating and screw engagement physically. Small
pre-existing case/mount overlaps inside the fixed base link remain unqualified.

Saved native CAD placements, all 104 source-instance transforms, base alignment,
mesh units/winding/bounds, and 31 independent kinematic configurations passed.
The new base-angle check covers 120 fixed-to-moving external component pairs;
all five native intersection candidates had zero overlap. Moving-to-moving
geometry is unchanged from the previously checked R01 reference. Internal servo
contacts and dynamic motion clearance are outside that static check.

See `evidence/calibration-checks.json`, `evidence/provenance.json`,
`evidence/zero-convention.json`, and `evidence/zero-datums.csv` for the numerical
records. These establish CAD/software consistency, not physical calibration
accuracy. ROS2 runtime and real motor motion have not been tested for R02.
