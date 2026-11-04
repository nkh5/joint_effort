# R02 model and software integration

## What is complete

The running iron-arm model now uses R02. The controllers, MoveIt Servo, joystick forwarding,
calibration tools and collision model use that geometry. See `reports/r02/` for the checks.
These are software results. They do not establish real motor torque, smoothness or tip accuracy.

## How the parts fit together

The controller sends a direction and speed. ROS turns that request into five joint movements.
RViz displays the resulting model; RViz itself does not control the motors.

```text
Pico (or synthetic joystick for tests)
    → joy / Joy messages
    → pico_forwarder + joy_teleop
    → /iron/task_velocity (tip translation or pointing velocity)
    → iron_task_bridge (five-joint motion math)
    → MoveIt Servo (limits, collision checks, smoothing)
    → iron_arm_controller (joint position targets)
    → GenericSystem for mock tests
      OR the STS driver for the physical arm after calibration
    → /joint_states feedback → TF, RViz and the motion checks
```

URDF describes the links and joints. TF is the set of coordinate frames: it tells ROS where
those links are now. SRDF defines the arm group and which collision pairs to check.
A controller is the program passing targets to the selected mock or real hardware interface.

## Geometry and zero angles

The untouched R02 source is:
`src/joint_effort_arm_calibration_description/urdf/joint_effort_arm_calibration.urdf`.
The active model used by both mock and real launch is:
`src/joint_effort_iron/urdf/iron.urdf`.

`tools/generate_iron.py` builds the active model from R02, adds the table/work area, collision
envelopes and mock interfaces, and updates `model.json` and `iron.srdf`. It does not rewrite
controller tuning or physical calibration. Model files need a rebuild and restart after editing.
Regenerating overwrites manual edits to the generated URDF, SRDF and collision meshes.

R02 zero means: upper arm upright, forearm forward, iron down. Base X is forward, Y is left,
and Z is up. The existing controller names are kept:

| Controller name | R02 name | Positive movement at zero |
| --- | --- | --- |
| joint_1 | base_rotation_joint | about upward Z; forward toward left |
| joint_2 | shoulder_joint | about Y; upper arm forward |
| joint_3 | elbow_joint | about Y; forearm downward |
| joint_4 | wrist_yaw_joint | about downward Z; wrist forward toward right |
| joint_5 | wrist_pitch_joint | about negative Y; iron down toward forward |

The base mesh's 30.340072-degree clocking correction is preserved in both its visual and
collision origins. World and base axes now face the same direction. The base mounting origin
remains 31 mm above the provisional tabletop; measure that on the real setup later.

The mock starts at `[0, -0.20, 0.20, 0, 0.15]` radians, about
`[0, -11.46, 11.46, 0, 8.59]` degrees. This is a software demonstration pose, not a torque-approved
physical home. At exact R02 zero, some joints have the same effect on tip pointing. The motion
math cannot independently produce all five requested directions there. The slightly bent mock
pose avoids that singularity. A physical zero reference does not have to be an operating pose.

The real startup still holds measured current positions. It never automatically copies the
mock starting pose to the arm.

## Why the tip frame needed a change

The motion solver treats local X as the iron's shaft direction. R02's iron-tip frame uses Z.
A fixed frame rotation now connects the two. It changes the coordinate axes, not the CAD tip
position. The nominal CAD tip still needs comparison with the actual installed iron.

A tip pose has six possible motions: three translations and three rotations. This arm has five
joints, so the task uses XYZ plus two directions of pointing. Rotation around the iron's own
shaft is left free. The Jacobian describes how each joint changes tip position and pointing.
Its damped inverse converts a requested tip velocity into joint velocities, with guards near
joint limits and singularities.

## Joystick changes

Translation still requests world X/Y/Z velocity. Rotation mode now uses world X/Y tilt;
vertical rotation would mainly ask a downward iron to spin around its shaft. The two modes
still use the same speed control and enable checks.

The synthetic Pico input now reads the configured button indices from `input.yaml`. It had
been sending an older seven-button layout. Current settings use button 7 for the Pico's virtual
enable state and button 0 for mode. The synthetic test represents those reports; it does not
prove the physical wiring or flashed firmware mapping.

The software requires centered input with enable released before enabling. Releasing enable,
losing input, a feedback fault, or a software stop prevents automatic resumption. On the panel,
"enable" is the virtual state supplied by the Pico; it is not necessarily a button held down.

## Collision model

Each of the six CAD links has a compact convex envelope: a solid shape enclosing its CAD
vertices. This reduces collision-checking cost. It fills holes and gaps, so it can reject a
pose that the detailed parts might physically clear. It does not prove clearance at adjacent
joint interfaces; adjacent pairs are exempt because their parts intentionally meet.

Moving links are checked against the table, example fixture and board. Their positions are
still provisional. The 800 by 1000 mm tabletop has its top at world Z = 0; the sample board
is centered at X = 430 mm, Y = 40 mm, Z = 151 mm.

The source +/-30-degree bounds are still software test bounds, not measured travel limits.
The collision check tested 243 combinations of lower/middle/upper joint positions. Six were
rejected for tool-to-table contact; the others were clear. Another 201 deterministic samples
were clear. An added block at the tip was detected, and Servo halted without moving through it.
These are sampled checks, not proof of every path or every physical obstacle.

## Calibration and driver changes

Old settings are saved in `calibration/sts_calibration_legacy_v1.yaml`. The new
`src/joint_effort_iron/config/sts_calibration.yaml` starts incomplete so old offsets cannot
silently be used with R02.

The new worksheet is `calibration/r02-measurements.csv`. Its angles must be measured from
R02 zero, in R02 positive directions. Use at least three measurements per joint, including
zero and measurements on both sides. Record more points to check consistency.

For a direct-drive 4096-count encoder:

```text
one count = 360 / 4096 = 0.08789 degrees
K = 4096 / (2*pi) = about 651.9 counts per radian
q = direction * (raw_count - zero_count) / K
raw_target = zero_count + direction * q * K
```

`direction` is +1 or -1, determined by measurements. The fitter checks slope and fit error.
It unwraps nearby samples for fitting but rejects powered travel that crosses the raw encoder
boundary. It does not automatically move a motor or reprogram its encoder offset.

The calibration stores the active URDF's hash, which is a fingerprint of that exact file.
Powered launch checks that fingerprint and the physical verification flags. Editing the model
requires reviewing/refitting the calibration. Controller-only testing can precede TCP approval;
full tip teleoperation also requires the physical TCP check.

The pinned STS driver gets the local `patches/sts-r02.patch` through `scripts/get_driver.sh`:

- Position commands and feedback use the measured encoder direction. Velocity feedback uses
  the same direction. The default remains the upstream driver's original convention.
- Measured position is not clipped to a software limit, which would hide a sagging joint.
- Activation refuses a position outside the configured range rather than silently moving it
  toward a limit. Valid startup targets still come from the measured pose.

The patch was built and tested in software. Its new direction handling still needs a small
physical direction check. It does not change EEPROM PID values. Mock results cannot tell us
whether real joystick jitter needs timing, acceleration, mechanical or PID changes.

## Main files

| File or folder | Purpose |
| --- | --- |
| src/joint_effort_arm_calibration_description | R02 source geometry and measurement references |
| src/joint_effort_iron/urdf/iron.urdf | Active model for mock and real launch |
| src/joint_effort_iron/meshes | Compact R02 collision envelopes |
| src/joint_effort_iron/config/iron.srdf | Arm group, mock pose and collision exceptions |
| src/joint_effort_iron/config/model.json | Motion-math settings and geometry metadata |
| src/joint_effort_iron/config/controllers.yaml | 50 Hz control loop and position controller |
| src/joint_effort_iron/config/servo.yaml | Servo timing, smoothing and collision settings |
| src/joint_effort_iron/joint_effort_iron/bridge.py | Tip velocity to five-joint commands |
| src/joint_effort_robot/joint_effort_robot/kinematics.py | Forward kinematics and Jacobian math; reused |
| src/joint_effort_input | Input normalization, enable checks, speed scaling and test joystick |
| src/joint_effort_iron/joint_effort_iron/calibration.py | Shared R02 calibration validation |
| tools/sts_calibration_monitor.py | Reconstruct raw counts using the selected encoder direction |
| tools/fit_r02_calibration.py | Fit R02 angles and raw counts without motor access |
| patches/sts-r02.patch | Direction and feedback changes to the pinned STS driver |
| scripts/get_driver.sh | Fetch the pinned driver and apply its patch |
| scripts/run_real.sh | Preflight and launch use the same selected calibration file |
| scripts/start_clean.sh | Load Jazzy and this local overlay without an old workspace chain |
| tests/test_r02.py, tests/r02_driver_conversions.cpp | R02 direction, fit and model-binding regressions |
| tools/check_r02_live.py | Mock motion, planning, input-loss and collision tests |
| tools/check_r02_commands.py | Run the documented demo commands on a mock |
| tools/check_r02_rviz.py | RViz model/TF loading and clean shutdown check |
| reports/r02 | Results for this integration; old reports are not R02 evidence |

## Run the isolated mock demo

Use two terminals in this `ros2_ws` folder. Domain 188 separates the demo from the normal
prototype domain 88. No serial hardware driver starts in these commands.

Terminal 1 — RViz and mock arm:

```bash
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  ros2 launch joint_effort_iron mock.launch.py
```

Terminal 2 — a visible joint move, then a tip move:

```bash
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  python3 tools/iron_demo.py joint --joint joint_1 --speed 0.05 --seconds 3
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  python3 tools/iron_demo.py up --seconds 3
```

To test the synthetic joystick, keep Terminal 1 running, start the following in Terminal 2,
and run the second command in Terminal 3. Stop any other motion tool first.

```bash
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  ros2 launch joint_effort_input input.launch.py joystick:=false mapping_confirmed:=true
```

```bash
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 python3 tools/input_demo.py
```

## Repeat the automated checks

Stop the interactive mock first. The live test requires an empty domain 188 and starts and
stops its own nodes. It explicitly checks for GenericSystem before sending motion.

```bash
./scripts/with_ros.sh env TMPDIR="$PWD/work/tmp" python3 -m unittest discover -s tests -v
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  python3 tools/check_r02_live.py
JE_ROS_DOMAIN_ID=188 ./scripts/with_ros.sh env JE_MOCK_TEST=1 \
  python3 tools/check_r02_rviz.py
```

See [REAL-ARM.md](REAL-ARM.md) for the physical work that remains.
