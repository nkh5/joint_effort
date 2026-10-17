# Code Info

## Model

The imported arm package contains the original joints and visible meshes. `generate_iron.py` adds the world mount, tip frame, workspace boxes, collision hulls, and mock hardware settings.

The URDF describes the links and joints. The SRDF defines the arm group and allowed collision pairs. Distances use meters; angles use radians.

## Movement math

`kinematics.py` reads the URDF. Forward kinematics combines the joint transforms to find the tip position. The Jacobian describes how small joint movements affect the tip.

The arm has five joints. We control XYZ position and two shaft-pointing directions. Shaft roll is free. The damped inverse converts a requested tip velocity into joint velocities. The code stops at poor configurations and checks speed and travel limits.

The code has been commented out. 

## Streamed movement

Pico or simulated input -> forwarder -> joy_teleop -> forwarder -> iron bridge -> MoveIt Servo -> mock controller.

The forwarder normalizes input, handles mode/enable, applies encoder speed, and checks readiness. The bridge uses the arm math. Servo handles the configured collision and joint-bound checks. The controller's hardware command interface uses positions.

Feedback travels from the mock hardware through the joint-state broadcaster and robot-state publisher. TF then updates the viewer.

## Planned movement

`iron_plan.py` chooses a target. `iron_cycle.py` builds the cold approach routine. Both use `iron_motion.py` to check readiness, request plans, inspect paths, execute, and monitor faults.

`iron_trajectory.py` checks the quintic curve between trajectory points. It checks joint-position extrema, peak speed, peak acceleration, and finite collision samples. Retiming slows a path without changing its geometric curve.

## Stopping and shutdown

The movement tools check ownership before creating their ROS node. The stop command looks for both project stop services together and sends each request as soon as it is found.

The local Servo runner lets its worker threads finish before closing ROS. The MoveIt planning runner also keeps its plugin loader alive until the planning objects are gone.

## Tests

From `ros2_ws`, with other model launches stopped:

```bash
JE_ROS_DOMAIN_ID=89 ./scripts/with_ros.sh python3 -m unittest discover -s tests -v
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/check_input_live.py
JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/check_iron_cycle.py
./scripts/with_ros.sh python3 tools/check_iron_calibration.py
```

The integration checks launch their own mock and viewer. They need a working display and ROS communication. Results are written under `reports`.

## Next Steps

We still need a real motor driver, measured joint/tool calibration, independent stopping, and physical accuracy tests. The current cycle stays above the board. 
