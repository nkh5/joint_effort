# Use JointEffort-ROS2-V1 for the arm

The ROS workspace in this folder (`ros2_ws`, the custom `joint_effort_hardware/IronSystem`
driver, `run_hardware*.sh`, heartbeat firmware and the "kory" button layout) is retired.
Do not build or launch it. The real arm now runs from `../JointEffort-ROS2-V1` on
sts_hardware_interface - see `../JointEffort-ROS2-V1/ros2_ws/REAL-ARM.md`.

Still used from this folder, only for servo ID work (no ROS):

```bash
./inspect_motors.sh --scan --label chain
./motor_test.sh assign-id --old-id 1 --new-id 4 --single-motor-connected --execute
./motor_test.sh verify-id --id 4 --single-motor-connected
```

`ros2_ws/hardware/Calibration.csv` was copied to `../JointEffort-ROS2-V1/ros2_ws/calibration/`.
