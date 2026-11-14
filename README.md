# Joint Effort arm — ROS 2

ROS 2 Jazzy software for the five-joint soldering-iron arm, with MoveIt Servo,
Pico joystick input, an STS motor interface and a Pi status display.

The current launch files use `joint_effort_iron/urdf/iron.urdf`, draft-description
visual meshes and the populated `joint_effort_iron/config/sts_calibration.yaml`.
The R02 source model and saved R02 checks are retained separately. Those saved
results describe a different model revision from the current physical-arm setup.

- [Build and workspace layout](JointEffort-ROS2-V1/ros2_ws/README.md)
- [Physical arm setup and calibration](JointEffort-ROS2-V1/ros2_ws/REAL-ARM.md)
- [Pi status display](JointEffort-ROS2-V1/ros2_ws/LCD-SETUP.md)
- [R02 integration reference](JointEffort-ROS2-V1/ros2_ws/R02-INTEGRATION.md)
- [Saved software evidence](JointEffort-ROS2-V1/ros2_ws/reports/README.md)
- [Cleanup review](CLEANUP_REVIEW.md)
- [November 13 preserved files](NOVEMBER_13_PRESERVED_FILES.md)

Model regeneration changes geometry and can invalidate physical calibration.
Review the selected model, measured travel and actual tip offset before powered use.
