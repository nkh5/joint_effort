# Joint Effort arm — ROS 2

ROS 2 Jazzy software for the five-joint soldering-iron arm. The active model uses R02 CAD
geometry, with mock controllers, MoveIt Servo, Pico joystick input and an STS motor interface.

- [Build and workspace layout](JointEffort-ROS2-V1/ros2_ws/README.md)
- [R02 model, software changes and mock demos](JointEffort-ROS2-V1/ros2_ws/R02-INTEGRATION.md)
- [Physical arm setup and calibration](JointEffort-ROS2-V1/ros2_ws/REAL-ARM.md)
- [Software test results](JointEffort-ROS2-V1/ros2_ws/reports/r02/README.md)

The R02 software has passed mock tests. Physical calibration, usable travel and the actual
iron-tip offset still need verification. The default R02 calibration blocks powered startup.
