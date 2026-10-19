# Physical ROS source

Start with the README in the parent folder. The base/shoulder tests can run
without ROS using `../motor_test.sh`. See `../docs/ROS-HARDWARE.md` for the full
five-joint driver, its dependencies and the calibration still required.

This source keeps shared model, kinematics and launch files from the earlier
workspace because the physical ROS path imports them. The old mock demo scripts
and mock test commands are not included in the main test workflow.
