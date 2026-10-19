# Files in this folder

Paths are relative to `JointEffort-Physical-Tests`.

| File | Purpose |
| --- | --- |
| `.gitignore` | Exclude local build files, sessions, logs and caches. |
| `README.md` | Setup, operating instructions, status or source credits. |
| `THIRD_PARTY.md` | Setup, operating instructions, status or source credits. |
| `check_code.sh` | Run offline code checks without opening the adapter. |
| `check_setup.sh` | Check Python and list adapter permissions. |
| `docs/ROS-HARDWARE.md` | Setup, operating instructions, status or source credits. |
| `docs/STATUS.md` | Setup, operating instructions, status or source credits. |
| `docs/TEST-DETAILS.md` | Setup, operating instructions, status or source credits. |
| `docs/VERIFICATION.json` | Record packaging checks and physical validation limits. |
| `inspect_motors.sh` | Read motor addresses and raw replies. |
| `motor_test.sh` | Run the physical motor test with Python. |
| `ros2_ws/README.md` | Setup, operating instructions, status or source credits. |
| `ros2_ws/firmware/COLCON_IGNORE` | Proposed, unflashed Pico firmware or its instructions/license. |
| `ros2_ws/firmware/Pico_Controller_iron/Pico_Controller_iron.ino` | Proposed, unflashed Pico firmware or its instructions/license. |
| `ros2_ws/firmware/Pico_Controller_iron/tinyusb.h` | Proposed, unflashed Pico firmware or its instructions/license. |
| `ros2_ws/firmware/README.md` | Proposed, unflashed Pico firmware or its instructions/license. |
| `ros2_ws/hardware/angle-measurements-template.csv` | Physical measurement worksheet; blanks still require measurement. |
| `ros2_ws/hardware/joint-measurements.csv` | Physical measurement worksheet; blanks still require measurement. |
| `ros2_ws/scripts/run_hardware.sh` | Start physical ROS on domain 89. |
| `ros2_ws/scripts/run_hardware_input.sh` | Start the physical input path. |
| `ros2_ws/scripts/run_motor_test.sh` | Run commissioning through the ROS entry point. |
| `ros2_ws/scripts/start_clean.sh` | Load a clean Jazzy environment and this workspace. |
| `ros2_ws/scripts/with_ros.sh` | Load a clean Jazzy environment and this workspace. |
| `ros2_ws/src/amw_pi_bridge/README.md` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/__init__.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/amw_motors.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/contract.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/demo.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/feedback.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/amw_pi_bridge/preview.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/config/iron_calibration.json` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/package.xml` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/resource/amw_pi_bridge` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/setup.cfg` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/setup.py` | Team read-only feedback, conversion or trajectory-preview dependency. |
| `ros2_ws/src/amw_pi_bridge/test/test_amw_motors.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/amw_pi_bridge/test/test_contract.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/joint_effort_arm_draft_description/CMakeLists.txt` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_arm_draft_description/README.md` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_arm_draft_description/config/display.rviz` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_arm_draft_description/export-recipe.json` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_arm_draft_description/launch/display.launch.py` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/base_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/base_rotation_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/forearm_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/tool_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/upper_arm_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/meshes/wrist_link.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_arm_draft_description/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_arm_draft_description/source-inventory.json` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_arm_draft_description/urdf/joint_effort_arm_draft.urdf` | Shared robot geometry; physical launch replaces the hardware section. |
| `ros2_ws/src/joint_effort_cad_review/CMakeLists.txt` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_cad_review/config/review.rviz` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_cad_review/launch/review.launch.py` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_cad_review/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_cad_review/scripts/review_pose.py` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_cad_review/scripts/review_sliders.py` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_cad_review/src/cad_rviz.cpp` | Shared CAD model or visualization dependency. |
| `ros2_ws/src/joint_effort_cad_review/urdf/cad_arm.urdf` | Shared robot geometry; physical launch replaces the hardware section. |
| `ros2_ws/src/joint_effort_hardware/CMakeLists.txt` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_hardware/config/iron_hardware.template.json` | Setup, operating instructions, status or source credits. |
| `ros2_ws/src/joint_effort_hardware/hardware.xml` | Register the physical hardware plugin. |
| `ros2_ws/src/joint_effort_hardware/include/joint_effort_hardware/sts_bus.hpp` | Declare serial and joint-conversion interfaces. |
| `ros2_ws/src/joint_effort_hardware/joint_effort_hardware/__init__.py` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_hardware/joint_effort_hardware/commission.py` | Run bounded base/shoulder motor movements. |
| `ros2_ws/src/joint_effort_hardware/joint_effort_hardware/configuration.py` | Validate calibration and generate the physical robot description. |
| `ros2_ws/src/joint_effort_hardware/launch/hardware.launch.py` | Start the inactive physical ROS system. |
| `ros2_ws/src/joint_effort_hardware/launch/input.launch.py` | Setup, operating instructions, status or source credits. |
| `ros2_ws/src/joint_effort_hardware/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_hardware/scripts/sts_commission` | Setup, operating instructions, status or source credits. |
| `ros2_ws/src/joint_effort_hardware/src/iron_system.cpp` | Connect five physical joints to ros2_control. |
| `ros2_ws/src/joint_effort_hardware/src/sts_bus.cpp` | Encode motor packets, verify replies and convert angles/counts. |
| `ros2_ws/src/joint_effort_hardware/src/sts_inspect.cpp` | Read native motor identity and status. |
| `ros2_ws/src/joint_effort_hardware/test/test_sts_bus.cpp` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/joint_effort_input/config/input.yaml` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/config/teleop.yaml` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/__init__.py` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/core.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/display.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/forwarder.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/lease.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/synthetic.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/joint_effort_input/teleop.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/launch/input.launch.py` | Joystick normalization, permission, heartbeat or status dependency. |
| `ros2_ws/src/joint_effort_input/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_input/resource/joint_effort_input` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_input/setup.cfg` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_input/setup.py` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_iron/config/controllers.yaml` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/iron.rviz` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/iron.srdf` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/joint_limits.yaml` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/kinematics.yaml` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/model.json` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/config/servo.yaml` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/joint_effort_iron/__init__.py` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_iron/joint_effort_iron/bridge.py` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/launch/mock.launch.py` | Shared iron-arm model, limits, Servo bridge or launch dependency. |
| `ros2_ws/src/joint_effort_iron/meshes/base_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/meshes/base_rotation_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/meshes/forearm_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/meshes/tool_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/meshes/upper_arm_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/meshes/wrist_link_hull.stl` | CAD visual mesh or simplified collision geometry. |
| `ros2_ws/src/joint_effort_iron/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_iron/resource/joint_effort_iron` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_iron/setup.cfg` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_iron/setup.py` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_iron/urdf/iron.urdf` | Shared robot geometry; physical launch replaces the hardware section. |
| `ros2_ws/src/joint_effort_moveit/CMakeLists.txt` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_moveit/UPSTREAM.md` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/controllers_plugin.xml` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_moveit/src/iron_collision_audit.cpp` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/src/mount_clearance.cpp` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/src/move_group_lifetime.cpp` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/src/no_execution_controller_manager.cpp` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_moveit/src/servo_lifetime.cpp` | MoveIt lifecycle, Servo or collision-checking dependency. |
| `ros2_ws/src/joint_effort_robot/config/controllers.yaml` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/iron_servo.yaml` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/joint_effort.srdf` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/joint_limits.yaml` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/kinematics.yaml` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/model.json` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/model.rviz` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/config/wire_servo.yaml` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/joint_effort_robot/__init__.py` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_robot/joint_effort_robot/kinematics.py` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/joint_effort_robot/task_bridge.py` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/launch/mock.launch.py` | Shared kinematics and robot-description dependency. |
| `ros2_ws/src/joint_effort_robot/package.xml` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_robot/resource/joint_effort_robot` | Python/ROS package registration. |
| `ros2_ws/src/joint_effort_robot/setup.cfg` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_robot/setup.py` | Build/install rules or package dependencies. |
| `ros2_ws/src/joint_effort_robot/urdf/joint_effort.urdf` | Shared robot geometry; physical launch replaces the hardware section. |
| `ros2_ws/src/vendor/teleop_tools/README.md` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/UPSTREAM.json` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/CHANGELOG.rst` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/README.md` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/config/joy_teleop_example.yaml` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/joy_teleop/__init__.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/joy_teleop/incrementer_server.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/joy_teleop/joy_teleop.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/launch/example.launch.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/package.xml` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/resource/joy_teleop` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/setup.cfg` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/setup.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/joy_teleop_testing_common.py` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_action_fibonacci.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_array_indexing.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_copyright.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_debounce.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_flake8.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_get_interface_type.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_multi_button_command.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_parameter_failures.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_pep257.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_service_add_two_ints.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_service_not_ready.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_service_trigger.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_topic_axis_mappings.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_topic_message_value.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/joy_teleop/test/test_xmllint.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs/CHANGELOG.rst` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs/CMakeLists.txt` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs/action/Increment.action` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs/package.xml` | Vendored joystick mapping/message source or retained upstream notice. |
| `ros2_ws/tests/test_commission.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/tests/test_hardware.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/tests/test_motor_identification.py` | Automated hardware-code check; does not use real motors. |
| `ros2_ws/tools/hardware_check.py` | Check calibration without serial access. |
| `ros2_ws/tools/hardware_stop.py` | Request a software hold without torque release. |
| `ros2_ws/tools/identify_motors.py` | Read addresses and capture diagnostic replies. |
| `ros2_ws/tools/iron_calibration.py` | Fit measured angle/count samples offline. |
| `scripts/check_setup.py` | Inspect Python and USB device paths without serial commands. |
| `scripts/motor_test.py` | Locate the bundled commissioning module. |
| `setup.sh` | Install Python dependencies on Ubuntu. |
| `tools/package_physical_tests.py` | Record the assembly recipe used for this folder. |
| `docs/FILES.md` | This file list. |
| `MANIFEST.json` | Source sizes and SHA-256 checksums. |
