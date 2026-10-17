# Files in this bundle

Paths are relative to the `ros` folder. Build files and local recordings are left out.


## Top level

| File | Purpose |
| --- | --- |
| `.gitignore` | Keep local build files and caches out of Git. |
| `README.md` | Start here: build and run the iron-arm mock. |
| `THIRD_PARTY.md` | Source credits and license notes. |

## docs

| File | Purpose |
| --- | --- |
| `CODE.md` | Explain the movement and input code. |
| `FILES.md` | List every bundled file and its purpose. |
| `README.md` | Guide index. |
| `SETUP.md` | Install dependencies and build on another computer. |

## iron_prototype

| File | Purpose |
| --- | --- |
| `PICO-CONTROLLER-GUIDE.md` | Check the Pico and test joystick input. |
| `README.md` | Run, move, plan, cycle, and stop the mock arm. |

## ros2_ws

| File | Purpose |
| --- | --- |
| `.gitignore` | Ignore local workspace build files. |
| `README.md` | Workspace layout and start commands. |

## ros2_ws/reports

| File | Purpose |
| --- | --- |
| `README.md` | Explain the saved studies and test results. |
| `collision-endpoints.json` | Earlier collision results used by the plot tool. |
| `joint-reference.csv` | Earlier dual-arm joint reference. |
| `workspace-study.json` | Earlier dual-arm reach and five-axis study. |

## ros2_ws/reports/iron

| File | Purpose |
| --- | --- |
| `calibration-check.json` | Offline tests using made-up encoder readings. |
| `collision-hulls.json` | Collision mesh sizes and simplification results. |
| `collision-samples.json` | Sampled iron-arm collision results. |
| `cycle-check.json` | Cold-cycle movement and interruption test results. |
| `input-check.json` | Synthetic joystick integration test results. |
| `proposed-motor-map.csv` | Proposed motor IDs and joint conversion settings. |
| `unit-tests.txt` | Python unit test results. |
| `workspace-study.json` | Iron-arm target study and checked hover candidates. |

## ros2_ws/scripts

| File | Purpose |
| --- | --- |
| `run_input.sh` | Start the joystick input nodes on domain 88. |
| `run_iron.sh` | Start the current iron-arm mock on domain 88. |
| `run_mock.sh` | Start the earlier dual-arm mock on domain 87. |
| `start_clean.sh` | Load Jazzy, the local build, domain, and log folders. |
| `with_ros.sh` | Run a command in a clean ROS Jazzy environment. |

## ros2_ws/src/joint_effort_arm_draft_description

| File | Purpose |
| --- | --- |
| `CMakeLists.txt` | Build and install this ROS package. |
| `README.md` | Explain the imported CAD model and visual preview. |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |

## ros2_ws/src/joint_effort_arm_draft_description/config

| File | Purpose |
| --- | --- |
| `display.rviz` | Save the RViz model, frame, and display settings. |

## ros2_ws/src/joint_effort_arm_draft_description/launch

| File | Purpose |
| --- | --- |
| `display.launch.py` | Start the isolated CAD visual preview. |

## ros2_ws/src/joint_effort_arm_draft_description/meshes

| File | Purpose |
| --- | --- |
| `base_link.stl` | CAD visual mesh for base_link. |
| `base_rotation_link.stl` | CAD visual mesh for base_rotation_link. |
| `forearm_link.stl` | CAD visual mesh for forearm_link. |
| `tool_link.stl` | CAD visual mesh for tool_link. |
| `upper_arm_link.stl` | CAD visual mesh for upper_arm_link. |
| `wrist_link.stl` | CAD visual mesh for wrist_link. |

## ros2_ws/src/joint_effort_arm_draft_description/urdf

| File | Purpose |
| --- | --- |
| `joint_effort_arm_draft.urdf` | Keep the imported CAD link and joint definitions. |

## ros2_ws/src/joint_effort_cad_review

| File | Purpose |
| --- | --- |
| `CMakeLists.txt` | Build and install this ROS package. |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |

## ros2_ws/src/joint_effort_cad_review/config

| File | Purpose |
| --- | --- |
| `review.rviz` | Save the RViz model, frame, and display settings. |

## ros2_ws/src/joint_effort_cad_review/launch

| File | Purpose |
| --- | --- |
| `review.launch.py` | Start the isolated CAD visual preview. |

## ros2_ws/src/joint_effort_cad_review/scripts

| File | Purpose |
| --- | --- |
| `review_pose.py` | Publish CAD review joint angles. |
| `review_sliders.py` | Adjust CAD review joint angles with sliders. |

## ros2_ws/src/joint_effort_cad_review/src

| File | Purpose |
| --- | --- |
| `cad_rviz.cpp` | Show the model and frame status in RViz. |

## ros2_ws/src/joint_effort_cad_review/urdf

| File | Purpose |
| --- | --- |
| `cad_arm.urdf` | Describe the arm for isolated CAD review. |

## ros2_ws/src/joint_effort_input

| File | Purpose |
| --- | --- |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |
| `setup.cfg` | Set the ROS Python installation folders. |
| `setup.py` | Install the Python package, commands, and data files. |

## ros2_ws/src/joint_effort_input/config

| File | Purpose |
| --- | --- |
| `input.yaml` | Set the joystick name, axes, buttons, speed, and timeouts. |
| `teleop.yaml` | Map normalized joystick values to raw tip velocity. |

## ros2_ws/src/joint_effort_input/joint_effort_input

| File | Purpose |
| --- | --- |
| `__init__.py` | Mark this folder as a Python package. |
| `core.py` | Map joystick input and enforce mode and enable rules. |
| `display.py` | Show input mode, speed, and errors in the terminal. |
| `forwarder.py` | Scale tip speed and reject stale or invalid commands. |
| `lease.py` | Allow only one project movement tool to own the arm. |
| `synthetic.py` | Provide simulated joystick messages for demos and tests. |
| `teleop.py` | Start joy_teleop with the project shutdown handling. |

## ros2_ws/src/joint_effort_input/launch

| File | Purpose |
| --- | --- |
| `input.launch.py` | Start joystick mapping, forwarding, and terminal status nodes. |

## ros2_ws/src/joint_effort_input/resource

| File | Purpose |
| --- | --- |
| `joint_effort_input` | Register the Python package in the ROS package index. |

## ros2_ws/src/joint_effort_iron

| File | Purpose |
| --- | --- |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |
| `setup.cfg` | Set the ROS Python installation folders. |
| `setup.py` | Install the Python package, commands, and data files. |

## ros2_ws/src/joint_effort_iron/config

| File | Purpose |
| --- | --- |
| `controllers.yaml` | Configure mock joint controllers and state publishing. |
| `iron.rviz` | Save the RViz model, frame, and display settings. |
| `iron.srdf` | Define planning groups and permitted collision pairs. |
| `joint_limits.yaml` | Set joint speed and acceleration limits. |
| `kinematics.yaml` | Select the MoveIt inverse-kinematics settings. |
| `model.json` | Store model dimensions and motion assumptions. |
| `servo.yaml` | Configure iron-arm Servo topics, limits, and collision checks. |

## ros2_ws/src/joint_effort_iron/joint_effort_iron

| File | Purpose |
| --- | --- |
| `__init__.py` | Mark this folder as a Python package. |
| `bridge.py` | Send checked iron-arm commands to Servo and the mock controller. |

## ros2_ws/src/joint_effort_iron/launch

| File | Purpose |
| --- | --- |
| `mock.launch.py` | Start mock hardware, controllers, MoveIt, Servo, and RViz. |

## ros2_ws/src/joint_effort_iron/meshes

| File | Purpose |
| --- | --- |
| `base_link_hull.stl` | Simplified collision shape for base_link. |
| `base_rotation_link_hull.stl` | Simplified collision shape for base_rotation_link. |
| `forearm_link_hull.stl` | Simplified collision shape for forearm_link. |
| `tool_link_hull.stl` | Simplified collision shape for tool_link. |
| `upper_arm_link_hull.stl` | Simplified collision shape for upper_arm_link. |
| `wrist_link_hull.stl` | Simplified collision shape for wrist_link. |

## ros2_ws/src/joint_effort_iron/resource

| File | Purpose |
| --- | --- |
| `joint_effort_iron` | Register the Python package in the ROS package index. |

## ros2_ws/src/joint_effort_iron/urdf

| File | Purpose |
| --- | --- |
| `iron.urdf` | Describe the current arm, fixtures, TCP, and mock hardware. |

## ros2_ws/src/joint_effort_moveit

| File | Purpose |
| --- | --- |
| `CMakeLists.txt` | Build and install this ROS package. |
| `UPSTREAM.md` | Explain the copied MoveIt source and local change. |
| `controllers_plugin.xml` | Register the MoveIt execution-blocking plugin. |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |

## ros2_ws/src/joint_effort_moveit/src

| File | Purpose |
| --- | --- |
| `iron_collision_audit.cpp` | Check model states and sampled paths for collisions. |
| `mount_clearance.cpp` | Measure clearance around the modeled mount. |
| `move_group_lifetime.cpp` | Run MoveIt with corrected shutdown object lifetimes. |
| `no_execution_controller_manager.cpp` | Disable ordinary MoveIt execution in the mock launch. |
| `servo_lifetime.cpp` | Stop Servo worker threads before closing the ROS context. |

## ros2_ws/src/joint_effort_robot

| File | Purpose |
| --- | --- |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |
| `setup.cfg` | Set the ROS Python installation folders. |
| `setup.py` | Install the Python package, commands, and data files. |

## ros2_ws/src/joint_effort_robot/config

| File | Purpose |
| --- | --- |
| `controllers.yaml` | Configure mock joint controllers and state publishing. |
| `iron_servo.yaml` | Configure Servo for the earlier iron arm. |
| `joint_effort.srdf` | Define planning groups and permitted collision pairs. |
| `joint_limits.yaml` | Set joint speed and acceleration limits. |
| `kinematics.yaml` | Select the MoveIt inverse-kinematics settings. |
| `model.json` | Store model dimensions and motion assumptions. |
| `model.rviz` | Save the RViz model, frame, and display settings. |
| `wire_servo.yaml` | Configure Servo for the earlier wire arm. |

## ros2_ws/src/joint_effort_robot/joint_effort_robot

| File | Purpose |
| --- | --- |
| `__init__.py` | Mark this folder as a Python package. |
| `kinematics.py` | Calculate tip position, Jacobians, and limited joint speeds. |
| `task_bridge.py` | Translate tip velocity into joint commands for the earlier mock. |

## ros2_ws/src/joint_effort_robot/launch

| File | Purpose |
| --- | --- |
| `mock.launch.py` | Start mock hardware, controllers, MoveIt, Servo, and RViz. |

## ros2_ws/src/joint_effort_robot/resource

| File | Purpose |
| --- | --- |
| `joint_effort_robot` | Register the Python package in the ROS package index. |

## ros2_ws/src/joint_effort_robot/urdf

| File | Purpose |
| --- | --- |
| `joint_effort.urdf` | Describe the earlier dual-arm mock. |

## ros2_ws/src/vendor/teleop_tools

| File | Purpose |
| --- | --- |
| `README.md` | Upstream teleop package guide. |
| `UPSTREAM.json` | Record the vendored teleop source version. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop

| File | Purpose |
| --- | --- |
| `CHANGELOG.rst` | Upstream release history and credits. |
| `README.md` | Upstream teleop package guide. |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |
| `setup.cfg` | Set the ROS Python installation folders. |
| `setup.py` | Install the Python package, commands, and data files. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop/config

| File | Purpose |
| --- | --- |
| `joy_teleop_example.yaml` | Upstream example joystick configuration. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop/joy_teleop

| File | Purpose |
| --- | --- |
| `__init__.py` | Mark this folder as a Python package. |
| `incrementer_server.py` | Upstream example action server. |
| `joy_teleop.py` | Upstream joystick-to-message, service, and action mappings. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop/launch

| File | Purpose |
| --- | --- |
| `example.launch.py` | Upstream example launch file. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop/resource

| File | Purpose |
| --- | --- |
| `joy_teleop` | Register the Python package in the ROS package index. |

## ros2_ws/src/vendor/teleop_tools/joy_teleop/test

| File | Purpose |
| --- | --- |
| `joy_teleop_testing_common.py` | Upstream joy_teleop test or shared test helper. |
| `test_action_fibonacci.py` | Upstream joy_teleop test or shared test helper. |
| `test_array_indexing.py` | Upstream joy_teleop test or shared test helper. |
| `test_copyright.py` | Upstream joy_teleop test or shared test helper. |
| `test_debounce.py` | Upstream joy_teleop test or shared test helper. |
| `test_flake8.py` | Upstream joy_teleop test or shared test helper. |
| `test_get_interface_type.py` | Upstream joy_teleop test or shared test helper. |
| `test_multi_button_command.py` | Upstream joy_teleop test or shared test helper. |
| `test_parameter_failures.py` | Upstream joy_teleop test or shared test helper. |
| `test_pep257.py` | Upstream joy_teleop test or shared test helper. |
| `test_service_add_two_ints.py` | Upstream joy_teleop test or shared test helper. |
| `test_service_not_ready.py` | Upstream joy_teleop test or shared test helper. |
| `test_service_trigger.py` | Upstream joy_teleop test or shared test helper. |
| `test_topic_axis_mappings.py` | Upstream joy_teleop test or shared test helper. |
| `test_topic_message_value.py` | Upstream joy_teleop test or shared test helper. |
| `test_xmllint.py` | Upstream joy_teleop test or shared test helper. |

## ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs

| File | Purpose |
| --- | --- |
| `CHANGELOG.rst` | Upstream release history and credits. |
| `CMakeLists.txt` | Build and install this ROS package. |
| `package.xml` | Declare the ROS package, dependencies, maintainer, and license. |

## ros2_ws/src/vendor/teleop_tools/teleop_tools_msgs/action

| File | Purpose |
| --- | --- |
| `Increment.action` | Upstream increment action message definition. |

## ros2_ws/tests

| File | Purpose |
| --- | --- |
| `test_bridge.py` | Test the earlier velocity bridge. |
| `test_input.py` | Test joystick mapping and enable rules. |
| `test_input_forwarding.py` | Test command timing, speed scaling, and forwarding. |
| `test_iron.py` | Test iron-arm model geometry and settings. |
| `test_iron_bridge.py` | Test the iron-arm velocity bridge. |
| `test_iron_workflows.py` | Test planning, trajectories, and calibration helpers. |
| `test_kinematics.py` | Test tip transforms, Jacobians, and motion limits. |

## ros2_ws/tools

| File | Purpose |
| --- | --- |
| `check_input_live.py` | Test synthetic input through the running mock. |
| `check_iron_calibration.py` | Test the offline calibration tool. |
| `check_iron_cycle.py` | Test cold cycles, stopping, and feedback loss. |
| `check_iron_demo.py` | Check the visible iron-arm demo. |
| `check_iron_live.py` | Check iron-arm controllers, motion, and collision handling. |
| `check_live.py` | Check the earlier dual-arm mock. |
| `check_shutdown.py` | Check that the earlier mock shuts down cleanly. |
| `demo_both.py` | Move both earlier mock arms at once. |
| `demo_motion.py` | Move one earlier mock arm. |
| `generate_assets.py` | Generate the earlier dual-arm model and settings. |
| `generate_iron.py` | Generate the current iron-arm model, fixtures, and collision hulls. |
| `input_demo.py` | Send simulated joystick input without a controller. |
| `input_probe.py` | Read joystick values and save their observed ranges. |
| `iron_calibration.py` | Fit joint zero, direction, and external ratio from a CSV. |
| `iron_cycle.py` | Preview or run the cold hover/lower/wait/retreat cycle. |
| `iron_demo.py` | Run small iron-arm demonstration movements. |
| `iron_motion.py` | Share motion checks, planning, feedback, and execution helpers. |
| `iron_plan.py` | Preview or execute a checked planned movement. |
| `iron_status.py` | Print iron-arm controller, feedback, and collision status. |
| `iron_stop.py` | Request a stop from the project movement tools. |
| `iron_trajectory.py` | Build and check timed joint trajectories. |
| `model_status.py` | Print status for the earlier dual-arm mock. |
| `plot_study.py` | Plot the earlier workspace and collision study. |
| `render_iron.py` | Draw the iron-arm model and workspace study. |
| `review_cad_urdf.py` | Inspect CAD joint frames, dimensions, and mesh bounds. |
| `study_iron_workspace.py` | Find candidate iron-tip positions and pointing directions. |
| `study_workspace.py` | Study reach and five-axis motion for the earlier model. |

## tools

| File | Purpose |
| --- | --- |
| `package_github.py` | Build this source ZIP and check its contents. |

## Generated file

| File | Purpose |
| --- | --- |
| `MANIFEST.json` | Record file sizes and SHA-256 checksums. |
