# MoveIt source

`src/move_group_lifetime.cpp` comes from MoveIt 2.12.4:
https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/src/move_group.cpp

The source keeps the upstream BSD license notice. Our change keeps the capability library loader alive until the ROS node finishes cleanup. This fixes the shutdown crash seen in the tested Jazzy installation.

`no_execution_controller_manager.cpp` supplies no ordinary MoveIt execution handles. The project plan/cycle tools check and execute trajectories on the mock controller.

System ROS files are unchanged. Check this local fix again when changing the MoveIt version.

`servo_lifetime.cpp` uses the installed Servo library. It stops and destroys Servo before closing the ROS context. This avoids a shutdown race in the collision thread seen with MoveIt 2.12.4. Both mock launches use this runner.
