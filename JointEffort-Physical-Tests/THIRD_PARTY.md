# Third-party code

## MoveIt

`ros2_ws/src/joint_effort_moveit/src/move_group_lifetime.cpp` is based on MoveIt 2.12.4. Its BSD license and copyright notice remain in the source.

Source: https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/src/move_group.cpp

## teleop_tools

`ros2_ws/src/vendor/teleop_tools` contains joy_teleop and teleop_tools_msgs from tag 2.0.0, commit `d162759de377d0afd2e62420fa89c2d1127f2fee`. The source and its existing notices are unchanged.

Source: https://github.com/ros-teleop/teleop_tools

## CAD and project source

The arm description and visual meshes came from the team's CAD export. Keep their origin separate from the code that adds mock control and collision geometry.

The team has not chosen a license for its own code or CAD. No new license is assigned by this bundle.

## Team hardware code

`ros2_ws/src/amw_pi_bridge` comes from
https://github.com/nkh5/joint_effort at commit
`7e63e7a929292e6bf87458223642f0ec8ba5b048`. Python bytecode was omitted.
The imported source files are unchanged. `ros2_ws/firmware/Pico_Controller_iron`
adds a heartbeat to the team's Pico firmware; its `tinyusb.h` keeps the original
touchgadget MIT notice. The proposed firmware has not been compiled or flashed here.

The native STS driver uses the documented packet and register formats; it does
not include an FTServo SDK copy:

- https://docs.waveshare.com/Servo_Communication_Protocol
- https://github.com/ftservo/FTServo_Python/blob/cbcfa64674d592f7e5028ae72af42580f60500b4/scservo_sdk/sms_sts.py
