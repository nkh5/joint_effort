# amw_moveit

`servo_lifetime` runs the installed MoveIt Servo node (`moveit_servo::ServoNode`) and destroys it
before the ROS context shuts down. With MoveIt 2.12.4 the stock `servo_node` could crash on Ctrl-C
in its collision-check thread; this ordering avoids that race. Recheck when MoveIt is upgraded:
if the stock `moveit_servo servo_node` shuts down cleanly, this package can be deleted and
`amw.launch.py` pointed at it instead.
