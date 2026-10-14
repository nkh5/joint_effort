"""amw.launch.py - starts the whole controller -> command chain in one go.

  joy_node    reads the Pico, publishes /joy                (amw_joy.yaml)
  joy_teleop  /joy -> /amw/cmd_raw (translate/rotate)       (amw_joy.yaml)
  amw_speed   /amw/cmd_raw x knob -> /servo_node/delta_twist_cmds
  amw_lcd     status screen

Run from this folder:
  ros2 launch ./amw.launch.py              # PC: LCD in a window
  ros2 launch ./amw.launch.py lcd:=fb      # Pi: LCD on the framebuffer
  ros2 launch ./amw.launch.py lcd:=none    # no LCD
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
PARAMS = os.path.join(HERE, "amw_joy.yaml")


def lcd_is(value):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("lcd"), "' == '", value, "'"]))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lcd", default_value="window",
                              description="window | fb | none"),
        Node(package="joy", executable="joy_node", name="joy_node",
             parameters=[PARAMS], output="screen"),
        Node(package="joy_teleop", executable="joy_teleop", name="joy_teleop",
             parameters=[PARAMS], output="screen"),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "amw_speed.py")],
                       name="amw_speed", output="screen"),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "amw_lcd.py")],
                       name="amw_lcd", output="screen", condition=lcd_is("window")),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "amw_lcd.py"), "--lcd"],
                       name="amw_lcd", output="screen", condition=lcd_is("fb")),
    ])
