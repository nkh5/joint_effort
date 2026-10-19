import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88': raise RuntimeError('Use scripts/run_input.sh (cold mock domain 88)')
    p = Path(get_package_share_directory('joint_effort_input'))
    return LaunchDescription([
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('mapping_confirmed', default_value='false'),
        DeclareLaunchArgument('display', default_value='true'),
        Node(package='joy', executable='joy_node', parameters=[str(p/'config/input.yaml')],
             condition=IfCondition(LaunchConfiguration('joystick')), output='screen'),
        Node(package='joint_effort_input', executable='joy_teleop_clean', name='joy_teleop',
             parameters=[str(p/'config/teleop.yaml')], remappings=[('joy', '/amw/joy_control')], output='screen'),
        Node(package='joint_effort_input', executable='pico_forwarder', parameters=[str(p/'config/input.yaml'),
             {'mapping_confirmed': ParameterValue(LaunchConfiguration('mapping_confirmed'), value_type=bool)}], output='screen'),
        Node(package='joint_effort_input', executable='display_preview',
             condition=IfCondition(LaunchConfiguration('display')), output='screen')])
