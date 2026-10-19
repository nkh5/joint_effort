import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from joint_effort_hardware.configuration import read, emulator_port


def start(context):
    emulated = LaunchConfiguration('emulated_bus').perform(context) == 'true'
    if os.environ.get('ROS_DOMAIN_ID') != ('90' if emulated else '89'): raise RuntimeError('Wrong ROS domain for physical/emulated input')
    calibration = LaunchConfiguration('calibration').perform(context)
    iron = Path(get_package_share_directory('joint_effort_iron'))
    config = read(calibration, iron/'urdf/iron.urdf')
    if emulated: emulator_port(config['serial_port'])
    confirmed = LaunchConfiguration('mapping_confirmed').perform(context)
    if confirmed not in ('true', 'false'): raise ValueError('mapping_confirmed must be true or false')
    p = Path(get_package_share_directory('joint_effort_input'))
    robot_path = Path(os.environ['ROS_HOME']).parent/('hardware-emulated-runtime' if emulated else 'hardware')/'iron_measured.urdf'
    if not robot_path.is_file(): raise ValueError('Start the physical model before its input nodes')
    return [
        Node(package='joy', executable='joy_node', parameters=[{'device_name': 'AMW Controller',
            'deadzone': 0., 'autorepeat_rate': 50., 'coalesce_interval_ms': 1, 'sticky_buttons': False}],
            condition=IfCondition(LaunchConfiguration('joystick')), output='screen'),
        Node(package='joint_effort_input', executable='joy_teleop_clean', name='joy_teleop',
            parameters=[str(p/'config/teleop.yaml')], remappings=[('joy', '/amw/joy_control')], output='screen'),
        Node(package='joint_effort_input', executable='pico_forwarder', parameters=[str(p/'config/input.yaml'),
            {'runtime_profile': 'hardware', 'hardware_calibration': calibration,
             'robot_path': str(robot_path), 'mapping_confirmed': confirmed == 'true',
             'emulated_bus': emulated,
             'enable_button': 0, 'mode_button': 1, 'mode_select_button': 9}], output='screen'),
        Node(package='joint_effort_input', executable='display_preview', output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('emulated_bus', default_value='false'),
        DeclareLaunchArgument('mapping_confirmed', default_value='false'), OpaqueFunction(function=start)])
