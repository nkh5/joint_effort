"""Datum measurement (visual only): RViz model of iron.urdf + sliders. No ros2_control, no motors.

Run through scripts/run_datum.sh (ROS domain 88). See REAL-ARM.md, step 2.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('Use scripts/run_datum.sh (ROS domain 88).')
    p = Path(get_package_share_directory('joint_effort_iron'))
    rviz_home = Path(os.environ['ROS_HOME']).parent / 'iron_rviz_home'
    (rviz_home / '.rviz2').mkdir(parents=True, exist_ok=True)
    sliders = Node(package='joint_effort_iron', executable='iron_datum_sliders', output='screen')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (p / 'urdf/iron.urdf').read_text()}], output='screen'),
        sliders,
        Node(package='joint_effort_cad_review', executable='cad_rviz',
             arguments=['-d', str(p / 'config/iron.rviz')], additional_env={'HOME': str(rviz_home)},
             output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=sliders,
                                           on_exit=[EmitEvent(event=Shutdown(reason='sliders closed'))])),
    ])
