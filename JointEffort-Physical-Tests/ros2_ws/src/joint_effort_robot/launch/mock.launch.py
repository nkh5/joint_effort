from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p = Path(get_package_share_directory('joint_effort_robot'))
    model = {'robot_description': (p/'urdf/joint_effort.urdf').read_text(),
             'robot_description_semantic': (p/'config/joint_effort.srdf').read_text(),
             'robot_description_kinematics': yaml.safe_load((p/'config/kinematics.yaml').read_text()),
             'robot_description_planning': yaml.safe_load((p/'config/joint_limits.yaml').read_text())}
    common = [model]
    js = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster','--controller-manager-timeout','30'], output='screen')
    controllers = Node(package='controller_manager', executable='spawner', arguments=['iron_arm_controller','wire_arm_controller','--controller-manager-timeout','30'], output='screen')
    nodes = [DeclareLaunchArgument('rviz', default_value='false'), DeclareLaunchArgument('selected_arm', default_value='iron'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description':model['robot_description']}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[str(p/'config/controllers.yaml')], output='screen'), js,
        RegisterEventHandler(OnProcessExit(target_action=js, on_exit=lambda event, context: [controllers] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Joint state broadcaster failed'))])),
        Node(package='joint_effort_moveit', executable='move_group_lifetime', parameters=common + [{
            'publish_robot_description_semantic':True, 'publish_planning_scene':True,
            'publish_geometry_updates':True, 'publish_state_updates':True, 'publish_transforms_updates':True,
            'planning_pipelines':['ompl'], 'default_planning_pipeline':'ompl',
            'ompl':{'planning_plugins':['ompl_interface/OMPLPlanner']},
            'moveit_controller_manager':'joint_effort_moveit/NoExecutionControllerManager',
            'disable_capabilities':'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
            'allow_trajectory_execution':False}], output='screen'),
        Node(package='joint_effort_robot', executable='task_bridge', parameters=[{'selected_arm':LaunchConfiguration('selected_arm')}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d',str(p/'config/model.rviz')], parameters=common, condition=IfCondition(LaunchConfiguration('rviz')), output='screen')]
    for arm in ['iron','wire']:
        nodes.append(Node(package='joint_effort_moveit', executable='servo_lifetime', name=arm+'_servo',
            parameters=common + [yaml.safe_load((p/'config'/f'{arm}_servo.yaml').read_text())], output='screen'))
    return LaunchDescription(nodes)
