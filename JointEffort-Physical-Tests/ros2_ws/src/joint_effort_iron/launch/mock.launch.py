import os,json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
 if os.environ.get('ROS_DOMAIN_ID')!='88':raise RuntimeError('Use scripts/run_iron.sh: isolated iron prototype requires ROS domain 88.')
 return build_launch()

def build_launch(description=None,controllers_file=None,bridge_parameters=None,physical=False,planning_limits=None):
 p=Path(get_package_share_directory('joint_effort_iron'));rviz_home=Path(os.environ['ROS_HOME']).parent/'iron_rviz_home';(rviz_home/'.rviz2').mkdir(parents=True,exist_ok=True)
 description=description or (p/'urdf/iron.urdf').read_text()
 common=[{'robot_description':description,'robot_description_semantic':(p/'config/iron.srdf').read_text(),'robot_description_kinematics':json.loads((p/'config/kinematics.yaml').read_text()),'robot_description_planning':planning_limits or json.loads((p/'config/joint_limits.yaml').read_text())}]
 js=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager-timeout','30'],output='screen')
 controller=Node(package='controller_manager',executable='spawner',arguments=['iron_arm_controller','--controller-manager-timeout','30']+(['--inactive'] if physical else []),output='screen')
 return LaunchDescription([
  DeclareLaunchArgument('rviz',default_value='false' if physical else 'true'),
  Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}],output='screen'),
  Node(package='controller_manager',executable='ros2_control_node',parameters=[controllers_file or str(p/'config/controllers.yaml')],output='screen'),js,
  RegisterEventHandler(OnProcessExit(target_action=js,on_exit=lambda event,context:[controller] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Joint broadcaster failed'))])),
  Node(package='joint_effort_moveit',executable='move_group_lifetime',parameters=common+[{'publish_robot_description_semantic':True,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':{'planning_plugins':['ompl_interface/OMPLPlanner'],'request_adapters':['default_planning_request_adapters/ResolveConstraintFrames','default_planning_request_adapters/ValidateWorkspaceBounds','default_planning_request_adapters/CheckStartStateBounds','default_planning_request_adapters/CheckStartStateCollision'],'response_adapters':['default_planning_response_adapters/AddTimeOptimalParameterization','default_planning_response_adapters/ValidateSolution']},'moveit_controller_manager':'joint_effort_moveit/NoExecutionControllerManager','disable_capabilities':'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction','allow_trajectory_execution':False}],output='screen'),
  Node(package='joint_effort_moveit',executable='servo_lifetime',name='iron_servo',parameters=common+[json.loads((p/'config/servo.yaml').read_text())],output='screen'),
  Node(package='joint_effort_iron',executable='iron_task_bridge',parameters=bridge_parameters or [],output='screen'),
  Node(package='joint_effort_cad_review',executable='cad_rviz',arguments=['-d',str(p/'config/iron.rviz')],additional_env={'HOME':str(rviz_home)},condition=IfCondition(LaunchConfiguration('rviz')),output='screen')])
