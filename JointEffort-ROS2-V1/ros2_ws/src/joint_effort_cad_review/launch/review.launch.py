import os,xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
 if os.environ.get('ROS_DOMAIN_ID')!='96':raise RuntimeError('CAD review requires isolated ROS_DOMAIN_ID=96. Use JE_ROS_DOMAIN_ID=96 with scripts/with_ros.sh.')
 share=Path(get_package_share_directory('joint_effort_cad_review'))
 rviz_home=Path(os.environ['ROS_HOME']).parent/'cad_review_home'
 (rviz_home/'.rviz2').mkdir(parents=True,exist_ok=True)
 root=ET.parse(share/'urdf/cad_arm.urdf').getroot();missing=[]
 for e in root.findall('.//mesh'):
  package,rel=e.get('filename').removeprefix('package://').split('/',1)
  try:path=Path(get_package_share_directory(package))/rel
  except PackageNotFoundError:path=None
  if path is None or not path.is_file():missing.append(e.get('filename'))
 if missing:
  for link in root.findall('link'):
   for visual in link.findall('visual'):link.remove(visual)
 description=ET.tostring(root,encoding='unicode')
 return LaunchDescription([
  DeclareLaunchArgument('rviz',default_value='true'),
  DeclareLaunchArgument('gui',default_value='true'),
  LogInfo(msg='CAD REVIEW ONLY: '+('six meshes unavailable; displaying coordinate axes only' if missing else 'all referenced meshes found; displaying CAD visuals')),
  Node(package='robot_state_publisher',executable='robot_state_publisher',name='cad_review_state_publisher',parameters=[{'robot_description':description}],remappings=[('joint_states','/cad_review/joint_states'),('robot_description','/cad_review/robot_description')]),
  Node(package='joint_effort_cad_review',executable='review_pose.py'),
  Node(package='joint_effort_cad_review',executable='review_sliders.py',condition=IfCondition(LaunchConfiguration('gui'))),
  Node(package='joint_effort_cad_review',executable='cad_rviz',additional_env={'HOME':str(rviz_home)},arguments=['-d',str(share/'config/review.rviz')],condition=IfCondition(LaunchConfiguration('rviz')))
 ])
