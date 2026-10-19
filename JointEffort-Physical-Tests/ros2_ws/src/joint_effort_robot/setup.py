from setuptools import setup
from glob import glob
setup(name='joint_effort_robot', version='0.1.0', packages=['joint_effort_robot'],
 data_files=[('share/ament_index/resource_index/packages',['resource/joint_effort_robot']),
 ('share/joint_effort_robot',['package.xml']),
 ('share/joint_effort_robot/config',glob('config/*')),
 ('share/joint_effort_robot/urdf',glob('urdf/*')),
 ('share/joint_effort_robot/launch',glob('launch/*'))],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts':['task_bridge = joint_effort_robot.task_bridge:main']})
