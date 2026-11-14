from setuptools import setup

setup(
    name='joint_effort_robot',
    version='0.1.0',
    description='Shared five-joint arm kinematics and reduced task motion math.',
    packages=['joint_effort_robot'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/joint_effort_robot']),
        ('share/joint_effort_robot', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
