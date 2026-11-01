"""Bench: the real sts_hardware_interface driving 5 loose motors, no arm model, no MoveIt.

  ros2 launch bench/bench.launch.py port:=/dev/serial/by-id/... rate:=50 ids:=1,2,3,4,5

Same hardware parameters as launch/real.launch.py (SyncWrite, hold position on activate), with a
forward_command_controller on 'position' so a test node can stream a new goal every cycle - the
same packet stream MoveIt Servo + the trajectory controller produce on the arm.
Joints are motor_<id>, centre 2048, limits +-pi (the test node keeps motion to +-15 deg).
"""
import os
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def setup(context):
    cfg = context.launch_configurations
    port, rate = cfg['port'], int(cfg['rate'])
    ids = [int(i) for i in cfg['ids'].split(',')]
    names = [f'motor_{i}' for i in ids]
    links = ['  <link name="base"/>']
    joints, ctrl = [], []
    for name, mid in zip(names, ids):
        links.append(f'  <link name="{name}_link"/>')
        joints.append(f'  <joint name="{name}" type="revolute"><parent link="base"/><child link="{name}_link"/>'
                      '<axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="1" velocity="5"/></joint>')
        ctrl.append(f'''    <joint name="{name}">
      <param name="motor_id">{mid}</param>
      <param name="operating_mode">0</param>
      <param name="position_center_steps">2048</param>
      <param name="min_position">-3.1</param>
      <param name="max_position">3.1</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>''')
    urdf = '\n'.join(['<?xml version="1.0"?>', '<robot name="sts_bench">', *links, *joints,
                      '  <ros2_control name="BenchSTS" type="system">', '    <hardware>',
                      '      <plugin>sts_hardware_interface/STSHardwareInterface</plugin>',
                      f'      <param name="serial_port">{port}</param>',
                      '      <param name="baud_rate">1000000</param>',
                      '      <param name="use_sync_write">true</param>',
                      '      <param name="reset_states_on_activate">false</param>',
                      '      <param name="hold_position_on_activate">true</param>',
                      '    </hardware>', *ctrl, '  </ros2_control>', '</robot>'])
    yaml = f'''controller_manager:
  ros__parameters:
    update_rate: {rate}
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    bench_position_controller:
      type: forward_command_controller/ForwardCommandController
bench_position_controller:
  ros__parameters:
    joints: [{", ".join(names)}]
    interface_name: position
'''
    fd, path = tempfile.mkstemp(prefix='bench_controllers_', suffix='.yaml')
    os.write(fd, yaml.encode())
    os.close(fd)

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '30'], output='screen')
    js, pc = spawner('joint_state_broadcaster'), spawner('bench_position_controller')
    print(f'[bench.launch] {len(ids)} motors {ids} on {port} at {rate} Hz', flush=True)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[path], output='screen'),
        js,
        RegisterEventHandler(OnProcessExit(target_action=js,
                                           on_exit=lambda e, c: [pc] if e.returncode == 0 else [])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port'),
        DeclareLaunchArgument('rate', default_value='50'),
        DeclareLaunchArgument('ids', default_value='1,2,3,4,5'),
        OpaqueFunction(function=setup),
    ])
