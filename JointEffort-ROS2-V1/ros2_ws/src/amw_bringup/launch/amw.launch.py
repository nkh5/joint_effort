"""The whole AMW stack in one launch. Normally started with `amw start` (see README.md).

  hardware:=mock|real   mock = simulated motors (default), real = STS servos on the Waveshare adapter
  readonly:=true        real only: torque OFF, joint states only (calibration and checking)
  teleop:=false         controllers only, no Servo/teleop/joystick (for `amw jog` and `amw track`)
  joystick:=false       teleop without joy_node (e.g. a synthetic /joy publisher)
  rviz:=true|false|auto auto (default) = RViz only if a display is available
  port:=/dev/...        override serial_port from arm.yaml
  arm_config:=/path     arm.yaml to use (`amw start` passes the one in the source tree)

With hardware:=real the launch first runs a read-only motor check (amw_tools.preflight). If a motor
does not answer, or a joint rests outside its limits (torque-on would make it jump), nothing starts.
"""
import os
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch_ros.actions import Node

from amw_bringup.arm import ArmConfigError, load, planning_limits, robot_description

SERVO_WAIT_TEXT = b'Waiting to receive robot state update'
SERVO_WAIT_FALLBACK_S = 20.0


def flag(context, name):
    value = context.launch_configurations[name].strip().lower()
    if value == 'auto':
        return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    if value not in ('true', 'false', '1', '0', 'yes', 'no'):
        raise RuntimeError(f'{name}:= must be true or false (is {value!r})')
    return value in ('true', '1', 'yes')


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text())


def setup(context):
    cfg = context.launch_configurations
    bringup = Path(get_package_share_directory('amw_bringup'))
    description_pkg = Path(get_package_share_directory('amw_description'))
    hardware = cfg['hardware'].strip()
    if hardware not in ('mock', 'real'):
        raise RuntimeError(f'hardware:= must be mock or real (is {hardware!r})')
    readonly = flag(context, 'readonly')
    if readonly and hardware != 'real':
        raise RuntimeError('readonly:=true only applies to hardware:=real')
    teleop = flag(context, 'teleop') and not readonly
    arm_path = cfg['arm_config'] or str(bringup / 'config/arm.yaml')

    try:
        arm = load(arm_path)
        if cfg['port']:
            arm.serial_port = cfg['port']
        description = robot_description((description_pkg / 'urdf/amw.urdf').read_text(),
                                        (description_pkg / 'srdf/amw.srdf').read_text(),
                                        arm, hardware, readonly)
    except ArmConfigError as e:
        raise RuntimeError(f'arm.yaml problem: {e}') from None

    mode = 'READ-ONLY, torque off' if readonly else ('controllers only' if not teleop else 'teleop')
    banner = LogInfo(msg=f'[amw] {hardware} arm, {mode}; config {arm_path}'
                         + (f'; port {arm.serial_port}' if hardware == 'real' else ''))

    moveit = {'robot_description': description,
              'robot_description_semantic': (description_pkg / 'srdf/amw.srdf').read_text(),
              'robot_description_kinematics': read_yaml(bringup / 'config/kinematics.yaml'),
              'robot_description_planning': planning_limits(arm)}

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '30'], output='screen')

    joint_states = spawner('joint_state_broadcaster')
    arm_controller = spawner('iron_arm_controller')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(bringup / 'config/controllers.yaml')], output='screen'),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_states,
            on_exit=lambda event, _: ([] if readonly else [arm_controller]) if event.returncode == 0
            else [EmitEvent(event=Shutdown(reason='joint_state_broadcaster failed to start'))])),
    ]

    if not teleop:
        nodes.append(joint_states)
    else:
        servo = Node(package='amw_moveit', executable='servo_lifetime', name='iron_servo',
                     parameters=[moveit, read_yaml(bringup / 'config/servo.yaml')], output='screen')
        # MoveIt Servo only starts once its planning scene sees a joint state that arrives AFTER Servo
        # itself started. With a still arm, joint states never change, so start the broadcaster only
        # when Servo says it is waiting (fallback: after SERVO_WAIT_FALLBACK_S seconds anyway).
        started = []

        def start_joint_states(text=SERVO_WAIT_TEXT):
            if started or SERVO_WAIT_TEXT not in text:
                return []
            started.append(True)
            return [joint_states]

        teleop_yaml = str(bringup / 'config/teleop.yaml')
        teleop_params = {'robot_description': description, 'max_joint_velocity_rad_s': arm.max_speed}
        nodes += [
            servo,
            RegisterEventHandler(OnProcessIO(target_action=servo,
                                             on_stdout=lambda e: start_joint_states(e.text),
                                             on_stderr=lambda e: start_joint_states(e.text))),
            TimerAction(period=SERVO_WAIT_FALLBACK_S,
                        actions=[OpaqueFunction(function=lambda _: start_joint_states())]),
            Node(package='amw_teleop', executable='amw_teleop', name='amw_teleop',
                 parameters=[teleop_yaml, teleop_params], output='screen'),
        ]
        if flag(context, 'joystick'):
            nodes.append(Node(package='joy', executable='joy_node', parameters=[teleop_yaml], output='screen'))

    if flag(context, 'rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(description_pkg / 'rviz/amw.rviz')], output='log'))

    if hardware == 'mock':
        return [banner, *nodes]

    # Real motors: read-only bus check first; start everything else only if it passes.
    check = [sys.executable, '-m', 'amw_tools.preflight', '--arm', arm_path, '--port', arm.serial_port]
    if readonly:
        check.append('--readonly')
    preflight = ExecuteProcess(cmd=check, output='screen', name='preflight')
    return [banner, preflight, RegisterEventHandler(OnProcessExit(
        target_action=preflight,
        on_exit=lambda event, _: nodes if event.returncode == 0 else
        [LogInfo(msg='[amw] Motor check failed (see above). Nothing was started.'),
         EmitEvent(event=Shutdown(reason='preflight failed'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('readonly', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='auto'),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('arm_config', default_value=''),
        OpaqueFunction(function=setup),
    ])
