"""Real iron arm: same stack as mock.launch.py, with the mock ros2_control hardware
swapped for sts_hardware_interface (Feetech STS servos on the Waveshare adapter).

  readonly:=true   torque OFF on every servo, joint states only (calibration / checking).
                   Starts with an incomplete calibration; a joint without center_steps uses its
                   csv_zero_counts + encoder_shift_counts, so it reads 0 at your physical 0 deg mark
                   (2048 if even that is missing). RViz is then off by the datum for that joint.
  readonly:=false  (default) servos hold their current position on start. Needs a complete
                   calibration (tools/fit_calibration.py says COMPLETE).
  teleop:=true     (default) also start move_group, Servo and the task bridge (joystick driving).
  teleop:=false    controllers only - for scripts/run_jog.sh single-joint tests.
  servo_acc:=N     servo acceleration 1-254 for every joint (default: the driver's 100). Lower = the
                   servo eases into each new goal instead of snapping to it (smoother, slightly laggier).
                   Applied by a forward_command_controller on the 'acceleration' interface.
Optional per-joint keys in the calibration file, written to servo EEPROM at start-up and kept after:
  p_coefficient, i_coefficient, d_coefficient (position PID, 0-255), deadband (0-255 counts).
  calibration:=/path/to/sts_calibration.yaml   (default: the installed config/sts_calibration.yaml)
  port:=/dev/serial/by-id/...                  overrides serial_port from the calibration file
  rviz:=true|false

Run through scripts/run_real.sh (ROS domain 88, same as the input stack).
"""
import json
import math
import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch_ros.actions import Node

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
PLUGIN = 'sts_hardware_interface/STSHardwareInterface'
READONLY_DEFAULT_CENTER = 2048   # also used by tools/sts_calibration_monitor.py
STEPS_PER_RAD = 4096 / (2 * math.pi)
SERVO_EEPROM_KEYS = ('p_coefficient', 'i_coefficient', 'd_coefficient', 'deadband')


def urdf_limits(urdf_text):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    return {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
            for j in root.findall('joint') if j.get('name') in JOINTS}


def readonly_center(j):
    """center_steps, else csv zero + shift (reads 0 at the physical 0 deg mark), else 2048."""
    if isinstance(j.get('center_steps'), int):
        return j['center_steps']
    if isinstance(j.get('csv_zero_counts'), int):
        return j['csv_zero_counts'] + int(j.get('encoder_shift_counts') or 0)
    return READONLY_DEFAULT_CENTER


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text())
    joints = data.get('joints') or {}
    missing = [j for j in JOINTS if j not in joints]
    if missing:
        raise RuntimeError(f'{path}: missing joints {missing}')
    return data


def calibration_problems(data, limits):
    """List of reasons the calibration is not complete enough to drive the motors."""
    problems = []
    ids = []
    for name in JOINTS:
        j = data['joints'][name]
        ids.append(j.get('motor_id'))
        c, lo, hi = j.get('center_steps'), j.get('min_rad'), j.get('max_rad')
        if not isinstance(c, int) or not 0 <= c <= 4095:
            problems.append(f'{name}: center_steps must be an integer 0-4095 (is {c!r})')
        if not all(isinstance(v, (int, float)) for v in (lo, hi)):
            problems.append(f'{name}: min_rad/max_rad not set')
        elif not -math.pi < lo < hi < math.pi:
            problems.append(f'{name}: need -pi < min_rad < max_rad < pi (is {lo}, {hi})')
        elif isinstance(c, int):
            u_lo, u_hi = limits[name]
            if not lo < u_lo < u_hi < hi:
                problems.append(f'{name}: driver limits {lo}..{hi} must enclose the URDF limit {u_lo}..{u_hi} '
                                '(rerun tools/fit_calibration.py --write after editing the URDF)')
            raw_lo, raw_hi = c - hi * STEPS_PER_RAD, c - lo * STEPS_PER_RAD
            if raw_lo < 0 or raw_hi > 4095:
                problems.append(f'{name}: travel {raw_lo:.0f}..{raw_hi:.0f} counts crosses the encoder wrap')
    if sorted(ids) != sorted(set(ids)) or not all(isinstance(i, int) and 1 <= i <= 253 for i in ids):
        problems.append(f'motor_id values must be distinct integers 1-253 (are {ids})')
    return problems


def control_block(data, port, readonly, acc=None):
    """The <ros2_control> element for the STS driver."""
    hw = [
        f'      <plugin>{PLUGIN}</plugin>',
        f'      <param name="serial_port">{port}</param>',
        f'      <param name="baud_rate">{int(data.get("baud_rate", 1000000))}</param>',
        '      <param name="use_sync_write">true</param>',
        '      <param name="reset_states_on_activate">false</param>',
        # seed each servo's goal from where it actually is, so torque-on does not jump
        '      <param name="hold_position_on_activate">true</param>',
    ]
    lines = ['  <ros2_control name="IronSTS" type="system">', '    <hardware>', *hw, '    </hardware>']
    for name in JOINTS:
        j = data['joints'][name]
        center = j['center_steps'] if not readonly else readonly_center(j)
        lines += [f'    <joint name="{name}">',
                  f'      <param name="motor_id">{int(j["motor_id"])}</param>',
                  '      <param name="operating_mode">0</param>',
                  f'      <param name="position_center_steps">{center}</param>']
        if not readonly:
            # limits clamp both commands and the reported position, so leave them out while calibrating
            lines += [f'      <param name="min_position">{float(j["min_rad"])!r}</param>',
                      f'      <param name="max_position">{float(j["max_rad"])!r}</param>',
                      '      <command_interface name="position"/>']
            if acc is not None:
                lines.append('      <command_interface name="acceleration"/>')
            for key in SERVO_EEPROM_KEYS:
                value = j.get(key)
                if value is not None:
                    if not isinstance(value, int) or not 0 <= value <= 255:
                        raise RuntimeError(f'{name}: {key} must be an integer 0-255 (is {value!r})')
                    lines.append(f'      <param name="{key}">{value}</param>')
        # no command interface = read-only joint: the driver keeps torque disabled
        lines += ['      <state_interface name="position"/>',
                  '      <state_interface name="velocity"/>',
                  '      <state_interface name="effort"/>',
                  '    </joint>']
    lines.append('  </ros2_control>')
    return '\n'.join(lines)


def real_description(urdf_text, data, port, readonly, acc=None):
    """iron.urdf with its mock <ros2_control> block replaced by the STS one."""
    pattern = re.compile(r'[ \t]*<ros2_control\b.*?</ros2_control>', re.S)
    if len(pattern.findall(urdf_text)) != 1:
        raise RuntimeError('iron.urdf must contain exactly one <ros2_control> block')
    return pattern.sub(lambda _: control_block(data, port, readonly, acc), urdf_text)


def setup(context):
    cfg = context.launch_configurations
    readonly = cfg['readonly'].lower() in ('1', 'true', 'yes')
    p = Path(get_package_share_directory('joint_effort_iron'))
    cal_path = cfg['calibration'] or str(p / 'config/sts_calibration.yaml')
    data = load_calibration(cal_path)
    port = cfg['port'] or data['serial_port']
    teleop = cfg['teleop'].lower() in ('1', 'true', 'yes')
    acc = None
    if cfg['servo_acc'].strip() and not readonly:
        acc = int(cfg['servo_acc'])
        if not 1 <= acc <= 254:
            raise RuntimeError('servo_acc must be 1-254 (0 makes STS servos creep and ignore speed)')
    urdf_text = (p / 'urdf/iron.urdf').read_text()
    if not readonly:
        problems = calibration_problems(data, urdf_limits(urdf_text))
        if problems:
            raise RuntimeError('Calibration incomplete; use readonly:=true until it is filled in:\n  '
                               + '\n  '.join(problems))
    description = real_description(urdf_text, data, port, readonly, acc)
    centers = {n: (data['joints'][n]['center_steps'] if not readonly else readonly_center(data['joints'][n]))
               for n in JOINTS}
    print(f'[real.launch] {"READ-ONLY (torque off)" if readonly else "DRIVING MOTORS"}'
          f'{"" if readonly or teleop else " (controllers only, teleop:=false)"} '
          f'on {port}, calibration {cal_path}, centers {centers}'
          f'{"" if acc is None else f", servo_acc {acc}"}')

    rviz_home = Path(os.environ['ROS_HOME']).parent / 'iron_rviz_home'
    (rviz_home / '.rviz2').mkdir(parents=True, exist_ok=True)
    common = [{'robot_description': description,
               'robot_description_semantic': (p / 'config/iron.srdf').read_text(),
               'robot_description_kinematics': json.loads((p / 'config/kinematics.yaml').read_text()),
               'robot_description_planning': json.loads((p / 'config/joint_limits.yaml').read_text())}]
    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '30'], output='screen')

    js = spawner('joint_state_broadcaster')
    controller = spawner('iron_arm_controller')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        # gets the (STS) robot_description from robot_state_publisher's topic, as in mock.launch.py
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(p / 'config/controllers.yaml')], output='screen'),
        RegisterEventHandler(OnProcessExit(
            target_action=js,
            on_exit=lambda event, context: ([] if readonly else [controller]) if event.returncode == 0 else
            [EmitEvent(event=Shutdown(reason='Joint broadcaster failed'))])),
    ]
    if acc is not None:
        acc_spawner = spawner('iron_acc_controller')
        set_acc = ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/iron_acc_controller/commands',
                                      'std_msgs/msg/Float64MultiArray', f'{{data: {[float(acc)] * len(JOINTS)}}}'],
                                 output='screen')
        nodes += [
            RegisterEventHandler(OnProcessExit(
                target_action=controller,
                on_exit=lambda event, context: [acc_spawner] if event.returncode == 0 else [])),
            RegisterEventHandler(OnProcessExit(
                target_action=acc_spawner,
                on_exit=lambda event, context: [set_acc] if event.returncode == 0 else
                [EmitEvent(event=Shutdown(reason='Acceleration controller failed'))])),
    ]
    if not (not readonly and teleop):
        nodes.append(js)
    else:
        # MoveIt 2.12.4 Servo only starts once its planning-scene monitor's last update is (a) on the
        # ROS clock and (b) newer than Servo's start. Joint-state *changes* set it on the ROS clock;
        # every planning-scene diff from move_group resets it to the SYSTEM clock. With a still arm
        # that leaves Servo "Waiting to receive robot state update" forever (the mock wins this race
        # by luck). So: Servo keeps its own scene (primary; table/PCB/fixture come from the URDF) and
        # ignores move_group's diffs, and joint_state_broadcaster starts only once Servo is waiting,
        # so the first joint state always counts as a fresh update.
        servo_params = json.loads((p / 'config/servo.yaml').read_text())
        servo_params['moveit_servo']['is_primary_planning_scene_monitor'] = True
        servo_params['moveit_servo']['monitored_planning_scene_topic'] = '/iron_servo/unused_external_scene'
        servo = Node(package='joint_effort_moveit', executable='servo_lifetime', name='iron_servo',
                     parameters=common + [servo_params], output='screen')
        js_started = []

        def start_js(event):
            if js_started or b'Waiting to receive robot state update' not in event.text:
                return []
            js_started.append(True)
            return [js]
        nodes += [
            RegisterEventHandler(OnProcessIO(target_action=servo, on_stdout=start_js, on_stderr=start_js)),
            Node(package='joint_effort_moveit', executable='move_group_lifetime', parameters=common + [{
                'publish_robot_description_semantic': True, 'publish_planning_scene': True,
                'publish_geometry_updates': True, 'publish_state_updates': True,
                'publish_transforms_updates': True, 'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                'ompl': {'planning_plugins': ['ompl_interface/OMPLPlanner'],
                         'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                                              'default_planning_request_adapters/ValidateWorkspaceBounds',
                                              'default_planning_request_adapters/CheckStartStateBounds',
                                              'default_planning_request_adapters/CheckStartStateCollision'],
                         'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                                               'default_planning_response_adapters/ValidateSolution']},
                'moveit_controller_manager': 'joint_effort_moveit/NoExecutionControllerManager',
                'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
                'allow_trajectory_execution': False}], output='screen'),
            servo,
            Node(package='joint_effort_iron', executable='iron_task_bridge', output='screen'),
        ]
    if cfg['rviz'].lower() in ('1', 'true', 'yes'):
        nodes.append(Node(package='joint_effort_cad_review', executable='cad_rviz',
                          arguments=['-d', str(p / 'config/iron.rviz')],
                          additional_env={'HOME': str(rviz_home)}, output='screen'))
    return nodes


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('Use scripts/run_real.sh: the iron arm runs on ROS domain 88.')
    return LaunchDescription([
        DeclareLaunchArgument('readonly', default_value='false'),
        DeclareLaunchArgument('calibration', default_value=''),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('servo_acc', default_value=''),
        OpaqueFunction(function=setup),
    ])
