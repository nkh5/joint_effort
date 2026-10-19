import copy
import importlib.util
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from joint_effort_hardware.configuration import read, description


def start(context):
    emulated = LaunchConfiguration('emulated_bus').perform(context) == 'true'
    if os.environ.get('ROS_DOMAIN_ID') != ('90' if emulated else '89'):
        raise RuntimeError('Physical arm requires domain 89; pseudo-terminal tests require domain 90')
    iron = Path(get_package_share_directory('joint_effort_iron'))
    config = read(LaunchConfiguration('calibration').perform(context), iron/'urdf/iron.urdf')
    allow = LaunchConfiguration('allow_motor_writes').perform(context)
    if allow not in ('true', 'false'): raise ValueError('allow_motor_writes must be true or false')
    xml = description(config, iron/'urdf/iron.urdf', allow == 'true', emulated)
    work = Path(os.environ['ROS_HOME']).parent/('hardware-emulated-runtime' if emulated else 'hardware')
    work.mkdir(parents=True, exist_ok=True)
    model_path = work/'iron_measured.urdf'; model_path.write_text(xml)
    controllers = json.loads((iron/'config/controllers.yaml').read_text())
    controllers['controller_manager']['ros__parameters']['hardware_components_initial_state'] = {'inactive': ['IronSTS']}
    controllers['iron_arm_controller']['ros__parameters'].update(
        interpolate_from_desired_state=False, set_last_command_interface_value_as_state_on_activation=False,
        allow_nonzero_velocity_at_trajectory_end=True)
    path = work/'controllers.json'; path.write_text(json.dumps(controllers))
    limits = copy.deepcopy(json.loads((iron/'config/joint_limits.yaml').read_text()))
    for row in config['joints']:
        limits['joint_limits'][row['name']]['max_velocity'] = row['max_velocity_rad_s']
        limits['joint_limits'][row['name']]['max_acceleration'] = .05
    spec = importlib.util.spec_from_file_location('iron_launch_common', iron/'launch/mock.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.build_launch(xml, str(path), [{'runtime_profile': 'hardware', 'robot_path': str(model_path),
        'hardware_calibration': LaunchConfiguration('calibration').perform(context), 'emulated_bus': emulated}],
        physical=True, planning_limits=limits).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration', description='Absolute path to measured iron hardware JSON'),
        DeclareLaunchArgument('allow_motor_writes', default_value='false'),
        DeclareLaunchArgument('emulated_bus', default_value='false'),
        OpaqueFunction(function=start)])
