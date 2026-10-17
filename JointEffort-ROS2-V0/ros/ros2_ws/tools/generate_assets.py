#!/usr/bin/env python3
"""Generate committed ROS assets from model.json. No ROS dependency."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/joint_effort_robot'
MODEL = json.loads((PKG / 'config/model.json').read_text())


def nums(values):
    return ' '.join(f'{v:.10g}' for v in values)


def elem(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})


def shape(link, kind, dimensions, center, color, rpy=(0, 0, 0)):
    for tag in ['visual', 'collision']:
        item = elem(link, tag)
        elem(item, 'origin', xyz=nums(center), rpy=nums(rpy))
        geometry = elem(item, 'geometry')
        if kind == 'box':
            elem(geometry, 'box', size=nums(dimensions))
        else:
            elem(geometry, 'cylinder', radius=dimensions[0], length=dimensions[1])
        if tag == 'visual':
            material = elem(item, 'material', name=f'{link.get("name")}_{len(link)}')
            elem(material, 'color', rgba=color)


def inertial(link, mass, size):
    # Placeholder positive inertias; not a dynamics model or measured CAD data.
    item = elem(link, 'inertial')
    elem(item, 'origin', xyz='0 0 0')
    elem(item, 'mass', value=mass)
    x, y, z = size
    elem(item, 'inertia', ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
         izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0)


def main():
    m = MODEL
    robot = ET.Element('robot', name=m['robot_name'])
    elem(robot, 'link', name='world')
    joints = []
    def link(name, mass=0.1, size=(0.05, 0.05, 0.05)):
        result = elem(robot, 'link', name=name)
        inertial(result, mass, size)
        return result
    def joint(name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0), axis=None, limits=None):
        item = elem(robot, 'joint', name=name, type='revolute' if axis is not None else 'fixed')
        elem(item, 'parent', link=parent)
        elem(item, 'child', link=child)
        elem(item, 'origin', xyz=nums(xyz), rpy=nums(rpy))
        if axis is not None:
            elem(item, 'axis', xyz=nums(axis))
            elem(item, 'limit', lower=limits[0], upper=limits[1],
                 velocity=m['mock_max_joint_velocity_rad_s'], effort=m['mock_effort_limit_nm'])
            joints.append(name)
        return item
    table = link('table', 10, m['table']['size_m'])
    shape(table, 'box', m['table']['size_m'], (0, 0, 0), '0.55 0.60 0.65 1')
    joint('table_fixed', 'world', 'table', m['table']['center_m'])
    surface = m['pcb']['surface_center_m']
    pcb = link('pcb', 0.05, m['pcb']['size_m'])
    shape(pcb, 'box', m['pcb']['size_m'], [0, 0, -m['pcb']['size_m'][2]/2], '0.15 0.55 0.25 1')
    joint('pcb_fixed', 'world', 'pcb', surface)
    fixture = link('pcb_fixture', 0.3, (0.10, 0.05, 0.03))
    shape(fixture, 'box', (0.10, 0.05, surface[2]-0.003), (0, 0, (surface[2]-0.003)/2), '0.35 0.38 0.42 1')
    joint('fixture_fixed', 'world', 'pcb_fixture', (surface[0], surface[1], 0))
    elem(robot, 'link', name='pcb_workspace')
    joint('workspace_fixed', 'pcb', 'pcb_workspace')
    for arm, base in m['arm_bases'].items():
        tool_offset = m.get('tool_offsets_by_arm_m', {}).get(arm, m['tool_offset_m'])
        color = '0.12 0.50 0.80 1' if arm == 'iron' else '0.90 0.45 0.10 1'
        base_link = link(f'{arm}_base')
        shape(base_link, 'box', (0.12, 0.10, 0.006), (0, 0, 0.003), color)
        joint(f'{arm}_mount', 'world', f'{arm}_base', base['xyz'], (0, 0, base['yaw_rad']))
        turret = link(f'{arm}_turret', 0.2)
        shape(turret, 'cylinder', (0.028, m['shoulder_height_m']-0.035), (0, 0, 0.006 + (m['shoulder_height_m']-0.035)/2), color)
        joint(f'{arm}_base_yaw', f'{arm}_base', f'{arm}_turret', axis=m.get('joint_axes_local', {}).get('base_yaw', [0,0,1]), limits=m['joint_limits_rad'][0])
        lengths = [m['upper_arm_m'], m['forearm_m'], m['wrist_bridge_m']]
        parents = ['turret', 'upper_arm', 'forearm']
        children = ['upper_arm', 'forearm', 'wrist_bridge']
        for i, (parent, child, length) in enumerate(zip(parents, children, lengths), 1):
            body = link(f'{arm}_{child}', 0.10, (length, 0.012, 0.020))
            shape(body, 'box', (max(length-0.03, 0.015), 0.012, 0.020), (length/2, 0, 0), color)
            shape(body, 'cylinder', (0.020, 0.045), (0, 0, 0), '0.35 0.38 0.42 1', (math.pi/2, 0, 0))
            # Motor envelope shown as part of this collision link, with nominal clearance.
            shape(body, 'box', (0.04523, 0.02473, 0.035), (0, -0.043, 0), '0.20 0.22 0.25 1')
            xyz = (0, 0, m['shoulder_height_m']) if i == 1 else (lengths[i-2], 0, 0)
            joint(f'{arm}_{m["joint_suffixes"][i]}', f'{arm}_{parent}', f'{arm}_{child}', xyz,
                  axis=m.get('joint_axes_local', {}).get(m['joint_suffixes'][i], [0,-1,0]), limits=m['joint_limits_rad'][i])
        tool = link(f'{arm}_tool', 0.15, (0.12, 0.036, 0.03))
        if arm == 'iron':
            shape(tool, 'box', (0.08, 0.036, 0.03), (0.01, 0, 0), color)
        else:
            shape(tool, 'box', (0.035, 0.020, 0.020), (0.01, 0, 0), color)
        shape(tool, 'cylinder', (0.007 if arm == 'iron' else 0.003, tool_offset-0.035),
              ((tool_offset+0.025)/2, 0, 0), '0.65 0.67 0.70 1', (0, math.pi/2, 0))
        joint(f'{arm}_wrist_yaw', f'{arm}_wrist_bridge', f'{arm}_tool', (m['wrist_bridge_m'], 0, 0),
              axis=m.get('joint_axes_local', {}).get('wrist_yaw', [0,0,1]), limits=m['joint_limits_rad'][4])
        tip = link(f'{arm}_tip', 0.005, (0.005, 0.006, 0.006))
        shape(tip, 'cylinder', (0.003 if arm == 'iron' else 0.001, 0.005), (-0.0025, 0, 0), color, (0, math.pi/2, 0))
        joint(f'{arm}_tip_fixed', f'{arm}_tool', f'{arm}_tip', (tool_offset, 0, 0))
        elem(robot, 'link', name=f'{arm}_tcp')
        joint(f'{arm}_tcp_fixed', f'{arm}_tip', f'{arm}_tcp')
    control = elem(robot, 'ros2_control', name='JointEffortMock', type='system')
    hardware = elem(control, 'hardware')
    elem(hardware, 'plugin').text = 'mock_components/GenericSystem'
    for arm in ['iron', 'wire']:
        for i, suffix in enumerate(m['joint_suffixes']):
            item = elem(control, 'joint', name=f'{arm}_{suffix}')
            elem(item, 'command_interface', name='position')
            state = elem(item, 'state_interface', name='position')
            elem(state, 'param', name='initial_value').text = str(m.get('initial_positions_by_arm_rad', {}).get(arm, m['initial_positions_rad'])[i])
            elem(item, 'state_interface', name='velocity')
    ET.indent(robot)
    (PKG / 'urdf').mkdir(exist_ok=True)
    ET.ElementTree(robot).write(PKG / 'urdf/joint_effort.urdf', encoding='utf-8', xml_declaration=True)
    semantic = ET.Element('robot', name=m['robot_name'])
    for arm in ['iron', 'wire']:
        group = elem(semantic, 'group', name=f'{arm}_arm')
        elem(group, 'chain', base_link=f'{arm}_base', tip_link=f'{arm}_tcp')
        pose = elem(semantic, 'group_state', name='park', group=f'{arm}_arm')
        for i, suffix in enumerate(m['joint_suffixes']):
            elem(pose, 'joint', name=f'{arm}_{suffix}', value=m.get('initial_positions_by_arm_rad', {}).get(arm, m['initial_positions_rad'])[i])
    both = elem(semantic, 'group', name='both_arms')
    elem(both, 'group', name='iron_arm')
    elem(both, 'group', name='wire_arm')
    # Adjacent links only: there is no blanket arm/PCB or inter-arm exemption.
    for j in robot.findall('joint'):
        elem(semantic, 'disable_collisions', link1=j.find('parent').get('link'),
             link2=j.find('child').get('link'), reason='Adjacent')
    for arm in ['iron', 'wire']:
        elem(semantic, 'disable_collisions', link1=f'{arm}_base', link2='table', reason='Mount contact')
        # The coaxial yaw cylinder's height/radius are invariant under all joints.
        elem(semantic, 'disable_collisions', link1=f'{arm}_turret', link2='table', reason='Invariant mounting clearance')
    elem(semantic, 'disable_collisions', link1='table', link2='pcb_fixture', reason='Fixture contact')
    # Both objects are fixed to world; no arm command can change this 1 mm gap.
    elem(semantic, 'disable_collisions', link1='pcb', link2='pcb_fixture', reason='Fixed fixture clearance')
    ET.indent(semantic)
    ET.ElementTree(semantic).write(PKG / 'config/joint_effort.srdf', encoding='utf-8', xml_declaration=True)
    controllers = {'controller_manager': {'ros__parameters': {'update_rate': 50,
                   'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}}}
    for arm in ['iron', 'wire']:
        name = f'{arm}_arm_controller'
        controllers['controller_manager']['ros__parameters'][name] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
        controllers[name] = {'ros__parameters': {'joints': [f'{arm}_{s}' for s in m['joint_suffixes']],
                            'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity'],
                            'allow_partial_joints_goal': False, 'interpolate_from_desired_state': False,
                            'allow_nonzero_velocity_at_trajectory_end': True}}
    # JSON is a YAML subset and avoids a generator dependency on PyYAML.
    (PKG / 'config/controllers.yaml').write_text(json.dumps(controllers, indent=2)+'\n')
    limits = {'joint_limits': {name: {'has_velocity_limits': True, 'max_velocity': m['mock_max_joint_velocity_rad_s'],
              'has_acceleration_limits': True, 'max_acceleration': m['mock_max_joint_acceleration_rad_s2']} for name in joints}}
    (PKG / 'config/joint_limits.yaml').write_text(json.dumps(limits, indent=2)+'\n')
    kinematics = {f'{arm}_arm': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                  'kinematics_solver_timeout': 0.05, 'position_only_ik': True} for arm in ['iron', 'wire']}
    (PKG / 'config/kinematics.yaml').write_text(json.dumps(kinematics, indent=2)+'\n')
    for arm in ['iron', 'wire']:
        servo = {'moveit_servo': {'move_group_name': f'{arm}_arm', 'publish_period': 0.02,
                 'command_in_type': 'speed_units', 'incoming_command_timeout': 0.12,
                 'max_expected_latency': 0.08,
                 'joint_command_in_topic': f'/{arm}_servo/delta_joint_cmds',
                 'cartesian_command_in_topic': f'/{arm}_servo/experimental_twist_disabled',
                 'pose_command_in_topic': f'/{arm}_servo/experimental_pose_disabled',
                 'command_out_topic': f'/{arm}_arm_controller/joint_trajectory',
                 'command_out_type': 'trajectory_msgs/JointTrajectory',
                 'publish_joint_positions': True, 'publish_joint_velocities': True,
                 'publish_joint_accelerations': False, 'joint_topic': '/joint_states',
                 'monitored_planning_scene_topic': '/monitored_planning_scene',
                 'is_primary_planning_scene_monitor': False, 'check_collisions': True,
                 'collision_check_rate': 25.0, 'self_collision_proximity_threshold': 0.008,
                 'scene_collision_proximity_threshold': 0.008, 'joint_limit_margins': [0.03],
                 'use_smoothing': True}}
        (PKG / f'config/{arm}_servo.yaml').write_text(json.dumps(servo, indent=2)+'\n')
    print('Generated URDF, SRDF, controller, joint-limit, kinematics and Servo configurations')


if __name__ == '__main__':
    main()
