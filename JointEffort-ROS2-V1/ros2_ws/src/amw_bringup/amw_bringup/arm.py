"""arm.yaml -> joint angles, limits and the robot description.

Pure Python (PyYAML only, no ROS), so the launch file, the `amw` command and the tests all use the
same code. The one formula:

    angle_rad = dir * (raw - zero) * 2*pi / 4096
"""
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
ROLES = {'joint_1': 'base', 'joint_2': 'shoulder', 'joint_3': 'elbow', 'joint_4': 'wrist 1', 'joint_5': 'wrist 2'}
COUNTS_PER_RAD = 4096 / (2 * math.pi)
STS_PLUGIN = 'sts_hardware_interface/STSHardwareInterface'
MOCK_PLUGIN = 'mock_components/GenericSystem'


class ArmConfigError(ValueError):
    """arm.yaml is missing something or is inconsistent. The message says which joint and why."""


@dataclass
class Joint:
    name: str
    motor_id: int
    zero: int
    dir: int
    lower: float          # rad
    upper: float          # rad
    reference: float      # rad
    acceleration: int

    def angle(self, raw):
        """Raw encoder count -> joint angle (rad)."""
        return self.dir * (raw - self.zero) / COUNTS_PER_RAD

    def raw(self, angle):
        """Joint angle (rad) -> raw encoder count (not rounded)."""
        return self.zero + self.dir * angle * COUNTS_PER_RAD

    def zero_for(self, raw, angle):
        """The `zero` that makes this raw count read `angle`."""
        return int(round(raw - self.dir * angle * COUNTS_PER_RAD))


@dataclass
class Arm:
    path: Path
    serial_port: str
    baud_rate: int
    margin: float           # rad
    max_speed: float        # rad/s
    max_accel: float        # rad/s^2
    joints: dict            # name -> Joint

    def __iter__(self):
        return iter(self.joints[n] for n in JOINTS)

    def driver_range(self, j):
        return j.lower - self.margin, j.upper + self.margin


def _number(value, where):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ArmConfigError(f'{where} must be a number (is {value!r})')
    return float(value)


def _integer(value, where, lo, hi):
    if type(value) is not int or not lo <= value <= hi:
        raise ArmConfigError(f'{where} must be a whole number {lo}..{hi} (is {value!r})')
    return value


def load(path):
    """Read and check arm.yaml. Raises ArmConfigError with a specific message on any problem."""
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as e:
        raise ArmConfigError(f'cannot read {path}: {e}') from None
    if not isinstance(data, dict) or not isinstance(data.get('joints'), dict):
        raise ArmConfigError(f'{path}: needs a "joints:" section')
    missing = [n for n in JOINTS if n not in data['joints']]
    if missing:
        raise ArmConfigError(f'{path}: missing joints {missing}')
    joints = {}
    for name in JOINTS:
        j = data['joints'][name] or {}
        where = f'{name}'
        joint = Joint(
            name=name,
            motor_id=_integer(j.get('motor_id'), f'{where}.motor_id', 1, 253),
            zero=_integer(j.get('zero'), f'{where}.zero', 0, 4095),
            dir=j.get('dir'),
            lower=math.radians(_number(j.get('lower_deg'), f'{where}.lower_deg')),
            upper=math.radians(_number(j.get('upper_deg'), f'{where}.upper_deg')),
            reference=math.radians(_number(j.get('reference_deg', 0.0), f'{where}.reference_deg')),
            acceleration=_integer(j.get('acceleration', 100), f'{where}.acceleration', 1, 254))
        if joint.dir not in (-1, 1) or type(joint.dir) is not int:
            raise ArmConfigError(f'{where}.dir must be 1 or -1 (is {j.get("dir")!r})')
        if not joint.lower < joint.upper:
            raise ArmConfigError(f'{where}: lower_deg must be less than upper_deg')
        joints[name] = joint
    ids = [j.motor_id for j in joints.values()]
    if len(set(ids)) != len(ids):
        raise ArmConfigError(f'motor_id values must all be different (are {ids})')
    arm = Arm(path=path,
              serial_port=str(data.get('serial_port') or ''),
              baud_rate=_integer(data.get('baud_rate', 1000000), 'baud_rate', 9600, 1000000),
              margin=math.radians(_number(data.get('limit_margin_deg', 3.0), 'limit_margin_deg')),
              max_speed=math.radians(_number(data.get('max_joint_speed_deg_s'), 'max_joint_speed_deg_s')),
              max_accel=math.radians(_number(data.get('max_joint_accel_deg_s2'), 'max_joint_accel_deg_s2')),
              joints=joints)
    if not arm.serial_port:
        raise ArmConfigError('serial_port is not set')
    for j in arm:
        lo, hi = arm.driver_range(j)
        if not (-math.pi < lo and hi < math.pi):
            raise ArmConfigError(f'{j.name}: limits plus margin must stay within one turn')
        raws = sorted((j.raw(lo), j.raw(hi)))
        if raws[0] < 0 or raws[1] > 4095:
            raise ArmConfigError(
                f'{j.name}: its travel ({math.degrees(lo):+.1f}..{math.degrees(hi):+.1f} deg incl. margin) needs raw '
                f'counts {raws[0]:.0f}..{raws[1]:.0f}, which crosses the encoder wrap at 0/4095. Narrow the limits '
                f'or re-centre the motor (amw motors recenter) and recalibrate its zero.')
    return arm


def update_values(path, joint, values):
    """Change `key: value` lines inside one joint's block of arm.yaml, keeping comments and layout."""
    path = Path(path)
    lines = path.read_text().splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines) if re.match(rf'\s+{joint}:\s*(#.*)?$', l)), None)
    if start is None:
        raise ArmConfigError(f'{path}: no "{joint}:" block')
    indent = len(lines[start]) - len(lines[start].lstrip())
    pending = dict(values)
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.strip() and len(line) - len(line.lstrip()) <= indent:
            break
        m = re.match(r'(\s+)(\w+):(\s*)([^#\n]*?)(\s*)(#.*)?(\n?)$', line)
        if m and m.group(2) in pending:
            value = pending.pop(m.group(2))
            text = f'{value:.3f}' if isinstance(value, float) else str(value)
            lines[i] = f'{m.group(1)}{m.group(2)}:{m.group(3) or " "}{text}{m.group(5) or ""}{m.group(6) or ""}{m.group(7)}'
    if pending:
        raise ArmConfigError(f'{path}: {joint} has no {sorted(pending)} line to update')
    path.write_text(''.join(lines))


# ------------------------------------------------------------------ robot description

def home_pose(srdf_text, state='home'):
    """Joint angles of an SRDF group_state (the mock start pose)."""
    root = ET.fromstring(srdf_text)
    node = root.find(f"group_state[@name='{state}']")
    if node is None:
        raise ArmConfigError(f'SRDF has no group_state "{state}"')
    return {j.get('name'): float(j.get('value')) for j in node.findall('joint')}


def _ros2_control(arm, hardware, readonly, home):
    rc = ET.Element('ros2_control', name='AMW' + ('Mock' if hardware == 'mock' else 'STS'), type='system')
    hw = ET.SubElement(rc, 'hardware')
    ET.SubElement(hw, 'plugin').text = MOCK_PLUGIN if hardware == 'mock' else STS_PLUGIN

    def param(parent, name, value):
        ET.SubElement(parent, 'param', name=name).text = str(value)

    if hardware == 'real':
        param(hw, 'serial_port', arm.serial_port)
        param(hw, 'baud_rate', arm.baud_rate)
        param(hw, 'use_sync_write', 'true')
        param(hw, 'reset_states_on_activate', 'false')
        param(hw, 'hold_position_on_activate', 'true')      # torque on holds where the arm is: no jump
    for j in arm:
        e = ET.SubElement(rc, 'joint', name=j.name)
        if hardware == 'real':
            param(e, 'motor_id', j.motor_id)
            param(e, 'operating_mode', 0)
            param(e, 'position_center_steps', j.zero)
            param(e, 'encoder_direction', j.dir)
            if not readonly:
                lo, hi = arm.driver_range(j)
                param(e, 'min_position', repr(lo))
                param(e, 'max_position', repr(hi))
                param(e, 'acceleration', j.acceleration)
        if not readonly:                                       # no command interface = torque stays off
            ET.SubElement(e, 'command_interface', name='position')
        pos = ET.SubElement(e, 'state_interface', name='position')
        if hardware == 'mock':
            param(pos, 'initial_value', repr(home[j.name]))
        ET.SubElement(e, 'state_interface', name='velocity')
        if hardware == 'real':
            ET.SubElement(e, 'state_interface', name='effort')
    return rc


def robot_description(urdf_text, srdf_text, arm, hardware='mock', readonly=False):
    """amw.urdf with arm.yaml's limits and speed written into every joint, plus the ros2_control block."""
    if hardware not in ('mock', 'real'):
        raise ArmConfigError(f'hardware must be mock or real (is {hardware!r})')
    root = ET.fromstring(urdf_text)
    if root.find('ros2_control') is not None:
        raise ArmConfigError('amw.urdf must not contain a <ros2_control> block (the launch adds it)')
    home = home_pose(srdf_text)
    for j in arm:
        node = root.find(f"joint[@name='{j.name}']")
        if node is None or node.find('limit') is None:
            raise ArmConfigError(f'amw.urdf has no revolute {j.name} with a <limit>')
        limit = node.find('limit')
        limit.set('lower', repr(j.lower))
        limit.set('upper', repr(j.upper))
        limit.set('velocity', repr(arm.max_speed))
        if not j.lower <= home[j.name] <= j.upper:
            raise ArmConfigError(f'SRDF home pose puts {j.name} at {math.degrees(home[j.name]):+.1f} deg, '
                                 f'outside its arm.yaml limits')
    root.append(_ros2_control(arm, hardware, readonly, home))
    ET.indent(root, '  ')
    return ET.tostring(root, encoding='unicode')


def planning_limits(arm):
    """robot_description_planning parameter for MoveIt Servo."""
    return {'joint_limits': {j.name: {'has_velocity_limits': True, 'max_velocity': arm.max_speed,
                                      'has_acceleration_limits': True, 'max_acceleration': arm.max_accel}
                             for j in arm}}
