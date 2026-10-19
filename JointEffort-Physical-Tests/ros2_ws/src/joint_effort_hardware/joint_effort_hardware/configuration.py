"""Check measured settings before creating a physical robot description."""
import hashlib
import json
import math
import os
import stat
from pathlib import Path
import xml.etree.ElementTree as ET
from amw_pi_bridge.contract import validate_config, number

PLUGIN = 'joint_effort_hardware/IronSystem'
CONFIRMATIONS = ('calibration_confirmed', 'motor_voltage_confirmed',
                 'workspace_confirmed', 'stop_arrangement_confirmed')


def validate(config, urdf):
    validate_config(config, urdf)
    for key in CONFIRMATIONS:
        if config.get(key) is not True:
            raise ValueError(f'{key} is not confirmed')
    if config.get('urdf_sha256') != hashlib.sha256(Path(urdf).read_bytes()).hexdigest():
        raise ValueError('Calibration does not match this URDF; review geometry and recalibrate')
    if config['baud_rate'] != 1000000:
        raise ValueError('Physical driver currently supports only 1 Mbaud')
    for key in ('voltage_min_v', 'voltage_max_v', 'supply_voltage_v', 'max_temperature_c'):
        if not number(config.get(key)):
            raise ValueError(f'Measured/label-verified {key} is required')
    if not 3 <= config['voltage_min_v'] < config['supply_voltage_v'] <= config['voltage_max_v'] <= 14:
        raise ValueError('Supply must fit the verified motor voltage range')
    if not 25 <= config['max_temperature_c'] <= 60:
        raise ValueError('Use a conservative temperature limit, 25..60 C')
    for row in config['joints']:
        if row['gear_ratio'] > 100:
            raise ValueError('gear_ratio is the external ratio, not the internal motor reduction')
        if not .002 <= row['max_velocity_rad_s'] <= .05:
            raise ValueError('First hardware profile is limited to 0.002..0.05 rad/s')
        tolerance = row.get('max_tracking_error_rad')
        quantum = math.tau/(4096*row['gear_ratio'])
        if not number(tolerance) or not quantum <= tolerance <= .05:
            raise ValueError('Set max_tracking_error_rad between one encoder count and 0.05 rad')
        if math.floor(row['max_velocity_rad_s']*4096*row['gear_ratio']/math.tau) < 1:
            raise ValueError('Native speed must be at least one count/s; zero means unlimited')
    return config


def read(path, urdf):
    return validate(json.loads(Path(path).read_text()), urdf)


def emulator_port(path):
    target = Path(path).resolve()
    if target.parent != Path('/dev/pts') or not target.name.isdigit():
        raise ValueError('Emulation requires a /dev/pts pseudo-terminal; real adapters are forbidden')
    device = target.stat()
    if not stat.S_ISCHR(device.st_mode) or not 136 <= os.major(device.st_rdev) <= 143:
        raise ValueError('Emulation requires a Linux pseudo-terminal slave')


def description(config, urdf, allow_writes=False, emulated=False):
    validate(config, urdf)
    if emulated: emulator_port(config['serial_port'])
    root = ET.parse(urdf).getroot()
    control = root.find('ros2_control')
    if control is None:
        raise ValueError('Missing ros2_control in iron URDF')
    root.remove(control)
    control = ET.SubElement(root, 'ros2_control', name='IronSTS', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = PLUGIN
    settings = {k: config[k] for k in (*CONFIRMATIONS, 'serial_port', 'baud_rate',
                'voltage_min_v', 'voltage_max_v', 'max_temperature_c')}
    settings['allow_motor_writes'] = bool(allow_writes)
    settings['emulated_bus'] = bool(emulated)
    for key, value in settings.items():
        ET.SubElement(hardware, 'param', name=key).text = str(value).lower() if type(value) is bool else str(value)
    by_name = {j.get('name'): j for j in root.findall('joint')}
    for row in sorted(config['joints'], key=lambda r: r['name']):
        limit = by_name[row['name']].find('limit')
        limit.set('lower', str(row['min_rad'])); limit.set('upper', str(row['max_rad']))
        limit.set('velocity', str(row['max_velocity_rad_s']))
        joint = ET.SubElement(control, 'joint', name=row['name'])
        for key, value in row.items():
            if key != 'name': ET.SubElement(joint, 'param', name=key).text = str(value)
        ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='velocity')
    return ET.tostring(root, encoding='unicode')
