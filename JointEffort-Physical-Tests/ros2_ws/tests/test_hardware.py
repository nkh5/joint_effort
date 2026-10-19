"""Hardware configuration checks. Fixtures describe an emulated bus only."""
import hashlib
import json
import math
import os
import pty
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from joint_effort_hardware.configuration import validate, description, CONFIRMATIONS, emulator_port
from joint_effort_input.core import InputState, Heartbeat

W = Path(__file__).resolve().parents[1]
URDF = W/'src/joint_effort_iron/urdf/iron.urdf'


def fixture():
    return {'schema': 1, **{key: True for key in CONFIRMATIONS},
            'serial_port': '/dev/pts/emulator-only', 'baud_rate': 1000000,
            'urdf_sha256': hashlib.sha256(URDF.read_bytes()).hexdigest(),
            'supply_voltage_v': 12.6, 'voltage_min_v': 9., 'voltage_max_v': 12.8,
            'max_temperature_c': 55,
            'joints': [dict(name=f'joint_{i}', motor_id=i, model_number=777, zero_counts=2048,
                           direction=1, gear_ratio=1., min_counts=1800, max_counts=2300,
                           min_rad=-.3, max_rad=.3, max_velocity_rad_s=.02,
                           max_tracking_error_rad=.02) for i in range(1, 6)]}


class HardwareConfigTests(unittest.TestCase):
    def test_emulation_cannot_select_a_real_adapter(self):
        for path in ['/dev/ttyUSB0', '/dev/null', '/dev/serial/by-id/ADAPTER']:
            with self.assertRaises(ValueError): emulator_port(path)
        master, slave = pty.openpty()
        try: emulator_port(os.ttyname(slave))
        finally: os.close(master); os.close(slave)
    def test_host_autorepeat_cannot_refresh_device_heartbeat(self):
        heartbeat = Heartbeat()
        self.assertFalse(heartbeat.update(0,0.))
        self.assertTrue(heartbeat.update(1,.025))
        self.assertTrue(heartbeat.update(1,.05))
        self.assertFalse(heartbeat.update(1,.20))
        self.assertTrue(heartbeat.update(0,.225))
    def test_template_cannot_enable_hardware(self):
        config = json.loads((W/'src/joint_effort_hardware/config/iron_hardware.template.json').read_text())
        with self.assertRaises(ValueError): validate(config, URDF)

    def test_no_writes_default_and_measured_limits_applied(self):
        root = ET.fromstring(description(fixture(), URDF))
        control = root.find('ros2_control')
        self.assertEqual(control.get('name'), 'IronSTS')
        self.assertEqual(control.find("hardware/param[@name='allow_motor_writes']").text, 'false')
        self.assertEqual(root.find("joint[@name='joint_1']/limit").get('lower'), '-0.3')
        self.assertEqual(control.find("joint[@name='joint_1']/param[@name='zero_counts']").text, '2048')

    def test_each_confirmation_required(self):
        for key in CONFIRMATIONS:
            config = fixture(); config[key] = False
            with self.assertRaises(ValueError, msg=key): validate(config, URDF)

    def test_geometry_supply_tracking_speed_and_duplicate_id(self):
        mutations = [lambda c: c.update(urdf_sha256='wrong'),
                     lambda c: c.update(voltage_max_v=7.4),
                     lambda c: c.update(voltage_max_v=math.nan),
                     lambda c: c['joints'][0].update(max_tracking_error_rad=.2),
                     lambda c: c['joints'][0].update(max_velocity_rad_s=.15),
                     lambda c: c['joints'][0].update(motor_id=2),
                     lambda c: c['joints'][0].update(min_rad=-1),
                     lambda c: c['joints'][0].update(direction=0)]
        for mutation in mutations:
            config = fixture(); mutation(config)
            with self.assertRaises(ValueError): validate(config, URDF)

    def test_input_mode_flag_and_hold_are_separate(self):
        state = InputState(hold=0, toggle=1, mode_select=9)
        def report(now, hold=0, rotate=0, virtual=0):
            buttons = [hold]+[0]*11; buttons[9] = rotate; buttons[7+rotate] = virtual
            return state.update([0.,0.,0.,0.], buttons, int((100+now)*1e9), int((100+now)*1e9), now)
        report(0., rotate=1, virtual=1); report(.1, rotate=1, virtual=1); report(.25, rotate=1, virtual=1)
        self.assertEqual(state.mode, 'rotation'); self.assertTrue(state.prepared)
        self.assertFalse(state.allowed(.25))
        report(.27, hold=1, rotate=1, virtual=1); self.assertTrue(state.allowed(.27))
        report(.28, hold=1, rotate=0, virtual=1); self.assertFalse(state.allowed(.28))


if __name__ == '__main__': unittest.main()
