"""Static checks that would otherwise only fail at launch time. No ROS needed."""
import ast
import py_compile
import re
import unittest
import xml.etree.ElementTree as ET

import yaml

import _paths

WS = _paths.WS
SRC = WS / 'src'
LAUNCH = SRC / 'amw_bringup/launch/amw.launch.py'
OUR_PACKAGES = ['amw_bringup', 'amw_description', 'amw_moveit', 'amw_teleop', 'amw_tools']


def executables(package):
    """console_scripts of a Python package, or install(TARGETS ...) of a CMake package."""
    setup = SRC / package / 'setup.py'
    if setup.exists():
        return set(re.findall(r"'(\w+) = ", setup.read_text()))
    cmake = (SRC / package / 'CMakeLists.txt').read_text()
    return set(re.findall(r'add_executable\((\w+)', cmake))


class WorkspaceTest(unittest.TestCase):
    def test_every_python_file_compiles(self):
        files = [f for f in SRC.rglob('*.py') if 'vendor' not in f.parts and 'sts_hardware_interface' not in f.parts]
        files += [WS / 'amw'] + list((WS / 'tests').glob('*.py'))
        for f in files:
            py_compile.compile(str(f), doraise=True)

    def test_package_names_match_folders(self):
        for name in OUR_PACKAGES:
            root = ET.parse(SRC / name / 'package.xml').getroot()
            self.assertEqual(root.findtext('name'), name)

    def test_launch_executables_exist(self):
        tree = ast.parse(LAUNCH.read_text())
        found = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
                kw = {k.arg: k.value for k in node.keywords}
                if isinstance(kw.get('package'), ast.Constant) and isinstance(kw.get('executable'), ast.Constant):
                    found.append((kw['package'].value, kw['executable'].value))
        ours = [(p, e) for p, e in found if p in OUR_PACKAGES]
        self.assertGreaterEqual(len(ours), 2)          # amw_teleop, servo_lifetime
        for package, exe in ours:
            self.assertIn(exe, executables(package), f'{package} has no executable {exe}')

    def test_config_files_are_yaml_and_installed(self):
        for f in (SRC / 'amw_bringup/config').glob('*.yaml'):
            self.assertIsInstance(yaml.safe_load(f.read_text()), dict, f.name)
        for name in ['arm.yaml', 'controllers.yaml', 'servo.yaml', 'kinematics.yaml', 'teleop.yaml']:
            self.assertIn(name, LAUNCH.read_text() + (SRC / 'amw_bringup/setup.py').read_text() + 'arm.yaml', name)
            self.assertTrue((SRC / 'amw_bringup/config' / name).exists(), name)

    def test_teleop_margin_is_outside_servo_margin(self):
        teleop = yaml.safe_load((SRC / 'amw_bringup/config/teleop.yaml').read_text())['amw_teleop']['ros__parameters']
        servo = yaml.safe_load((SRC / 'amw_bringup/config/servo.yaml').read_text())['moveit_servo']
        self.assertGreater(teleop['joint_limit_margin_rad'], max(servo['joint_limit_margins']),
                           'teleop must stop pushing into a limit before MoveIt Servo halts everything')

    def test_controller_names_agree(self):
        controllers = yaml.safe_load((SRC / 'amw_bringup/config/controllers.yaml').read_text())
        servo = yaml.safe_load((SRC / 'amw_bringup/config/servo.yaml').read_text())['moveit_servo']
        self.assertIn('iron_arm_controller', controllers['controller_manager']['ros__parameters'])
        self.assertTrue(servo['command_out_topic'].startswith('/iron_arm_controller/'))
        srdf = ET.parse(SRC / 'amw_description/srdf/amw.srdf').getroot()
        self.assertEqual(srdf.find('group').get('name'), servo['move_group_name'])

    def test_meshes_referenced_by_urdf_exist(self):
        text = (SRC / 'amw_description/urdf/amw.urdf').read_text()
        for path in re.findall(r'package://amw_description/([^"]+)', text):
            self.assertTrue((SRC / 'amw_description' / path).exists(), path)

    def test_no_old_package_names_left(self):
        pattern = re.compile(r'joint_effort_(iron|input|robot|moveit|cad_review|arm_draft|arm_calibration)'
                             r'|amw_forwarder|iron_task_bridge|joy_teleop|display_state')
        skip = {'archive', 'vendor', 'sts_hardware_interface', 'build', 'install', 'log'}
        for f in WS.rglob('*'):
            if f.is_file() and not skip & set(f.relative_to(WS).parts) and f.suffix in ('.py', '.md', '.yaml', '.xml',
                                                                                       '.txt', '.urdf', '.srdf', '.rviz', ''):
                if f.name == 'test_workspace.py' or f.name.endswith('-arm-check.md'):   # history on purpose
                    continue
                self.assertIsNone(pattern.search(f.read_text(errors='ignore')), f)

    def test_driver_patch_present(self):
        patch = (WS / 'driver/amw_driver.patch').read_text()
        for needle in ('encoder_direction', 'default_acceleration_', 'outside its calibrated range'):
            self.assertIn(needle, patch)


if __name__ == '__main__':
    unittest.main()
