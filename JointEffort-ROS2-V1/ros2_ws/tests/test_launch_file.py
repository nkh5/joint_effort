"""Run amw.launch.py's setup() with stand-in launch classes, so its logic is checked without ROS:
which nodes start in each mode, their parameters, and the preflight step for the real arm."""
import importlib.util
import sys
import types
import unittest

import _paths

SRC = _paths.WS / 'src'


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs

    def __repr__(self):
        return f'{type(self).__name__}({self.kwargs})'


def install_stubs():
    """Minimal fake `launch`, `launch_ros` and `ament_index_python` modules."""
    names = ['DeclareLaunchArgument', 'EmitEvent', 'ExecuteProcess', 'LogInfo', 'OpaqueFunction',
             'RegisterEventHandler', 'TimerAction', 'OnProcessExit', 'OnProcessIO', 'Shutdown', 'Node',
             'LaunchDescription']
    classes = {n: type(n, (Action,), {}) for n in names}
    mods = {
        'launch': {'LaunchDescription': classes['LaunchDescription']},
        'launch.actions': {n: classes[n] for n in names[:7]},
        'launch.event_handlers': {'OnProcessExit': classes['OnProcessExit'], 'OnProcessIO': classes['OnProcessIO']},
        'launch.events': {'Shutdown': classes['Shutdown']},
        'launch_ros': {},
        'launch_ros.actions': {'Node': classes['Node']},
        'ament_index_python': {},
        'ament_index_python.packages': {'get_package_share_directory': lambda pkg: str(SRC / pkg)},
    }
    saved = {name: sys.modules.get(name) for name in mods}
    for name, attrs in mods.items():
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
    return classes, saved


class LaunchFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.classes, cls.saved = install_stubs()
        spec = importlib.util.spec_from_file_location('amw_launch', SRC / 'amw_bringup/launch/amw.launch.py')
        cls.launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.launch)

    @classmethod
    def tearDownClass(cls):
        for name, module in cls.saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def run_setup(self, **args):
        cfg = dict(hardware='mock', readonly='false', teleop='true', joystick='true', rviz='false',
                   port='', arm_config='')
        cfg.update(args)
        return self.launch.setup(types.SimpleNamespace(launch_configurations=cfg))

    def nodes(self, actions):
        """All Node actions, including those started later by event handlers (preflight, Servo wait)."""
        out = []
        for a in actions:
            if type(a).__name__ == 'Node':
                out.append(a)
            if type(a).__name__ == 'RegisterEventHandler':
                handler = a.args[0]
                if type(handler).__name__ == 'OnProcessExit' and 'preflight' in repr(handler.kwargs['target_action']):
                    out += self.nodes(handler.kwargs['on_exit'](types.SimpleNamespace(returncode=0), None))
        return out

    def executables(self, actions):
        return sorted(n.kwargs['executable'] for n in self.nodes(actions))

    def test_declares_arguments(self):
        ld = self.launch.generate_launch_description()
        declared = [a.args[0] for a in ld.args[0] if type(a).__name__ == 'DeclareLaunchArgument']
        self.assertEqual(declared, ['hardware', 'readonly', 'teleop', 'joystick', 'rviz', 'port', 'arm_config'])

    def test_mock_teleop_starts_the_full_stack(self):
        exes = self.executables(self.run_setup())
        for exe in ['robot_state_publisher', 'ros2_control_node', 'servo_lifetime', 'amw_teleop', 'amw_joycon']:
            self.assertIn(exe, exes)
        self.assertNotIn('rviz2', exes)
        self.assertNotIn('amw_joycon', self.executables(self.run_setup(joystick='false')))

    def test_teleop_gets_the_description_and_teleop_yaml(self):
        nodes = {n.kwargs['executable']: n for n in self.nodes(self.run_setup())}
        yaml_file, params = nodes['amw_teleop'].kwargs['parameters']
        self.assertTrue(yaml_file.endswith('config/teleop.yaml'))
        self.assertIn('<ros2_control', params['robot_description'])
        self.assertAlmostEqual(params['max_joint_velocity_rad_s'], 0.15, places=4)
        self.assertEqual(nodes['amw_teleop'].kwargs['name'], 'amw_teleop')
        self.assertTrue(nodes['amw_joycon'].kwargs['parameters'][0].endswith('config/teleop.yaml'))
        servo = nodes['servo_lifetime'].kwargs['parameters']
        self.assertTrue(servo[1]['moveit_servo']['is_primary_planning_scene_monitor'])
        self.assertIn('joint_limits', servo[0]['robot_description_planning'])

    def test_joint_states_start_once_after_servo_waits(self):
        actions = self.run_setup()
        io = [a.args[0] for a in actions if type(a).__name__ == 'RegisterEventHandler'
              and type(a.args[0]).__name__ == 'OnProcessIO'][0]
        timer = [a for a in actions if type(a).__name__ == 'TimerAction'][0]
        fallback = timer.kwargs['actions'][0].kwargs['function']
        self.assertEqual(io.kwargs['on_stdout'](types.SimpleNamespace(text=b'Servo starting')), [])
        started = io.kwargs['on_stderr'](types.SimpleNamespace(text=b'[WARN] Waiting to receive robot state update'))
        self.assertEqual([n.kwargs['arguments'][0] for n in started], ['joint_state_broadcaster'])
        self.assertEqual(fallback(None), [])          # already started: the fallback does nothing
        fresh = self.run_setup()                      # if Servo never says it, the fallback starts it
        timer = [a for a in fresh if type(a).__name__ == 'TimerAction'][0]
        self.assertEqual(len(timer.kwargs['actions'][0].kwargs['function'](None)), 1)

    def test_no_teleop_has_no_servo_or_input(self):
        exes = self.executables(self.run_setup(teleop='false'))
        self.assertNotIn('servo_lifetime', exes)
        self.assertNotIn('amw_teleop', exes)
        self.assertIn('spawner', exes)

    def test_real_runs_preflight_first(self):
        actions = self.run_setup(hardware='real', port='/tmp/x')
        pre = [a for a in actions if type(a).__name__ == 'ExecuteProcess']
        self.assertEqual(len(pre), 1)
        cmd = pre[0].kwargs['cmd']
        self.assertEqual(cmd[1:3], ['-m', 'amw_tools.preflight'])
        self.assertIn('/tmp/x', cmd)
        self.assertNotIn('--readonly', cmd)
        top_level = [type(a).__name__ for a in actions]
        self.assertNotIn('Node', top_level)              # nothing starts before the check passes
        self.assertIn('servo_lifetime', self.executables(actions))

    def test_real_readonly(self):
        actions = self.run_setup(hardware='real', readonly='true')
        cmd = [a for a in actions if type(a).__name__ == 'ExecuteProcess'][0].kwargs['cmd']
        self.assertIn('--readonly', cmd)
        exes = self.executables(actions)
        self.assertNotIn('servo_lifetime', exes)
        self.assertNotIn('amw_teleop', exes)

    def test_bad_arguments_are_explained(self):
        with self.assertRaisesRegex(RuntimeError, 'readonly'):
            self.run_setup(readonly='true')
        with self.assertRaisesRegex(RuntimeError, 'hardware'):
            self.run_setup(hardware='sim')
        with self.assertRaisesRegex(RuntimeError, 'arm.yaml problem'):
            self.run_setup(arm_config='/nonexistent/arm.yaml')


if __name__ == '__main__':
    unittest.main()
