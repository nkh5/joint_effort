"""Drive the real amw_teleop node with stand-in ROS classes: Joy-Con in, JointJog and /amw/status out.
Checks the wiring between the node, the supervisor and the kinematics without ROS installed."""
import json
import math
import sys
import types
import unittest

import numpy as np

import _paths


class Auto:
    """A message stand-in: any attribute you read appears (as another Auto), any you set sticks."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Auto()
        setattr(self, name, value)
        return value


class Clock:
    t = 100.0

    def now(self):
        ns = int(Clock.t * 1e9)
        return types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: Auto(sec=ns // 10**9, nanosec=ns % 10**9))


class FakeNode:
    def __init__(self, name):
        self.name, self.params, self.pubs, self.subs, self.clients, self.timers = name, {}, {}, {}, {}, []
        self.logs = []
        FakeNode.instance = self

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=FakeNode.overrides.get(name, default))

    def create_publisher(self, kind, topic, qos):
        self.pubs[topic] = []
        return types.SimpleNamespace(publish=self.pubs[topic].append)

    def create_subscription(self, kind, topic, cb, qos):
        self.subs[topic] = cb

    def create_client(self, kind, name):
        client = types.SimpleNamespace(service_is_ready=lambda: False)
        self.clients[name] = client
        return client

    def create_service(self, kind, name, cb):
        self.subs[name] = cb

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_clock(self):
        return Clock()

    def get_logger(self):
        logs = self.logs
        return types.SimpleNamespace(info=lambda m: logs.append(('info', m)), warning=lambda m: logs.append(('warn', m)))

    def destroy_node(self):
        pass


def install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    msg = lambda **consts: type('Msg', (Auto,), consts)    # noqa: E731
    status_cls = msg(OK=0, WARN=1, ERROR=2)
    mod('rclpy', init=lambda **k: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)
    mod('rclpy.executors', ExternalShutdownException=RuntimeError)
    mod('rclpy.node', Node=FakeNode)
    mod('rclpy.qos', DurabilityPolicy=Auto(), QoSProfile=lambda **k: None, ReliabilityPolicy=Auto(),
        qos_profile_sensor_data=None)
    mod('control_msgs'), mod('control_msgs.msg', DynamicJointState=msg(), JointJog=msg())
    mod('controller_manager_msgs'), mod('controller_manager_msgs.srv', ListControllers=msg())
    mod('diagnostic_msgs'), mod('diagnostic_msgs.msg', DiagnosticArray=type('A', (Auto,), {'__init__': lambda s: Auto.__init__(s, status=[])}),
                                 DiagnosticStatus=status_cls, KeyValue=msg())
    mod('moveit_msgs'), mod('moveit_msgs.msg', ServoStatus=msg())
    mod('moveit_msgs.srv', ServoCommandType=types.SimpleNamespace(Request=msg(JOINT_JOG=0)))
    mod('sensor_msgs'), mod('sensor_msgs.msg', JointState=msg(), Joy=msg(), Imu=msg())
    mod('std_msgs'), mod('std_msgs.msg', String=msg())
    mod('std_srvs'), mod('std_srvs.srv', Trigger=msg())


L, ZL, UP, DOWN, ON, LEFT = 0, 1, 2, 3, 4, 5


class TeleopNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = dict(sys.modules)
        install_stubs()
        from amw_bringup.arm import load, robot_description
        import yaml
        src = _paths.WS / 'src'
        arm = load(src / 'amw_bringup/config/arm.yaml')
        params = yaml.safe_load((src / 'amw_bringup/config/teleop.yaml').read_text())['amw_teleop']['ros__parameters']
        params.update(robot_description=robot_description((src / 'amw_description/urdf/amw.urdf').read_text(),
                                                          (src / 'amw_description/srdf/amw.srdf').read_text(), arm),
                      max_joint_velocity_rad_s=arm.max_speed)
        FakeNode.overrides = params
        sys.modules.pop('amw_teleop.node', None)
        from amw_teleop import node as node_module
        cls.node_module = node_module
        cls.home = {'joint_1': 0.0, 'joint_2': 0.5, 'joint_3': -1.2, 'joint_4': 0.0, 'joint_5': 1.5}

    @classmethod
    def tearDownClass(cls):
        for name in list(sys.modules):
            if name not in cls.saved:
                del sys.modules[name]
        sys.modules.update(cls.saved)

    def setUp(self):
        self.mono = 0.0
        self.node_module.time.monotonic = lambda: self.mono
        self.n = self.node_module.Teleop()
        self.f = FakeNode.instance
        self.buttons = [0] * 11
        self.axes = [0.0, 0.0]
        self.gyro = None                           # rad/s in Joy-Con axes; None = no IMU messages
        self.accel = (0.0, 9.80665, 0.0)           # Joy-Con upright: its y axis (the long side) points up
        self.q = dict(self.home)

    def step(self, seconds=0.02):
        """Advance time, feeding joint states, Servo status, controller state and joystick at 50 Hz."""
        end = self.mono + seconds
        while self.mono < end - 1e-9:
            self.mono = round(self.mono + 0.02, 6)
            Clock.t += 0.02
            self.f.subs['/joint_states'](Auto(name=list(self.q), position=list(self.q.values()), velocity=[]))
            self.f.subs['/iron_servo/status'](Auto(code=0, message=''))
            self.n.health.controller = 'active'
            self.n.servo_switched = True
            self.f.subs['/joy'](Auto(axes=list(self.axes), buttons=list(self.buttons)))
            if self.gyro is not None:
                for k in range(4):                 # 200 Hz IMU, arriving bundled
                    stamp = Clock.t - 0.02 + 0.005 * (k + 1)
                    self.f.subs['/amw/hand_imu'](Auto(
                        header=Auto(stamp=Auto(sec=int(stamp), nanosec=int((stamp % 1) * 1e9))),
                        angular_velocity=Auto(x=self.gyro[0], y=self.gyro[1], z=self.gyro[2]),
                        linear_acceleration=Auto(x=self.accel[0], y=self.accel[1], z=self.accel[2])))
            self.n.tick()

    def press(self, i):
        self.buttons[i] = 1
        self.step()
        self.buttons[i] = 0
        self.step()

    def status(self):
        return json.loads(self.f.pubs['/amw/status'][-1].data)

    def test_full_cycle(self):
        self.step(0.2)
        st = self.status()
        self.assertEqual((st['state'], st['stop']['code']), ('off', 'OFF'))
        self.assertEqual(self.f.pubs['/iron_servo/delta_joint_cmds'], [], 'nothing sent to Servo while off')

        self.press(ON)
        self.step(0.3)
        self.assertEqual(self.status()['state'], 'active')

        self.axes[0] = 1.0                         # full stick +X
        self.step(0.1)
        jog = self.f.pubs['/iron_servo/delta_joint_cmds'][-1]
        self.assertEqual(jog.joint_names, ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        self.assertTrue(any(abs(v) > 1e-6 for v in jog.velocities))
        self.assertLessEqual(max(abs(v) for v in jog.velocities), 0.15 + 1e-9)
        st = self.status()
        self.assertEqual(len(st['joints']), 5)
        self.assertAlmostEqual(st['joints'][2]['deg'], math.degrees(-1.2), places=1)

        self.buttons[L] = 1                        # hold L while pushing: z mode waits for a centred stick
        self.step()
        st = self.status()
        self.assertEqual((st['state'], st['mode'], st['stop']['code']), ('centre', 'z', 'MODE_CHANGED'))
        jogs = self.f.pubs['/iron_servo/delta_joint_cmds']
        self.assertEqual(jogs[-1].velocities, [0.0] * 5, 'a zero command right after stopping')
        sent = len(jogs)
        self.step(0.2)
        self.assertEqual(len(jogs), sent, 'and nothing more while paused')
        self.axes[0] = 0.0
        self.step(0.3)
        self.assertEqual(self.status()['state'], 'active')
        self.buttons[L] = 0
        self.step()

        response = self.f.subs['/amw/stop'](None, Auto())
        self.assertTrue(response.success)
        self.step()
        self.assertEqual(self.status()['stop']['code'], 'STOP_REQUESTED')
        self.assertTrue(any(level == 'warn' and m.startswith('[STOP] Stopped by `amw stop`') for level, m in self.f.logs))
        events = [t for _, t in self.status()['events']]
        self.assertIn('moving', events)

    def test_joint_limit_warning_keeps_moving(self):
        self.step(0.1)
        self.press(ON)
        self.step(0.3)
        upper = self.n.limits[2][1]
        self.q['joint_3'] = upper - 0.01
        for axis in range(3):                      # find a direction (x, y, or z with L) that pushes the elbow up
            for sign in (1, -1):
                self.axes = [0.0, 0.0]
                self.buttons[L] = int(axis == 2)
                self.step(0.06)
                self.axes[min(axis, 1)] = sign
                self.step(0.4)
                st = self.status()
                if st['warning'] and st['warning']['code'] == 'JOINT_LIMIT':
                    self.assertEqual(st['state'], 'active')
                    self.assertIn('joint_3', st['warning']['text'])
                    self.assertEqual(self.f.pubs['/iron_servo/delta_joint_cmds'][-1].velocities[2], 0.0)
                    return
        self.fail('no stick direction produced a joint-limit warning for joint_3')

    def test_z_mode_moves_up_and_down(self):
        self.step(0.1)
        self.press(ON)
        self.step(0.3)
        self.buttons[L] = 1
        self.step(0.1)
        self.axes[1] = 1.0
        self.step(0.1)
        linear, angular = self.n.command([self.q[n] for n in self.n.names])
        self.assertEqual((linear[0], linear[1]), (0.0, 0.0))
        self.assertGreater(linear[2] * self.n.z_sign, 0.0, 'stick up -> up when z_sign is 1, down when -1')
        self.assertTrue(any(abs(v) > 1e-6 for v in self.f.pubs['/iron_servo/delta_joint_cmds'][-1].velocities))

    def iron(self):
        return self.n.robot.fk('iron', [self.q[n] for n in self.n.names])[:3, 0]

    def test_zl_points_the_iron_like_the_joycon(self):
        self.gyro = (0.0, 0.0, 0.0)
        self.step(0.1)
        self.press(LEFT)                           # Joy-Con upright: this is "iron straight down"
        self.press(ON)
        self.step(0.3)
        self.buttons[ZL] = 1
        self.step(0.1)
        self.assertEqual(self.status()['mode'], 'rotation')
        q = [self.q[n] for n in self.n.names]
        _, angular = self.n.command(q)
        target = np.array([0.0, 0.0, -1.0])
        expected = np.cross(self.iron(), target)   # Joy-Con upright: iron swings toward straight down
        self.assertGreater(float(np.dot(angular, expected)), 0.0)
        self.assertAlmostEqual(float(np.dot(angular, self.iron())), 0.0, places=9, msg='no roll about the iron')
        self.assertTrue(any(abs(v) > 1e-6 for v in self.f.pubs['/iron_servo/delta_joint_cmds'][-1].velocities))

    def test_twisting_the_joycon_does_not_move_the_target(self):
        self.gyro = (0.0, 0.0, 0.0)
        self.step(0.1)
        self.press(LEFT)
        before = self.n.orient.r.copy()
        self.gyro = (0.0, 0.8, 0.0)                # twist about the long (y) axis for 0.5 s
        self.step(0.5)
        self.gyro = (0.0, 0.0, 0.0)
        from amw_teleop.hand import hand_tilt
        tilt = hand_tilt(self.n.neutral, self.n.orient.r, self.n.long_axis)
        self.assertGreater(np.degrees(np.linalg.norm(self.n.neutral.T @ self.n.orient.r - np.eye(3))), 1.0)
        self.assertLess(np.degrees(np.linalg.norm(tilt)), 0.5)
        self.assertIsNotNone(before)

    def test_tilting_the_joycon_tilts_the_target(self):
        self.gyro = (0.0, 0.0, 0.0)
        self.step(0.1)
        self.press(LEFT)
        self.n.orient.gain = 0.0                   # the fake accel below does not turn with the gyro
        self.gyro = (0.5, 0.0, 0.0)                # tilt about Joy-Con x for 0.4 s (~11 deg)
        self.step(0.4)
        self.gyro = (0.0, 0.0, 0.0)
        from amw_teleop.hand import hand_tilt, iron_target
        target = iron_target(hand_tilt(self.n.neutral, self.n.orient.r, self.n.long_axis), **self.n.hand_cfg)
        tilt_deg = np.degrees(np.arccos(-target[2]))
        self.assertAlmostEqual(tilt_deg, np.degrees(0.2) * self.n.hand_cfg['scale'], delta=1.5)

    def test_zl_without_imu_turns_off(self):
        self.step(0.1)
        self.press(ON)
        self.step(0.3)
        self.buttons[ZL] = 1
        self.step(0.3)
        st = self.status()
        self.assertEqual((st['state'], st['stop']['code']), ('off', 'IMU_LOST'))

    def test_lost_feedback_turns_off(self):
        self.step(0.1)
        self.press(ON)
        self.step(0.3)
        self.f.subs['/joint_states'] = lambda msg: None          # joint states stop arriving
        self.step(0.4)
        st = self.status()
        self.assertEqual((st['state'], st['stop']['code']), ('off', 'FEEDBACK_STALE'))

    def test_diagnostics(self):
        self.step(0.1)
        self.n.publish_diagnostics()
        arr = self.f.pubs['/diagnostics'][-1]
        self.assertEqual(arr.status[0].name, 'amw: teleop')
        self.assertEqual(len(arr.status), 6)


if __name__ == '__main__':
    unittest.main()
