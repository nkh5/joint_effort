"""amw_teleop: Joy-Con -> ON/OFF and mode logic -> five-joint velocity -> MoveIt Servo.

One node owns the whole teleop path (earlier versions split it over four nodes):

  in   /joy                    Joy-Con stick and buttons (from amw_joycon; order in teleop.yaml)
       /amw/hand_imu           Joy-Con gyro, bias removed (from amw_joycon), for the ZL rotation clutch
       /joint_states           arm feedback
       /iron_servo/status      MoveIt Servo status
       /dynamic_joint_states   motor temperature / voltage / current (real arm)
  out  /iron_servo/delta_joint_cmds   JointJog, 50 Hz while moving (one zero command after stopping)
       /amw/status             JSON (latched): state, mode, speed, stop reason, warning, joints, events
       /diagnostics            standard DiagnosticArray (rqt_robot_monitor) at 1 Hz
  srv  /amw/stop               Trigger: turn the arm OFF (like the ON/OFF button)

Every stop is printed once in this terminal as `[STOP] what happened -- what to do`.
"""
import json
import math
import time

import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, JointJog
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import velocity
from .hand import HandRotation, follow_velocity, target_rotation
from .kinematics import Robot, differential_step
from .supervisor import Config, Health, Supervisor

PERIOD = 0.02            # s, 50 Hz like Servo and the controller
STATUS_PERIOD = 0.1      # s
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SERVO_NAMES = {-1: 'invalid', 0: 'ok', 1: 'slowing (singularity)', 2: 'halted (singularity)',
               3: 'slowing (collision)', 4: 'slowing (collision)', 5: 'halted (collision)', 6: 'at a joint limit'}


class Teleop(Node):
    def __init__(self):
        super().__init__('amw_teleop')

        def param(name, default):
            return self.declare_parameter(name, default).value

        description = param('robot_description', '')
        if not description:
            raise RuntimeError('amw_teleop needs the robot_description parameter (start it with `amw start`)')
        self.robot = Robot(description)
        self.names = self.robot.names('iron')
        self.limits = self.robot.limits('iron')
        self.sup = Supervisor(Config(
            stick_axes=tuple(param('stick_axes', [0, 1])), stick_signs=tuple(param('stick_signs', [1.0, 1.0])),
            on_button=param('on_button', 4), speed_up_button=param('speed_up_button', 2),
            speed_down_button=param('speed_down_button', 3), z_button=param('z_button', 0),
            rotate_button=param('rotate_button', 1), speed_step=param('speed_step', 2),
            start_level=param('start_level', 10), deadzone=param('stick_deadzone', 0.12),
            expo=param('stick_expo', 0.6), centre_time=param('centre_time_s', 0.2),
            input_timeout=param('input_timeout_s', 0.15), imu_timeout=param('imu_timeout_s', 0.15),
            feedback_timeout=param('feedback_timeout_s', 0.3),
            limit_tolerance=math.radians(param('limit_tolerance_deg', 2.0)),
            motor_warm_c=param('motor_warm_c', 55.0), motor_hot_c=param('motor_hot_c', 65.0),
            motor_volt_range=(param('motor_volt_min', 9.5), param('motor_volt_max', 14.0))))
        self.speeds = {k: param(k, d) for k, d in [('linear_speed_m_s', 0.01), ('angular_speed_rad_s', 0.1),
                                                     ('z_speed_ratio', 0.5)]}
        self.hand_cfg = dict(axes=tuple(param('hand_axes', [0, 1])), signs=tuple(param('hand_signs', [1.0, 1.0])),
                             scale=param('hand_scale', 0.5), max_angle=math.radians(param('hand_max_angle_deg', 60.0)))
        self.hand_gain = param('hand_gain_per_s', 3.0)
        if (len(self.hand_cfg['axes']) != 2 or any(a not in (0, 1, 2) for a in self.hand_cfg['axes'])
                or any(s not in (-1.0, 1.0) for s in self.hand_cfg['signs'])):
            raise RuntimeError('hand_axes must be two of 0, 1, 2 and hand_signs two of 1.0 / -1.0 (teleop.yaml)')
        self.hand = HandRotation()
        self.imu_time = -math.inf
        self.rotating = False
        self.tool0 = None
        self.model = {k: param(k, d) for k, d in [
            ('task_max_linear_speed_m_s', 0.02), ('task_max_angular_speed_rad_s', 0.2),
            ('task_orientation_length_m', 0.12), ('task_damping_m', 0.002), ('task_min_singular_value_m', 0.003),
            ('joint_limit_margin_rad', 0.05), ('max_joint_velocity_rad_s', 0.15)]}

        self.health = Health(limits={n: tuple(l) for n, l in zip(self.names, self.limits)})
        self.joint_time = -math.inf
        self.servo_time = -math.inf
        self.servo_future = None
        self.servo_switched = False
        self.controller_future = None
        self.was_active = False
        self.last_status = None
        self.last_status_time = -math.inf
        self.last_stop_logged = None
        self.qdot = np.zeros(5)

        self.jog_pub = self.create_publisher(JointJog, '/iron_servo/delta_joint_cmds', 1)
        self.status_pub = self.create_publisher(String, '/amw/status', STATUS_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, qos_profile_sensor_data)
        self.create_subscription(Imu, '/amw/hand_imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(ServoStatus, '/iron_servo/status', self.on_servo, 10)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.on_dynamic, 10)
        self.servo_client = self.create_client(ServoCommandType, '/iron_servo/switch_command_type')
        self.controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.create_service(Trigger, '/amw/stop', self.on_stop)
        self.create_timer(PERIOD, self.tick)
        self.create_timer(1.0, self.slow_tick)
        self.get_logger().info('Ready. The arm starts OFF: centre the stick and press ON (D-pad right).')

    # ------------------------------------------------------------------ inputs
    def on_joy(self, msg):
        self.refresh_health()
        self.sup.joy(list(msg.axes), list(msg.buttons), time.monotonic(), self.health)

    def on_imu(self, msg):
        self.imu_time = time.monotonic()
        if self.rotating:
            w = msg.angular_velocity
            self.hand.add((w.x, w.y, w.z), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_joint_states(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in self.names) and all(math.isfinite(values[n]) for n in self.names):
            self.health.joints = {n: values[n] for n in self.names}
            self.joint_time = time.monotonic()

    def on_servo(self, msg):
        self.health.servo_code, self.health.servo_message = msg.code, msg.message
        self.servo_time = time.monotonic()

    def on_dynamic(self, msg):
        for name, values in zip(msg.joint_names, msg.interface_values):
            if name in self.names:
                self.health.motors[name] = dict(zip(values.interface_names, values.values))

    def on_stop(self, request, response):
        self.sup.request_stop(time.monotonic())
        response.success = True
        response.message = 'Arm turned OFF. Press ON (D-pad right) to continue.'
        return response

    # ------------------------------------------------------------------ background checks (1 Hz)
    def slow_tick(self):
        f = self.controller_future
        if f is not None and f.done():
            try:
                states = {c.name: c.state for c in f.result().controller}
                self.health.controller = states.get('iron_arm_controller', 'not loaded')
            except Exception:
                self.health.controller = None
            self.controller_future = None
        if self.controller_future is None and self.controllers_client.service_is_ready():
            self.controller_future = self.controllers_client.call_async(ListControllers.Request())
        # Servo accepts JointJog only after it has been switched to that command type (once).
        f = self.servo_future
        if f is not None and f.done():
            self.servo_switched = not f.exception() and f.result().success
            if not self.servo_switched:
                self.get_logger().warning('MoveIt Servo refused the switch to joint-jog input; retrying')
            self.servo_future = None
        if not self.servo_switched and self.servo_future is None and self.servo_client.service_is_ready():
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.JOINT_JOG
            self.servo_future = self.servo_client.call_async(request)
        self.publish_diagnostics()

    def refresh_health(self):
        now = time.monotonic()
        self.health.joints_age = now - self.joint_time
        self.health.imu_age = now - self.imu_time
        self.health.servo_ready = self.servo_switched and now - self.servo_time < 1.0

    # ------------------------------------------------------------------ 50 Hz loop
    def tick(self):
        now = time.monotonic()
        self.refresh_health()
        moving = self.sup.step(self.health, now)
        qdot = np.zeros(5)
        rotating = moving and self.sup.mode == 'rotation'
        if rotating and not self.rotating:          # rotation (ZL) just started: hold the iron where it is
            self.hand.reset()
            self.tool0 = None
        self.rotating = rotating
        if moving:
            q = np.array([self.health.joints[n] for n in self.names])
            linear, angular = self.command(q)
            if any(linear) or any(angular):
                qdot, info = differential_step(self.robot, 'iron', q, linear, angular, self.model, PERIOD)
                self.sup.kinematics_warning(info, self.names, self.limits, q, now)
        if moving or self.was_active:          # stream while moving, plus one zero after stopping
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.joint_names = self.names
            msg.velocities = qdot.tolist()
            msg.duration = PERIOD
            self.jog_pub.publish(msg)
        self.was_active = moving
        self.qdot = qdot
        self.log_changes()
        self.publish_status(now)

    def command(self, q):
        """(linear m/s, angular rad/s) for this tick, world frame, from the current mode."""
        if self.sup.mode != 'rotation':
            return velocity(self.sup.axes, self.sup.mode, self.sup.speed, self.speeds['linear_speed_m_s'],
                            self.model['task_max_linear_speed_m_s'], self.speeds['z_speed_ratio']), [0., 0., 0.]
        tool = self.robot.fk('iron', q)[:3, :3]
        if self.tool0 is None:
            self.tool0 = tool
        target = target_rotation(self.tool0, self.hand.vector(), **self.hand_cfg)
        w = follow_velocity(target, tool, self.hand_gain)
        axis = tool[:, 0]                          # the iron; roll about it is left free by the solve
        w = w - axis * float(axis @ w)
        cap = min(self.speeds['angular_speed_rad_s'] * self.sup.speed, self.model['task_max_angular_speed_rad_s'])
        n = float(np.linalg.norm(w))
        if n > cap:
            w *= cap / n
        if n < 1e-3:                               # within ~0.02 deg/s of the target: hold still
            w = np.zeros(3)
        return [0., 0., 0.], w.tolist()

    # ------------------------------------------------------------------ outputs
    def log_changes(self):
        stop = self.sup.stop
        key = (self.sup.state, stop and stop['text'], (self.sup.warning or {}).get('text'))
        if key == self.last_stop_logged:
            return
        self.last_stop_logged = key
        log = self.get_logger()
        if self.sup.state == 'active':
            warn = self.sup.warning
            log.info(f'[ON] moving ({self.sup.mode})' + (f'  [WARN] {warn["text"]} -- {warn["fix"]}' if warn else ''))
        elif stop and stop['kind'] == 'pause':
            log.info(f'[WAIT] {stop["text"]} -- {stop["fix"]}')
        elif stop and stop['code'] in ('OFF', 'TURNED_OFF'):
            log.info(f'[OFF] {stop["text"]}')
        elif stop:
            log.warning(f'[STOP] {stop["text"]} -- {stop["fix"]}')

    def joint_rows(self):
        rows = []
        for name, (lo, hi) in zip(self.names, self.limits):
            q = (self.health.joints or {}).get(name)
            m = self.health.motors.get(name, {})
            rows.append({'name': name, 'deg': None if q is None else round(math.degrees(q), 2),
                         'lower': round(math.degrees(lo), 2), 'upper': round(math.degrees(hi), 2),
                         'margin_deg': None if q is None else round(math.degrees(min(q - lo, hi - q)), 2),
                         'temp': m.get('temperature'), 'volt': m.get('voltage'), 'amps': m.get('current')})
        return rows

    def publish_status(self, now):
        data = self.sup.status(now)
        data.update(schema=2, servo={'code': self.health.servo_code, 'message': self.health.servo_message,
                                     'text': SERVO_NAMES.get(self.health.servo_code, 'no status')},
                    controller=self.health.controller, joints=self.joint_rows(),
                    feedback_age=round(min(self.health.joints_age, 999.0), 3),
                    command_deg_s=[round(math.degrees(v), 2) for v in self.qdot])
        compare = {k: v for k, v in data.items() if k not in ('events', 'joints', 'feedback_age', 'command_deg_s')}
        if compare == self.last_status and now - self.last_status_time < STATUS_PERIOD:
            return
        self.last_status, self.last_status_time = compare, now
        data['stamp_ns'] = self.get_clock().now().nanoseconds
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False, default=str)))

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus(name='amw: teleop', hardware_id='amw')
        stop, warn = self.sup.stop, self.sup.warning
        if stop and stop['kind'] == 'stop' and stop['code'] not in ('OFF', 'TURNED_OFF'):
            st.level, st.message = DiagnosticStatus.ERROR, stop['text']
        elif warn or (stop and stop['kind'] == 'pause'):
            st.level, st.message = DiagnosticStatus.WARN, (warn or stop)['text']
        else:
            st.level, st.message = DiagnosticStatus.OK, self.sup.state
        st.values = [KeyValue(key='state', value=self.sup.state), KeyValue(key='mode', value=self.sup.mode),
                     KeyValue(key='level', value=str(self.sup.level))]
        arr.status.append(st)
        for row in self.joint_rows():
            js = DiagnosticStatus(name=f'amw: {row["name"]}', hardware_id='amw')
            js.level, js.message = DiagnosticStatus.OK, 'ok'
            if row['margin_deg'] is not None and row['margin_deg'] < 0:
                js.level, js.message = DiagnosticStatus.WARN, 'outside its limits'
            js.values = [KeyValue(key=k, value=str(row[k])) for k in ('deg', 'lower', 'upper', 'temp', 'volt', 'amps')]
            arr.status.append(js)
        self.diag_pub.publish(arr)

    def close(self):
        """Stop the arm cleanly: one zero command before the node goes away."""
        if self.was_active:
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.names
            msg.velocities = [0.0] * 5
            msg.duration = PERIOD
            self.jog_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
