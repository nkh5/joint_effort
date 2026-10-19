"""Own the mock bridge while enabled; normalize Joy, then scale joy_teleop output."""
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import GetStateValidity
from joint_effort_robot.kinematics import Robot
from .core import InputState, Heartbeat, velocity, multiplier
from .lease import Lease

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
STATUS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
INPUT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class Forwarder(Node):
    def __init__(self):
        super().__init__('amw_forwarder')
        def parameter(name, default): return self.declare_parameter(name, default).value
        self.profile = parameter('runtime_profile', 'mock')
        self.expected_plugin = 'mock_components/GenericSystem'
        self.physical = self.profile == 'hardware'
        emulated = parameter('emulated_bus', False)
        iron = Path(get_package_share_directory('joint_effort_iron'))
        if self.physical:
            from joint_effort_hardware.configuration import read, PLUGIN, emulator_port
            config = read(parameter('hardware_calibration', ''), iron/'urdf/iron.urdf')
            if emulated: emulator_port(config['serial_port'])
            self.expected_plugin = PLUGIN
        if self.profile not in ('mock', 'hardware') or os.environ.get('ROS_DOMAIN_ID') != (('90' if emulated else '89') if self.physical else '88'):
            raise RuntimeError('Mock input requires domain 88; physical input requires measured settings and domain 89')
        self.confirmed = parameter('mapping_confirmed', False)
        self.input = InputState(tuple(parameter('stick_axes', [0, 1, 2])),
                                tuple(parameter('stick_signs', [1., 1., 1.])),
                                parameter('slider_axis', 3), parameter('slider_low_axis', 1.),
                                parameter('enable_button', 0), parameter('mode_button', 1),
                                parameter('stick_deadzone', .08), parameter('input_timeout', .15),
                                parameter('mode_select_button', -1))
        self.command_timeout = parameter('command_timeout', .12)
        if not .05 <= self.command_timeout <= .12: raise ValueError('Command timeout must be 0.05–0.12 s')
        root = Path(os.environ['ROS_HOME']).parent
        self.lease = Lease(str(root/('iron-emulated-motion.lock' if self.physical and emulated else 'iron-motion.lock')))
        self.robot = Robot(parameter('robot_path', str(iron/'urdf/iron.urdf')))
        self.names = self.robot.names('iron'); self.bounds = self.robot.limits('iron')
        self.state_stamp = -1; self.state_receipt = -math.inf; self.state_valid = False; self.q = None
        self.collision_clear = False; self.valid_receipt = self.valid_poll = -math.inf; self.valid_future = None
        self.phase = 'idle'; self.pending = None; self.pending_start = 0.; self.last_command = -math.inf
        self.last_raw_stamp = -1; self.min_raw_stamp = -1; self.active_start = 0.
        self.servo = None; self.servo_receipt = -math.inf; self.bridge = 'unknown'; self.bridge_receipt = -math.inf
        self.hw_ok = self.controllers_ok = False; self.hw_receipt = self.controllers_receipt = -math.inf
        self.hw_future = self.controllers_future = None; self.poll_time = -math.inf
        self.reason = 'Waiting for arm controllers'; self.output = [0.]*6
        self.last_fault = None
        self.heartbeat = Heartbeat()
        self.canonical = self.create_publisher(Joy, '/amw/joy_control', LATEST)
        self.task = self.create_publisher(TwistStamped, '/iron/task_velocity', LATEST)
        self.status = self.create_publisher(String, '/amw/display_state', STATUS)
        self.create_subscription(Joy, '/joy', self.joy, INPUT)
        self.create_subscription(TwistStamped, '/amw/cmd_raw', self.raw, LATEST)
        self.create_subscription(JointState, '/joint_states', self.feedback, INPUT)
        self.create_subscription(ServoStatus, '/iron_servo/status', self.servo_status, LATEST)
        self.create_subscription(String, '/iron_task_bridge/status', self.bridge_status, LATEST)
        self.enable = self.create_client(SetBool, '/iron_task_bridge/enable')
        self.hw = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.create_service(Trigger, '/amw/stop', self.stop)
        self.create_timer(.02, self.tick); self.create_timer(.1, self.publish_status)

    def ns(self, stamp): return stamp.sec*10**9+stamp.nanosec

    def feedback(self, msg):
        self.state_valid = False
        if len(msg.name) != 5 or len(msg.position) != 5 or set(msg.name) != set(self.names): return
        q = dict(zip(msg.name, msg.position))
        if any(not math.isfinite(q[n]) or not lo <= q[n] <= hi for n, (lo, hi) in zip(self.names, self.bounds)): return
        self.state_stamp = self.ns(msg.header.stamp); self.state_receipt = time.monotonic(); self.state_valid = True
        self.q = [q[n] for n in self.names]

    def servo_status(self, msg):
        self.servo = msg; self.servo_receipt = time.monotonic()

    def bridge_status(self, msg):
        self.bridge = msg.data; self.bridge_receipt = time.monotonic()

    def joy(self, msg):
        now = time.monotonic()
        if self.physical and msg.header.frame_id == 'synthetic_joy':
            self.fail('Synthetic input is not allowed on the physical arm'); return
        if self.physical:
            # Pico enable is a separate permission. It never replaces the held button.
            if (len(msg.buttons) <= 10 or any(v not in (0,1) for v in msg.buttons)
                    or msg.buttons[7]+msg.buttons[8] != 1
                    or bool(msg.buttons[8]) != bool(msg.buttons[9])):
                self.fail('Pico disabled or mode flags disagree; release and center')
                return
            if not self.heartbeat.update(msg.buttons[10], now):
                self.fail('Pico heartbeat missing; center and release required'); return
        valid = self.input.update(msg.axes, msg.buttons, self.ns(msg.header.stamp),
                                  self.get_clock().now().nanoseconds, now)
        if not valid: self.reason = self.input.reason
        canonical = Joy(); canonical.header = msg.header
        canonical.axes = list(self.input.axes) if valid else [0., 0., 0.]
        allowed = valid and self.confirmed and self.input.allowed(now)
        canonical.buttons = [int(allowed and self.input.mode == 'translation'),
                             int(allowed and self.input.mode == 'rotation')]
        # Exactly three axes and two synthetic buttons avoid upstream missing-index behavior.
        self.canonical.publish(canonical)

    def publish_velocity(self, values):
        msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'world'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = values[:3]
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = values[3:]
        self.output = list(values); self.task.publish(msg)

    def fail(self, reason):
        self.last_fault = {'reason': reason, 'readiness': self.readiness(time.monotonic()),
                           'stamp_ns': self.get_clock().now().nanoseconds}
        self.reason = reason; self.input.fault(reason)

    def raw(self, msg):
        now = time.monotonic(); stamp = self.ns(msg.header.stamp)
        values = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                  msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        if (msg.header.frame_id != 'world' or not 0 <= age <= self.command_timeout
                or stamp <= self.last_raw_stamp or not all(math.isfinite(v) for v in values)):
            if self.phase == 'active': self.fail('Invalid, stale or replayed joy_teleop command')
            return
        self.last_raw_stamp = stamp
        if self.phase != 'active' or stamp < self.min_raw_stamp: return
        expected = velocity(self.input.axes, self.input.mode)
        if any(abs(v-e) > 1e-7 for v, e in zip(values, expected)):
            # A late sample from a previous axis state is discarded; never restamp it into motion.
            return
        if not self.input.allowed(now) or not self.ready(now): return
        self.last_command = now
        if now-self.active_start < .15: return
        scaled = velocity(self.input.axes, self.input.mode, multiplier(self.input.level))
        self.publish_velocity(scaled)

    def ready(self, now):
        return all(self.readiness(now).values())

    def readiness(self, now):
        age = (self.get_clock().now().nanoseconds-self.state_stamp)*1e-9
        return {'mapping': self.confirmed, 'hardware': self.hw_ok, 'controllers': self.controllers_ok,
                'hardware_status_fresh': now-self.hw_receipt < 3.,
                'controller_status_fresh': now-self.controllers_receipt < 3.,
                'joint_values': self.state_valid, 'joint_stamp_fresh': 0 <= age < .3,
                'joint_receipt_fresh': now-self.state_receipt < .3,
                'servo_status_fresh': self.servo is not None and now-self.servo_receipt < .3,
                'device_heartbeat': not self.physical or self.heartbeat.fresh(now),
                'bridge_status_fresh': now-self.bridge_receipt < .3}

    def poll(self, now):
        if self.valid_future is not None and self.valid_future.done():
            self.collision_clear = not self.valid_future.exception() and self.valid_future.result().valid
            self.valid_receipt = now; self.valid_future = None
        if (now-self.valid_poll > .2 and self.valid_future is None and self.state_valid
                and self.validity.service_is_ready()):
            self.valid_poll = now
            request = GetStateValidity.Request(); request.group_name = 'iron_arm'
            request.robot_state.joint_state.name = self.names; request.robot_state.joint_state.position = self.q
            self.valid_future = self.validity.call_async(request)
        for label, client, kind in [('hw', self.hw, ListHardwareComponents),
                                    ('controllers', self.controllers, ListControllers)]:
            future = getattr(self, label+'_future')
            if future is not None and future.done():
                ok = False
                if not future.exception():
                    result = future.result()
                    if label == 'hw':
                        ok = (len(result.component) == 1 and result.component[0].plugin_name == self.expected_plugin
                              and (not self.physical or result.component[0].state.id == 3))
                    else:
                        ok = {'iron_arm_controller', 'joint_state_broadcaster'} <= {c.name for c in result.controller if c.state == 'active'}
                setattr(self, label+'_ok', ok); setattr(self, label+'_receipt', now)
                setattr(self, label+'_future', None)
        if now-self.poll_time > 1.:
            self.poll_time = now
            for label, client, kind in [('hw', self.hw, ListHardwareComponents), ('controllers', self.controllers, ListControllers)]:
                if getattr(self, label+'_future') is None and client.service_is_ready():
                    setattr(self, label+'_future', client.call_async(kind.Request()))

    def transition(self, enabled, now):
        if not self.enable.service_is_ready(): return False
        self.pending = self.enable.call_async(SetBool.Request(data=enabled)); self.pending_start = now
        self.phase = 'enabling' if enabled else 'disabling'; return True

    def tick(self):
        now = time.monotonic(); self.poll(now)
        if len(self.get_publishers_info_by_topic('/joy')) > 1:
            self.fail('Multiple joystick publishers; stop duplicate input sources')
        desired = self.confirmed and self.input.allowed(now)
        if self.pending is not None and self.pending.done():
            success = not self.pending.exception() and self.pending.result().success
            self.pending = None
            if self.phase == 'enabling':
                if success:
                    self.phase = 'active'; self.active_start = now; self.last_command = now
                    self.min_raw_stamp = self.get_clock().now().nanoseconds
                else:
                    self.fail('Arm bridge refused enable'); self.phase = 'idle'; self.lease.release()
            elif success:
                self.phase = 'idle'; self.lease.release()
            else:
                self.reason = 'Bridge disable not acknowledged; retaining ownership and retrying'
        if self.pending is not None and now-self.pending_start > .5:
            self.fail('Bridge service response delayed; retaining ownership until disabled')
            desired = False
        if self.phase == 'active':
            if not self.ready(now): self.fail('Arm feedback/controller/Servo fault; center and release required')
            elif self.bridge == 'disabled' and now-self.active_start > .1: self.fail('Bridge disabled unexpectedly')
            elif now-self.active_start > .15 and self.servo.code not in (0, 1, 3, 4):
                self.fail('Servo halted: '+self.servo.message+'; center and release required')
            elif now-self.active_start > .2 and now-self.last_command > self.command_timeout: self.fail('joy_teleop command input lost')
            desired = self.confirmed and self.input.allowed(now) and self.ready(now)
            if not desired:
                self.publish_velocity([0.]*6)
                if self.pending is None: self.transition(False, now)
            elif now-self.active_start < .15:
                self.publish_velocity([0.]*6)
        elif self.phase == 'idle' and desired:
            if not self.ready(now):
                self.fail('Arm system not ready; center and release required')
            elif not self.collision_clear or now-self.valid_receipt > .5:
                self.fail('Current modeled collision check not clear/fresh; center and release required')
            elif not self.lease.acquire():
                self.fail('Another motion tool owns the arm; center and release after it ends')
            elif not self.transition(True, now):
                self.fail('Arm bridge service unavailable'); self.lease.release()
        elif self.phase == 'disabling' and self.pending is None:
            self.transition(False, now)
        if self.phase in ('enabling', 'disabling'):
            self.publish_velocity([0.]*6)
        if self.phase == 'idle': self.output = [0.]*6

    def stop(self, request, response):
        self.fail('Software stop requested; center and release required')
        if self.lease.file: self.publish_velocity([0.]*6)
        response.success = True; response.message = 'Joystick stop latched; release enable and center before restarting'
        return response

    def publish_status(self):
        warning = self.servo.message if self.servo else 'Waiting for Servo status'
        reason = self.input.reason if self.phase == 'idle' else self.reason
        if not self.confirmed: reason = 'Joystick mapping unconfirmed; motion inhibited'
        elif self.phase == 'active':
            reason = 'Preparing Servo with zero command' if time.monotonic()-self.active_start < .15 else 'Moving arm while enable is held'
        elif self.input.prepared and (not self.ready(time.monotonic()) or not self.collision_clear):
            reason = 'Waiting for arm feedback/controllers and a clear collision check'
        data = {'schema': 1, 'scope': 'iron cold hardware' if self.physical else 'iron cold mock only', 'mode': self.input.mode,
                'level': self.input.level, 'multiplier': round(multiplier(self.input.level), 4),
                'speed_fraction': (self.input.level-1)/19., 'phase': self.phase,
                'enabled': self.phase == 'active', 'input_fresh': self.input.fresh(time.monotonic()),
                'prepared': self.input.prepared,
                'mapping_confirmed': self.confirmed, 'message': reason, 'servo_message': warning,
                'servo_code': self.servo.code if self.servo else None, 'bridge': self.bridge,
                'command_world': self.output,
                'mock_ready': self.ready(time.monotonic()),
                'readiness': self.readiness(time.monotonic()), 'last_fault': self.last_fault,
                'hardware_verified': self.hw_ok, 'controllers_active': self.controllers_ok,
                'collision_clear': self.collision_clear,
                'feedback_valid': self.state_valid,
                'feedback_receipt_age': min(999., time.monotonic()-self.state_receipt),
                'feedback_stamp_age': min(999., (self.get_clock().now().nanoseconds-self.state_stamp)*1e-9),
                'bridge_age': min(999., time.monotonic()-self.bridge_receipt),
                'stamp_ns': self.get_clock().now().nanoseconds}
        self.status.publish(String(data=json.dumps(data, allow_nan=False)))

    def close(self):
        self.confirmed = False
        self.fail('Input node shutting down')
        deadline = time.monotonic()+.7
        while self.phase != 'idle' and rclpy.ok() and time.monotonic() < deadline:
            self.tick(); rclpy.spin_once(self, timeout_sec=.02)
        self.lease.release(); self.destroy_node()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO); node = Forwarder()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.close()
        if rclpy.ok(): rclpy.shutdown()
