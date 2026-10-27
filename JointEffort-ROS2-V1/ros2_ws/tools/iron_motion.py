"""Supervised cold-mock trajectory client shared by plan and cycle tools."""
import copy
import fcntl
import hashlib
import json
import math
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.convert import message_to_ordereddict
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from moveit_msgs.srv import GetMotionPlan, GetStateValidity, GetPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from joint_effort_robot.kinematics import Robot
from iron_trajectory import audit, retime

W = Path(__file__).resolve().parents[1]


def signature(message):
    # CDR byte streams may contain unspecified alignment padding. Compare values.
    return json.dumps(message_to_ordereddict(message), sort_keys=True, separators=(',', ':'))


class Motion(Node):
    def __init__(self):
        if os.environ.get('ROS_DOMAIN_ID') != '88':
            raise RuntimeError('Use JE_ROS_DOMAIN_ID=88 with scripts/with_ros.sh')
        (W/'work').mkdir(exist_ok=True)
        lock = (W/'work/iron-motion.lock').open('w')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise RuntimeError('Another motion tool owns the mock; release joystick enable or stop that tool first')
        try:
            super().__init__('iron_motion')
        except Exception:
            lock.close()
            raise
        self.lock = lock
        self.robot = Robot(W/'src/joint_effort_iron/urdf/iron.urdf')
        self.model = json.loads((W/'src/joint_effort_iron/config/model.json').read_text())
        self.names = self.robot.names('iron'); self.state = None; self.receipt = -math.inf
        self.stop_reason = None; self.goal_handle = None; self.scene_parts = {}; self.service_clients = {}
        self.metrics = {'executions': [], 'retiming': [], 'stop': None}
        self.create_subscription(JointState, '/joint_states', self.feedback, 1)
        self.create_subscription(PlanningScene, '/monitored_planning_scene', self.scene, 10)
        self.create_service(Trigger, '/iron_motion/stop', self.stop)
        self.action = ActionClient(self, FollowJointTrajectory, '/iron_arm_controller/follow_joint_trajectory')

    def feedback(self, msg):
        if (len(msg.name) == len(msg.position) == 5 and set(msg.name) == set(self.names)
                and all(math.isfinite(v) for v in msg.position)):
            self.state = msg; self.receipt = time.monotonic()
        else:
            self.state = None

    def scene(self, msg):
        # State-only diffs do not invalidate a path. Geometry/ACM/transform changes do.
        world = copy.deepcopy(msg.world)
        for obj in world.collision_objects:
            obj.header.stamp.sec = obj.header.stamp.nanosec = 0
        world.octomap.header.stamp.sec = world.octomap.header.stamp.nanosec = 0
        world.octomap.octomap.header.stamp.sec = world.octomap.octomap.header.stamp.nanosec = 0
        parts = {'world': world, 'acm': msg.allowed_collision_matrix}
        for key, value in parts.items():
            nonempty = (bool(value.collision_objects) or bool(value.octomap.octomap.data)) if key == 'world' else bool(value.entry_names or value.default_entry_names)
            if msg.is_diff and not nonempty:
                continue
            digest = signature(value)
            if key in self.scene_parts and digest != self.scene_parts[key]:
                self.stop_reason = 'Planning-scene geometry or collision rules changed; replan required'
            self.scene_parts[key] = digest
        if msg.robot_state.attached_collision_objects and self.scene_parts.get('ready'):
            self.stop_reason = 'Attached collision geometry changed; replan required'
        for key, value in [('transforms', msg.fixed_frame_transforms), ('padding', msg.link_padding), ('scale', msg.link_scale)]:
            if value:
                normalized = copy.deepcopy(value)
                if key == 'transforms':
                    for transform in normalized:
                        transform.header.stamp.sec = transform.header.stamp.nanosec = 0
                    normalized.sort(key=lambda t: (t.header.frame_id, t.child_frame_id))
                else:
                    normalized.sort(key=lambda v: v.link_name)
                digest = '|'.join(signature(v) for v in normalized)
                if key in self.scene_parts and digest != self.scene_parts[key]:
                    self.stop_reason = 'Planning-scene '+key+' changed; replan required'
                elif key not in self.scene_parts and self.scene_parts.get('ready'):
                    self.stop_reason = 'Planning-scene '+key+' changed; replan required'
                self.scene_parts[key] = digest

    def stop(self, request, response):
        self.stop_reason = 'User requested software stop'
        response.success = True
        response.message = 'Stop latched; active mock trajectory will be canceled. Restart the command to move again.'
        return response

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(.01, max(0., deadline-time.monotonic())))

    def fresh(self):
        if self.state is None or time.monotonic()-self.receipt > .3:
            return False
        age = (self.get_clock().now().nanoseconds-self.state.header.stamp.sec*10**9-self.state.header.stamp.nanosec)*1e-9
        q = self.q(); limits = self.robot.limits('iron')
        return 0 <= age < .3 and np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1])

    def q(self):
        return np.array([dict(zip(self.state.name, self.state.position))[j] for j in self.names])

    def guard(self):
        if self.stop_reason:
            raise RuntimeError(self.stop_reason)
        if not self.fresh():
            raise RuntimeError('Joint feedback lost, stale, invalid or outside travel limits')

    def wait(self, future, timeout, interrupt=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            self.pump(.01)
            if interrupt and self.stop_reason:
                raise RuntimeError(self.stop_reason)
            if time.monotonic() > deadline:
                raise RuntimeError('ROS request timed out')
        result = future.result()
        if result is None:
            raise RuntimeError('ROS request returned no result')
        return result

    def client(self, kind, path):
        if path not in self.service_clients:
            self.service_clients[path] = self.create_client(kind, path)
        client = self.service_clients[path]
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=.3 if self.goal_handle else 15):
            raise RuntimeError('Start scripts/run_iron.sh first; missing '+path)
        return client

    def call(self, kind, path, request):
        return self.wait(self.client(kind, path).call_async(request), .3 if self.goal_handle else 15)

    def mock(self):
        hw = self.call(ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request())
        if len(hw.component) != 1 or hw.component[0].plugin_name != 'mock_components/GenericSystem':
            raise RuntimeError('Expected one GenericSystem mock component; refusing motion')
        controllers = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        if not {'iron_arm_controller', 'joint_state_broadcaster'} <= {c.name for c in controllers.controller if c.state == 'active'}:
            raise RuntimeError('Mock controllers are not active')

    def ready(self):
        self.mock(); deadline = time.monotonic()+10
        while not self.fresh():
            if time.monotonic() > deadline:
                raise RuntimeError('No complete fresh mock feedback')
            self.pump(.05)
        description = (W/'src/joint_effort_iron/urdf/iron.urdf').read_text()
        for node in ['robot_state_publisher', 'move_group']:
            req = GetParameters.Request(names=['robot_description'])
            if self.call(GetParameters, '/'+node+'/get_parameters', req).values[0].string_value != description:
                raise RuntimeError('Running robot description differs from local model; rebuild/restart')
        req = SetBool.Request(data=False)
        if not self.call(SetBool, '/iron_task_bridge/enable', req).success:
            raise RuntimeError('Could not disable task bridge')
        # Obtain a full baseline: the monitor may publish only state diffs after startup.
        baseline = self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene
        self.scene(baseline)
        self.pump(.5)
        self.scene_parts['ready'] = True; self.guard()
        if not self.valid(self.q()):
            raise RuntimeError('Current modeled state is in collision')

    def valid(self, q):
        req = GetStateValidity.Request(); req.group_name = 'iron_arm'
        req.robot_state.joint_state.name = self.names
        req.robot_state.joint_state.position = [float(v) for v in q]
        return self.call(GetStateValidity, '/check_state_validity', req).valid

    def plan(self, start, target):
        self.guard()
        if np.max(np.abs(np.asarray(start)-target)) < .0002:
            self.noop_target = np.array(target)
            return None
        req = GetMotionPlan.Request(); m = req.motion_plan_request
        m.group_name = 'iron_arm'; m.start_state.joint_state.name = self.names
        m.start_state.joint_state.position = [float(v) for v in start]
        m.allowed_planning_time = 5.; m.num_planning_attempts = 2
        m.max_velocity_scaling_factor = .5; m.max_acceleration_scaling_factor = .5
        constraint = Constraints()
        constraint.joint_constraints = [JointConstraint(joint_name=j, position=float(v), tolerance_above=.0001, tolerance_below=.0001, weight=1.) for j, v in zip(self.names, target)]
        m.goal_constraints = [constraint]
        result = self.call(GetMotionPlan, '/plan_kinematic_path', req).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError('MoveIt could not find a valid path: '+str(result.error_code.val))
        trajectory = result.trajectory.joint_trajectory
        timing = retime(trajectory, self.names, self.robot.limits('iron'),
                        .5*self.model['mock_max_joint_velocity_rad_s'],
                        .5*self.model['mock_max_joint_acceleration_rad_s2'])
        self.metrics['retiming'].append(timing)
        if timing['time_stretch_factor'] > 1:
            print(f"Retimed controller spline: {timing['time_stretch_factor']:.3f}x duration to meet speed/acceleration limits.", flush=True)
        return trajectory

    def check(self, trajectory, cartesian_check=None):
        self.guard()
        if trajectory is None:
            if not self.valid(self.q()):
                raise RuntimeError('Current no-motion state is in collision')
            return {'duration_s': 0., 'collision_samples': 1, 'no_motion': True,
                    'max_speed_rad_s': 0., 'max_acceleration_rad_s2': 0.}
        samples, summary = audit(trajectory, self.names, self.robot.limits('iron'),
                                 self.model['mock_max_joint_velocity_rad_s'],
                                 self.model['mock_max_joint_acceleration_rad_s2'])
        for _, q in samples:
            self.guard()
            if not self.valid(q):
                raise RuntimeError('Interpolated path sample is in collision; motion refused')
            if cartesian_check:
                cartesian_check(q)
        return summary

    def cancel(self):
        if self.goal_handle is None:
            return
        reason = self.stop_reason or 'Motion interrupted'
        started = time.monotonic()
        future = self.goal_handle.cancel_goal_async()
        response = self.wait(future, 3., interrupt=False)
        terminal = self.wait(self.goal_handle.get_result_async(), 3., interrupt=False)
        latency = time.monotonic()-started
        self.pump(.3); before = self.q() if self.fresh() else None
        self.pump(.3); drift = float(np.max(np.abs(self.q()-before))) if before is not None and self.fresh() else None
        self.metrics['stop'] = {'reason': reason, 'cancel_acknowledged': bool(response.goals_canceling),
                                'terminal_status': terminal.status, 'cancel_to_terminal_s': latency,
                                'settled_drift_rad': drift}
        self.goal_handle = None
        print('Mock cancellation:', json.dumps(self.metrics['stop']), flush=True)
        if drift is not None and drift > .001:
            raise RuntimeError('Mock failed to hold after cancellation; stop the launch')

    def execute(self, trajectory):
        self.guard(); self.mock()
        if trajectory is None:
            error = float(np.max(np.abs(self.q()-self.noop_target)))
            if error > .0002 or not self.valid(self.q()):
                raise RuntimeError('No-motion target changed or is colliding; replan required')
            self.metrics['executions'].append({'live_collision_checks': 1, 'joint_error_rad': error, 'no_motion': True})
            return
        if np.max(np.abs(self.q()-np.array(trajectory.points[0].positions))) > .002:
            raise RuntimeError('Robot moved since planning; replan required')
        if not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('Mock trajectory action unavailable')
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        # Await goal acceptance even if stopped, so that no accepted goal is abandoned.
        self.goal_handle = self.wait(self.action.send_goal_async(goal), 10, interrupt=False)
        if not self.goal_handle.accepted:
            self.goal_handle = None; raise RuntimeError('Mock controller rejected trajectory')
        future = self.goal_handle.get_result_async(); deadline = time.monotonic()+trajectory.points[-1].time_from_start.sec+15
        checks = 0; next_check = 0.
        try:
            while not future.done():
                self.pump(.01); self.guard()
                if time.monotonic() >= next_check:
                    if not self.valid(self.q()):
                        raise RuntimeError('Live modeled collision detected')
                    checks += 1; next_check = time.monotonic()+.1
                if time.monotonic() > deadline:
                    raise RuntimeError('Mock execution timed out')
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError('Mock trajectory failed: '+result.result.error_string)
            self.goal_handle = None; self.pump(.15); self.guard()
            error = float(np.max(np.abs(self.q()-trajectory.points[-1].positions)))
            if error > .001:
                raise RuntimeError('Mock did not reach requested joint target')
            self.metrics['executions'].append({'live_collision_checks': checks, 'joint_error_rad': error})
        except (RuntimeError, KeyboardInterrupt) as error:
            self.stop_reason = self.stop_reason or str(error) or 'Keyboard interrupt'
            self.cancel(); raise

    def dwell(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.guard()
            if not self.valid(self.q()):
                raise RuntimeError('Live modeled collision during dwell')
            self.pump(.1)

    def close(self):
        try:
            self.cancel()
        finally:
            self.action.destroy(); self.destroy_node(); self.lock.close()


def hover(motion):
    study = json.loads((W/'reports/iron/workspace-study.json').read_text())
    digest = hashlib.sha256((W/'src/joint_effort_iron/urdf/iron.urdf').read_bytes()).hexdigest()
    if study.get('urdf_sha256') != digest:
        raise RuntimeError('Hover study is outdated; repeat workspace/integration checks')
    candidates = [t for t in study['targets'] if t['geometric_solution'] and (t.get('collision_check') or {}).get('valid')]
    if not candidates:
        raise RuntimeError('No screened hover candidates')
    return min(candidates, key=lambda t: np.linalg.norm(np.array(t['geometric_solution']['q_rad'])-motion.q()))
