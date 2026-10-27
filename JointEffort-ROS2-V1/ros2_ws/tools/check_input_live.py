#!/usr/bin/env python3
"""Synthetic Pico -> actual joy_teleop -> forwarding -> Servo -> GenericSystem test."""
import json
import hashlib
import math
import os
import signal
import re
import subprocess
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped, Pose
from moveit_msgs.srv import GetStateValidity, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from joint_effort_input.forwarder import STATUS
from joint_effort_input.lease import Lease
from joint_effort_input.synthetic import Source
from joint_effort_robot.kinematics import Robot

W = Path(__file__).resolve().parents[1]; OUT = W/'reports/iron'
assert os.environ.get('ROS_DOMAIN_ID') == '88'
logs = []; processes = []; report = {'scope': 'Synthetic USB-HID-equivalent Joy; cold mock only', 'checks': {}}
source_paths = sorted(p for p in (W/'src/joint_effort_input').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
source_paths += [W/'tools'/name for name in ['input_demo.py', 'input_probe.py', 'iron_demo.py', 'iron_motion.py', 'iron_stop.py', 'check_input_live.py']]
report['source_sha256'] = {str(p.relative_to(W)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
def start(name, command):
    log = (OUT/(name+'.log')).open('w'); logs.append(log)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process); return process

mock = start('input-mock-launch', ['ros2', 'launch', 'joint_effort_iron', 'mock.launch.py', 'rviz:=true'])
inputs = start('input-launch', ['ros2', 'launch', 'joint_effort_input', 'input.launch.py',
                              'joystick:=false', 'mapping_confirmed:=true'])
rclpy.init(); node = Node('amw_input_check'); state = {}; q = None; tasks = []; canonical = []
robot = Robot(W/'src/joint_effort_iron/urdf/iron.urdf')
source = Source()
raw = node.create_publisher(TwistStamped, '/amw/cmd_raw', 1)
def status(msg):
    global state
    state = json.loads(msg.data)
def feedback(msg):
    global q
    if len(msg.name) == 5:
        values = dict(zip(msg.name, msg.position)); q = np.array([values[n] for n in robot.names('iron')])
node.create_subscription(String, '/amw/display_state', status, STATUS)
node.create_subscription(JointState, '/joint_states', feedback, 1)
node.create_subscription(TwistStamped, '/iron/task_velocity', lambda msg: tasks.append(msg), 1)
node.create_subscription(Joy, '/amw/joy_control', lambda msg: canonical.append(msg), 1)

def pump(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=min(.01, max(0., deadline-time.monotonic())))
def emit(seconds, axes=(0., 0., 0.), hold=0, toggle=0, level=10, bad=None):
    if source.closed: raise RuntimeError('Synthetic source already closed')
    source.spec = (tuple(axes), hold, toggle, level, bad); pump(seconds)
def call(kind, path, request):
    client = node.create_client(kind, path)
    try:
        assert client.wait_for_service(timeout_sec=5), path
        future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and not future.exception(), path
        return future.result()
    finally: node.destroy_client(client)
def prepare():
    emit(.8); deadline = time.monotonic()+3
    while not state.get('prepared') or state.get('phase') != 'idle':
        assert time.monotonic() < deadline, ('Did not qualify centered release', state)
        emit(.1)
def moving(axes=(0., 0., .8), level=10, seconds=.5):
    prepare(); emit(.3, hold=1, level=level); emit(seconds, axes, hold=1, level=level)
    if not state.get('enabled'): print('Motion did not enable:', state, flush=True)
    assert state.get('enabled'), state
def settled():
    pump(.4); before = q.copy(); pump(.3)
    drift = float(np.max(np.abs(q-before))); assert drift < 1e-5, drift
    assert not state.get('enabled'), state
    return drift
def validity():
    req = GetStateValidity.Request(); req.group_name = 'iron_arm'
    req.robot_state.joint_state.name = robot.names('iron'); req.robot_state.joint_state.position = q.tolist()
    return call(GetStateValidity, '/check_state_validity', req).valid
def obstacle(remove=False):
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    obj = CollisionObject(); obj.id = 'input_tip_block'; obj.header.frame_id = 'world'
    obj.operation = obj.REMOVE if remove else obj.ADD
    if not remove:
        tip = robot.fk('iron', q)[:3, 3]; pose = Pose(); pose.orientation.w = 1.
        pose.position.x, pose.position.y, pose.position.z = map(float, tip)
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.04]*3)]
        obj.primitive_poses = [pose]
    req.scene.world.collision_objects = [obj]
    assert call(ApplyPlanningScene, '/apply_planning_scene', req).success

try:
    deadline = time.monotonic()+35
    while q is None or not state or not state.get('mock_ready'):
        assert time.monotonic() < deadline, state; pump(.1)
    prepare(); assert validity()
    emit(.4, hold=1, bad='short'); emit(.4, hold=1)
    assert not state['enabled']; report['checks']['startup_held_after_invalid_blocked'] = True
    before = robot.fk('iron', q); moving(seconds=2.)
    emit(.3); pump(.3); after = robot.fk('iron', q)
    delta = 1000*(after[:3, 3]-before[:3, 3]); assert delta[2] > 1., (delta, state)
    report['checks']['translation_tip_delta_mm'] = delta.tolist()
    report['checks']['release_hold_drift_rad'] = settled()
    for level in [1, 10, 20]:
        tasks.clear(); moving(level=level, seconds=.3)
        value = max(abs(m.twist.linear.z) for m in tasks)
        expected = 0.01*(.8-.08)/.92*({1: .1, 10: 1., 20: 2.}[level])
        assert abs(value-expected) < 1e-7, (level, value, expected, state)
        report['checks'][f'level_{level}_z_m_s'] = value; emit(.4)
    prepare(); emit(.1, toggle=1); emit(.6); assert state['mode'] == 'rotation'
    before = q.copy(); moving(axes=(.8, 0., 0.), seconds=1.)
    emit(.5); assert np.max(np.abs(q-before)) > .002
    report['checks']['rotation_joint_change_rad'] = (q-before).tolist()
    # Mode switch while held must stop and cannot resume until release/center.
    moving(axes=(.8, 0., 0.)); emit(.1, axes=(.8, 0., 0.), hold=1, toggle=1)
    emit(.4, axes=(.8, 0., 0.), hold=1); assert not state['enabled']; settled()
    report['checks']['mode_change_held_stops'] = True
    moving(); source.spec = None; pump(.45); report['checks']['joy_loss_drift_rad'] = settled()
    emit(.4, hold=1); assert not state['enabled']; report['checks']['joy_recovery_held_stays_disabled'] = True
    for bad in ['nan', 'short', 'old']:
        moving(); emit(.04, hold=1, bad=bad); emit(.4, hold=1)
        assert not state['enabled']; settled(); report['checks'][f'{bad}_joy_stops'] = True
    moving(); assert call(Trigger, '/amw/stop', Trigger.Request()).success
    emit(.4, hold=1); settled(); report['checks']['software_stop_latches'] = True
    lease = Lease(W/'work/iron-motion.lock'); prepare(); assert lease.acquire()
    emit(.5, hold=1); assert not state['enabled']; lease.release()
    emit(.3, hold=1); assert not state['enabled']; report['checks']['other_owner_and_no_auto_retry'] = True
    moving(); denied = subprocess.Popen(['python3', 'tools/iron_demo.py', 'joint', '--seconds', '1'],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.monotonic()+8
    while denied.poll() is None:
        assert time.monotonic() < deadline
        emit(.05, hold=1)
    denial = denied.communicate()[0]
    assert denied.returncode != 0 and 'owns' in denial, denial
    report['checks']['demo_refuses_while_joystick_owns'] = True
    req = SwitchController.Request(); req.deactivate_controllers = ['joint_state_broadcaster']; req.strictness = req.STRICT
    assert call(SwitchController, '/controller_manager/switch_controller', req).ok
    emit(.6, hold=1); assert not state['enabled']
    req = SwitchController.Request(); req.activate_controllers = ['joint_state_broadcaster']; req.strictness = req.STRICT
    assert call(SwitchController, '/controller_manager/switch_controller', req).ok
    emit(.6, hold=1); assert not state['enabled']; report['checks']['feedback_loss_and_recovery_requires_release'] = True
    moving(); obstacle(); emit(.7, hold=1); assert not state['enabled']
    assert state['servo_code'] == 5, state; settled(); obstacle(True); emit(.6, hold=1)
    assert not state['enabled']; report['checks']['collision_halt_no_auto_resume'] = True
    # Invalid raw commands must be rejected rather than rescaled/restamped.
    for case in ['nan', 'old', 'future', 'frame', 'replay']:
        moving(); assert raw.get_subscription_count() > 0, 'Injected command publisher has no subscriber'
        bad = TwistStamped(); bad.header.stamp = node.get_clock().now().to_msg()
        bad.header.frame_id = 'world'; bad.twist.linear.z = 0.01*(.8-.08)/.92
        if case == 'nan': bad.twist.linear.x = math.nan
        elif case == 'old': bad.header.stamp.sec -= 1
        elif case == 'future': bad.header.stamp.sec += 1
        elif case == 'frame': bad.header.frame_id = 'base_link'
        if case == 'replay':
            raw.publish(bad); emit(.03, hold=1)
        deadline = time.monotonic()+.25
        while time.monotonic() < deadline:
            raw.publish(bad); emit(.01, hold=1)
        emit(.3, hold=1); assert not state['enabled'], (case, state); settled()
        report['checks'][case+'_raw_command_stops'] = True
    moving(); pid = int(re.search(r'joy_teleop_clean-1.*process started with pid \[(\d+)\]',
                                 (OUT/'input-launch.log').read_text()).group(1))
    os.kill(pid, signal.SIGINT); emit(.5, hold=1); assert not state['enabled']; settled()
    report['checks']['joy_teleop_loss_stops'] = True
    start('input-teleop-restart', [str(W/'install/joint_effort_input/lib/joint_effort_input/joy_teleop_clean'), '--ros-args',
          '--params-file', str(W/'src/joint_effort_input/config/teleop.yaml'), '-r', 'joy:=/amw/joy_control'])
    emit(3., hold=1); assert not state['enabled']; report['checks']['teleop_restart_no_auto_resume'] = True
    moving(); emit(.4)
    prepare(); assert validity(); assert all(len(m.axes) == 3 and len(m.buttons) == 2 for m in canonical)
    report['checks']['canonical_shape_and_final_collision_clear'] = True
    plan_log = (OUT/'input-idle-plan-output.txt').open('w')
    planned = subprocess.Popen(['python3', 'tools/iron_plan.py', '--joint', 'joint_1', '--degrees', '3', '--execute'],
                               stdout=plan_log, stderr=subprocess.STDOUT)
    deadline = time.monotonic()+35
    while planned.poll() is None:
        assert time.monotonic() < deadline
        emit(.05)
    plan_log.close(); assert planned.returncode == 0, (OUT/'input-idle-plan-output.txt').read_text()
    report['checks']['planned_motion_with_idle_input_succeeds'] = True
    source.close(); deadline = time.monotonic()+4
    while node.get_publishers_info_by_topic('/joy'):
        assert time.monotonic() < deadline, 'Synthetic source endpoint did not disappear'
        pump(.1)
    demo_log = (OUT/'input-demo-output.txt').open('w')
    before_demo = q.copy()
    demo = subprocess.Popen(['python3', 'tools/input_demo.py'], stdout=demo_log, stderr=subprocess.STDOUT)
    deadline = time.monotonic()+30
    while demo.poll() is None:
        assert time.monotonic() < deadline
        pump(.05)
    demo_log.close(); assert demo.returncode == 0, (OUT/'input-demo-output.txt').read_text()
    assert np.max(np.abs(q-before_demo)) > .001
    report['checks']['documented_synthetic_demo_succeeds'] = True
    report['result'] = 'PASS'; print(json.dumps(report, indent=2), flush=True)
finally:
    try: emit(.3)
    except Exception: pass
    if not source.closed: source.close()
    node.destroy_node(); rclpy.shutdown()
    codes = []
    for process in reversed(processes):
        process.send_signal(signal.SIGINT)
        try: codes.append(process.wait(timeout=20))
        except subprocess.TimeoutExpired: process.terminate(); codes.append(process.wait(timeout=5))
    for log in logs: log.close()
    texts = [(OUT/(name+'.log')).read_text() for name in ['input-mock-launch', 'input-launch', 'input-teleop-restart']]
    report['shutdown'] = {'codes': codes, 'children_clean': all('process has died' not in t and 'Traceback' not in t for t in texts),
                          'rviz_model_ok': 'Iron prototype model: Status: Ok' in texts[0], 'tf_ok': 'Iron frames: Status: Ok' in texts[0]}
    if not all(code == 0 for code in codes) or not report['shutdown']['children_clean']: report['result'] = 'FAIL'
    (OUT/'input-check.json').write_text(json.dumps(report, indent=2)+'\n')
assert all(code == 0 for code in codes) and report['shutdown']['children_clean'], report['shutdown']
print('PASS: input pipeline and clean shutdown', flush=True)
