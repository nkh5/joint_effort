#!/usr/bin/env python3
"""Self-contained ROS mock cycle and interruption tests; never opens a serial port."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

W = Path(__file__).resolve().parents[1]; OUT = W/'reports/iron'
assert os.environ.get('ROS_DOMAIN_ID') == '88'
launch_log = (OUT/'cycle-test-launch.log').open('w')
launch = subprocess.Popen(['ros2', 'launch', 'joint_effort_iron', 'mock.launch.py', 'rviz:=true'],
                          stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init(); node = Node('iron_cycle_check'); q = None; received = -1; active = None; report = {'checks': {}}


def feedback(msg):
    global q, received
    if len(msg.name) == 5 and len(msg.position) == 5:
        data = dict(zip(msg.name, msg.position)); q = np.array([data[f'joint_{i}'] for i in range(1, 6)])
        received = time.monotonic()


node.create_subscription(JointState, '/joint_states', feedback, 1)


def pump(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)


def call(kind, path, request):
    client = node.create_client(kind, path)
    try:
        assert client.wait_for_service(timeout_sec=10), path
        future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done(), path
        return future.result()
    finally:
        node.destroy_client(client)


def start(case, tool, args):
    global active
    output = (OUT/(case+'-output.txt')).open('w')
    active = subprocess.Popen(['python3', str(W/'tools'/tool), *args], stdout=output, stderr=subprocess.STDOUT)
    return active, output


def finish(case, process, output, expected):
    deadline = time.monotonic()+95
    while process.poll() is None:
        assert time.monotonic() < deadline, case+' timed out'
        pump(.1)
    output.close(); text = (OUT/(case+'-output.txt')).read_text()
    print(case+': '+text, flush=True); assert process.returncode == expected, text
    result_path = OUT/('cycle-last.json' if 'cycle' in case else 'plan-last.json')
    result = json.loads(result_path.read_text()); (OUT/(case+'.json')).write_text(json.dumps(result, indent=2)+'\n')
    return result


def moving_approach(case):
    process, output = start(case, 'iron_cycle.py', ['--execute'])
    deadline = time.monotonic()+60
    while 'Cycle 1: approach' not in (OUT/(case+'-output.txt')).read_text():
        assert process.poll() is None, (OUT/(case+'-output.txt')).read_text()
        assert time.monotonic() < deadline
        pump(.1)
    pump(.4); return process, output


def obstacle(remove=False):
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    box = CollisionObject(); box.id = 'cycle_scene_change'; box.header.frame_id = 'world'
    box.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
    if not remove:
        pose = Pose(); pose.orientation.w = 1.; pose.position.x = .8; pose.position.y = .3; pose.position.z = .2
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.01]*3)]; box.primitive_poses = [pose]
    req.scene.world.collision_objects = [box]
    assert call(ApplyPlanningScene, '/apply_planning_scene', req).success


try:
    deadline = time.monotonic()+30
    while q is None or time.monotonic()-received > .3:
        assert time.monotonic() < deadline, 'No mock feedback'
        pump(.1)
    pump(3.)
    process, output = start('cycle-normal', 'iron_cycle.py', ['--execute', '--cycles', '2'])
    normal = finish('cycle-normal', process, output, 0)
    assert normal['result'] == 'PASS' and len(normal['cycles']) == 2
    for phase in normal['cycles']:
        baseline = phase.get('hover_xyz_m', normal['target']['hover_xyz_m'])
        down = np.array(phase['approach_xyz_m'])-baseline
        back = np.array(phase['retreat_xyz_m'])-baseline
        assert abs(down[2]+.010) < .0002 and np.linalg.norm(down[:2]) < .0005
        assert np.linalg.norm(back) < .0002
    assert normal['target']['closest_tip_clearance_mm'] >= 9.8
    report['checks']['two_complete_cycles'] = normal

    process, output = moving_approach('cycle-stop')
    # A second motion client must refuse ownership while the first one is active.
    other = subprocess.run(['python3', str(W/'tools/iron_plan.py')], capture_output=True, text=True, timeout=10)
    assert other.returncode == 1 and 'Another motion tool owns the mock' in other.stdout
    report['checks']['exclusive_motion_client'] = True
    stopped = subprocess.run(['python3', str(W/'tools/iron_stop.py')], capture_output=True, text=True, timeout=10)
    assert stopped.returncode == 0, stopped.stdout+stopped.stderr
    result = finish('cycle-stop', process, output, 1)
    assert result['supervision']['stop']['cancel_acknowledged']
    assert result['supervision']['stop']['settled_drift_rad'] < .00001
    report['checks']['software_stop'] = result['supervision']['stop']

    process, output = moving_approach('cycle-interrupt'); process.send_signal(signal.SIGINT)
    result = finish('cycle-interrupt', process, output, 1)
    assert result['supervision']['stop']['cancel_acknowledged']
    assert result['supervision']['stop']['settled_drift_rad'] < .00001
    report['checks']['keyboard_interrupt'] = result['supervision']['stop']

    process, output = moving_approach('cycle-scene-change'); obstacle()
    result = finish('cycle-scene-change', process, output, 1)
    assert 'scene' in result['reason'].lower()
    assert result['supervision']['stop']['cancel_acknowledged']
    assert result['supervision']['stop']['settled_drift_rad'] < .00001
    report['checks']['scene_change'] = result['supervision']['stop']; obstacle(True); pump(.6)

    process, output = moving_approach('cycle-feedback-loss')
    req = SwitchController.Request(); req.deactivate_controllers = ['joint_state_broadcaster']; req.strictness = req.STRICT
    assert call(SwitchController, '/controller_manager/switch_controller', req).ok
    result = finish('cycle-feedback-loss', process, output, 1)
    assert 'feedback' in result['reason'].lower() and result['supervision']['stop']['cancel_acknowledged']
    req = SwitchController.Request(); req.activate_controllers = ['joint_state_broadcaster']; req.strictness = req.STRICT
    assert call(SwitchController, '/controller_manager/switch_controller', req).ok
    pump(.5); before = q.copy(); pump(.5)
    drift = float(np.max(abs(q-before))); assert drift < .00001
    report['checks']['feedback_loss'] = {'stop': result['supervision']['stop'], 'recovery_drift_rad': drift}

    # Existing joint and hover plan interfaces use the same supervisor now.
    process, output = start('plan-supervised-joint', 'iron_plan.py', ['--joint', 'joint_1', '--degrees', '8', '--execute'])
    result = finish('plan-supervised-joint', process, output, 0)
    assert result['supervision']['executions'][0]['live_collision_checks'] > 0
    report['checks']['existing_joint_plan_interface'] = result
    process, output = start('plan-supervised', 'iron_plan.py', ['--hover', '--execute'])
    result = finish('plan-supervised', process, output, 0)
    assert result['supervision']['executions'][0]['live_collision_checks'] > 0
    report['checks']['existing_plan_interface'] = result
    report['result'] = 'PASS'
finally:
    if active is not None and active.poll() is None:
        active.send_signal(signal.SIGINT)
        try: active.wait(timeout=5)
        except subprocess.TimeoutExpired: active.terminate(); active.wait(timeout=5)
    node.destroy_node(); rclpy.shutdown(); launch.send_signal(signal.SIGINT)
    try: code = launch.wait(timeout=20)
    except subprocess.TimeoutExpired: launch.terminate(); code = launch.wait(timeout=5)
    launch_log.close(); text = (OUT/'cycle-test-launch.log').read_text()
    report['shutdown'] = {'launch_code': code, 'children_clean': 'process has died' not in text,
                          'rviz_model_ok': 'Iron prototype model: Status: Ok' in text,
                          'tf_ok': 'Iron frames: Status: Ok' in text}
    complete = report.get('result') == 'PASS' and code == 0 and all(report['shutdown'][key] for key in ['children_clean', 'rviz_model_ok', 'tf_ok'])
    report['result'] = 'PASS' if complete else 'FAIL'
    (OUT/'cycle-check.json').write_text(json.dumps(report, indent=2)+'\n')
assert report['result'] == 'PASS' and code == 0 and all(report['shutdown'][key] for key in ['children_clean', 'rviz_model_ok', 'tf_ok'])
print('PASS: repeated cold cycle, stop, Ctrl+C, scene change, feedback loss, existing plan and clean GUI shutdown.')
