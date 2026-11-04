#!/usr/bin/env python3
"""Synthetic joystick exercise for the already running cold mock input launch."""
import json
import os
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareComponents
from std_msgs.msg import String
from joint_effort_input.forwarder import STATUS
from rclpy.signals import SignalHandlerOptions
from joint_effort_input.synthetic import Source

if os.environ.get('ROS_DOMAIN_ID') != '88' and not (os.environ.get('ROS_DOMAIN_ID') == '188' and os.environ.get('JE_MOCK_TEST') == '1'): raise SystemExit('Use JE_ROS_DOMAIN_ID=88 with scripts/with_ros.sh')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node = Node('amw_synthetic_demo'); state = {}
# The forwarding stack can serve real motors too; this demo must not.
client = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
if not client.wait_for_service(timeout_sec=10): raise SystemExit('Start the mock arm first.')
future = client.call_async(ListHardwareComponents.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=10)
if not future.done() or future.exception(): raise SystemExit('Cannot verify mock hardware.')
hardware = future.result().component
if len(hardware) != 1 or hardware[0].plugin_name != 'mock_components/GenericSystem':
    raise SystemExit('Synthetic demo requires GenericSystem mock hardware; physical motors refused.')
node.destroy_client(client)
source = Source('amw_demo_source')
def status(msg):
    global state
    state = json.loads(msg.data)
node.create_subscription(String, '/amw/display_state', status, STATUS)


def emit(seconds, axes=(0., 0., 0.), hold=0, toggle=0, level=10):
    source.spec = (tuple(axes), hold, toggle, level, None)
    if len(node.get_publishers_info_by_topic('/joy')) > 1:
        raise RuntimeError('Another joystick publisher is present. Run input launch with joystick:=false for this synthetic demo.')
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(.01, max(0., end-time.monotonic())))


try:
    emit(1.)
    deadline = time.monotonic()+10
    while not state and time.monotonic() < deadline: emit(.2)
    if not state or not state.get('mapping_confirmed'):
        raise RuntimeError('Start scripts/run_input.sh joystick:=false mapping_confirmed:=true after run_iron.sh')
    deadline = time.monotonic()+25
    while not state.get('mock_ready') or not state.get('collision_clear'):
        if time.monotonic() > deadline: raise RuntimeError('Mock system not ready or modeled collision check not clear; inspect the launch terminals')
        emit(.4)
    def require_enabled():
        if not state.get('enabled'): raise RuntimeError(state.get('message', 'No supervisor status'))
    print('MOCK ONLY: synthetic joystick. Watch RViz and display preview.', flush=True)
    for label, axes, level in [('up at normal speed', (0., 0., .8), 10),
                               ('down at high speed', (0., 0., -.8), 20)]:
        emit(.6); emit(.3, hold=1, level=level); print(label, flush=True)
        require_enabled(); emit(2., axes, hold=1, level=level); require_enabled(); emit(.6)
        if not state.get('input_fresh'): raise RuntimeError('Input supervisor status missing')
    print('Switch to rotation; point sideways, then return', flush=True)
    emit(.1, toggle=1); emit(.6); emit(.3, hold=1)
    require_enabled(); emit(2., (.8, 0., 0.), hold=1); require_enabled(); emit(.6); emit(.3, hold=1)
    require_enabled(); emit(2., (-.8, 0., 0.), hold=1); require_enabled(); emit(.6)
    emit(.1, toggle=1); emit(.6)
    print('Finished. Enable released; mock holds position.', flush=True)
except (RuntimeError, KeyboardInterrupt) as exc:
    print('Input demo stopped:', exc, flush=True)
    raise SystemExit(1)
finally:
    try: emit(.4)
    except RuntimeError: pass
    source.close()
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
