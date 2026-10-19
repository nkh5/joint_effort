#!/usr/bin/env python3
"""Request a latched software stop. This is not a physical emergency stop."""
import os
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import SetHardwareComponentState

if os.environ.get('ROS_DOMAIN_ID') != '89': raise SystemExit('Use JE_ROS_DOMAIN_ID=89 with scripts/with_ros.sh')
rclpy.init(); node = Node('iron_hardware_stop')
request = SetHardwareComponentState.Request(); request.name = 'IronSTS'; request.target_state.id = 2
requests = [(Trigger, '/amw/stop', Trigger.Request()),
            (SetBool, '/iron_task_bridge/enable', SetBool.Request(data=False)),
            (SetHardwareComponentState, '/controller_manager/set_hardware_component_state', request)]
clients = [(node.create_client(kind, name), name, req) for kind, name, req in requests]
pending = []; hardware_ack = False
try:
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        for client, name, req in clients[:]:
            if client.service_is_ready():
                pending.append((client.call_async(req), name)); clients.remove((client,name,req))
        for future, name in pending[:]:
            if future.done():
                ok = not future.exception() and (future.result().ok if 'hardware_component' in name else future.result().success)
                print(name+': '+('acknowledged' if ok else 'not acknowledged'))
                if 'hardware_component' in name: hardware_ack = ok
                pending.remove((future,name))
        if not clients and not pending: break
        rclpy.spin_once(node, timeout_sec=.01)
    print('Torque stays enabled to support the arm. Support it before disconnecting servo power.')
    if not hardware_ack: raise SystemExit('Hardware hold was not acknowledged. Use the physical stop/power disconnect as arranged.')
finally:
    node.destroy_node(); rclpy.shutdown()
