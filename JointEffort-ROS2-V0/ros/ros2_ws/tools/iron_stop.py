#!/usr/bin/env python3
"""Request all project mock supervisors to stop (not a physical emergency stop)."""
import os
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

if os.environ.get('ROS_DOMAIN_ID') != '88':
    raise SystemExit('Use JE_ROS_DOMAIN_ID=88 with scripts/with_ros.sh')
rclpy.init(); node = Node('iron_motion_stop')
try:
    clients = {path: node.create_client(Trigger, path)
               for path in ['/amw/stop', '/iron_motion/stop']}
    pending = dict(clients); requests = {}
    deadline = time.monotonic()+3
    while pending and time.monotonic() < deadline:
        for path, client in list(pending.items()):
            if client.service_is_ready():
                requests[path] = client.call_async(Trigger.Request())
                del pending[path]
        rclpy.spin_once(node, timeout_sec=.05)
    for path, future in requests.items():
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or future.exception() or not future.result().success:
            raise SystemExit('Stop request was not acknowledged for '+path+'; inspect the motion terminal.')
        print(future.result().message)
    if not requests:
        raise SystemExit('No project motion supervisor found. This stop does not control arbitrary ROS publishers or hardware.')
    print('Check the motion terminal for cancellation/hold confirmation. This is not a physical emergency stop.')
finally:
    node.destroy_node(); rclpy.shutdown()
