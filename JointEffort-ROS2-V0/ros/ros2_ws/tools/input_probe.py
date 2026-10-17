#!/usr/bin/env python3
"""Read-only Pico Joy mapping probe. Publishes no arm commands."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from rclpy.signals import SignalHandlerOptions

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=30.)
args = parser.parse_args()
if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300: parser.error('Use 1–300 seconds')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node = Node('amw_mapping_probe'); records = []; last = None


def receive(msg):
    global last
    snapshot = {'axes': [round(v, 4) for v in msg.axes], 'buttons': list(msg.buttons),
                'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec}
    records.append(snapshot)
    value = (snapshot['axes'], snapshot['buttons'])
    if value != last:
        print('axes:', snapshot['axes'], 'buttons:', snapshot['buttons'], flush=True); last = value


node.create_subscription(Joy, '/joy', receive, qos_profile_sensor_data)
print('READ ONLY: move one control at a time; turn encoder fully low then high. Ctrl+C ends.', flush=True)
try:
    end = time.monotonic()+args.seconds
    while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.1)
except KeyboardInterrupt: pass
finally:
    out = Path(__file__).resolve().parents[1]/'reports/iron/input-mapping-observed.json'
    out.write_text(json.dumps({'scope': 'Observed Joy data only; not calibration approval', 'samples': records}, indent=2)+'\n')
    print(f'{len(records)} reports saved to {out}', flush=True)
    if not records: print('No joystick reports. Check Pico USB passthrough, device name, and input permissions.', flush=True)
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
