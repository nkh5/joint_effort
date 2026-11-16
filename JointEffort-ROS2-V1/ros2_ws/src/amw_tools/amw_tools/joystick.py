"""amw joystick: print the Joy-Con input that amw_teleop receives. Sends no arm commands.

Needs amw_joycon running (`amw start ...` starts it). One line, updated 10x per second:
stick x/y (right/up should be +), held buttons by name, IMU sample rate and gyro (deg/s, bias removed).
Ctrl-C to stop.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy

BUTTONS = ['L', 'ZL', 'DPAD_UP', 'DPAD_DOWN', 'DPAD_RIGHT', 'DPAD_LEFT', 'MINUS', 'CAPTURE', 'STICK', 'SL', 'SR']


def main(seconds=None):
    rclpy.init()
    node = Node('amw_joystick_probe')
    s = {'joy': None, 'joys': 0, 'imus': 0, 'gyro': (0.0, 0.0, 0.0), 'window': time.monotonic(), 'rate': 0.0}

    def joy(msg):
        s['joy'], s['joys'] = msg, s['joys'] + 1

    def imu(msg):
        w = msg.angular_velocity
        s['gyro'], s['imus'] = (math.degrees(w.x), math.degrees(w.y), math.degrees(w.z)), s['imus'] + 1

    node.create_subscription(Joy, '/joy', joy, qos_profile_sensor_data)
    node.create_subscription(Imu, '/amw/hand_imu', imu, qos_profile_sensor_data)
    print(__doc__.split('\n\n')[1], flush=True)
    end = time.monotonic() + seconds if seconds else None
    last_print, imu_count = 0.0, 0
    try:
        while end is None or time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - s['window'] >= 1.0:
                s['rate'] = (s['imus'] - imu_count) / (now - s['window'])
                imu_count, s['window'] = s['imus'], now
            if now - last_print >= 0.1 and s['joy'] is not None:
                last_print = now
                m = s['joy']
                held = [BUTTONS[i] if i < len(BUTTONS) else str(i) for i, b in enumerate(m.buttons) if b]
                g = s['gyro']
                line = (f"stick x {m.axes[0]:+.2f} y {m.axes[1]:+.2f} | held {' '.join(held) or '-':24s} | "
                        f"IMU {s['rate']:4.0f} Hz gyro {g[0]:+6.1f} {g[1]:+6.1f} {g[2]:+6.1f} deg/s")
                sys.stdout.write('\r' + line)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if s['joys'] == 0:
            print('No /joy messages. Is `amw start` running? Does the amw_joycon log say "Ready."? '
                  'It stays silent until the Joy-Con has been held still for a second (see docs/joycon.md).')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
