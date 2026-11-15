"""amw joystick: print the Pico's raw /joy axes and buttons whenever they change. Sends no arm commands.

Needs joy_node running (`amw start ...` starts it). Move one control at a time; turn the encoder fully
low and then high. Ctrl-C to stop.

Expected with Pico_Controller.ino: axes [X, Y, twist, speed level], buttons 0-5 = stick + panel 1-5,
6 = encoder push, 7 = ON (latched by the Pico), 8 = same as 7.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def main(seconds=None):
    rclpy.init()
    node = Node('amw_joystick_probe')
    state = {'last': None, 'count': 0}

    def receive(msg):
        state['count'] += 1
        value = ([round(v, 3) for v in msg.axes], list(msg.buttons))
        if value != state['last']:
            state['last'] = value
            pressed = [i for i, b in enumerate(msg.buttons) if b]
            print(f'axes {value[0]}   pressed {pressed}', flush=True)

    node.create_subscription(Joy, '/joy', receive, qos_profile_sensor_data)
    print(__doc__.split('\n\n')[1], flush=True)
    end = time.monotonic() + seconds if seconds else None
    try:
        while end is None or time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if state['count'] == 0:
            print('No /joy messages. Is `amw start` running? Is the Pico plugged in '
                  '(ls /dev/input/js*)? Does joy_node log "Opened joystick: AMW Controller"?')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
