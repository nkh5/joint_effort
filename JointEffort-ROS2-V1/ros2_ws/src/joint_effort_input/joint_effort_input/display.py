"""Terminal LCD preview; physical display transport awaits display identification."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .forwarder import STATUS


class Display(Node):
    def __init__(self):
        super().__init__('amw_display_preview'); self.last = -float('inf'); self.text = None
        self.create_subscription(String, '/amw/display_state', self.receive, STATUS)
        self.create_timer(.2, self.watchdog)

    def show(self, text):
        if text != self.text: print(text, flush=True); self.text = text

    def receive(self, msg):
        try:
            data = json.loads(msg.data)
            age = (self.get_clock().now().nanoseconds-data['stamp_ns'])*1e-9
            if not 0 <= age <= .5: return
            self.last = time.monotonic(); bars = round(data['speed_fraction']*10)
            self.show(f"{data['mode'].upper()} | {data['level']:02d}/20 {data['multiplier']:.2f}x "
                      f"[{'#'*bars}{'.'*(10-bars)}] | {data['phase']}\n"
                      f"{data['message']} | Servo: {data['servo_message']}")
        except (KeyError, TypeError, ValueError): self.show('DISPLAY DATA ERROR — movement status unknown')

    def watchdog(self):
        if time.monotonic()-self.last > .5: self.show('STATUS LOST — movement status unknown')


def main(args=None):
    rclpy.init(args=args); node = Display()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
