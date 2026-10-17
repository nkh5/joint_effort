"""Run unchanged upstream JoyTeleop with one orderly ROS-context shutdown."""
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from joy_teleop.joy_teleop import JoyTeleop


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = JoyTeleop()
    try:
        while rclpy.ok(): rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
