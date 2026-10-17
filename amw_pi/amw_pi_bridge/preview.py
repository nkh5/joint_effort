"""Observe Kory's Servo output and show calibrated motor targets. No serial I/O."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from .contract import read_config, preview_trajectory


class Preview(Node):
    def __init__(self):
        super().__init__('nick_trajectory_preview')
        p = Path(get_package_share_directory('amw_pi_bridge'))
        iron = Path(get_package_share_directory('joint_effort_iron'))
        path = self.declare_parameter('calibration', str(p/'config/iron_calibration.json')).value
        self.config = read_config(path, iron/'urdf/iron.urdf')
        self.publisher = self.create_publisher(String, '/nick/motor_targets_preview', 1)
        self.subscription = self.create_subscription(JointTrajectory,
            '/iron_arm_controller/joint_trajectory', self.receive, 1)
        self.last = -float('inf')
        self.timed_out = False
        self.create_timer(.05, self.watchdog)

    def emit(self, data):
        data.update(scope='dry_run_only', writes_motors=False)
        self.publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def receive(self, msg):
        try:
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            # JointTrajectory permits stamp=0 for "start now". Keep that semantic explicit.
            if stamp and not 0 <= age <= .12:
                raise ValueError('Stale/future trajectory header')
            points = [{'positions': list(p.positions), 'velocities': list(p.velocities),
                       'accelerations': list(p.accelerations), 'effort': list(p.effort),
                       'time_s': p.time_from_start.sec+p.time_from_start.nanosec*1e-9}
                      for p in msg.points]
            result = preview_trajectory(self.config, list(msg.joint_names), points)
            self.last = time.monotonic()
            self.timed_out = False
            self.emit({'valid': True, 'points': result})
        except (ValueError, TypeError) as exc:
            self.emit({'valid': False, 'points': [], 'error': str(exc)})

    def watchdog(self):
        if time.monotonic()-self.last > .12 and not self.timed_out:
            self.emit({'valid': False, 'points': [], 'error': 'No recent valid trajectory'})
            self.timed_out = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Preview()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
