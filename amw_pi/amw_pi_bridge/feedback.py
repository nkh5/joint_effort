"""Read-only preview of measured joints using the unchanged amw_motor_tools SDK layer."""
import json
import time
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .amw_motors import open_bus, BusError
from .contract import read_config, sample_all


class Feedback(Node):
    def __init__(self):
        super().__init__('nick_motor_feedback')
        self.bus_context = None
        package = Path(get_package_share_directory('amw_pi_bridge'))
        iron = Path(get_package_share_directory('joint_effort_iron'))
        path = self.declare_parameter('calibration', str(package/'config/iron_calibration.json')).value
        self.config = read_config(path, iron/'urdf/iron.urdf')
        sdk = self.declare_parameter('sdk_path', '').value
        hz = float(self.declare_parameter('poll_hz', 5.).value)
        if not 1 <= hz <= 10:
            raise ValueError('poll_hz must be 1..10; sequential bench reads are not a 50 Hz control loop')
        self.states = self.create_publisher(JointState, '/nick/joint_states_preview', 1)
        self.status = self.create_publisher(String, '/nick/motor_feedback_status', 1)
        args = SimpleNamespace(sdk=sdk, port=self.config['serial_port'],
                               baud=self.config['baud_rate'], retries=2)
        context = open_bus(args)
        self.bus = context.__enter__()
        self.bus_context = context
        self.create_timer(1/hz, self.tick)
        self.get_logger().info('Read-only hardware preview; no motor writes; stop other serial owners first')

    def tick(self):
        start = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        try:
            data = sample_all(self.bus, self.config)
            elapsed = time.monotonic()-start
            if elapsed > .25:
                raise ValueError(f'Feedback batch took {elapsed:.3f}s; discarded as stale')
            msg = JointState()
            # Oldest-read timestamp, not the end of a slow scan. Never restamp cached values.
            msg.header.stamp = stamp
            msg.name = [r['name'] for r in data]
            msg.position = [r['position'] for r in data]
            # No velocity/effort estimate is fabricated from these bench samples.
            self.states.publish(msg)
            status = {'ok': True, 'scope': 'read_only_preview', 'batch_seconds': elapsed, 'motors': data}
        except (BusError, OSError, ValueError) as exc:
            status = {'ok': False, 'scope': 'read_only_preview', 'error': str(exc)}
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))

    def close(self):
        if self.bus_context is not None:
            self.bus_context.__exit__(None, None, None)
            self.bus_context = None
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Feedback()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
