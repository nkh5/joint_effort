"""amw_joycon: read a Nintendo Joy-Con (L) through the Linux hid-nintendo driver and publish

  /joy            sensor_msgs/Joy, 100 Hz: axes [stick x, stick y] (-1..1, rest position removed,
                  right = +x, up = +y), buttons in BUTTONS order (1 = held)
  /amw/hand_imu   sensor_msgs/Imu, every IMU sample (~200 Hz): angular_velocity in rad/s with the gyro
                  bias removed, linear_acceleration in m/s^2, stamped from the Joy-Con's own sample clock
                  (MSC_TIMESTAMP), so samples that arrive bundled still get their real spacing

At start (and after every reconnect) it measures the gyro bias and the stick rest position: leave the
Joy-Con still, e.g. on the table, for about a second. Nothing is published until that is done.

The kernel shows a Joy-Con as two input devices: "Joy-Con (L)" (stick + buttons) and
"Joy-Con (L) (IMU)". joycond must NOT be running: it grabs the devices and this node then sees nothing.
`device_name:=Pro Controller` also works (left stick, L, ZL, D-pad), over USB or Bluetooth.
"""
import math
import select
import threading
import time

BUTTONS = ['L', 'ZL', 'DPAD_UP', 'DPAD_DOWN', 'DPAD_RIGHT', 'DPAD_LEFT', 'MINUS', 'CAPTURE', 'STICK', 'SL', 'SR']

# Linux input event codes (linux/input-event-codes.h), so the parsing below needs no evdev import.
EV_SYN, EV_KEY, EV_ABS, EV_MSC = 0, 1, 3, 4
SYN_REPORT, MSC_TIMESTAMP = 0, 5
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y = 0, 1, 2, 3, 4, 5, 16, 17
KEYS = {310: 'L', 312: 'ZL', 544: 'DPAD_UP', 545: 'DPAD_DOWN', 546: 'DPAD_LEFT', 547: 'DPAD_RIGHT',
        314: 'MINUS', 309: 'CAPTURE', 317: 'STICK', 311: 'SL', 313: 'SR'}
ACCEL_COUNTS_PER_G, GYRO_COUNTS_PER_DPS = 4096.0, 14247.0     # hid-nintendo defaults (absinfo overrides)
G = 9.80665


class Pad:
    """Stick and buttons from the Joy-Con's main input device."""

    def __init__(self, stick_max=32767.0, invert_y=True):
        self.stick_max, self.invert_y = float(stick_max), invert_y
        self.raw = [0.0, 0.0]
        self.rest = [0.0, 0.0]
        self.held = dict.fromkeys(BUTTONS, 0)

    def event(self, kind, code, value):
        if kind == EV_KEY and code in KEYS:
            self.held[KEYS[code]] = 1 if value else 0
        elif kind == EV_ABS and code in (ABS_X, ABS_Y):
            self.raw[code] = max(-1.0, min(1.0, value / self.stick_max))
        elif kind == EV_ABS and code == ABS_HAT0X:        # Pro Controller D-pad
            self.held['DPAD_LEFT'], self.held['DPAD_RIGHT'] = int(value < 0), int(value > 0)
        elif kind == EV_ABS and code == ABS_HAT0Y:
            self.held['DPAD_UP'], self.held['DPAD_DOWN'] = int(value < 0), int(value > 0)

    def stick(self):
        x = self.raw[0] - self.rest[0]
        y = self.raw[1] - self.rest[1]
        return [max(-1.0, min(1.0, x)), max(-1.0, min(1.0, -y if self.invert_y else y))]

    def buttons(self):
        return [self.held[b] for b in BUTTONS]


class ImuStream:
    """IMU device events -> samples (device time s, accel m/s^2, gyro rad/s, raw gyro bias not removed)."""

    def __init__(self, accel_per_g=ACCEL_COUNTS_PER_G, gyro_per_dps=GYRO_COUNTS_PER_DPS):
        self.accel_scale = G / accel_per_g
        self.gyro_scale = math.radians(1.0) / gyro_per_dps
        self.values = [0.0] * 6
        self.stamp_us = None          # MSC_TIMESTAMP of the current sample
        self.prev_us = None
        self.device_time = 0.0        # unwrapped seconds

    def event(self, kind, code, value, arrival):
        """Feed one event. Returns a sample tuple on SYN_REPORT, else None."""
        if kind == EV_ABS and ABS_X <= code <= ABS_RZ:
            self.values[code] = value
        elif kind == EV_MSC and code == MSC_TIMESTAMP:
            self.stamp_us = value & 0xFFFFFFFF
        elif kind == EV_SYN and code == SYN_REPORT:
            if self.stamp_us is None:                     # no timestamps from this kernel: use arrival
                self.device_time = arrival
            elif self.prev_us is not None:
                self.device_time += ((self.stamp_us - self.prev_us) & 0xFFFFFFFF) * 1e-6   # 32-bit wrap
            self.prev_us, self.stamp_us = self.stamp_us, None
            v = self.values
            return (self.device_time, [a * self.accel_scale for a in v[:3]], [g * self.gyro_scale for g in v[3:]])
        return None


class Clock:
    """Joy-Con sample clock -> ROS time. Keeps the smallest delay seen (samples arrive bundled, so the
    earliest arrival is closest to the true offset); re-syncs on a jump."""

    def __init__(self, resync_s=0.2):
        self.offset = None
        self.resync_s = resync_s

    def to_ros(self, device_time, arrival):
        candidate = arrival - device_time
        if self.offset is None or candidate < self.offset or candidate - self.offset > self.resync_s:
            self.offset = candidate
        return device_time + self.offset


class Calibration:
    """Gyro bias and stick rest position, measured while the Joy-Con lies still."""

    def __init__(self, seconds=1.0, still_deg_s=1.5):
        self.seconds, self.still = seconds, math.radians(still_deg_s)
        self.reset()

    def reset(self):
        self.samples, self.sticks, self.start = [], [], None
        self.bias = None

    @property
    def done(self):
        return self.bias is not None

    def add(self, t, gyro, stick_raw):
        """Returns True once calibrated. Motion restarts the window."""
        if self.done:
            return True
        if self.start is None:
            self.start = t
        self.samples.append(gyro)
        self.sticks.append(list(stick_raw))
        mean = [sum(s[i] for s in self.samples) / len(self.samples) for i in range(3)]
        if any(abs(s[i] - mean[i]) > self.still for s in self.samples[-1:] for i in range(3)):
            self.samples, self.sticks, self.start = [], [], None      # it moved: start over
            return False
        if t - self.start >= self.seconds and len(self.samples) >= 20:
            self.bias = mean
            self.rest = [sum(s[i] for s in self.sticks) / len(self.sticks) for i in range(2)]
            if any(abs(r) > 0.25 for r in self.rest):                  # stick was held, not resting
                self.rest = [0.0, 0.0]
            return True
        return False


# ---------------------------------------------------------------------------------------- ROS node

def main(args=None):
    import evdev
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, Joy

    class JoyConNode(Node):
        def __init__(self):
            super().__init__('amw_joycon')
            # Attribute names below avoid rclpy.Node's own (handle, name, clock, executor, context...).
            self.device_name = self.declare_parameter('device_name', 'Joy-Con (L)').value
            rate = self.declare_parameter('publish_rate_hz', 100.0).value
            self.invert_y = self.declare_parameter('invert_y', True).value
            self.cal = Calibration(self.declare_parameter('calibration_s', 1.0).value,
                                   self.declare_parameter('still_deg_s', 1.5).value)
            self.joy_pub = self.create_publisher(Joy, '/joy', qos_profile_sensor_data)
            self.imu_pub = self.create_publisher(Imu, '/amw/hand_imu', qos_profile_sensor_data)
            self.lock = threading.Lock()
            self.pad = self.imu = None
            self.devices = []
            self.last_imu = -math.inf
            self.running = True
            self.note = None
            self.create_timer(1.0 / rate, self.publish_joy)
            self.thread = threading.Thread(target=self.read_loop, daemon=True)
            self.thread.start()

        def say(self, text, warn=False):
            if text != self.note:
                self.note = text
                (self.get_logger().warning if warn else self.get_logger().info)(text)

        # ---- device discovery
        def open_devices(self):
            found = [evdev.InputDevice(p) for p in evdev.list_devices()]
            imu = [d for d in found if self.device_name in d.name and 'IMU' in d.name]
            pad = [d for d in found if self.device_name in d.name and 'IMU' not in d.name and 'Combined' not in d.name]
            if not pad or not imu:
                for d in found:
                    d.close()
                self.say(f'Waiting for "{self.device_name}": pair it over Bluetooth (bluetoothctl) and check that '
                         f'/dev/input/event* is readable (udev rule, docs/joycon.md). Found: '
                         f'{sorted(d.name for d in found) or "no readable input devices"}', warn=True)
                return False
            for d in found:
                if d is not pad[0] and d is not imu[0]:
                    d.close()
            pad, imu = pad[0], imu[0]
            stick = pad.absinfo(ABS_X)
            accel, gyro = imu.absinfo(ABS_X), imu.absinfo(ABS_RX)
            with self.lock:
                self.pad = Pad(stick.max if stick and stick.max else 32767, self.invert_y)
                self.stream = ImuStream(accel.resolution or ACCEL_COUNTS_PER_G, gyro.resolution or GYRO_COUNTS_PER_DPS)
                self.sample_clock = Clock()
                self.cal.reset()
            self.devices = [pad, imu]
            self.imu_dev = imu
            self.opened_at = time.monotonic()
            self.say(f'Opened "{pad.name}" and "{imu.name}". Leave the Joy-Con still for a second '
                     '(measuring gyro bias and stick centre)...')
            return True

        def close_devices(self):
            for d in self.devices:
                try:
                    d.close()
                except OSError:
                    pass
            self.devices = []
            with self.lock:
                self.pad = None

        # ---- reader thread
        def read_loop(self):
            while self.running and rclpy.ok():
                if not self.devices:
                    if not self.open_devices():
                        time.sleep(1.0)
                        continue
                try:
                    ready, _, _ = select.select(self.devices, [], [], 0.1)
                    for dev in ready:
                        for e in dev.read():
                            self.on_event(dev, e)
                except (OSError, ValueError):
                    self.say('Joy-Con disconnected; waiting for it to come back...', warn=True)
                    self.close_devices()
                    continue
                if time.monotonic() - max(self.last_imu, self.opened_at) > 2.0:
                    self.say('No motion data from the Joy-Con. Is joycond running (it grabs the device)? '
                             'Run: sudo systemctl stop joycond', warn=True)

        def on_event(self, dev, e):
            now = time.monotonic()
            with self.lock:
                if dev is not self.imu_dev:
                    self.pad.event(e.type, e.code, e.value)
                    return
                sample = self.stream.event(e.type, e.code, e.value, now)
                if sample is None:
                    return
                self.last_imu = now
                t, accel, gyro = sample
                if not self.cal.done:
                    if self.cal.add(t, gyro, self.pad.raw):
                        self.pad.rest = self.cal.rest
                        b = [math.degrees(v) for v in self.cal.bias]
                        self.say(f'Ready. Gyro bias {b[0]:+.2f} {b[1]:+.2f} {b[2]:+.2f} deg/s, stick centre '
                                 f'{self.cal.rest[0]:+.3f} {self.cal.rest[1]:+.3f}.')
                    return
                stamp = self.sample_clock.to_ros(t, time.time())
            msg = Imu()
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            msg.header.frame_id = 'joycon'
            msg.orientation_covariance[0] = -1.0                  # no orientation estimate
            g = [w - b for w, b in zip(gyro, self.cal.bias)]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
            self.imu_pub.publish(msg)

        # ---- 100 Hz Joy
        def publish_joy(self):
            with self.lock:
                if self.pad is None or not self.cal.done or time.monotonic() - self.last_imu > 0.2:
                    return                                  # not connected / not calibrated: stay silent
                axes, buttons = self.pad.stick(), self.pad.buttons()
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'joycon'
            msg.axes, msg.buttons = axes, buttons
            self.joy_pub.publish(msg)

    rclpy.init(args=args)
    node = JoyConNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.close_devices()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
