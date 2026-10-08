#!/usr/bin/env python3
"""
amw_speed.py - applies the encoder speed level (1-20) to joy_teleop's commands.
Also publishes /amw/speed_level (1-20) and /amw/speed_multiplier for the LCD.
Run: python3 amw_speed.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32


class AmwSpeed(Node):
    def __init__(self):
        super().__init__("amw_speed")
        self.level_axis = self.declare_parameter("level_axis", 3).value
        self.level_min = self.declare_parameter("level_min", 1).value # must match Pico LEVEL_MIN
        self.level_max = self.declare_parameter("level_max", 20).value # must match Pico LEVEL_MAX
        self.level_neutral = self.declare_parameter("level_neutral", 10).value # level = 1.0x
        self.speed_min = self.declare_parameter("speed_min", 0.1).value # multiplier at level_min
        self.speed_max = self.declare_parameter("speed_max", 2.0).value # multiplier at level_max

        self.level = self.level_neutral
        self.mult = 1.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.level_pub = self.create_publisher(Int32, "/amw/speed_level", latched)
        self.mult_pub = self.create_publisher(Float32, "/amw/speed_multiplier", latched)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(TwistStamped, "/amw/cmd_raw", self.on_cmd, 10)
        self.publish_status()

    def level_from_axis(self, a):
        # Pico sends level_min..level_max as 0..255; joy turns that into -1..+1
        t = (max(-1.0, min(1.0, a)) + 1.0) / 2.0
        return int(round(self.level_min + t * (self.level_max - self.level_min)))

    def multiplier(self, lv):
        # neutral -> 1.0x; log-spaced so each click changes speed by the same ratio
        n = self.level_neutral
        if lv >= n:
            return self.speed_max ** ((lv - n) / (self.level_max - n))
        return self.speed_min ** ((n - lv) / (n - self.level_min))

    def publish_status(self):
        self.level_pub.publish(Int32(data=self.level))
        self.mult_pub.publish(Float32(data=float(self.mult)))

    def on_joy(self, msg):
        if self.level_axis >= len(msg.axes):
            return
        lv = self.level_from_axis(msg.axes[self.level_axis])
        if lv != self.level:
            self.level = lv
            self.mult = self.multiplier(lv)
            self.publish_status()

    def on_cmd(self, msg):
        k = self.mult
        msg.twist.linear.x *= k
        msg.twist.linear.y *= k
        msg.twist.linear.z *= k
        msg.twist.angular.x *= k
        msg.twist.angular.y *= k
        msg.twist.angular.z *= k
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = AmwSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
