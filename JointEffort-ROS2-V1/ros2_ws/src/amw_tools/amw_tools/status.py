"""amw status: why the arm is (not) moving, on one screen. Reads /amw/status from amw_teleop.

Shows the state (ON / OFF / CENTRE STICK), the stop reason or warning with what to do, mode and
speed, MoveIt Servo and the controller, every joint against its limits with motor temperature,
voltage and current, and the recent events with how long ago they happened.

  amw status          live (Ctrl-C to quit)
  amw status --once   one snapshot

With `amw start ... --no-teleop` there is no /amw/status; it then shows joint angles only.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .common import ROLES, load_arm

STALE = 1.0
LABEL = {'active': 'ON', 'off': 'OFF', 'centre': 'CENTRE STICK'}


class Status(Node):
    def __init__(self):
        super().__init__('amw_status')
        self.arm = load_arm()
        self.data, self.data_time = None, -math.inf
        self.q, self.q_time = None, -math.inf
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/amw/status', self.on_status, qos)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_status(self, msg):
        try:
            self.data, self.data_time = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def on_joints(self, msg):
        self.q, self.q_time = dict(zip(msg.name, msg.position)), time.monotonic()

    def render(self):
        now = time.monotonic()
        out = [f'AMW status  {time.strftime("%H:%M:%S")}', '']
        d = self.data if now - self.data_time < STALE else None
        if d is None:
            if now - self.q_time > STALE:
                out.append('Arm is not running (no /amw/status, no /joint_states). Start it with `amw start`.')
                return '\n'.join(out)
            out.append('Teleop is not running (started with --no-teleop?). Joint angles only:')
            out += [''] + self.joint_table([{'name': j.name, 'deg': math.degrees(self.q.get(j.name, math.nan)),
                                             'lower': math.degrees(j.lower), 'upper': math.degrees(j.upper)}
                                            for j in self.arm])
            return '\n'.join(out)
        stop, warn = d.get('stop'), d.get('warning')
        out.append(f'{LABEL.get(d["state"], d["state"])}   mode {d["mode"]}   speed {d["level"]}/20 '
                   f'(x{d["multiplier"]:.2f})   joystick {d["joystick"]}')
        if stop:
            out.append(f'  {"WHY" if stop["kind"] == "stop" else "WAIT"}: {stop["text"]}  [{stop["code"]}]')
            out.append(f'  FIX: {stop["fix"]}')
        if warn:
            out.append(f'  WARNING: {warn["text"]}  [{warn["code"]}]')
            out.append(f'  FIX: {warn["fix"]}')
        servo = d.get('servo') or {}
        out.append(f'Servo: {servo.get("text")}{"  - " + servo["message"] if servo.get("message") else ""}'
                   f'    controller: {d.get("controller") or "unknown"}'
                   f'    joint states {d.get("feedback_age", 0) * 1000:.0f} ms old')
        out += [''] + self.joint_table(d.get('joints') or [], d.get('command_deg_s'))
        out += ['', 'Recent (seconds ago):'] + [f'  {age:6.1f}  {text}' for age, text in reversed(d.get('events', []))]
        return '\n'.join(out)

    @staticmethod
    def joint_table(rows, command=None):
        out = [f'{"joint":17} {"angle":>8} {"limits deg":>15} {"margin":>7} {"cmd/s":>6} {"temp":>5} {"volt":>5} {"amps":>5}']
        for i, r in enumerate(rows):
            deg = r.get('deg')
            margin = None if deg is None or deg != deg else min(deg - r['lower'], r['upper'] - deg)
            flag = '  OUTSIDE' if margin is not None and margin < 0 else '  <- at limit' if margin is not None and margin < 3 else ''

            def num(x, f):
                return f'{x:{f}}' if isinstance(x, (int, float)) and x == x else '-'
            cmd = command[i] if command and i < len(command) else None
            out.append(f'{r["name"] + " " + ROLES.get(r["name"], ""):17} {num(deg, "+8.2f"):>8} '
                       f'{r["lower"]:+6.1f}..{r["upper"]:+6.1f} {num(margin, "+7.1f"):>7} {num(cmd, "+6.2f"):>6} '
                       f'{num(r.get("temp"), "5.0f"):>5} {num(r.get("volt"), "5.1f"):>5} {num(r.get("amps"), "5.2f"):>5}{flag}')
        return out


def main(once=False):
    rclpy.init()
    node = Status()
    try:
        if once:
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(node.render())
            return 0
        next_draw = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_draw:
                next_draw = time.monotonic() + 0.5
                print('\033[H\033[J' + node.render(), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
