#!/usr/bin/env python3
"""Live calibration readout for the real iron arm.

Run while `scripts/run_real.sh readonly:=true` is running (torque off). Prints, per joint:
the angle ROS sees, the raw servo step behind it, and the min/max seen so far.
Ctrl-C prints a summary you can copy into sts_calibration.yaml.

The raw step is recomputed from /joint_states with the same center the launch used:
center_steps, else csv_zero_counts + encoder_shift_counts (angle then reads 0 at your physical
0 deg mark), else 2048. That is only exact in readonly mode, where the driver does not clamp
positions to limits.
"""
import math
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
DEFAULT_CENTER = 2048                       # must match READONLY_DEFAULT_CENTER in real.launch.py
STEPS_PER_RAD = 4096 / (2 * math.pi)


class Monitor(Node):
    def __init__(self, cal):
        super().__init__('sts_calibration_monitor')
        self.ids, self.centers, self.kind = {}, {}, {}
        for name in JOINTS:
            j = cal['joints'][name]
            self.ids[name] = j.get('motor_id')
            c = j.get('center_steps')
            z = j.get('csv_zero_counts')
            if isinstance(c, int):
                self.centers[name] = c
            elif isinstance(z, int):
                self.centers[name] = z + int(j.get('encoder_shift_counts') or 0)
            else:
                self.centers[name] = DEFAULT_CENTER
            self.kind[name] = 'URDF angle' if isinstance(c, int) else 'from your 0 mark'
        self.q = {}
        self.seen = {}                      # name -> [min_rad, max_rad]
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_timer(0.25, self.show)

    def on_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.centers and math.isfinite(pos):
                self.q[name] = pos
                lo_hi = self.seen.setdefault(name, [pos, pos])
                lo_hi[0], lo_hi[1] = min(lo_hi[0], pos), max(lo_hi[1], pos)

    def raw(self, name, q):
        return round(self.centers[name] - q * STEPS_PER_RAD)

    def table(self):
        rows = [f'{"joint":8} {"id":>3} {"angle deg":>10} {"angle rad":>10} {"raw step":>9}   '
                f'{"seen min..max rad":>20}   angle is']
        for name in JOINTS:
            if name not in self.q:
                rows.append(f'{name:8} {self.ids[name]!s:>3} {"(no data)":>10}')
                continue
            q = self.q[name]
            lo, hi = self.seen[name]
            rows.append(f'{name:8} {self.ids[name]!s:>3} {math.degrees(q):10.2f} {q:10.4f} '
                        f'{self.raw(name, q):9d}   {lo:9.4f} .. {hi:8.4f}   {self.kind[name]}')
        return '\n'.join(rows)

    def show(self):
        print('\033[2J\033[H' + self.table()
              + '\n\nPositive angle = raw step DEcreasing.  Ctrl-C for a summary.', flush=True)

    def summary(self):
        print('\nLast reading and the range seen per joint:')
        for name in JOINTS:
            if name in self.q:
                lo, hi = self.seen[name]
                print(f'  {name}: raw {self.raw(name, self.q[name])}  angle {self.q[name]:+.4f} rad   '
                      f'seen {lo:+.4f} .. {hi:+.4f} rad ({math.degrees(lo):+.1f} .. {math.degrees(hi):+.1f} deg)'
                      f'   [{self.kind[name]}]')


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parent.parent / 'src/joint_effort_iron/config/sts_calibration.yaml'
    cal = yaml.safe_load(path.read_text())
    rclpy.init()
    node = Monitor(cal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
