"""Move ONE joint by a small angle through iron_arm_controller, then report what happened.

  ./scripts/run_jog.sh joint_2 +3        # degrees (default)
  ./scripts/run_jog.sh joint_2 -0.05 --rad
  ./scripts/run_jog.sh joint_2 +3 --time 4

Needs `./scripts/run_real.sh teleop:=false` (or the mock: `./scripts/run_iron.sh`) running, so
nothing else is sending trajectories. The other four joints keep their current *commanded* targets
(read from the controller), not their measured positions: re-commanding a joint to where it has
sagged under gravity would ratchet it a little further every jog.
Refuses steps over 10 deg, targets outside the URDF limit, and stale joint states.
"""
import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
MAX_STEP = math.radians(10)


def limits():
    root = ET.parse(Path(get_package_share_directory('joint_effort_iron')) / 'urdf/iron.urdf').getroot()
    return {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
            for j in root.findall('joint') if j.get('name') in JOINTS}


class Jog(Node):
    def __init__(self):
        super().__init__('iron_jog')
        self.q = None
        self.stamp = 0.0
        self.cmd = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(JointTrajectoryControllerState, '/iron_arm_controller/controller_state',
                                 self.on_ctrl, 10)
        self.client = ActionClient(self, FollowJointTrajectory, '/iron_arm_controller/follow_joint_trajectory')

    def on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINTS):
            self.q = [d[n] for n in JOINTS]
            self.stamp = time.monotonic()

    def on_ctrl(self, msg):
        point = msg.output if len(msg.output.positions) == len(msg.joint_names) else msg.reference
        d = dict(zip(msg.joint_names, point.positions))
        if all(n in d for n in JOINTS):
            self.cmd = [d[n] for n in JOINTS]

    def wait_command(self, timeout=3.0):
        end = time.monotonic() + timeout
        self.cmd = None
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cmd is not None:
                return list(self.cmd)
        raise SystemExit('No /iron_arm_controller/controller_state - is iron_arm_controller active?')

    def wait_state(self, timeout=3.0):
        end = time.monotonic() + timeout
        self.q = None
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None:
                return list(self.q)
        raise SystemExit('No /joint_states with all five joints - is run_real.sh (or run_iron.sh) running?')

    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.q)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('joint', choices=JOINTS)
    ap.add_argument('delta', type=float, help='step, degrees unless --rad')
    ap.add_argument('--rad', action='store_true')
    ap.add_argument('--time', type=float, default=3.0, help='seconds for the move (default 3)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])
    delta = args.delta if args.rad else math.radians(args.delta)
    if abs(delta) > MAX_STEP:
        raise SystemExit(f'Step {math.degrees(delta):.1f} deg is larger than {math.degrees(MAX_STEP):.0f} deg')
    if not 1.0 <= args.time <= 20.0:
        raise SystemExit('--time must be 1-20 s')

    rclpy.init()
    node = Jog()
    try:
        i = JOINTS.index(args.joint)
        lo, hi = limits()[args.joint]
        start = node.wait_state()
        command = node.wait_command()
        target = list(command)
        target[i] += delta
        if not lo <= target[i] <= hi:
            raise SystemExit(f'{args.joint}: target {target[i]:+.4f} rad is outside its URDF limit {lo:+.3f}..{hi:+.3f}')
        if not node.client.wait_for_server(timeout_sec=3.0):
            raise SystemExit('iron_arm_controller action server not available - is the controller active?')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * 5
        point.time_from_start = Duration(sec=int(args.time), nanosec=int((args.time % 1) * 1e9))
        goal.trajectory.points = [point]
        print(f'{args.joint}: command {command[i]:+.4f} -> {target[i]:+.4f} rad ({math.degrees(delta):+.2f} deg) '
              f'over {args.time:.1f} s (measured {start[i]:+.4f})')
        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise SystemExit('Goal rejected by iron_arm_controller')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.time + 5.0)
        result = result_future.result()
        end = node.settle(0.5)
        moved = [e - s for e, s in zip(end, start)]
        print(f'controller result: error_code {result.result.error_code if result else "timeout"}')
        print(f'{"joint":8} {"start":>9} {"end":>9} {"moved deg":>10} {"end - target deg":>17}')
        for n, s, e, m, g in zip(JOINTS, start, end, moved, target):
            flag = '  <- jogged' if n == args.joint else ('  <- MOVED?' if abs(m) > math.radians(1) else '')
            print(f'{n:8} {s:+9.4f} {e:+9.4f} {math.degrees(m):+10.2f} {math.degrees(e - g):+17.2f}{flag}')
        err = math.degrees(moved[i] - delta)
        print(f'\n{args.joint}: asked {math.degrees(delta):+.2f} deg, got {math.degrees(moved[i]):+.2f} deg '
              f'(error {err:+.2f} deg)')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
