#!/usr/bin/env python3
"""Jog one joint (or all five at once) through a fixed sequence while recording every joint, then plot it.

Needs `./scripts/run_real.sh teleop:=false` running (or the mock `./scripts/run_iron.sh`). From ros2_ws:

  JE_ROS_DOMAIN_ID=88 ./scripts/with_ros.sh python3 tools/jog_track.py joint_3
  ... joint_3 --note "after tightening horn"                (text added to the plot title)
  ... joint_2 --moves +3,-3,+1,-1 --time 2 --hold 1.5      (these are the defaults)
  ... all                                                   (all five joints do the same moves together)

First it drives the whole arm to the same start pose (default: DEFAULT_START below; --start park for the SRDF
park pose, --start q1,q2,q3,q4,q5 in radians, or --start none to test from wherever the arm is), so every
test runs at the same load.
The move there is slow (about 8 deg/s); it asks for Enter first if any joint has to travel more than 30 deg.
Moves are relative steps in degrees, each over --time seconds, with a --hold pause after each one.
The other joints keep their *commanded* targets (same as run_jog.sh), so anything they do is the
arm reacting to the jogged joint. With 'all', every joint moves by the same steps at the same time (like
the joystick moving several joints at once) and each one gets its own row in the table and plot.
Every /joint_states message is recorded (rate = the controller's update_rate, 50 Hz by default) together
with the controller's reference for every joint.

Output: results/jogtrack-<joint>-<time>/  plot.png, tip.png, summary.txt, samples.csv, controller.csv
  track err  = measured - controller reference during the move (rms / max), from the controller's own state
  shake      = rms of the measured motion around its own 0.2 s running average (jitter, steps)
  reversals  = times the jogged joint changed direction during a one-way move (0 = smooth)
  settle     = final position - target after the hold;   hold pk-pk = wobble while holding
  others     = largest peak-peak motion of any other joint (the arm shaking in response)
  tip block  = the same motion at the soldering tip (iron_tcp, forward kinematics of all 5 joints), in mm:
               err = |measured tip - where the reference joint angles put it| (lag + offsets),
               jitter = the measured tip around its own 0.2 s running average (the jaggedness; rms and max),
               settle = tip error after the hold, hold span = how far the tip wanders while holding
"""
import argparse
import csv
import math
import statistics
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
WS = Path(__file__).resolve().parent.parent
D = math.degrees
COUNT_DEG = 360 / 4096
# Default start pose for every test (rad): the arm's test pose, measured Nov 6 2026 with the monitor.
DEFAULT_START = [0.0, 0.1396, -0.4694, 0.0, 1.5002]


def urdf_limits():
    candidates = [WS / 'src/joint_effort_iron/urdf/iron.urdf']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, Path(get_package_share_directory('joint_effort_iron')) / 'urdf/iron.urdf')
    except Exception:
        pass
    for p in candidates:
        if p.exists():
            root = ET.parse(p).getroot()
            return {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
                    for j in root.findall('joint') if j.get('name') in JOINTS}
    raise SystemExit('iron.urdf not found')


def park_pose():
    """The SRDF 'park' pose (the arm's home), installed copy first."""
    candidates = [WS / 'src/joint_effort_iron/config/iron.srdf']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, Path(get_package_share_directory('joint_effort_iron')) / 'config/iron.srdf')
    except Exception:
        pass
    for p in candidates:
        if p.exists():
            for gs in ET.parse(p).getroot().findall('group_state'):
                if gs.get('name') == 'park':
                    d = {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
                    return [d[n] for n in JOINTS]
    raise SystemExit("No 'park' pose in iron.srdf - pass --start q1,q2,q3,q4,q5 or --start none")


GAIN_KEYS = [('p_coefficient', 'P'), ('i_coefficient', 'I'), ('d_coefficient', 'D'), ('deadband', 'db')]


def gains_from_description(text):
    """Gains and servo_acc exactly as the running arm was launched (from /robot_description)."""
    root = ET.fromstring(text)
    out, acc = {}, False
    for rc in root.iter('ros2_control'):
        for j in rc.findall('joint'):
            prm = {p.get('name'): (p.text or '').strip() for p in j.findall('param')}
            out[j.get('name')] = {k: prm.get(k) for k, _ in GAIN_KEYS}
            acc |= any(c.get('name') == 'acceleration' for c in j.findall('command_interface'))
    return out, acc


def gains_from_yaml():
    import yaml
    data = yaml.safe_load((WS / 'src/joint_effort_iron/config/sts_calibration.yaml').read_text())
    return {n: {k: data['joints'][n].get(k) for k, _ in GAIN_KEYS} for n in JOINTS}


def gains_line(gains, acc, note, source):
    def one(n):
        g = gains.get(n, {})
        if all(g.get(k) in (None, '') for k, _ in GAIN_KEYS):
            return f'{n[-1]}: servo own'
        return f'{n[-1]}: ' + '/'.join(str(g.get(k)) if g.get(k) not in (None, '') else '?' for k, _ in GAIN_KEYS)
    txt = 'P/I/D/db per joint  ' + '   '.join(one(n) for n in JOINTS)
    if source != 'running arm':
        txt += f'  [{source}]'
    return txt + (f'  |  {note}' if note else '')


class Recorder(Node):
    def __init__(self):
        super().__init__('iron_jog_track')
        self.lock = threading.Lock()
        # Time base = the message header stamps (when the controller sampled the motors), not the time this
        # script happened to receive them: on a busy laptop messages arrive in bursts.
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.events = [(-1e9, 'start')]     # (t, segment) - when each phase began, same time base
        self.samples = []            # (t, [q1..q5], segment label)          from /joint_states
        self.ctrl = []               # (t, [reference x5], [feedback x5], segment)   from the controller
        self.cmd = None              # latest controller output/reference (for holding the others)
        self.q = None
        self.segment = 'start'
        self.description = None
        self.create_subscription(String, '/robot_description', self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_state, 50)
        self.create_subscription(JointTrajectoryControllerState, '/iron_arm_controller/controller_state',
                                 self.on_ctrl, 50)
        self.client = ActionClient(self, FollowJointTrajectory, '/iron_arm_controller/follow_joint_trajectory')

    def stamp(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0:                                   # no stamp: fall back to the receive time
            t = self.get_clock().now().nanoseconds * 1e-9
        return t - self.t0

    def set_segment(self, name):
        with self.lock:
            self.segment = name
            self.events.append((self.get_clock().now().nanoseconds * 1e-9 - self.t0, name))

    def on_description(self, msg):
        self.description = msg.data

    def on_ctrl(self, msg):
        d = dict(zip(msg.joint_names, msg.reference.positions)) if msg.reference.positions else {}
        fb = dict(zip(msg.joint_names, msg.feedback.positions)) if msg.feedback.positions else {}
        out = msg.output if len(msg.output.positions) == len(msg.joint_names) else msg.reference
        o = dict(zip(msg.joint_names, out.positions))
        with self.lock:
            if all(n in o for n in JOINTS):
                self.cmd = [o[n] for n in JOINTS]
            if all(n in d and n in fb for n in JOINTS):
                self.ctrl.append((self.stamp(msg), [d[n] for n in JOINTS], [fb[n] for n in JOINTS], self.segment))

    def on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        if not all(n in d for n in JOINTS):
            return
        q = [d[n] for n in JOINTS]
        with self.lock:
            self.q = q
            self.samples.append((self.stamp(msg), q, self.segment))


def wait_for(fn, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = fn()
        if v is not None:
            return v
        time.sleep(0.05)
    raise SystemExit(what)


def running_mean(x, n):
    h = n // 2
    return [statistics.fmean(x[max(0, i - h):i + h + 1]) for i in range(len(x))]


def load_robot():
    """Forward kinematics from the team's kinematics module (same URDF the arm runs)."""
    try:
        from joint_effort_robot.kinematics import Robot
        from ament_index_python.packages import get_package_share_directory
        urdf = Path(get_package_share_directory('joint_effort_iron')) / 'urdf/iron.urdf'
        if not urdf.exists():
            urdf = WS / 'src/joint_effort_iron/urdf/iron.urdf'
        return Robot(urdf)
    except Exception as e:
        print(f'(tip analysis skipped: {e})')
        return None


def tip_series(robot, ctrl):
    """[(t, measured tip xyz [m], reference tip xyz [m], segment)] from the controller's own state."""
    out = []
    for t, ref, fb, seg in ctrl:
        try:
            pm = robot.fk('iron', fb)[:3, 3]
            pr = robot.fk('iron', ref)[:3, 3]
        except Exception:
            continue
        out.append((t, pm, pr, seg))
    return out


def tip_analyse(tips, n_moves):
    rows = []
    for k in range(n_moves):
        mv = [x for x in tips if x[3] == f'move {k + 1}']
        hd = [x for x in tips if x[3] == f'hold {k + 1}']
        if len(mv) < 12 or len(hd) < 3:
            rows.append(None)
            continue
        err = [1000 * float(((a - b) ** 2).sum()) ** 0.5 for _, a, b, _ in mv]
        pts = [a for _, a, _, _ in mv]
        h = 5
        sm = [sum(pts[max(0, i - h):i + h + 1]) / len(pts[max(0, i - h):i + h + 1]) for i in range(len(pts))]
        resid = [1000 * float(((a - b) ** 2).sum()) ** 0.5 for a, b in zip(pts, sm)][h:-h] or [0.0]
        hold = [1000 * float(((a - b) ** 2).sum()) ** 0.5 for _, a, b, _ in hd]
        hold_pts = [a for t, a, _, _ in hd if t - hd[0][0] > 0.3] or [hd[-1][1]]
        span = max(1000 * float(((a - b) ** 2).sum()) ** 0.5 for a in hold_pts for b in hold_pts)
        rows.append({'err_rms': math.sqrt(statistics.fmean([e * e for e in err])), 'err_max': max(err),
                     'jitter_rms': math.sqrt(statistics.fmean([r * r for r in resid])), 'jitter_max': max(resid),
                     'settle': hold[-1], 'hold_span': span})
    return rows


def tip_plot(tips, path, title):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t = [x[0] for x in tips]
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for i, c in enumerate('xyz'):
        ax[0].plot(t, [1000 * (x[1][i] - x[2][i]) for x in tips], lw=0.8, label=f'{c}')
    ax[0].axhline(0, color='k', lw=0.5)
    ax[0].set_ylabel('tip: measured - reference (mm)')
    ax[0].legend(loc='best', fontsize=8, ncol=3)
    ax[1].plot(t, [1000 * float(((x[1] - x[2]) ** 2).sum()) ** 0.5 for x in tips], '.-', ms=2, lw=0.7,
               color='tab:red')
    ax[1].set_ylabel('|tip error| (mm)')
    pts = [x[1] for x in tips]
    h = 5
    sm = [sum(pts[max(0, i - h):i + h + 1]) / len(pts[max(0, i - h):i + h + 1]) for i in range(len(pts))]
    ax[2].plot(t, [1000 * float(((a - b) ** 2).sum()) ** 0.5 for a, b in zip(pts, sm)], '.-', ms=2, lw=0.7,
               color='tab:purple')
    ax[2].set_ylabel('tip jitter (mm)\n(vs own 0.2 s average)')
    ax[2].set_xlabel('time (s)')
    spans = {}
    for x in tips:
        if x[3].startswith('move'):
            spans.setdefault(x[3], [x[0], x[0]])[1] = x[0]
    for a in ax:
        a.grid(alpha=0.3)
        for lo, hi in spans.values():
            a.axvspan(lo, hi, color='tab:blue', alpha=0.07)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def analyse(samples, ctrl, moves, jogged, moved):
    """moves = [(step_deg, target_rad)] for joint index `jogged`; `moved` = all joints that were moving."""
    out = []
    for k, (delta_deg, target) in enumerate(moves):
        mv = [s for s in samples if s[2] == f'move {k + 1}']
        hd = [s for s in samples if s[2] == f'hold {k + 1}']
        if len(mv) < 5 or len(hd) < 3:
            out.append(None)
            continue
        q = [s[1][jogged] for s in mv]
        err = [c[2][jogged] - c[1][jogged] for c in ctrl if c[3] == f'move {k + 1}']
        sm = running_mean(q, 11)
        resid = [a - b for a, b in zip(q, sm)][5:-5] or [0.0]
        steps = [b - a for a, b in zip(q, q[1:])]
        sign = 1 if delta_deg > 0 else -1
        moving = [x for x in steps if abs(x) > 0.5 * math.radians(COUNT_DEG)]
        reversals = sum(1 for x in moving if x * sign < 0)
        hold_q = [s[1][jogged] for s in hd if s[0] - hd[0][0] > 0.3] or [hd[-1][1][jogged]]
        seg = mv + hd
        others = {}
        for j in range(5):
            if j not in moved:
                v = [s[1][j] for s in seg]
                others[JOINTS[j]] = D(max(v) - min(v))
        worst = max(others, key=others.get) if others else None
        dt = [b[0] - a[0] for a, b in zip(mv, mv[1:])]
        out.append({'move': f'{delta_deg:+g} deg',
                    'got_deg': D(hd[-1][1][jogged] - mv[0][1][jogged]),
                    'track_rms_deg': D(math.sqrt(statistics.fmean([e * e for e in err]))) if err else float('nan'),
                    'track_max_deg': D(max(abs(e) for e in err)) if err else float('nan'),
                    'shake_rms_deg': D(math.sqrt(statistics.fmean([r * r for r in resid]))),
                    'max_jump_deg': D(max(abs(x) for x in steps)),
                    'reversals': reversals,
                    'settle_deg': D(hd[-1][1][jogged] - target),
                    'hold_pkpk_deg': D(max(hold_q) - min(hold_q)),
                    'others_worst': f'{worst} {others[worst]:.2f}' if worst else '- (all moving)',
                    'rate_hz': 1 / statistics.fmean(dt) if dt else float('nan')})
    return out


def plot_all(samples, ctrl, steps, path, gline):
    """All five joints moved together: one row per joint, measured vs reference and the error."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 'matplotlib missing (sudo apt install python3-matplotlib) - plot skipped'
    t = [s[0] for s in samples]
    q0 = samples[0][1]
    fig, ax = plt.subplots(5, 2, figsize=(15, 13), sharex=True)
    spans = {}
    for s in samples:
        if s[2].startswith('move'):
            spans.setdefault(s[2], [s[0], s[0]])[1] = s[0]
    for j, name in enumerate(JOINTS):
        a, b = ax[j][0], ax[j][1]
        a.plot(t, [D(s[1][j] - q0[j]) for s in samples], '.-', ms=2, lw=0.7, label='measured')
        if ctrl:
            a.plot([c[0] for c in ctrl], [D(c[1][j] - q0[j]) for c in ctrl], lw=1.1, label='reference')
            b.plot([c[0] for c in ctrl], [D(c[2][j] - c[1][j]) for c in ctrl], '.-', ms=2, lw=0.7, color='tab:red')
        b.axhline(0, color='k', lw=0.5)
        a.set_ylabel(f'{name}\nfrom start (deg)')
        b.set_ylabel('meas - ref (deg)')
        for x in (a, b):
            x.grid(alpha=0.3)
            for lo, hi in spans.values():
                x.axvspan(lo, hi, color='tab:blue', alpha=0.07)
    ax[0][0].legend(loc='best', fontsize=8)
    ax[4][0].set_xlabel('time (s)')
    ax[4][1].set_xlabel('time (s)')
    fig.suptitle(f'all joints together: moves {", ".join(f"{m:+g}" for m in steps)} deg  (shaded = moving; '
                 f'1 encoder count = {COUNT_DEG:.3f} deg)\n{gline}', fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    return None


def plot(samples, ctrl, moves, jogged, path, gline):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 'matplotlib missing (sudo apt install python3-matplotlib) - plot skipped'
    t = [s[0] for s in samples]
    q0 = samples[0][1]
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    name = JOINTS[jogged]
    ax[0].plot(t, [D(s[1][jogged] - q0[jogged]) for s in samples], '.-', ms=3, lw=0.8, label=f'{name} measured')
    if ctrl:
        ax[0].plot([c[0] for c in ctrl], [D(c[1][jogged] - q0[jogged]) for c in ctrl], lw=1.2, label='controller reference')
    ax[0].set_ylabel(f'{name} from start (deg)')
    if ctrl:
        ax[1].plot([c[0] for c in ctrl], [D(c[2][jogged] - c[1][jogged]) for c in ctrl], '.-', ms=3, lw=0.8,
                   color='tab:red')
    ax[1].axhline(0, color='k', lw=0.5)
    ax[1].set_ylabel('measured - reference (deg)')
    for j in range(5):
        if j != jogged:
            ax[2].plot(t, [D(s[1][j] - q0[j]) for s in samples], lw=0.9, label=JOINTS[j])
    ax[2].set_ylabel('other joints from start (deg)')
    ax[2].set_xlabel('time (s)')
    for a in ax:
        a.grid(alpha=0.3)
    spans = {}
    for s in samples:
        if s[2].startswith('move'):
            spans.setdefault(s[2], [s[0], s[0]])[1] = s[0]
    for a in ax:
        for lo, hi in spans.values():
            a.axvspan(lo, hi, color='tab:blue', alpha=0.07)
    ax[0].legend(loc='best', fontsize=8)
    ax[2].legend(loc='best', fontsize=8, ncol=4)
    fig.suptitle(f'{name}: moves {", ".join(f"{m:+g}" for m, _ in moves)} deg  (shaded = moving; '
                 f'1 encoder count = {COUNT_DEG:.3f} deg)\n{gline}', fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('joint', choices=JOINTS + ['all'], help="joint to jog, or 'all' for all five together")
    ap.add_argument('--moves', default='+3,-3,+1,-1', help='relative steps in degrees (default +3,-3,+1,-1)')
    ap.add_argument('--time', type=float, default=2.0, help='seconds per move (default 2)')
    ap.add_argument('--hold', type=float, default=1.5, help='seconds to hold after each move (default 1.5)')
    ap.add_argument('--note', default='', help='free text added to the title')
    ap.add_argument('--start', default='default',
                    help="'default' (DEFAULT_START), 'park', 'none', or q1,q2,q3,q4,q5 in radians")
    ap.add_argument('--yes', action='store_true', help='do not ask before a long move to the start pose')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    steps = [float(x) for x in args.moves.replace(' ', '').split(',') if x]
    if not steps or any(abs(s) > 10 for s in steps):
        raise SystemExit('--moves: 1..n steps, each at most 10 deg')
    if not 0.5 <= args.time <= 20 or not 0.3 <= args.hold <= 20:
        raise SystemExit('--time 0.5-20 s, --hold 0.3-20 s')
    moved = list(range(5)) if args.joint == 'all' else [JOINTS.index(args.joint)]
    limits = urdf_limits()
    if args.start == 'none':
        start_pose = None
    elif args.start == 'default':
        start_pose = list(DEFAULT_START)
    elif args.start == 'park':
        start_pose = park_pose()
    else:
        start_pose = [float(x) for x in args.start.split(',')]
        if len(start_pose) != 5:
            raise SystemExit('--start needs five joint angles in radians, e.g. 0,0.5,-1.2,0,1.5')
    if start_pose:
        bad = [f'{n} {v:+.3f} (limit {limits[n][0]:+.3f}..{limits[n][1]:+.3f})'
               for n, v in zip(JOINTS, start_pose) if not limits[n][0] + 0.02 <= v <= limits[n][1] - 0.02]
        if bad:
            raise SystemExit('Start pose outside the URDF limits: ' + ', '.join(bad))

    rclpy.init()
    node = Recorder()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        wait_for(lambda: node.q, 3, 'No /joint_states with all five joints - is run_real.sh teleop:=false running?')
        cmd = wait_for(lambda: node.cmd, 3, 'No /iron_arm_controller/controller_state - is iron_arm_controller active?')
        end = time.monotonic() + 2
        while node.description is None and time.monotonic() < end:
            time.sleep(0.05)
        try:
            gains, acc = gains_from_description(node.description)
            source = 'running arm'
        except Exception:
            gains, acc, source = gains_from_yaml(), False, 'from yaml - may differ from the running arm'
        gline = gains_line(gains, acc, args.note, source)
        print(gline, flush=True)
        if not node.client.wait_for_server(timeout_sec=3.0):
            raise SystemExit('iron_arm_controller action server not available')

        def run_goal(positions, seconds):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.velocities = [0.0] * 5
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            fut = node.client.send_goal_async(goal)
            handle = wait_for(lambda: fut.result() if fut.done() else None, 3, 'goal not answered')
            if not handle.accepted:
                raise SystemExit('Goal rejected by iron_arm_controller')
            res = handle.get_result_async()
            wait_for(lambda: True if res.done() else None, seconds + 5, 'move did not finish')
            return res.result().result.error_code

        if start_pose:
            travel = max(abs(D(a - b)) for a, b in zip(start_pose, cmd))
            seconds = max(3.0, travel / 8.0)
            print('Start pose (rad): ' + ', '.join(f'{n} {v:+.3f}' for n, v in zip(JOINTS, start_pose))
                  + f'  - largest joint move {travel:.1f} deg over {seconds:.1f} s', flush=True)
            if travel > 30 and not args.yes:
                input('Large move. Make sure the path is clear, then press Enter (Ctrl-C aborts): ')
            node.set_segment('goto start')
            code = run_goal(start_pose, seconds)
            if code != 0:
                raise SystemExit(f'Move to the start pose failed (controller error_code {code})')
            time.sleep(2.0)                     # let it settle under load before measuring
            cmd = list(start_pose)
        targets = {}                            # joint index -> target after each step
        for j in moved:
            lo, hi = limits[JOINTS[j]]
            pos, tj = cmd[j], []
            for s in steps:
                pos += math.radians(s)
                if not lo <= pos <= hi:
                    raise SystemExit(f'{JOINTS[j]}: the sequence reaches {pos:+.3f} rad, outside its URDF limit '
                                     f'{lo:+.3f}..{hi:+.3f}. Start further inside the range or change --moves.')
                tj.append(pos)
            targets[j] = tj
        print(f'{args.joint}: moves {steps} deg, {args.time:g} s each, hold {args.hold:g} s; recording...',
              flush=True)
        node.set_segment('before')
        time.sleep(0.5)
        held = list(cmd)
        for k, s in enumerate(steps):
            node.set_segment(f'move {k + 1}')
            code = run_goal([targets[j][k] if j in targets else held[j] for j in range(5)], args.time)
            node.set_segment(f'hold {k + 1}')
            time.sleep(args.hold)
            print(f'  move {k + 1}: {s:+g} deg  controller error_code {code}', flush=True)
        node.set_segment('after')
        time.sleep(0.3)
    finally:
        ex.shutdown(timeout_sec=1.0)
        spin.join(timeout=2.0)
        import logging, io, contextlib
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            node.destroy_node()
            rclpy.try_shutdown()
            import gc
            gc.collect()      # rclpy prints 'exception was never retrieved' for leftover futures here

    with node.lock:
        samples, ctrl, events = list(node.samples), list(node.ctrl), list(node.events)

    def label(t):
        name = events[0][1]
        for te, n in events:
            if te <= t:
                name = n
        return name
    samples = sorted(((t, q, label(t)) for t, q, _ in samples), key=lambda s: s[0])
    ctrl = sorted(((t, r, fb, label(t)) for t, r, fb, _ in ctrl), key=lambda c: c[0])
    all_samples = samples
    samples = [s for s in samples if s[2] not in ('start', 'goto start')] or samples
    ctrl = [c for c in ctrl if c[3] not in ('start', 'goto start')] or ctrl
    out_dir = WS / 'results' / f'jogtrack-{args.joint}-{time.strftime("%Y%m%d-%H%M%S")}'
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / 'samples.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'segment', *[f'{n}_rad' for n in JOINTS]])
        for t, q, seg in all_samples:
            w.writerow([f'{t:.4f}', seg, *[f'{v:.5f}' for v in q]])
    with open(out_dir / 'controller.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'segment', *[f'{n}_reference_rad' for n in JOINTS], *[f'{n}_feedback_rad' for n in JOINTS]])
        for t, r, fb, seg in ctrl:
            w.writerow([f'{t:.4f}', seg, *[f'{v:.5f}' for v in r], *[f'{v:.5f}' for v in fb]])
    start_txt = 'from where it was' if not start_pose else \
        'start pose ' + ', '.join(f'{v:+.2f}' for v in start_pose) + ' rad'
    lines = [f'{args.joint}  moves {args.moves} deg  {args.time:g} s each, hold {args.hold:g} s  '
             f'({len(samples)} samples, {start_txt})', gline, '']
    head = (f'{"move":>8} {"got":>7} {"track rms":>10} {"track max":>10} {"shake rms":>10} {"max jump":>9} '
            f'{"reversals":>9} {"settle":>7} {"hold pk-pk":>10}  {"others (worst pk-pk deg)":<26} {"Hz":>5}')
    for j in moved:
        res = analyse(samples, ctrl, list(zip(steps, targets[j])), j, moved)
        if len(moved) > 1:
            lines.append(JOINTS[j])
        lines.append(head)
        for r in res:
            if r is None:
                lines.append('   (too few samples for this move)')
                continue
            lines.append(f'{r["move"]:>8} {r["got_deg"]:+7.2f} {r["track_rms_deg"]:10.3f} {r["track_max_deg"]:10.3f} '
                         f'{r["shake_rms_deg"]:10.3f} {r["max_jump_deg"]:9.3f} {r["reversals"]:9d} '
                         f'{r["settle_deg"]:+7.2f} {r["hold_pkpk_deg"]:10.3f}  {r["others_worst"]:<26} {r["rate_hz"]:5.0f}')
        if len(moved) > 1:
            lines.append('')
    robot = load_robot()
    if robot is not None and ctrl:
        tips = tip_series(robot, ctrl)
        trows = tip_analyse(tips, len(steps))
        lines += ['tip (iron_tcp, all joints combined), mm',
                  f'{"move":>8} {"err rms":>8} {"err max":>8} {"jitter rms":>11} {"jitter max":>11} {"settle":>7} '
                  f'{"hold span":>10}']
        for s_, r in zip(steps, trows):
            if r is None:
                lines.append('   (too few samples for this move)')
                continue
            lines.append(f'{s_:+7g}d {r["err_rms"]:8.2f} {r["err_max"]:8.2f} {r["jitter_rms"]:11.3f} '
                         f'{r["jitter_max"]:11.3f} {r["settle"]:7.2f} {r["hold_span"]:10.2f}')
        tip_plot(tips, out_dir / 'tip.png', f'tip (iron_tcp): {args.joint} moves {args.moves} deg\n{gline}')
    lines += ['', 'All angles in degrees; 1 encoder count = 0.088 deg. Smooth: shake rms < ~0.05, reversals 0, '
              'max jump ~1-2 counts per sample;', 'shaking: shake rms and max jump grow, reversals > 0, others '
              'pk-pk well above 0.1.']
    if len(moved) > 1:
        msg = plot_all(samples, ctrl, steps, out_dir / 'plot.png', gline)
    else:
        msg = plot(samples, ctrl, list(zip(steps, targets[moved[0]])), moved[0], out_dir / 'plot.png', gline)
    if msg:
        lines.append(msg)
    text = '\n'.join(lines)
    (out_dir / 'summary.txt').write_text(text + '\n')
    print('\n' + text)
    print(f'\nSaved: {out_dir}/ (plot.png, tip.png, summary.txt, samples.csv, controller.csv)')
    return 0


if __name__ == '__main__':
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    import os
    os._exit(rc or 0)       # skip interpreter teardown: rclpy prints harmless 'exception was never retrieved' there
