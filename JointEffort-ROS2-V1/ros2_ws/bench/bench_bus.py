#!/usr/bin/env python3
"""ROS side of the bench: stream a new position goal to all motors every control cycle through
sts_hardware_interface (bench.launch.py) and check every motor follows, at the given rate.

  python3 bench/bench_bus.py --rate 50 --out results/bus_50.json [--quick]

Phases: hold 2 s, slow sine (0.25 Hz, +-15 deg), fast sine (1 Hz, +-10 deg), hold 2 s, back to start.
Each motor gets a different phase so all five move at different times and directions.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

DEG = math.pi / 180


class BusTest(Node):
    def __init__(self, rate, phases):
        super().__init__('bench_bus_test')
        self.rate, self.phases = rate, phases
        self.names = None
        self.start = None
        self.js = []          # (t_recv, t_stamp, positions)
        self.dyn = []         # (t, {joint: {iface: value}})
        self.cmds = []        # (t, targets)
        self.pub = self.create_publisher(Float64MultiArray, '/bench_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.on_dyn, 50)
        self.t0 = None

    def on_js(self, msg):
        if self.names is None:
            self.names = sorted(msg.name, key=lambda n: int(n.split('_')[1]))
        d = dict(zip(msg.name, msg.position))
        if self.start is None:
            self.start = [d[n] for n in self.names]
            return
        if self.t0 is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.js.append((time.monotonic() - self.t0, stamp, [d[n] for n in self.names]))

    def on_dyn(self, msg):
        if self.t0 is None:
            return
        vals = {j: dict(zip(v.interface_names, v.values)) for j, v in zip(msg.joint_names, msg.interface_values)}
        self.dyn.append((time.monotonic() - self.t0, vals))

    def target(self, t):
        n = len(self.names)
        out = list(self.start)
        t_phase = 0.0
        for kind, dur, amp, freq in self.phases:
            if t < t_phase + dur:
                if kind == 'sine':
                    tt = t - t_phase
                    ramp = min(1.0, tt / 0.5, (t_phase + dur - t) / 0.5)   # fade in/out, no jerk at edges
                    out = [s + ramp * amp * DEG * math.sin(2 * math.pi * freq * tt + 2 * math.pi * i / n)
                           for i, s in enumerate(self.start)]
                return out, kind
            t_phase += dur
        return out, 'done'


def analyse(node, rate):
    names = node.names
    period = 1.0 / rate
    res = {'rate_hz': rate, 'joint_states': {}, 'motors': {}}
    tr = [r[0] for r in node.js]
    gaps = [b - a for a, b in zip(tr, tr[1:])]
    if gaps:
        res['joint_states'] = {'messages': len(tr), 'mean_hz': (len(tr) - 1) / (tr[-1] - tr[0]),
                               'max_gap_ms': 1000 * max(gaps),
                               'gaps_over_2_periods': sum(g > 2 * period for g in gaps)}
    phase_spans, t_phase = [], 0.0
    for kind, dur, amp, freq in node.phases:
        phase_spans.append((kind, t_phase, t_phase + dur, amp, freq))
        t_phase += dur
    for i, name in enumerate(names):
        m = {}
        for kind, a, b, amp, freq in phase_spans:
            if kind != 'sine':
                continue
            samples = [(t, p[i]) for t, _, p in node.js if a + 0.7 <= t <= b - 0.7]
            best = None
            for lag_ms in range(0, 301, 5):
                lag = lag_ms / 1000
                errs = [p - node.target(t - lag)[0][i] for t, p in samples]
                if not errs:
                    continue
                rms = math.sqrt(sum(e * e for e in errs) / len(errs))
                if best is None or rms < best[1]:
                    best = (lag, rms, max(abs(e) for e in errs))
            # stalls: reported position frozen >= 4 cycles while the goal kept moving the same way > 3 counts
            stalls, since, counted = 0, None, False
            for (t1, p1), (t2, p2) in zip(samples, samples[1:]):
                if p1 == p2:
                    if since is None:
                        since, counted = t1, False
                    g = [node.target(x)[0][i] for x in (since - 0.2, since, t2)]
                    reversal = (g[1] - g[0]) * (g[2] - g[1]) < 0   # turnaround dwell = deadband, not a stall
                    if not counted and not reversal and t2 - since >= 3.5 / rate and abs(g[2] - g[1]) > 3 * 2 * math.pi / 4096:
                        stalls += 1
                        counted = True
                else:
                    since = None
            m[f'{freq:g}Hz'] = {'lag_ms': best[0] * 1000 if best else None,
                                'rms_err_deg': best[1] / DEG if best else None,
                                'max_err_deg': best[2] / DEG if best else None,
                                'stalls': stalls, 'samples': len(samples)}
        volts = [d[name].get('voltage') for _, d in node.dyn if name in d and 'voltage' in d[name]]
        cur = [d[name].get('current') for _, d in node.dyn if name in d and 'current' in d[name]]
        temp = [d[name].get('temperature') for _, d in node.dyn if name in d and 'temperature' in d[name]]
        m['min_voltage'] = min(volts) if volts else None
        m['max_current_a'] = max(cur) if cur else None
        m['max_temp_c'] = max(temp) if temp else None
        res['motors'][name] = m
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=50)
    ap.add_argument('--out', required=True)
    ap.add_argument('--quick', action='store_true')
    args, _ = ap.parse_known_args()
    q = args.quick
    phases = [('hold', 2.0, 0, 0), ('sine', 8.0 if q else 16.0, 15.0, 0.25), ('sine', 6.0 if q else 10.0, 10.0, 1.0),
              ('hold', 2.0, 0, 0)]
    rclpy.init()
    node = BusTest(args.rate, phases)
    deadline = time.monotonic() + 15
    while (node.start is None or node.pub.get_subscription_count() == 0) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.start is None or node.pub.get_subscription_count() == 0:
        print('No /joint_states or position controller - did bench.launch.py start?', file=sys.stderr)
        return 2
    print(f'[bench_bus] {len(node.names)} motors at {args.rate:g} Hz, start (deg) '
          + ', '.join(f'{s / DEG:.1f}' for s in node.start), flush=True)
    for s in node.start:
        if abs(s) > (180 - 16) * DEG:
            print('A motor is within 16 deg of the encoder wrap - turn it to the middle and rerun', file=sys.stderr)
            return 2
    period = 1.0 / args.rate
    node.t0 = time.monotonic()
    k = 0
    while True:
        t = time.monotonic() - node.t0
        tgt, kind = node.target(t)
        if kind == 'done':
            break
        node.pub.publish(Float64MultiArray(data=tgt))
        node.cmds.append((t, tgt))
        k += 1
        while time.monotonic() - node.t0 < k * period:
            rclpy.spin_once(node, timeout_sec=max(0.0, node.t0 + k * period - time.monotonic()))
    node.pub.publish(Float64MultiArray(data=node.start))
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    res = analyse(node, args.rate)
    res['start_deg'] = [s / DEG for s in node.start]
    res['phases'] = phases
    res['trace'] = {'t': [r[0] for r in node.js], 'pos': [r[2] for r in node.js]}
    with open(args.out, 'w') as f:
        json.dump(res, f)
    js = res['joint_states']
    print(f'[bench_bus] joint_states {js.get("mean_hz", 0):.1f} Hz, max gap {js.get("max_gap_ms", 0):.0f} ms; '
          + '; '.join(f'{n}: rms {m["0.25Hz"]["rms_err_deg"]:.2f}/{m["1Hz"]["rms_err_deg"]:.2f} deg'
                      for n, m in res['motors'].items()), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
