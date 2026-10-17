#!/usr/bin/env python3
"""Plan one CAD joint/hover target; optional supervised GenericSystem execution."""
import argparse, json, math
import rclpy
from rclpy.signals import SignalHandlerOptions
from iron_motion import Motion, W, hover


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint', choices=[f'joint_{i}' for i in range(1, 6)], default='joint_1')
    p.add_argument('--degrees', type=float, default=10.)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--hover', action='store_true')
    a = p.parse_args()
    if not math.isfinite(a.degrees) or abs(a.degrees) > 25:
        p.error('Use a finite absolute angle within +/-25 degrees')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); motion = None; report = {'result': 'FAIL'}
    try:
        motion = Motion(); motion.ready(); target = motion.q()
        if a.hover:
            endpoint = hover(motion); target = endpoint['geometric_solution']['q_rad']
            print('Hover target XYZ (m):', endpoint['target_m'], 'downward approach:', endpoint['downward_approach_deg'], 'degrees', flush=True)
        else:
            target[motion.names.index(a.joint)] = math.radians(a.degrees)
        trajectory = motion.plan(motion.q(), target); report['path'] = motion.check(trajectory)
        print(f"MOCK: checked path; {len(trajectory.points) if trajectory is not None else 0} points, {report['path']['duration_s']:.2f} seconds.", flush=True)
        if a.execute:
            motion.execute(trajectory)
            print('Mock execution complete; maximum joint error:', round(math.degrees(motion.metrics['executions'][-1]['joint_error_rad']), 4), 'degrees.', flush=True)
            if a.hover:
                print('Observed modeled tip XYZ (m):', [round(float(v), 6) for v in motion.robot.fk('iron', motion.q())[:3, 3]], flush=True)
        else:
            print('Plan only. Add --execute to move the mock. Keep other motion publishers stopped.', flush=True)
        report['result'] = 'PASS'; report['executed'] = a.execute
        return 0
    except (RuntimeError, ValueError, KeyboardInterrupt) as e:
        report['reason'] = str(e) or 'Keyboard interrupt'; print('Plan stopped:', report['reason'], flush=True)
        return 1
    finally:
        if motion:
            try:
                motion.close()
            finally:
                report['supervision'] = motion.metrics
                (W/'reports/iron/plan-last.json').write_text(json.dumps(report, indent=2)+'\n')
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
