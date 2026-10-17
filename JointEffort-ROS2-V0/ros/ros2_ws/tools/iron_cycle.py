#!/usr/bin/env python3
"""Cold mock: hover 20 mm, approach to 10 mm clearance, dwell, retreat."""
import argparse
import json
import math
import numpy as np
from scipy.optimize import least_squares
import rclpy
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from iron_motion import Motion, W, hover


def clearance_path(motion, endpoint):
    robot = motion.robot; q = np.array(endpoint['geometric_solution']['q_rad'])
    start = robot.fk('iron', q); direction = start[:3, 0]; positions = [q.copy()]
    bounds = robot.limits('iron')+np.array([.04, -.04])
    for depth in np.linspace(.001, .010, 10):
        target = start[:3, 3]-np.array([0., 0., depth])
        def residual(values):
            t = robot.fk('iron', values)
            return np.r_[t[:3, 3]-target, .12*(t[:3, 0]-direction)]
        solution = least_squares(residual, q, bounds=(bounds[:, 0], bounds[:, 1]),
                                 max_nfev=150, ftol=1e-10, xtol=1e-10, gtol=1e-10)
        q = solution.x; t = robot.fk('iron', q)
        if np.linalg.norm(t[:3, 3]-target) > .0001 or np.linalg.norm(t[:3, 0]-direction) > .001:
            raise RuntimeError('Could not solve a precise constant-pointing approach; no motion')
        positions.append(q.copy())
    def make(values):
        trajectory = JointTrajectory(joint_names=motion.names); elapsed = 0.
        for i, value in enumerate(values):
            if i:
                delta = np.max(np.abs(value-values[i-1]))
                # Rest-to-rest quintic: peaks 1.875*d/T and 5.774*d/T^2.
                elapsed += max(.8, 1.875*delta/.06, math.sqrt(5.774*delta/.12))
            trajectory.points.append(JointTrajectoryPoint(positions=value.tolist(), velocities=[0.]*5,
                                     accelerations=[0.]*5, time_from_start=Duration(sec=int(elapsed), nanosec=int((elapsed%1)*1e9))))
        return trajectory
    top = motion.model['pcb_center_m'][2]+motion.model['pcb_size_m'][2]/2
    def corridor(values):
        t = robot.fk('iron', values); xyz = t[:3, 3]
        if np.linalg.norm(xyz[:2]-start[:2, 3]) > .0005:
            raise RuntimeError('Approach leaves the 0.5 mm vertical corridor')
        if not top+.0098 <= xyz[2] <= start[2, 3]+.0002:
            raise RuntimeError('Approach clearance violated')
        if np.linalg.norm(t[:3, 0]-direction) > .002:
            raise RuntimeError('Approach shaft pointing changed excessively')
    return make(positions), make(positions[::-1]), corridor, start[:3, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--cycles', type=int, choices=range(1, 4), default=1)
    a = parser.parse_args(); rclpy.init(signal_handler_options=SignalHandlerOptions.NO); motion = None
    report = {'scope': 'Cold GenericSystem only; no heat, contact or serial driver', 'result': 'FAIL', 'cycles': []}
    try:
        motion = Motion(); motion.ready(); endpoint = hover(motion)
        approach, retreat, corridor, xyz = clearance_path(motion, endpoint)
        transit = motion.plan(motion.q(), approach.points[0].positions)
        paths = [('hover', transit, None), ('approach', approach, corridor), ('retreat', retreat, corridor)]
        report['target'] = {'hover_xyz_m': xyz.tolist(), 'downward_approach_deg': endpoint['downward_approach_deg'],
                            'approach_depth_mm': 10, 'closest_tip_clearance_mm': float((xyz[2]-.010-(motion.model['pcb_center_m'][2]+motion.model['pcb_size_m'][2]/2))*1000),
                            'dwell_s': 1.}
        report['paths'] = {name: motion.check(path, check) for name, path, check in paths}
        print('Cold cycle:', json.dumps(report['target']), flush=True)
        for name, summary in report['paths'].items():
            print(f"Checked {name}: {summary['duration_s']:.2f}s, {summary['collision_samples']} spline samples", flush=True)
        if a.execute:
            for cycle in range(a.cycles):
                observed = {}
                phases = paths if cycle == 0 else paths[1:]
                for name, path, check in phases:
                    motion.check(path, check)  # Recheck current scene immediately before each phase.
                    print(f'Cycle {cycle+1}: {name}', flush=True); motion.execute(path)
                    observed[name+'_xyz_m'] = motion.robot.fk('iron', motion.q())[:3, 3].tolist()
                    if name == 'approach':
                        print('Dwell: 1 second above the board', flush=True); motion.dwell(1.)
                report['cycles'].append(observed)
            print('Cold cycle complete; tip is holding at the hover position.', flush=True)
        else:
            print('Preview only. Add --execute to move the mock arm.', flush=True)
        report['result'] = 'PASS'; report['executed'] = a.execute
        return 0
    except (RuntimeError, ValueError, KeyboardInterrupt) as e:
        report['reason'] = str(e) or 'Keyboard interrupt'
        print('Cycle stopped:', report['reason'], flush=True); return 1
    finally:
        if motion:
            try:
                motion.close()
            finally:
                report['supervision'] = motion.metrics
                (W/'reports/iron/cycle-last.json').write_text(json.dumps(report, indent=2)+'\n')
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
