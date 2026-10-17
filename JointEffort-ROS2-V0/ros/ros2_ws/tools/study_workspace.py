#!/usr/bin/env python3
"""Offline five-dimensional feasibility study; never emits ROS commands."""
from pathlib import Path
import json
import numpy as np
from joint_effort_robot.kinematics import Robot,task_jacobian
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'src/joint_effort_robot'
model=json.loads((P/'config/model.json').read_text());robot=Robot(P/'urdf/joint_effort.urdf')
rng=np.random.default_rng(20261002)

def solve(arm,target,direction):
 limits=robot.limits(arm)+np.array([.04,-.04])
 seeds=[model['initial_positions_by_arm_rad'][arm]]+[rng.uniform(limits[:,0],limits[:,1]) for _ in range(5)]
 best=None
 for seed in seeds:
  q=np.array(seed,dtype=float)
  for step in range(100):
   t,j=robot.fk(arm,q,True);reduced,basis=task_jacobian(t,j,model['task_orientation_length_m'])
   position=target-t[:3,3];d=t[:3,0]
   # Rotation moving current tool direction toward desired, no tool roll.
   cross=np.cross(d,direction);angle=np.arctan2(np.linalg.norm(cross),np.clip(d@direction,-1,1))
   angular=cross/max(np.linalg.norm(cross),1e-9)*angle
   err=np.r_[position,.12*basis.T@angular]
   if np.linalg.norm(position)<.0005 and angle<np.deg2rad(.5):
    singular=np.linalg.svd(reduced,compute_uv=False)
    result={'q_rad':q.tolist(),'position_error_m':float(np.linalg.norm(position)),'direction_error_deg':float(np.rad2deg(angle)),'min_singular_value_m':float(singular[-1])}
    if best is None or result['min_singular_value_m']>best['min_singular_value_m']:best=result
    break
   dq=np.linalg.solve(reduced.T@reduced+.003**2*np.eye(5),reduced.T@err)
   dq*=min(1,.18/max(np.max(np.abs(dq)),1e-10))
   q=np.clip(q+dq,limits[:,0],limits[:,1])
 return best

report={'assumptions':'Draft geometry and limits; candidate endpoints only; not path validity or measured accuracy.', 'random_seed':20261002,'arms':{},'target_candidates':[]}
for arm in ['iron','wire']:
 samples=rng.uniform(robot.limits(arm)[:,0],robot.limits(arm)[:,1],size=(1000,5))
 minima=[]
 for q in samples:
  t,j=robot.fk(arm,q,True);minima.append(np.linalg.svd(task_jacobian(t,j)[0],compute_uv=False)[-1])
 report['arms'][arm]={'random_postures':len(samples),'below_rank_threshold_fraction':float(np.mean(np.array(minima)<.001)),'min_singular_value_percentiles_m':np.percentile(minima,[0,5,50,95,100]).tolist()}
 # Twenty-five XY board points 10 mm above the draft surface, tool pointing down.
 for x in np.linspace(-.05,.05,5):
  for y in np.linspace(.18,.22,5):
   target=np.array([x,y,.055]);direction=np.array([0.,0.,-1.])
   result=solve(arm,target,direction)
   report['target_candidates'].append({'arm':arm,'target_m':target.tolist(),'direction':direction.tolist(),'solution':result})
 results=[v for v in report['target_candidates'] if v['arm']==arm]
 report['arms'][arm]['board_targets_solved']=sum(v['solution'] is not None for v in results)
 report['arms'][arm]['board_targets_total']=len(results)
(ROOT/'reports/workspace-study.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['arms'],indent=2))
