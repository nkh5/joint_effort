#!/usr/bin/env python3
"""Reduced position+pointing IK above the provisional PCB, without roll target."""
import json,math,time,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from joint_effort_robot.kinematics import Robot
W=Path(__file__).resolve().parents[1];P=W/'src/joint_effort_iron';R=Robot(P/'urdf/iron.urdf');M=json.loads((P/'config/model.json').read_text());bounds=R.limits('iron')+np.array([.04,-.04]);rng=np.random.default_rng(88);records=[];started=time.monotonic()
home=np.array(json.loads((W/'src/joint_effort_iron/config/model.json').read_text())['initial_positions_rad'])  # V1: q=0 is outside the elbow limit
seeds=[np.clip(s,bounds[:,0],bounds[:,1]) for s in (home,home+np.array([0,.35,0,-.2,0]))]+list(rng.uniform(bounds[:,0],bounds[:,1],(6,5)))
for x in [M['pcb_center_m'][0]+d for d in [-.04,0,.04]]:
 for y in [M['pcb_center_m'][1]+d for d in [-.02,0,.02]]:
  target=np.array([x,y,.172])
  for angle in [15,30,45]:
   direction=np.array([math.cos(math.radians(angle)),0,-math.sin(math.radians(angle))]);best=None
   def residual(q):
    t=R.fk('iron',q);return np.r_[t[:3,3]-target,.12*(t[:3,0]-direction)]
   for seed in seeds:
    result=least_squares(residual,seed,bounds=(bounds[:,0],bounds[:,1]),max_nfev=120,ftol=1e-8,xtol=1e-8,gtol=1e-8);t=R.fk('iron',result.x);position=float(np.linalg.norm(t[:3,3]-target));pointing=math.acos(float(np.clip(np.dot(t[:3,0],direction),-1,1)))
    candidate={'q_rad':result.x.tolist(),'position_error_m':position,'pointing_error_rad':pointing,'score':float(np.linalg.norm(result.fun))}
    if best is None or candidate['score']<best['score']:best=candidate
    if position<.001 and pointing<math.radians(1):break
   feasible=best['position_error_m']<.001 and best['pointing_error_rad']<math.radians(1)
   records.append({'target_m':target.tolist(),'downward_approach_deg':angle,'pointing_world':direction.tolist(),'geometric_solution':best if feasible else None,'best_position_error_m':best['position_error_m'],'best_pointing_error_rad':best['pointing_error_rad'],'collision_check':None})
report={'urdf_sha256':hashlib.sha256((P/'urdf/iron.urdf').read_bytes()).hexdigest(),'scope':'Cold mock endpoint feasibility above provisional PCB. Software +/-30 deg travel with 0.04 rad margin; free shaft roll. No path or complete workspace proof.','target_tip_z_m':.172,'pcb_top_z_m':.152,'geometric_solution_count':sum(r['geometric_solution'] is not None for r in records),'candidate_count':len(records),'position_tolerance_m':.001,'pointing_tolerance_deg':1,'elapsed_s':time.monotonic()-started,'targets':records};(W/'reports/iron/workspace-study.json').write_text(json.dumps(report,indent=2)+'\n');print('CAD iron task endpoint candidates:',report['geometric_solution_count'],'/',len(records),'geometric solutions in',round(report['elapsed_s'],1),'seconds.')
