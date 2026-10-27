#!/usr/bin/env python3
"""Audit supplied CAD URDF without assuming its limits/TCP are calibrated."""
import argparse,csv,json,hashlib,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from joint_effort_robot.kinematics import origin,rotation

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('urdf',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
root=ET.parse(a.urdf).getroot();links={l.get('name') for l in root.findall('link')};joints=root.findall('joint')
children=[j.find('child').get('link') for j in joints]
assert len(links)==len(root.findall('link')) and len(children)==len(set(children)),'Duplicate links/children'
assert all(j.find('parent').get('link') in links and j.find('child').get('link') in links for j in joints)
roots=links-set(children);assert len(roots)==1
base=next(iter(roots));pending=joints.copy();ordered=[];known={base}
while pending:
 before=len(pending)
 for j in pending.copy():
  if j.find('parent').get('link') in known:known.add(j.find('child').get('link'));ordered.append(j);pending.remove(j)
 assert len(pending)<before,'Disconnected/cyclic tree'
active=[j for j in ordered if j.get('type')=='revolute'];names=[j.get('name') for j in active]
limits=[]
for j in active:
 axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');limit=j.find('limit')
 assert np.isfinite(axis).all() and abs(np.linalg.norm(axis)-1)<1e-8
 lo,hi=float(limit.get('lower')),float(limit.get('upper'));assert math.isfinite(lo) and math.isfinite(hi) and lo<hi
 limits.append([lo,hi])
def fk(q):
 frames={base:np.eye(4)};axes={};points={};index=0
 for j in ordered:
  t=frames[j.find('parent').get('link')]@origin(j.find('origin'))
  if j.get('type')=='revolute':
   axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axes[j.get('name')]=t[:3,:3]@axis;points[j.get('name')]=t[:3,3].copy()
   motion=np.eye(4);motion[:3,:3]=rotation(axis,q[index]);t=t@motion;index+=1
  frames[j.find('child').get('link')]=t
 return frames,axes,points
frames,axes,points=fk(np.zeros(len(active)));tip=frames['iron_tip'];jac=np.zeros((6,len(active)))
for i,n in enumerate(names):jac[:3,i]=np.cross(axes[n],tip[:3,3]-points[n]);jac[3:,i]=axes[n]
error=0.;angular_error=0.
for i in range(len(active)):
 q=np.zeros(len(active));q[i]=1e-6;plus=fk(q)[0]['iron_tip'];q[i]=-1e-6;minus=fk(q)[0]['iron_tip']
 error=max(error,float(np.max(np.abs((plus[:3,3]-minus[:3,3])/2e-6-jac[:3,i]))))
 rd=(plus[:3,:3]-minus[:3,:3])/2e-6;skew=rd@tip[:3,:3].T
 w=np.array([skew[2,1],skew[0,2],skew[1,0]])
 angular_error=max(angular_error,float(np.max(np.abs(w-jac[3:,i]))))
assert max(error,angular_error)<1e-7
rows=[]
for i,j in enumerate(active):
 xyz=np.fromstring(j.find('origin').get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(j.find('origin').get('rpy','0 0 0'),sep=' ')
 rows.append({'joint':j.get('name'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'origin_parent_mm':(xyz*1000).tolist(),'origin_rpy_deg':np.degrees(rpy).tolist(),'axis_joint':np.fromstring(j.find('axis').get('xyz'),sep=' ').tolist(),'axis_in_base_at_cad_zero':axes[j.get('name')].tolist(),'axis_center_in_base_mm':(points[j.get('name')]*1000).tolist(),'origin_displacement_mm':float(np.linalg.norm(xyz)*1000),'slider_bounds_deg':np.degrees(limits[i]).tolist()})
meshes=[]
for e in root.findall('.//mesh'):
 uri=e.get('filename');parts=uri.removeprefix('package://').split('/',1);rel=parts[1] if len(parts)==2 else uri
 candidates=[a.urdf.parent/rel,a.urdf.parent.parent/rel,a.urdf.parent.parent.parent/'ros2_ws/src'/parts[0]/rel]
 meshes.append({'uri':uri,'found':any(c.exists() for c in candidates)})
result={'source_sha256':hashlib.sha256(a.urdf.read_bytes()).hexdigest(),'robot':root.get('name'),'base':base,'links':len(links),'revolute_joints':len(active),'fixed_joints':len(joints)-len(active),'joint_rows':rows,'nominal_tip_in_base_mm':(tip[:3,3]*1000).tolist(),'tip_rotation_in_base':tip[:3,:3].tolist(),'jacobian_rank_at_cad_zero':int(np.linalg.matrix_rank(jac)),'finite_difference_linear_error':error,'finite_difference_angular_error':angular_error,'mesh_references':meshes,'collision_shapes':len(root.findall('.//collision')),'inertials':len(root.findall('.//inertial')),'ros2_control_blocks':len(root.findall('ros2_control')),'scope':'CAD saved pose, placeholder limits, uncalibrated nominal tip; no mechanical/hardware approval'}
(a.output/'cad-urdf-audit.json').write_text(json.dumps(result,indent=2)+'\n')
with (a.output/'cad-joint-transforms.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['joint','parent','child','origin_x_mm','origin_y_mm','origin_z_mm','roll_deg','pitch_deg','yaw_deg','axis_x','axis_y','axis_z','axis_base_x_at_q0','axis_base_y_at_q0','axis_base_z_at_q0','CAD_zero_not_motor_zero','limits_status'])
 for r in rows:writer.writerow([r['joint'],r['parent'],r['child'],*r['origin_parent_mm'],*r['origin_rpy_deg'],*r['axis_joint'],*r['axis_in_base_at_cad_zero'],'YES','Visualization placeholders'])
print(json.dumps({k:result[k] for k in ['links','revolute_joints','nominal_tip_in_base_mm','jacobian_rank_at_cad_zero','finite_difference_linear_error','finite_difference_angular_error','collision_shapes','inertials']},indent=2))
