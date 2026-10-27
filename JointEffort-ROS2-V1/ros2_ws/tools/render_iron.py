#!/usr/bin/env python3
"""Static figure of the cold prototype, collision hulls and chosen workspace."""
import json,struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from joint_effort_robot.kinematics import Robot
W=Path(__file__).resolve().parents[1];P=W/'src/joint_effort_iron';R=Robot(P/'urdf/iron.urdf');frames=R.frames({});fig=plt.figure(figsize=(12,8));ax=fig.add_subplot(111,projection='3d');bounds=[]
for name,link in R.links.items():
 collision=link.find('collision/geometry')
 if collision is None:continue
 mesh=collision.find('mesh');box=collision.find('box');t=frames[name]
 if mesh is not None:
  raw=(P/'meshes'/mesh.get('filename').rsplit('/',1)[1]).read_bytes();num=struct.unpack('<I',raw[80:84])[0];vertices=np.frombuffer(raw,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),offset=84,count=num)['v'].astype(float);color=link.find('visual/material/color').get('rgba').split()[:3];color=list(map(float,color));alpha=.8
 else:
  size=np.fromstring(box.get('size'),sep=' ');corners=np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*size
  from scipy.spatial import ConvexHull
  vertices=corners[ConvexHull(corners).simplices];color=[.1,.55,.2] if name=='pcb' else [.5,.5,.5];alpha=.25
 vertices=vertices@t[:3,:3].T+t[:3,3];ax.add_collection3d(Poly3DCollection(vertices,facecolor=color,edgecolor=(0,0,0,.15),linewidth=.15,alpha=alpha));bounds.append(vertices.reshape(-1,3))
tip=R.fk('iron',np.zeros(5));ax.scatter(*tip[:3,3],color='red',s=45);ax.quiver(*tip[:3,3],*tip[:3,0],length=.07,color='red');ax.text(*tip[:3,3],' saved-pose tip',fontsize=9)
study_path=W/'reports/iron/workspace-study.json'
if study_path.exists():
 study=json.loads(study_path.read_text());clear=[v for v in study['targets'] if v['geometric_solution'] and v['collision_check'] and v['collision_check']['valid']]
 if clear:
  points=np.array([v['target_m'] for v in clear]);ax.scatter(*points.T,color='lime',s=25,label='Collision-free hover endpoints (angles sampled separately)');ax.legend(loc='upper left',fontsize=8)
ax.set(xlim=(-.2,.75),ylim=(-.4,.4),zlim=(-.08,.65),xlabel='World X: forward (m)',ylabel='World Y (m)',zlabel='World Z: up (m)');ax.set_box_aspect([.95,.8,.73]);ax.view_init(elev=20,azim=-65);ax.set_title('Iron-arm cold mock: conservative collision hulls and provisional work area\nOriginal CAD visuals are used in RViz; the wire arm is excluded')
fig.savefig(W/'reports/iron/prototype-workspace.png',dpi=160,bbox_inches='tight');print('Saved iron prototype workspace/collision figure.')
