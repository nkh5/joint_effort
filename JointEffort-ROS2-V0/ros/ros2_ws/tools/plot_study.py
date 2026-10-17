from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from joint_effort_robot.kinematics import Robot
R=Path(__file__).resolve().parents[1];p=R/'src/joint_effort_robot';m=json.loads((p/'config/model.json').read_text());robot=Robot(p/'urdf/joint_effort.urdf')
endpoints=json.loads((R/'reports/collision-endpoints.json').read_text())
fig=plt.figure(figsize=(13,6));ax=fig.add_subplot(121,projection='3d')
positions={name:q for a in ['iron','wire'] for name,q in zip(robot.names(a),m['initial_positions_by_arm_rad'][a])};frames=robot.frames(positions)
for arm,color in [('iron','#2377b9'),('wire','#da7723')]:
 names=['base','upper_arm','forearm','wrist_bridge','tool','tcp'];pts=np.array([frames[f'{arm}_{v}'][:3,3] for v in names]);ax.plot(*pts.T,'o-',lw=4,color=color,label=arm.title()+' arm')
ax.plot([-.06,.06,.06,-.06,-.06],[.17,.17,.23,.23,.17],[.045]*5,color='green',lw=3,label='Draft PCB')
ax.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',xlim=(-.3,.3),ylim=(-.1,.45),zlim=(0,.4),title='Draft dual-arm startup pose');ax.set_box_aspect((.6,.55,.4));ax.legend(loc='upper left',fontsize=9);ax.view_init(25,-60)
ax2=fig.add_subplot(122)
for arm,marker,offset in [('iron','o',-.8),('wire','s',.8)]:
 for e in endpoints:
  if e['arm']!=arm:continue
  color='#229954' if e['collision_check'] and e['collision_check']['valid'] else '#c0392b' if e['solution'] else '#b7bdc3'
  x,y,_=e['target_m'];ax2.scatter(x*1000+offset,y*1000,s=90,marker=marker,color=color,edgecolor='white',linewidth=.5)
ax2.plot([ -60,60,60,-60,-60],[170,170,230,230,170],color='green',lw=1.5)
ax2.set(xlabel='World X (mm)',ylabel='World Y (mm)',title='Downward-pointing target endpoints, Z = 55 mm',xlim=(-65,65),ylim=(168,232));ax2.grid(alpha=.2)
from matplotlib.lines import Line2D
ax2.legend(handles=[Line2D([0],[0],marker='o',color='w',markerfacecolor='#229954',label='IK + collision check passed'),Line2D([0],[0],marker='o',color='w',markerfacecolor='#b7bdc3',label='No IK solution found'),Line2D([0],[0],marker='o',color='gray',linestyle='',label='Iron'),Line2D([0],[0],marker='s',color='gray',linestyle='',label='Wire')],fontsize=8,loc='upper right')
fig.suptitle('Joint Effort — provisional model and five-DOF feasibility',fontsize=16)
fig.text(.5,.015,'Draft dimensions and travel limits. Other arm parked. Endpoint checks do not validate a path or simultaneous operation.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.045,1,.95));fig.savefig(R/'reports/model-and-workspace.png',dpi=160)
