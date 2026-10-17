#!/usr/bin/env python3
"""Compare stock/fixed MoveIt teardown with SIGINT and SIGTERM; no motion."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
ROOT=Path(__file__).resolve().parents[1];P=ROOT/'src/joint_effort_robot'
args=argparse.ArgumentParser();args.add_argument('--stock-runs',type=int,default=2);args.add_argument('--fixed-runs',type=int,default=6);opt=args.parse_args()
params={'robot_description':(P/'urdf/joint_effort.urdf').read_text(),'robot_description_semantic':(P/'config/joint_effort.srdf').read_text(),'robot_description_kinematics':yaml.safe_load((P/'config/kinematics.yaml').read_text()),'robot_description_planning':yaml.safe_load((P/'config/joint_limits.yaml').read_text()),'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':{'planning_plugins':['ompl_interface/OMPLPlanner']},'allow_trajectory_execution':False}
logs=ROOT/'work/shutdown';logs.mkdir(parents=True,exist_ok=True)
f=logs/'parameters.yaml';f.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
results=[]
for variant,count in [('stock',opt.stock_runs),('fixed',opt.fixed_runs)]:
 exe='/opt/ros/jazzy/lib/moveit_ros_move_group/move_group' if variant=='stock' else str(Path(get_package_prefix('joint_effort_moveit'))/'lib/joint_effort_moveit/move_group_lifetime')
 for i in range(count):
  sig=signal.SIGINT if i%2==0 else signal.SIGTERM
  path=logs/f'{variant}-{i}-{sig.name}.log'
  with path.open('w') as log:
   p=subprocess.Popen([exe,'--ros-args','--params-file',str(f)],stdout=log,stderr=subprocess.STDOUT)
   # Readiness is observed in output rather than assumed from a fixed delay.
   deadline=time.monotonic()+15
   while 'You can start planning now!' not in path.read_text():
    if p.poll() is not None or time.monotonic()>deadline:raise RuntimeError(f'{variant} did not start: {path}')
    time.sleep(.05)
   time.sleep(.3)
   p.send_signal(sig)
   forced=False
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:p.kill();p.wait();forced=True
  result={'variant':variant,'run':i+1,'signal':sig.name,'exit_code':p.returncode,'forced':forced}
  print(json.dumps(result),flush=True);results.append(result)
report={'results':results,'fixed_pass':all(r['exit_code']==0 and not r['forced'] for r in results if r['variant']=='fixed')}
(ROOT/'reports/shutdown-check.json').write_text(json.dumps(report,indent=2)+'\n')
if not report['fixed_pass']:raise SystemExit('Fixed executable failed shutdown regression')
