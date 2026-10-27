#!/usr/bin/env python3
import json,os,signal,subprocess,time
from pathlib import Path
W=Path(__file__).resolve().parents[1];out=W/'reports/iron';assert os.environ.get('ROS_DOMAIN_ID')=='88'
log=(out/'desktop-demo-launch.log').open('w');launch=subprocess.Popen(['ros2','launch','joint_effort_iron','mock.launch.py','rviz:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
report={}
try:
 time.sleep(8)
 for tool,args in [('iron_status.py',[]),('iron_demo.py',['sequence','--seconds','3']),('iron_plan.py',['--joint','joint_1','--degrees','8','--execute']),('iron_plan.py',['--hover','--execute'])]:
  result=subprocess.run(['python3',str(W/'tools'/tool),*args],capture_output=True,text=True,timeout=75);(out/(tool.replace('.py','')+('-hover' if '--hover' in args else '')+'-output.txt')).write_text(result.stdout+result.stderr);print(result.stdout,flush=True);assert result.returncode==0,result.stderr;report[tool+('-hover' if '--hover' in args else '')]='PASS'
 phases=json.loads((out/'demo-last.json').read_text());assert len(phases)==4;assert abs(phases[0]['joint_delta_deg']['joint_1'])>3;assert phases[2]['tcp_delta_mm'][2]>2;assert phases[3]['tcp_delta_mm'][2]<-2;assert all(p['end_collision_free'] for p in phases)
finally:
 launch.send_signal(signal.SIGINT)
 try:code=launch.wait(timeout=20)
 except subprocess.TimeoutExpired:launch.terminate();code=launch.wait(timeout=5)
 log.close();text=(out/'desktop-demo-launch.log').read_text();report['launch_code']=code;report['shutdown_children_clean']='process has died' not in text;report['robotmodel_ok']='Iron prototype model: Status: Ok' in text;report['tf_display_ok']='Iron frames: Status: Ok' in text;(out/'desktop-demo-check.json').write_text(json.dumps(report,indent=2)+'\n')
assert code==0 and report['shutdown_children_clean'] and report['robotmodel_ok'] and report['tf_display_ok'];print('PASS: user commands, CAD RViz displays, planned mock execution and clean GUI shutdown.')
