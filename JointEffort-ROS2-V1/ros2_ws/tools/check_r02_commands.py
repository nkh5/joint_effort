#!/usr/bin/env python3
"""Run the documented demo commands on an isolated mock. No serial device access."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
W=Path(__file__).resolve().parents[1];OUT=W/'reports/r02';processes=[];logs=[];report={}
assert os.environ.get('ROS_DOMAIN_ID')=='188' and os.environ.get('JE_MOCK_TEST')=='1'
def start(name,args):
 log=(OUT/(name+'.log')).open('w');logs.append(log)
 p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
 return p
def run(name,args):
 result=subprocess.run(['python3',str(W/'tools'/name),*args],capture_output=True,text=True,timeout=50)
 (OUT/(name.replace('.py','')+'-command.txt')).write_text(result.stdout+result.stderr)
 assert result.returncode==0,(name,result.stdout,result.stderr)
 report[name]='PASS';print(result.stdout,flush=True)
try:
 start('command-mock',['ros2','launch','joint_effort_iron','mock.launch.py','rviz:=false'])
 run('iron_status.py',[])
 time.sleep(1.)
 run('iron_demo.py',['joint','--joint','joint_1','--speed','0.05','--seconds','3'])
 run('iron_demo.py',['up','--seconds','3'])
 start('command-input',['ros2','launch','joint_effort_input','input.launch.py','joystick:=false','mapping_confirmed:=true','display:=false'])
 run('input_demo.py',[])
finally:
 exits=[]
 for p in reversed(processes):
  if p.poll() is None:p.send_signal(signal.SIGINT)
  try:exits.append(p.wait(timeout=20))
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);exits.append(p.wait())
 for log in logs:log.close()
 report['exit_codes']=exits;(OUT/'demo-command-check.json').write_text(json.dumps(report,indent=2)+'\n')
assert all(x==0 for x in exits)
