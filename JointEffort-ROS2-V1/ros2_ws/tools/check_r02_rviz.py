#!/usr/bin/env python3
"""Open the R02 mock in RViz and check its model and TF displays. No serial driver."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
W=Path(__file__).resolve().parents[1];out=W/'reports/r02';logpath=out/'rviz-launch.log'
assert os.environ.get('ROS_DOMAIN_ID')=='188' and os.environ.get('JE_MOCK_TEST')=='1'
checks={'scope':'R02 mock RViz smoke check'}
with logpath.open('w') as log:
 p=subprocess.Popen(['ros2','launch','joint_effort_iron','mock.launch.py','rviz:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  end=time.monotonic()+35
  while time.monotonic()<end:
   text=logpath.read_text()
   checks['model_ok']='Iron prototype model: Status: Ok' in text
   checks['tf_ok']='Iron frames: Status: Ok' in text
   if all(checks[k] for k in ['model_ok','tf_ok']):break
   if p.poll() is not None:break
   time.sleep(.2)
 finally:
  if p.poll() is None:p.send_signal(signal.SIGINT)
  try:checks['exit_code']=p.wait(timeout=20)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);checks['exit_code']=p.wait()
text=logpath.read_text();checks['clean_shutdown']='process has died' not in text and 'Segmentation fault' not in text and checks['exit_code']==0
checks['result']='PASS' if checks['model_ok'] and checks['tf_ok'] and checks['clean_shutdown'] else 'FAIL'
(out/'rviz-check.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
raise SystemExit(0 if checks['result']=='PASS' else 1)
