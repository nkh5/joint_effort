#!/usr/bin/env python3
"""Visible iron CAD motion through Servo and verified GenericSystem only."""
import argparse,json,math,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import SetBool,Trigger
from rclpy.signals import SignalHandlerOptions
from joint_effort_input.lease import Lease
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from moveit_msgs.srv import ServoCommandType,GetStateValidity
from tf2_ros import Buffer,TransformListener
P=argparse.ArgumentParser(description=__doc__);P.add_argument('motion',choices=['joint','up','down','forward','sequence']);P.add_argument('--joint',choices=[f'joint_{i}' for i in range(1,6)],default='joint_1');P.add_argument('--seconds',type=float,default=3);P.add_argument('--speed',type=float,default=.05);a=P.parse_args()
if not math.isfinite(a.seconds) or not 0<a.seconds<=10 or not math.isfinite(a.speed) or abs(a.speed)>.1:P.error('Use duration 0–10 seconds and joint speed within +/-0.1 rad/s.')
if os.environ.get('ROS_DOMAIN_ID')!='88':raise SystemExit('Use JE_ROS_DOMAIN_ID=88 with scripts/with_ros.sh')
lease=Lease(Path(__file__).resolve().parents[1]/'work/iron-motion.lock')
if not lease.acquire():raise SystemExit('Another motion tool owns the mock; release joystick enable or stop that tool first.')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Node('iron_prototype_demo');names=[f'joint_{i}' for i in range(1,6)];state={};received=-math.inf;stamp=-1;buf=Buffer();listener=TransformListener(buf,n);stop_requested=False
def request_stop(req,res):
 global stop_requested
 stop_requested=True;res.success=True;res.message='Demo stop requested; inspect the motion terminal for hold confirmation.';return res
n.create_service(Trigger,'/iron_motion/stop',request_stop)
def feedback(msg):
 global state,received,stamp
 state={};received=-math.inf
 if len(msg.name)==5 and len(msg.position)==5 and set(msg.name)==set(names) and all(math.isfinite(v) for v in msg.position):state=dict(zip(msg.name,msg.position));received=time.monotonic();stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
n.create_subscription(JointState,'/joint_states',feedback,1)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def fresh():return len(state)==5 and time.monotonic()-received<.3 and 0<=(n.get_clock().now().nanoseconds-stamp)*1e-9<.3
def call(kind,path,req):
 c=n.create_client(kind,path)
 try:
  if not c.wait_for_service(timeout_sec=15):raise RuntimeError('Missing '+path+'; start scripts/run_iron.sh first.')
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
  if not f.done():raise RuntimeError('Service timeout '+path)
  return f.result()
 finally:n.destroy_client(c)
def valid():
 req=GetStateValidity.Request();req.robot_state.joint_state.name=names;req.robot_state.joint_state.position=[state[j] for j in names]
 return call(GetStateValidity,'/check_state_validity',req).valid
def tip():
 t=buf.lookup_transform('world','iron_tcp',rclpy.time.Time()).transform.translation;return [t.x,t.y,t.z]
commands=[];joint=n.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1);task=n.create_publisher(TwistStamped,'/iron/task_velocity',1);active_mode=None

def enable(value):
 req=SetBool.Request();req.data=value
 if not call(SetBool,'/iron_task_bridge/enable',req).success:raise RuntimeError('Task bridge is not ready for fresh mock commands.')
def send(mode,speed):
 if mode=='joint':
  msg=JointJog();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=names;msg.velocities=[speed if j==a.joint else 0. for j in names];msg.duration=.02;joint.publish(msg)
 else:
  msg=TwistStamped();msg.header.stamp=n.get_clock().now().to_msg();msg.header.frame_id='world'
  if mode=='forward':msg.twist.linear.x=speed
  else:msg.twist.linear.z=speed
  task.publish(msg)
def stop():
 if active_mode:
  for _ in range(8):send(active_mode,0.);spin(.02)
  if active_mode!='joint':enable(False)
  spin(.6)
try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
 if len(hw.component)!=1 or hw.component[0].plugin_name!='mock_components/GenericSystem':raise RuntimeError('Refusing commands: expected GenericSystem mock hardware.')
 deadline=time.monotonic()+25
 while True:
  controllers=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
  if {'iron_arm_controller','joint_state_broadcaster'}<={c.name for c in controllers.controller if c.state=='active'} and fresh() and buf.can_transform('world','iron_tcp',rclpy.time.Time()):break
  if time.monotonic()>deadline:raise RuntimeError('Controllers or fresh feedback not ready.')
  spin(.1)
 enable(False);req=ServoCommandType.Request();req.command_type=req.JOINT_JOG
 if not call(ServoCommandType,'/iron_servo/switch_command_type',req).success:raise RuntimeError('Servo rejected joint commands.')
 spin(.3)
 phases=[('joint',a.speed,a.seconds),('joint',-a.speed,a.seconds),('up',.003,a.seconds),('down',-.003,a.seconds)] if a.motion=='sequence' else [(a.motion,a.speed if a.motion=='joint' else (-.003 if a.motion=='down' else .003),a.seconds)]
 print('MOCK ONLY: detailed CAD iron arm. Keep other command tools stopped.',flush=True)
 for mode,speed,seconds in phases:
  if not fresh() or not valid():raise RuntimeError('Feedback is stale or current model is in collision.')
  active_mode=mode
  if mode!='joint':enable(True)
  before=state.copy();old_tip=tip();end=time.monotonic()+seconds
  print(f'{mode}: {seconds:.1f} seconds',flush=True)
  while time.monotonic()<end:
   if stop_requested:raise RuntimeError('Software stop requested.')
   if not fresh():raise RuntimeError('Stale feedback: stopped the demo.')
   send(mode,speed);spin(.02)
  stop()
  if not fresh():raise RuntimeError('Fresh final feedback missing.')
  delta={j:round(math.degrees(state[j]-before[j]),3) for j in names};tcp_delta=[round(1000*(v-b),3) for v,b in zip(tip(),old_tip)];print('  joint change (degrees):',delta,'tip XYZ change (mm):',tcp_delta,flush=True)
  commands.append({'mode':mode,'joint_delta_deg':delta,'tcp_delta_mm':tcp_delta,'end_collision_free':valid()})
 (Path(__file__).resolve().parents[1]/'reports/iron/demo-last.json').write_text(json.dumps(commands,indent=2)+'\n');print('Complete. The mock arm holds its final position.',flush=True)
except (RuntimeError,KeyboardInterrupt) as e:print('Demo stopped:',e,flush=True);raise SystemExit(1)
finally:
 if active_mode and rclpy.ok():stop()
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 lease.release()
