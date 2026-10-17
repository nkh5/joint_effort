#!/usr/bin/env python3
"""Bounded simultaneous mock demo. Requires a freshly launched park pose."""
import json, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType, GetStateValidity
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[1]
MODEL=json.loads((ROOT/'src/joint_effort_robot/config/model.json').read_text())
ARMS=['iron','wire'];SUFFIXES=MODEL['joint_suffixes']
EXPECTED={a+'_'+s for a in ARMS for s in SUFFIXES}
rclpy.init();node=Node('joint_effort_both_demo')
state={};received=-math.inf;source_ns=-1;codes={};seen={a:set() for a in ARMS};pubs={}
buffer=Buffer();listener=TransformListener(buffer,node)

def feedback(msg):
 global received,source_ns
 state.clear()
 if len(msg.name)!=10 or len(msg.position)!=10 or set(msg.name)!=EXPECTED or not all(math.isfinite(v) for v in msg.position):
  received=-math.inf;return
 state.update(zip(msg.name,msg.position));received=time.monotonic()
 source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
node.create_subscription(JointState,'/joint_states',feedback,1)
def status(arm,msg):codes[arm]=msg.code;seen[arm].add(msg.code)
for arm in ARMS:
 node.create_subscription(ServoStatus,f'/{arm}_servo/status',lambda msg,a=arm:status(a,msg),10)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def call(kind,name,req):
 client=node.create_client(kind,name)
 try:
  if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing '+name+'. Start the mock model first.')
  future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
  if not future.done():raise RuntimeError('Service timeout: '+name)
  return future.result()
 finally:node.destroy_client(client)
def fresh():
 age=(node.get_clock().now().nanoseconds-source_ns)*1e-9
 return set(state)==EXPECTED and time.monotonic()-received<.3 and 0<=age<.3

def publish(velocities):
 stamp=node.get_clock().now().to_msg()
 for arm,pub in pubs.items():
  msg=JointJog();msg.header.stamp=stamp;msg.joint_names=[arm+'_'+s for s in SUFFIXES]
  msg.velocities=list(velocities.get(arm,[0.]*5));msg.duration=.02;pub.publish(msg)
def stop():
 for _ in range(5):publish({});spin(.02)
 spin(.5)
def clear_scene():
 req=GetStateValidity.Request();req.robot_state.joint_state.name=list(state);req.robot_state.joint_state.position=list(state.values())
 if not call(GetStateValidity,'/check_state_validity',req).valid:raise RuntimeError('Current modeled robot is in collision. Restart in the park pose and check obstacles.')
def tcp(arm):
 p=buffer.lookup_transform('world',arm+'_tcp',rclpy.time.Time()).transform.translation
 return [p.x,p.y,p.z]

try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
 if len(hw.component)!=1 or hw.component[0].plugin_name!='mock_components/GenericSystem':raise RuntimeError('Refusing motion: expected GenericSystem mock hardware.')
 deadline=time.monotonic()+15
 while True:
  ctr=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
  active={c.name for c in ctr.controller if c.state=='active'}
  if {'iron_arm_controller','wire_arm_controller','joint_state_broadcaster'}<=active and fresh() and all(buffer.can_transform('world',a+'_tcp',rclpy.time.Time()) for a in ARMS):break
  if time.monotonic()>deadline:raise RuntimeError('Controllers, complete feedback or TCP transforms are not ready.')
  spin(.1)
 if any(abs(state[a+'_'+s]-MODEL['initial_positions_by_arm_rad'][a][i])>.03 for a in ARMS for i,s in enumerate(SUFFIXES)):
  raise RuntimeError('Start a fresh mock launch before this sequence; the arms have already moved.')
 clear_scene()
 req=SetBool.Request();req.data=False
 if not call(SetBool,'/task_bridge/enable',req).success:raise RuntimeError('Could not disable the task bridge.')
 for arm in ARMS:
  req=ServoCommandType.Request();req.command_type=req.JOINT_JOG
  if not call(ServoCommandType,f'/{arm}_servo/switch_command_type',req).success:raise RuntimeError(arm+' Servo did not accept joint commands.')
  pubs[arm]=node.create_publisher(JointJog,f'/{arm}_servo/delta_joint_cmds',1)
 spin(.5)
 phases=[('Swing apart',{'iron':[.05,0,0,0,0],'wire':[-.05,0,0,0,0]}),
         ('Lift together', {a:[0,.025,0,-.025,0] for a in ARMS}),
         ('Lower together',{a:[0,-.025,0,.025,0] for a in ARMS}),
         ('Swing back',{'iron':[-.05,0,0,0,0],'wire':[.05,0,0,0,0]})]
 report=[]
 print('MOCK ONLY: both arms, four phases. Keep other motion publishers stopped.',flush=True)
 for title,velocities in phases:
  if not fresh():raise RuntimeError('Feedback became stale; stopping both arms.')
  clear_scene();before=state.copy();tips={a:tcp(a) for a in ARMS}
  for a in ARMS:seen[a].clear()
  print(title+' (3 seconds)',flush=True);end=time.monotonic()+3
  while time.monotonic()<end:
   if not fresh():raise RuntimeError('Feedback became stale; stopping both arms.')
   if any(codes.get(a)==ServoStatus.HALT_FOR_COLLISION for a in ARMS):raise RuntimeError('Servo reported a collision halt; stopping both arms.')
   publish(velocities);spin(.02)
  stop()
  if not fresh():raise RuntimeError('Missing fresh feedback after stopping.')
  clear_scene();record={'phase':title,'arms':{}}
  for a in ARMS:
   degrees={s:round(math.degrees(state[a+'_'+s]-before[a+'_'+s]),3) for s in SUFFIXES}
   movement=[round(1000*(v-b),3) for v,b in zip(tcp(a),tips[a])]
   record['arms'][a]={'joint_delta_deg':degrees,'tcp_delta_mm':movement,'servo_codes':sorted(seen[a])}
   print(f'  {a}: joint degrees {degrees}; tip XYZ mm {movement}',flush=True)
   if ServoStatus.DECELERATE_FOR_COLLISION in seen[a]:print('  Servo reduced speed near modeled geometry.',flush=True)
  report.append(record)
 (ROOT/'reports/both-demo-last.json').write_text(json.dumps(report,indent=2)+'\n')
 print('Complete. Both mock arms hold their final positions. Restart the launch before repeating.',flush=True)
except (RuntimeError,KeyboardInterrupt) as error:
 print('Demo stopped:',error,flush=True);raise SystemExit(1)
finally:
 if pubs and rclpy.ok():stop()
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
