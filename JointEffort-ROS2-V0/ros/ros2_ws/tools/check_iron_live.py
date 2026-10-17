#!/usr/bin/env python3
"""Iron-only mock integration: TF, FCL, Servo, planning, mock execution, watchdog."""
import json,math,os,signal,subprocess,time,hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose,TwistStamped
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import JointJog
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers,SwitchController
from moveit_msgs.srv import GetStateValidity,ApplyPlanningScene,ServoCommandType,GetMotionPlan
from moveit_msgs.msg import CollisionObject,JointConstraint,Constraints,ServoStatus
from std_srvs.srv import SetBool
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from joint_effort_robot.kinematics import Robot,task_jacobian
W=Path(__file__).resolve().parents[1];OUT=W/'reports/iron';R=Robot(W/'src/joint_effort_iron/urdf/iron.urdf');names=R.names('iron');assert os.environ.get('ROS_DOMAIN_ID')=='88'
log=(OUT/'integration-launch.log').open('w');launch=subprocess.Popen(['ros2','launch','joint_effort_iron','mock.launch.py','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=Node('iron_integration_check');state={};receipt=-math.inf;stamp=-1;servo_codes=[];bridge_status=[];ages=[];buf=Buffer();listener=TransformListener(buf,n)
report={'scope':'CAD iron-only mock; provisional geometry and calibration','checks':{}}
def feedback(msg):
 global state,receipt,stamp
 state=dict(zip(msg.name,msg.position));receipt=time.monotonic();stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;ages.append((n.get_clock().now().nanoseconds-stamp)*1e-9)
n.create_subscription(JointState,'/joint_states',feedback,1);n.create_subscription(ServoStatus,'/iron_servo/status',lambda m:servo_codes.append(m.code),10);n.create_subscription(String,'/iron_task_bridge/status',lambda m:bridge_status.append(m.data),10)
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def fresh():return set(state)==set(names) and time.monotonic()-receipt<.3 and 0<=(n.get_clock().now().nanoseconds-stamp)*1e-9<.3

def call(kind,path,req):
 c=n.create_client(kind,path)
 try:
  assert c.wait_for_service(timeout_sec=25),'Missing '+path
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15);assert f.done(),'Timeout '+path;return f.result()
 finally:n.destroy_client(c)
def validity(q=None):
 req=GetStateValidity.Request();req.robot_state.joint_state.name=names;req.robot_state.joint_state.position=[state[j] for j in names] if q is None else list(q)
 result=call(GetStateValidity,'/check_state_validity',req);return {'valid':result.valid,'contacts':[[c.contact_body_1,c.contact_body_2] for c in result.contacts]}
def obstacle(position,remove=False):
 req=ApplyPlanningScene.Request();req.scene.is_diff=True;co=CollisionObject();co.header.frame_id='world';co.id='iron_test_block';co.operation=CollisionObject.REMOVE if remove else CollisionObject.ADD
 if not remove:
  box=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.04,.04,.04]);pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,position);co.primitives=[box];co.primitive_poses=[pose]
 req.scene.world.collision_objects=[co];assert call(ApplyPlanningScene,'/apply_planning_scene',req).success;pump(.6)
def enable(value):
 req=SetBool.Request();req.data=value;assert call(SetBool,'/iron_task_bridge/enable',req).success
pub=n.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1)
def jog(speed):
 msg=JointJog();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=names;msg.velocities=[speed,0.,0.,0.,0.];msg.duration=.02;pub.publish(msg)
try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request());assert len(hw.component)==1 and hw.component[0].plugin_name=='mock_components/GenericSystem';report['hardware']=hw.component[0].plugin_name
 deadline=time.monotonic()+25
 while True:
  ctr=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request());active={c.name for c in ctr.controller if c.state=='active'}
  if {'iron_arm_controller','joint_state_broadcaster'}==active and fresh() and buf.can_transform('world','iron_tcp',rclpy.time.Time()):break
  assert time.monotonic()<deadline,'Controllers/feedback/TF not ready';pump(.2)
 deadline=time.monotonic()+10
 while not bridge_status:
  assert time.monotonic()<deadline,'Task status discovery timed out';pump(.1)
 pump(.6);report['controllers']={c.name:c.state for c in ctr.controller};report['checks']['startup_collision']=validity();assert report['checks']['startup_collision']['valid']
 q=np.array([state[j] for j in names]);tf=buf.lookup_transform('world','iron_tcp',rclpy.time.Time()).transform.translation;expected=R.fk('iron',q)[:3,3];error=float(np.linalg.norm(expected-[tf.x,tf.y,tf.z]));report['checks']['startup_tf_error_m']=error;assert error<1e-8
 report['checks']['startup_bridge_disabled']='disabled' in bridge_status;assert report['checks']['startup_bridge_disabled']
 samples=json.loads((OUT/'collision-samples.json').read_text())['samples'];colliding=next(s for s in samples if s['collision']);report['checks']['fixture_collision_endpoint']=validity(colliding['q_rad']);assert not report['checks']['fixture_collision_endpoint']['valid']
 # Compare a small deterministic set of offline/live collision classifications.
 for row in samples[:12]:assert validity(row['q_rad'])['valid']==(not row['collision'])
 report['checks']['offline_live_collision_agreement_samples']=12
 study=json.loads((OUT/'workspace-study.json').read_text());assert study.get('urdf_sha256')==hashlib.sha256((W/'src/joint_effort_iron/urdf/iron.urdf').read_bytes()).hexdigest(),'Outdated workspace study'
 for target in study['targets']:
  solution=target['geometric_solution']
  if solution:
   target['collision_check']=validity(solution['q_rad']);t,j=R.fk('iron',solution['q_rad'],True);target['min_scaled_task_singular_value_m']=float(np.linalg.svd(task_jacobian(t,j,.12)[0],compute_uv=False)[-1])
 study['collision_free_geometric_solution_count']=sum(t['collision_check'] is not None and t['collision_check']['valid'] for t in study['targets']);(OUT/'workspace-study.json').write_text(json.dumps(study,indent=2)+'\n');report['checks']['board_hover_candidates']={'tested_targets':len(study['targets']),'geometric_solutions':study['geometric_solution_count'],'collision_free_endpoints':study['collision_free_geometric_solution_count']}

 enable(False);req=ServoCommandType.Request();req.command_type=req.JOINT_JOG;assert call(ServoCommandType,'/iron_servo/switch_command_type',req).success;pump(.3)
 before=state.copy();servo_codes.clear();deadline=time.monotonic()+1.5
 while time.monotonic()<deadline:assert fresh();jog(.05);pump(.02)
 pump(.8);settled=state.copy();pump(.4);delta=state['joint_1']-before['joint_1'];drift=max(abs(state[j]-settled[j]) for j in names)
 report['checks']['joint_jog_delta_rad']=delta;report['checks']['servo_timeout_settled_drift_rad']=drift;report['checks']['clear_motion_servo_codes']=sorted(set(servo_codes));assert delta>.025,'Servo motion unexpectedly slow';assert drift<1e-5
 print('PASS: live mock controllers, TF, fixture collision, joint jog and Servo timeout.',flush=True)
 # Add a world obstacle over the actual modeled tip and verify Servo refuses motion.
 tip=R.fk('iron',[state[j] for j in names])[:3,3];obstacle(tip);report['checks']['world_obstacle_collision']=validity();assert not report['checks']['world_obstacle_collision']['valid'];before=state.copy();servo_codes.clear()
 for _ in range(45):jog(.05);pump(.02)
 report['checks']['servo_obstacle_halt_seen']=ServoStatus.HALT_FOR_COLLISION in servo_codes;report['checks']['servo_obstacle_drift_rad']=max(abs(state[j]-before[j]) for j in names);assert report['checks']['servo_obstacle_halt_seen'];assert report['checks']['servo_obstacle_drift_rad']<1e-5
 obstacle(tip,True);assert validity()['valid'];pump(.4)
 # Plan a collision-checked joint path, then execute only on the verified mock JTC.
 request=GetMotionPlan.Request();m=request.motion_plan_request;m.group_name='iron_arm';m.allowed_planning_time=3.;m.num_planning_attempts=2;m.max_velocity_scaling_factor=.5;m.max_acceleration_scaling_factor=.5;m.start_state.joint_state.name=names;m.start_state.joint_state.position=[state[j] for j in names]
 goal=[state[j]+d for j,d in zip(names,[.025,.03,-.03,0.,0.])];constraints=Constraints();constraints.joint_constraints=[JointConstraint(joint_name=j,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for j,v in zip(names,goal)];m.goal_constraints=[constraints]
 plan=call(GetMotionPlan,'/plan_kinematic_path',request).motion_plan_response;assert plan.error_code.val==1,plan.error_code.val;trajectory=plan.trajectory.joint_trajectory;assert trajectory.points
 durations=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points];assert all(b>a for a,b in zip(durations,durations[1:])),durations;report['checks']['planned_duration_s']=durations[-1]
 report['checks']['planned_max_speed_rad_s']=max(abs(v) for p in trajectory.points for v in p.velocities);report['checks']['planned_max_acceleration_rad_s2']=max(abs(v) for p in trajectory.points for v in p.accelerations);assert report['checks']['planned_max_speed_rad_s']<=.075001;assert report['checks']['planned_max_acceleration_rad_s2']<=.150001
 # Verify actual planned points plus interpolation at <=0.01 rad joint spacing.
 prev=np.array([state[j] for j in names]);checked=0
 for point in trajectory.points:
  values=dict(zip(trajectory.joint_names,point.positions));next_q=np.array([values[j] for j in names]);steps=max(1,math.ceil(np.max(abs(next_q-prev))/.01))
  for i in range(1,steps+1):assert validity(prev+(next_q-prev)*i/steps)['valid'];checked+=1
  prev=next_q
 report['checks']['planned_path_collision_samples']=checked;report['checks']['planned_points']=len(trajectory.points)
 # The launch creates no hardware driver; recheck before mock execution.
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request());assert hw.component[0].plugin_name=='mock_components/GenericSystem'
 action=ActionClient(n,FollowJointTrajectory,'/iron_arm_controller/follow_joint_trajectory');assert action.wait_for_server(timeout_sec=10);g=FollowJointTrajectory.Goal();g.trajectory=trajectory;f=action.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10);handle=f.result();assert handle.accepted;future=handle.get_result_async();rclpy.spin_until_future_complete(n,future,timeout_sec=15);assert future.done() and future.result().result.error_code==0;pump(.3);maxerr=max(abs(state[j]-v) for j,v in zip(names,goal));report['checks']['planned_mock_execution_error_rad']=maxerr;assert maxerr<.001
 print('PASS: world obstacle halted Servo; MoveIt planned path was clear and executed on mock.',flush=True)
 # Reduced XYZ + shaft pointing; stale commands must settle without resuming.
 task=n.create_publisher(TwistStamped,'/iron/task_velocity',1);pump(.3);enable(True);before=np.array([state[j] for j in names]);start=R.fk('iron',before);bridge_status.clear();deadline=time.monotonic()+2.
 while time.monotonic()<deadline:
  assert fresh();msg=TwistStamped();msg.header.stamp=n.get_clock().now().to_msg();msg.header.frame_id='world';msg.twist.linear.z=.003;task.publish(msg);pump(.02)
 pump(.8);stopped=state.copy();end=R.fk('iron',[state[j] for j in names]);pump(.4);drift=max(abs(state[j]-stopped[j]) for j in names);movement=end[:3,3]-start[:3,3];angle=math.acos(float(np.clip(np.dot(start[:3,0],end[:3,0]),-1,1)))
 report['checks']['task_up_delta_mm']=(movement*1000).tolist();report['checks']['task_pointing_error_rad']=angle;report['checks']['task_timeout_seen']='command_timeout' in bridge_status;report['checks']['task_timeout_drift_rad']=drift
 assert movement[2]>.002,'Reduced XYZ task did not produce visible upward motion';assert angle<.005;assert drift<1e-5;assert report['checks']['task_timeout_seen'];assert 'feedback_fault_reenable_required' not in bridge_status;enable(False)
 # Loss of joint feedback must disable the bridge and require a new enable.
 enable(True);req=SwitchController.Request();req.deactivate_controllers=['joint_state_broadcaster'];req.strictness=SwitchController.Request.STRICT;assert call(SwitchController,'/controller_manager/switch_controller',req).ok;bridge_status.clear();pump(.7)
 report['checks']['feedback_loss_fault_seen']='feedback_fault_reenable_required' in bridge_status;assert report['checks']['feedback_loss_fault_seen']
 req=SwitchController.Request();req.activate_controllers=['joint_state_broadcaster'];req.strictness=req.STRICT;assert call(SwitchController,'/controller_manager/switch_controller',req).ok;pump(.6)
 report['checks']['feedback_recovery_stays_disabled']=bridge_status[-1]=='disabled';assert report['checks']['feedback_recovery_stays_disabled'];assert fresh();enable(False)
 report['checks']['final_collision_free']=validity()['valid'];assert report['checks']['final_collision_free'];report['feedback_age_p95_s']=float(np.percentile(ages,95));report['result']='PASS';print('PASS: reduced task moved upward, held pointing, and stopped on command timeout.',flush=True)
finally:
 report['servo_codes_seen']=sorted(set(servo_codes));n.destroy_node();rclpy.shutdown();launch.send_signal(signal.SIGINT)
 try:report['launch_exit_code']=launch.wait(timeout=20)
 except subprocess.TimeoutExpired:launch.terminate();report['launch_exit_code']=launch.wait(timeout=5)
 log.close();text=(OUT/'integration-launch.log').read_text();report['shutdown_no_segfault']='exit code -11' not in text and 'Segmentation fault' not in text;report['shutdown_children_clean']='process has died' not in text;(OUT/'live-check.json').write_text(json.dumps(report,indent=2)+'\n')
assert report['launch_exit_code']==0 and report['shutdown_no_segfault'] and report['shutdown_children_clean']
print(json.dumps(report,indent=2))
