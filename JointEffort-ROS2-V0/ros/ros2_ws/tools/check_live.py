#!/usr/bin/env python3
"""Integration verification in a mock-only ROS domain. Refuse non-mock hardware."""
import json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity,ApplyPlanningScene,ServoCommandType
from moveit_msgs.msg import CollisionObject,PlanningScene,ServoStatus
from geometry_msgs.msg import Pose,TwistStamped
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListHardwareComponents,ListControllers
from std_srvs.srv import SetBool
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from joint_effort_robot.kinematics import Robot

ROOT=Path(__file__).resolve().parents[1]
model=json.loads((ROOT/'src/joint_effort_robot/config/model.json').read_text())
robot=Robot(ROOT/'src/joint_effort_robot/urdf/joint_effort.urdf')
rclpy.init();n=Node('joint_effort_live_check');state={};statuses=[];bridge_status=[];feedback_ages=[]
def receive_state(m):
 state.update(zip(m.name,m.position));feedback_ages.append((n.get_clock().now().nanoseconds-(m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec))*1e-9)
n.create_subscription(JointState,'/joint_states',receive_state,1)
n.create_subscription(ServoStatus,'/iron_servo/status',lambda m:statuses.append(m.code),10)
n.create_subscription(String,'/task_bridge/status',lambda m:bridge_status.append(m.data),10)
buffer=Buffer();listener=TransformListener(buffer,n)

def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def call(kind,path,req):
 c=n.create_client(kind,path)
 if not c.wait_for_service(timeout_sec=15):raise RuntimeError('Missing service '+path)
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15)
 if not f.done(): raise RuntimeError('Service timeout '+path)
 result=f.result();n.destroy_client(c);return result
def validity(values):
 req=GetStateValidity.Request();req.robot_state.joint_state.name=list(values);req.robot_state.joint_state.position=list(values.values())
 req.group_name='' # Full robot including inactive arm and fixtures
 res=call(GetStateValidity,'/check_state_validity',req)
 return {'valid':res.valid,'contacts':[[c.contact_body_1,c.contact_body_2] for c in res.contacts]}
def scene_obstacle(position,remove=False):
 req=ApplyPlanningScene.Request();req.scene.is_diff=True
 co=CollisionObject();co.header.frame_id='world';co.id='test_block';co.operation=CollisionObject.REMOVE if remove else CollisionObject.ADD
 if not remove:
  box=SolidPrimitive();box.type=SolidPrimitive.BOX;box.dimensions=[.04,.04,.04]
  pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,position)
  co.primitives=[box];co.primitive_poses=[pose]
 req.scene.world.collision_objects=[co]
 assert call(ApplyPlanningScene,'/apply_planning_scene',req).success
 pump(.5)

try:
 hw=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
 assert len(hw.component)==1 and hw.component[0].plugin_name=='mock_components/GenericSystem', 'Refusing a non-mock hardware system'
 deadline=time.monotonic()+30
 while True:
  ctr=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
  if {c.name:c.state for c in ctr.controller}=={'joint_state_broadcaster':'active','iron_arm_controller':'active','wire_arm_controller':'active'}:break
  if time.monotonic()>deadline:raise RuntimeError('Mock controllers did not activate')
  pump(.2)
 pump(1)
 assert len(state)==10,'Ten joint states required'
 baseline=state.copy();report={'hardware_plugin':hw.component[0].plugin_name,'controllers':{c.name:c.state for c in ctr.controller},'startup_collision':validity(state)}
 # Independently compare ROS TF with URDF numerical forward kinematics.
 report['tf_error_m']={}
 for arm in ['iron','wire']:
  tf=buffer.lookup_transform('world',arm+'_tcp',rclpy.time.Time())
  actual=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
  expected=robot.fk(arm,[state[j] for j in robot.names(arm)])[:3,3]
  report['tf_error_m'][arm]=float(np.linalg.norm(actual-expected));assert report['tf_error_m'][arm]<1e-8
 assert report['startup_collision']['valid'],report['startup_collision']
 # Collision service must reject a world block overlapping the iron tool.
 tip=robot.fk('iron',[state[j] for j in robot.names('iron')])[:3,3]
 scene_obstacle(tip)
 report['world_obstacle_collision']=validity(state)
 assert not report['world_obstacle_collision']['valid'],'World collision was missed'
 # Verify the obstacle also reaches Servo's collision monitor.
 blocked_pub=n.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1)
 before_block=state.copy();statuses.clear();pump(.5)
 for _ in range(35):
  msg=JointJog();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=robot.names('iron');msg.velocities=[.02,0,0,0,0];msg.duration=.02
  blocked_pub.publish(msg);pump(.02)
 report['servo_collision_status_seen']=ServoStatus.HALT_FOR_COLLISION in statuses
 report['servo_collision_blocked_delta_rad']=max(abs(state[j]-before_block[j]) for j in robot.names('iron'))
 assert report['servo_collision_status_seen'],'Servo collision halt status missing'
 assert report['servo_collision_blocked_delta_rad']<1e-5,'Servo moved despite overlapping obstacle'
 scene_obstacle(tip,True)
 assert validity(state)['valid']
 # Named joint jog through Servo; inactive arm must remain still.
 pub=n.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1);pump(.5)
 req=ServoCommandType.Request();req.command_type=req.JOINT_JOG
 assert call(ServoCommandType,'/iron_servo/switch_command_type',req).success
 for _ in range(30):
  msg=JointJog();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=robot.names('iron');msg.velocities=[.02,0,0,0,0];msg.duration=.02
  pub.publish(msg);pump(.02)
 report['servo_streaming_delta_rad']=state['iron_base_yaw']-baseline['iron_base_yaw']
 print('Streaming motion',report['servo_streaming_delta_rad'],'Servo statuses',set(statuses),flush=True)
 assert report['servo_streaming_delta_rad']>1e-5,'No continuous motion while Servo stream active'
 pump(.8)
 report['servo_joint_jog_delta_rad']=state['iron_base_yaw']-baseline['iron_base_yaw']
 report['inactive_arm_delta_rad']=max(abs(state[j]-baseline[j]) for j in robot.names('wire'))
 assert report['servo_joint_jog_delta_rad']>.006,'Clear-space motion is unexpectedly slow; check invariant mounting pairs in the collision matrix'
 assert report['inactive_arm_delta_rad']<1e-9,'Inactive arm moved'
 # Enable bridge, send stamped reduced XYZ input, and allow watchdog to expire.
 before=state.copy();bridge_status.clear();req=SetBool.Request();req.data=True
 assert call(SetBool,'/task_bridge/enable',req).success
 task=n.create_publisher(TwistStamped,'/iron/task_velocity',1);pump(.3)
 for _ in range(30):
  msg=TwistStamped();msg.header.stamp=n.get_clock().now().to_msg();msg.header.frame_id='world';msg.twist.linear.z=.002
  task.publish(msg);pump(.02)
 pump(.6);stopped=state.copy();pump(.4)
 report['bridge_motion_delta_rad']=max(abs(stopped[j]-before[j]) for j in robot.names('iron'))
 report['watchdog_settled_delta_rad']=max(abs(state[j]-stopped[j]) for j in robot.names('iron'))
 report['bridge_timeout_seen']='command_timeout' in bridge_status[-100:]
 report['bridge_feedback_fault_during_motion_test']='feedback_fault_reenable_required' in bridge_status
 print('Bridge check',report['bridge_motion_delta_rad'],'statuses',set(bridge_status),'servo',set(statuses))
 assert report['bridge_motion_delta_rad']>.0001
 assert report['watchdog_settled_delta_rad']<1e-5
 assert report['bridge_timeout_seen']
 assert not report['bridge_feedback_fault_during_motion_test'], 'Feedback fault confounded command-watchdog test'
 req.data=False;assert call(SetBool,'/task_bridge/enable',req).success
 # Classify offline IK endpoints with the entire second arm parked.
 study=json.loads((ROOT/'reports/workspace-study.json').read_text())
 endpoints=[]
 for candidate in study['target_candidates']:
  if candidate['solution'] is None:
   endpoints.append({**candidate,'collision_check':None});continue
  values=baseline.copy()
  values.update(zip(robot.names(candidate['arm']),candidate['solution']['q_rad']))
  endpoints.append({**candidate,'collision_check':validity(values)})
 (ROOT/'reports/collision-endpoints.json').write_text(json.dumps(endpoints,indent=2)+'\n')
 report['board_endpoint_counts']={a:{'ik_solutions':sum(e['solution'] is not None for e in endpoints if e['arm']==a),
  'collision_free_with_other_arm_parked':sum(e['collision_check'] is not None and e['collision_check']['valid'] for e in endpoints if e['arm']==a)} for a in ['iron','wire']}
 # Both tips at the same above-board endpoint must be identified as inter-arm collision.
 iron=next(e for e in endpoints if e['arm']=='iron' and e['solution'] and abs(e['target_m'][0])<1e-8 and abs(e['target_m'][1]-.2)<1e-8)
 wire=next(e for e in endpoints if e['arm']=='wire' and e['target_m']==iron['target_m'])
 same_tip=baseline.copy()
 for e in [iron,wire]:same_tip.update(zip(robot.names(e['arm']),e['solution']['q_rad']))
 report['inter_arm_same_tip_collision']=validity(same_tip)
 assert not report['inter_arm_same_tip_collision']['valid']
 report['feedback_age_p95_s']=float(np.percentile(feedback_ages,95))
 report['feedback_age_max_s']=float(max(feedback_ages))
 report['result']='PASS' 
 (ROOT/'reports/live-check.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
finally:
 n.destroy_node();rclpy.shutdown()
