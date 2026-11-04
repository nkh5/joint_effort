#!/usr/bin/env python3
"""Isolated R02 mock checks. Never starts a serial driver or opens a device."""
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TwistStamped
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SwitchController
from moveit_msgs.msg import CollisionObject, ServoStatus, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity, ServoCommandType, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from joint_effort_robot.kinematics import Robot
from joint_effort_input.synthetic import Source
from joint_effort_input.forwarder import STATUS

W=Path(__file__).resolve().parents[1];OUT=W/'reports/r02';OUT.mkdir(exist_ok=True,parents=True)
if os.environ.get('ROS_DOMAIN_ID')!='188' or os.environ.get('JE_MOCK_TEST')!='1':
    raise SystemExit('Use domain 188 and JE_MOCK_TEST=1. This test must be isolated from hardware.')
R=Robot(W/'src/joint_effort_iron/urdf/iron.urdf');NAMES=R.names('iron')
rclpy.init();node=Node('r02_mock_check');q=None;received=-math.inf;stamps=[];receipts=[];codes=[];display={};source=None
processes=[];logs=[];report={'result':'INCOMPLETE','checks':{},'scope':'isolated domain 188, GenericSystem only'}
buf=Buffer();listener=TransformListener(buf,node)
def feedback(m):
    global q,received
    if set(m.name)==set(NAMES) and len(m.position)==5:
        values=dict(zip(m.name,m.position));q=np.array([values[n] for n in NAMES]);received=time.monotonic()
        receipts.append(received);stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
def status(m):
    global display
    display=json.loads(m.data)
node.create_subscription(JointState,'/joint_states',feedback,qos_profile_sensor_data)
node.create_subscription(ServoStatus,'/iron_servo/status',lambda m:codes.append(m.code),10)
node.create_subscription(String,'/amw/display_state',status,STATUS)
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.01,max(0,end-time.monotonic())))
def call(kind,path,request,timeout=10):
    c=node.create_client(kind,path)
    try:
        assert c.wait_for_service(timeout_sec=timeout),'Missing '+path
        f=c.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
        assert f.done() and not f.exception(),'Failed '+path
        return f.result()
    finally:node.destroy_client(c)
def start(name,args):
    log=(OUT/(name+'.log')).open('w');logs.append(log)
    p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p

def valid(values=None):
    req=GetStateValidity.Request();req.robot_state.joint_state.name=NAMES
    req.robot_state.joint_state.position=list(map(float,q if values is None else values))
    return call(GetStateValidity,'/check_state_validity',req)
def mock_only():
    h=call(ListHardwareComponents,'/controller_manager/list_hardware_components',ListHardwareComponents.Request())
    assert len(h.component)==1 and h.component[0].plugin_name=='mock_components/GenericSystem','Not mock hardware'
def settle():
    pump(.65);before=q.copy();pump(.3);drift=float(max(abs(q-before)));assert drift<1e-5,drift;return drift

def obstacle(position,remove=False):
    req=ApplyPlanningScene.Request();req.scene.is_diff=True
    obj=CollisionObject();obj.id='r02_tip_obstacle';obj.header.frame_id='world';obj.operation=obj.REMOVE if remove else obj.ADD
    if not remove:
        pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,position)
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.04]*3)];obj.primitive_poses=[pose]
    req.scene.world.collision_objects=[obj]
    assert call(ApplyPlanningScene,'/apply_planning_scene',req).success;pump(.6)

def emit(seconds,axes=(0.,0.,0.),hold=0,toggle=0,level=10):
    source.spec=(tuple(axes),hold,toggle,level,None);pump(seconds)
def prepare():
    emit(.8);assert display.get('prepared') and display.get('phase')=='idle',display
    emit(.4,hold=1);assert display.get('enabled'),display

try:
    pump(.5);others=[n for n in node.get_node_names() if n!='r02_mock_check']
    assert not others,('Test domain already has nodes',others)
    launch=start('mock-launch',['ros2','launch','joint_effort_iron','mock.launch.py','rviz:=false'])
    mock_only();deadline=time.monotonic()+45
    while True:
        c=call(ListControllers,'/controller_manager/list_controllers',ListControllers.Request())
        active={x.name for x in c.controller if x.state=='active'}
        if q is not None and time.monotonic()-received<.3 and {'iron_arm_controller','joint_state_broadcaster'}<=active and codes:break
        assert time.monotonic()<deadline,('Not ready',active,q,codes[-5:]);pump(.25)
    assert valid().valid
    pump(.5);tf=buf.lookup_transform('world','iron_tcp',rclpy.time.Time()).transform.translation
    error=float(np.linalg.norm(R.fk('iron',q)[:3,3]-[tf.x,tf.y,tf.z]));assert error<1e-8
    report['checks']['tf_error_m']=error;report['checks']['startup_clear']=True
    # Test all corners plus a coarse grid, rather than rely only on random endpoints.
    collisions=[];bounds=R.limits('iron')
    for values in itertools.product(*[np.linspace(a,b,3) for a,b in bounds]):
        result=valid(values)
        if not result.valid:collisions.append({'q':values,'contacts':[(x.contact_body_1,x.contact_body_2) for x in result.contacts]})
    report['checks']['grid_poses_checked']=243;report['checks']['grid_collisions']=collisions
    req=ServoCommandType.Request();req.command_type=req.JOINT_JOG
    assert call(ServoCommandType,'/iron_servo/switch_command_type',req).success;pump(.4)
    pub=node.create_publisher(JointJog,'/iron_servo/delta_joint_cmds',1);pump(.3)
    def jog(index,speed,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert time.monotonic()-received<.3,'Feedback stale'
            m=JointJog();m.header.stamp=node.get_clock().now().to_msg();m.joint_names=NAMES
            m.velocities=[speed if i==index else 0. for i in range(5)];m.duration=.02;pub.publish(m);pump(.02)
    deltas=[]
    for index in range(5):
        mock_only();before=q.copy();jog(index,.04,1.2);settle();delta=float(q[index]-before[index]);assert delta>.02,(index,delta,codes[-10:])
        deltas.append(delta);jog(index,-.04,1.2);settle();assert valid().valid
    report['checks']['five_joint_positive_changes_rad']=deltas
    report['checks']['command_timeout_hold']=True
    tip=R.fk('iron',q)[:3,3];obstacle(tip);assert not valid().valid
    before=q.copy();codes.clear();jog(0,.04,1.);settle()
    assert ServoStatus.HALT_FOR_COLLISION in codes,codes
    assert max(abs(q-before))<1e-5
    report['checks']['world_obstacle_blocks_motion']=True
    obstacle(tip,True);assert valid().valid
    # Plan and execute a short joint path, checking the interpolated path first.
    from iron_trajectory import audit
    req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='iron_arm'
    m.allowed_planning_time=3.;m.num_planning_attempts=2
    m.max_velocity_scaling_factor=.5;m.max_acceleration_scaling_factor=.5
    m.start_state.joint_state.name=NAMES;m.start_state.joint_state.position=q.tolist()
    goal=q.copy();goal[0]+=.03;constraints=Constraints()
    constraints.joint_constraints=[JointConstraint(joint_name=n,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,v in zip(NAMES,goal)]
    m.goal_constraints=[constraints]
    result=call(GetMotionPlan,'/plan_kinematic_path',req).motion_plan_response
    assert result.error_code.val==1,result.error_code.val
    trajectory=result.trajectory.joint_trajectory
    samples,timing=audit(trajectory,NAMES,bounds,.15,.3)
    for _,values in samples:assert valid(values).valid
    mock_only();action=ActionClient(node,FollowJointTrajectory,'/iron_arm_controller/follow_joint_trajectory')
    assert action.wait_for_server(timeout_sec=10)
    g=FollowJointTrajectory.Goal();g.trajectory=trajectory;f=action.send_goal_async(g)
    rclpy.spin_until_future_complete(node,f,timeout_sec=10);handle=f.result();assert handle.accepted
    f=handle.get_result_async();rclpy.spin_until_future_complete(node,f,timeout_sec=15)
    assert f.done() and f.result().result.error_code==0;pump(.3)
    assert max(abs(q-goal))<.001
    report['checks']['planned_path_samples_checked']=len(samples)
    report['checks']['planned_mock_execution_error_rad']=float(max(abs(q-goal)))
    print('PASS: five joints, TF, endpoint grid, timeout, obstacle halt and planned mock path.',flush=True)
    # Follow the full current input chain, including the configured Pico button indices.
    inputs=start('input-launch',['ros2','launch','joint_effort_input','input.launch.py','joystick:=false','mapping_confirmed:=true','display:=false'])
    source=Source('r02_synthetic_pico');emit(1.)
    deadline=time.monotonic()+25
    while not display.get('mock_ready') or not display.get('collision_clear'):
        assert time.monotonic()<deadline,display;emit(.2)
    movements=[]
    for axis,sign in [(2,1),(2,-1),(0,1),(0,-1)]:
        prepare();before=R.fk('iron',q);axes=[0.,0.,0.];axes[axis]=sign*.45
        emit(1.,axes,hold=1);assert display.get('enabled'),display
        emit(.2);settle();after=R.fk('iron',q);delta=(after[:3,3]-before[:3,3])*1000
        assert sign*delta[axis]>.8,delta
        movements.append(delta.tolist())
    report['checks']['joystick_translation_mm']=movements
    emit(.1,toggle=1);emit(.8);assert display.get('mode')=='rotation',display
    rotation=[]
    for axis in (0,1):
        prepare();before=R.fk('iron',q)[:3,0];axes=[0.,0.,0.];axes[axis]=.4
        emit(1.,axes,hold=1);assert display.get('enabled'),display;emit(.2);settle()
        after=R.fk('iron',q)[:3,0];angle=math.acos(float(np.clip(before@after,-1,1)))
        assert angle>.005,angle;rotation.append(angle)
        prepare();axes[axis]=-.4;emit(1.,axes,hold=1);emit(.2);settle()
    report['checks']['joystick_pointing_changes_rad']=rotation
    emit(.1,toggle=1);emit(.8);prepare();emit(.6,(0.,0.,.4),hold=1)
    source.spec=None;pump(.7);assert not display.get('enabled'),display;settle()
    emit(.6,hold=1);assert not display.get('enabled'),'Reconnect resumed with enable held'
    emit(.8);prepare();emit(.4,(0.,0.,.4),hold=1)
    assert call(Trigger,'/amw/stop',Trigger.Request()).success
    emit(.5,hold=1);assert not display.get('enabled');settle()
    report['checks']['input_loss_and_stop_require_release']=True
    emit(.8);prepare();emit(.4,(0.,0.,.4),hold=1)
    req=SwitchController.Request();req.deactivate_controllers=['joint_state_broadcaster'];req.strictness=req.STRICT
    assert call(SwitchController,'/controller_manager/switch_controller',req).ok
    emit(.7,hold=1);assert not display.get('enabled'),display
    req=SwitchController.Request();req.activate_controllers=['joint_state_broadcaster'];req.strictness=req.STRICT
    assert call(SwitchController,'/controller_manager/switch_controller',req).ok
    emit(.7,hold=1);assert not display.get('enabled'),'Feedback recovery resumed movement'
    emit(.8);settle();report['checks']['feedback_loss_requires_release']=True
    report['checks']['final_collision_clear']=valid().valid;assert report['checks']['final_collision_clear']
    # Frequencies are measured from this test, rather than inferred from configuration.
    intervals=np.diff(stamps);intervals=intervals[intervals>0]
    report['joint_state_stamp_rate_hz']=float(1/np.median(intervals))
    report['result']='PASS'
    print('PASS: configured synthetic Pico, translation/pointing, release, disconnect and stop.',flush=True)
except BaseException as e:
    report['error']=repr(e);raise
finally:
    if source is not None:source.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    exits=[]
    for p in reversed(processes):
        if p.poll() is None:p.send_signal(signal.SIGINT)
        try:exits.append(p.wait(timeout=20))
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);exits.append(p.wait())
    for log in logs:log.close()
    report['launch_exit_codes']=exits
    report['clean_shutdown']=all(x==0 for x in exits) and all('process has died' not in p.read_text() and 'Segmentation fault' not in p.read_text() for p in (OUT/'mock-launch.log',OUT/'input-launch.log') if p.exists())
    (OUT/'live-check.json').write_text(json.dumps(report,indent=2)+'\n')
if not report['clean_shutdown']:raise SystemExit('Shutdown check failed')
print(json.dumps(report,indent=2))
