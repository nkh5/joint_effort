#!/usr/bin/env python3
"""CAD-based iron-only cold mock model; explicit provisional collision hulls."""
import json,math,struct,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
W=Path(__file__).resolve().parents[1];P=W/'src/joint_effort_iron';SOURCE=W/'src/joint_effort_arm_draft_description';root=ET.parse(SOURCE/'urdf/joint_effort_arm_draft.urdf').getroot();root.set('name','joint_effort_iron')
model={'scope':'CAD iron-only cold mock. Provisional workspace, travel, calibration and collision hulls. No physical driver.','base_xyz_m':[0,0,.031],'base_yaw_rad':math.pi,'joint_limits_deg':[-30,30],'proposed_future_mechanical_stop_deg':[-35,35],'mechanical_stops_status':'Concept only, not modeled hardware or fabrication approval; validate +/-35 deg against the physical assembly before installing stops','proposed_ids':list(range(1,6)),'proposed_encoder_zero_counts':[2048]*5,'proposed_encoder_sign':[1]*5,'external_joint_to_servo_ratio':[1.]*5,'calibration_confirmed':False,'provisional_servo_supply_voltage_v':12.6,'motor_variant':'Provisional 12.6V supply per user; matching motor-variant datasheet pending','supplied_datasheet_variant_voltage_range_v':[4,7.4],'mock_effort_status':'0.49 Nm is a software/mock metadata choice, not a rating for the provisional 12.6V variant','encoder_counts_per_output_turn':4096,'internal_motor_reduction_not_external_ratio':345,'mock_max_joint_velocity_rad_s':.15,'mock_max_joint_acceleration_rad_s2':.3,'mock_effort_limit_nm':.49,'task_orientation_length_m':.12,'task_damping_m':.002,'task_min_singular_value_m':.003,'task_max_linear_speed_m_s':.003,'task_max_angular_speed_rad_s':.03,'table_center_m':[.25,0,-.025],'table_size_m':[1.,.8,.05],'pcb_center_m':[.43,.04,.151],'pcb_size_m':[.12,.08,.002],'pcb_fixture_center_m':[.43,.04,.075],'pcb_fixture_size_m':[.12,.08,.15],'initial_positions_rad':[0.]*5}
ET.SubElement(root,'link',name='world');j=ET.SubElement(root,'joint',name='iron_mount',type='fixed');ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link='base_link');ET.SubElement(j,'origin',xyz='0 0 .031',rpy=f'0 0 {math.pi}')
colors=['.3 .4 .5 1','.4 .5 .6 1','.85 .65 .3 1','.3 .65 .55 1','.6 .4 .75 1','.75 .45 .55 1'];mesh_report=[]
for i,link in enumerate(root.findall('link')):
 visual=link.find('visual');mesh=link.find('visual/geometry/mesh')
 if mesh is None:continue
 material=ET.SubElement(visual,'material',name='cad_color_'+str(i));ET.SubElement(material,'color',rgba=colors[i])
 name=link.get('name');raw=(SOURCE/'meshes'/f'{name}.stl').read_bytes();count=struct.unpack('<I',raw[80:84])[0]
 dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]);vertices=np.frombuffer(raw,dtype=dtype,offset=84,count=count)['v'].reshape(-1,3).astype(float)
 vertices=np.unique(vertices,axis=0);hull=ConvexHull(vertices);faces=hull.simplices.copy()
 # Orient faces outward, then emit a compact binary STL without rescaling.
 triangles=vertices[faces];normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);flip=np.sum(normals*hull.equations[:,:3],axis=1)<0;faces[flip]=faces[flip][:,[0,2,1]];triangles=vertices[faces];normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
 payload=np.zeros(len(faces),dtype=dtype);payload['normal']=normals;payload['v']=triangles
 file=P/'meshes'/f'{name}_hull.stl';file.write_bytes(b'Joint Effort conservative CAD convex hull'.ljust(80,b'\0')+struct.pack('<I',len(faces))+payload.tobytes())
 collision=ET.SubElement(link,'collision');ET.SubElement(collision,'origin',xyz='0 0 0',rpy='0 0 0');geometry=ET.SubElement(collision,'geometry');ET.SubElement(geometry,'mesh',filename=f'package://joint_effort_iron/meshes/{file.name}',scale='1 1 1')
 # All visual vertices satisfy the hull planes (within float32 tessellation error).
 error=max(float(np.max(vertices@eq[:3]+eq[3])) for eq in hull.equations);assert error<1e-8
 mesh_report.append({'link':name,'visual_triangles':count,'hull_triangles':len(faces),'max_vertex_outside_hull_m':error,'collision_sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'policy':'Convex envelope of entire visual link; fills gaps conservatively. Adjacent-link collisions exempted; not physical clearance certification.'})
for j in root.findall('joint'):
 if j.get('type')=='revolute':j.find('limit').set('effort','.49');j.find('limit').set('velocity','.15')
# TCP +X is the nominal shaft-forward direction, tip-frame +Y.
ET.SubElement(root,'link',name='iron_tcp');j=ET.SubElement(root,'joint',name='iron_tcp_axis',type='fixed');ET.SubElement(j,'parent',link='iron_tip');ET.SubElement(j,'child',link='iron_tcp');ET.SubElement(j,'origin',xyz='0 0 0',rpy=f'0 0 {math.pi/2}')
for name,xyz,size,color in [('table',model['table_center_m'],model['table_size_m'],'.5 .5 .5 1'),('pcb_fixture',model['pcb_fixture_center_m'],model['pcb_fixture_size_m'],'.45 .45 .5 .6'),('pcb',model['pcb_center_m'],model['pcb_size_m'],'.1 .5 .2 1')]:
 link=ET.SubElement(root,'link',name=name)
 for tag in ['visual','collision']:
  obj=ET.SubElement(link,tag);geo=ET.SubElement(obj,'geometry');ET.SubElement(geo,'box',size=' '.join(map(str,size)))
  if tag=='visual':mat=ET.SubElement(obj,'material',name=name+'_color');ET.SubElement(mat,'color',rgba=color)
 j=ET.SubElement(root,'joint',name=name+'_fixed',type='fixed');ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link=name);ET.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy='0 0 0')
control=ET.SubElement(root,'ros2_control',name='IronCADMock',type='system');hardware=ET.SubElement(control,'hardware');ET.SubElement(hardware,'plugin').text='mock_components/GenericSystem'
names=[f'joint_{i}' for i in range(1,6)]
for name in names:
 j=ET.SubElement(control,'joint',name=name);ET.SubElement(j,'command_interface',name='position');state=ET.SubElement(j,'state_interface',name='position');ET.SubElement(state,'param',name='initial_value').text='0';ET.SubElement(j,'state_interface',name='velocity')
ET.indent(root);ET.ElementTree(root).write(P/'urdf/iron.urdf',encoding='utf-8',xml_declaration=True)
srdf=ET.Element('robot',name='joint_effort_iron');group=ET.SubElement(srdf,'group',name='iron_arm');ET.SubElement(group,'chain',base_link='base_link',tip_link='iron_tcp');park=ET.SubElement(srdf,'group_state',name='park',group='iron_arm')
for name in names:ET.SubElement(park,'joint',name=name,value='0')
for j in root.findall('joint'):
 ET.SubElement(srdf,'disable_collisions',link1=j.find('parent').get('link'),link2=j.find('child').get('link'),reason='Adjacent')
# Only invariant mounting/static-fixture pairs, never moving-link vs tabletop.
for a,b in [('base_link','table'),('table','pcb_fixture'),('pcb_fixture','pcb'),('table','pcb')]:ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Fixed mounting or fixture contact')
ET.indent(srdf);ET.ElementTree(srdf).write(P/'config/iron.srdf',encoding='utf-8',xml_declaration=True)
(P/'config/model.json').write_text(json.dumps(model,indent=2)+'\n')
controllers={'controller_manager':{'ros__parameters':{'update_rate':50,'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},'iron_arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},'iron_arm_controller':{'ros__parameters':{'joints':names,'command_interfaces':['position'],'state_interfaces':['position','velocity'],'allow_partial_joints_goal':False,'allow_nonzero_velocity_at_trajectory_end':True}}}
(P/'config/controllers.yaml').write_text(json.dumps(controllers,indent=2)+'\n')
(P/'config/kinematics.yaml').write_text(json.dumps({'iron_arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.05,'position_only_ik':True}},indent=2)+'\n')
(P/'config/joint_limits.yaml').write_text(json.dumps({'joint_limits':{n:{'has_velocity_limits':True,'max_velocity':.15,'has_acceleration_limits':True,'max_acceleration':.3} for n in names}},indent=2)+'\n')
servo=json.loads((W/'src/joint_effort_robot/config/iron_servo.yaml').read_text());servo['moveit_servo']['joint_limit_margins']=[.04];servo['moveit_servo']['self_collision_proximity_threshold']=.004;servo['moveit_servo']['scene_collision_proximity_threshold']=.004;(P/'config/servo.yaml').write_text(json.dumps(servo,indent=2)+'\n')
(W/'reports/iron/collision-hulls.json').write_text(json.dumps(mesh_report,indent=2)+'\n');print('Generated iron-only CAD mock, fixtures and',sum(m['hull_triangles'] for m in mesh_report),'collision triangles.')
