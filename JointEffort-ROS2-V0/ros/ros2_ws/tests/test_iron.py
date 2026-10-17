import json,unittest,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from joint_effort_robot.kinematics import Robot,origin,rotation,differential_step,task_jacobian
W=Path(__file__).resolve().parents[1];P=W/'src/joint_effort_iron';R=Robot(P/'urdf/iron.urdf');M=json.loads((P/'config/model.json').read_text())
class IronTests(unittest.TestCase):
 def test_preserves_cad_origins_and_visual_meshes(self):
  source=ET.parse(W/'src/joint_effort_arm_draft_description/urdf/joint_effort_arm_draft.urdf').getroot()
  for j in source.findall('joint'):
   new=next(v for v in R.joints if v.get('name')==j.get('name'));np.testing.assert_allclose(origin(new.find('origin')),origin(j.find('origin')),atol=1e-14)
  for link in source.findall('link'):
   mesh=link.find('visual/geometry/mesh')
   if mesh is not None:self.assertEqual(mesh.attrib,R.links[link.get('name')].find('visual/geometry/mesh').attrib)
 def test_nominal_tip_and_pointing_convention(self):
  t=R.fk('iron',np.zeros(5));np.testing.assert_allclose(t[:3,3],[.394285284139,.043978599593,.381178433023],atol=1e-10)
  frames=R.frames({});np.testing.assert_allclose(t[:3,0],frames['iron_tip'][:3,1],atol=1e-12)
 def test_geometric_jacobian_at_varied_poses(self):
  rng=np.random.default_rng(88)
  for q in rng.uniform(-.5,.5,(12,5)):
   t,j=R.fk('iron',q,True)
   for k in range(5):
    h=np.eye(5)[k]*1e-6;a=R.fk('iron',q+h);b=R.fk('iron',q-h);np.testing.assert_allclose((a[:3,3]-b[:3,3])/2e-6,j[:3,k],atol=1e-8)
 def test_xyz_pointing_residual_and_free_roll(self):
  q=np.zeros(5);t,j=R.fk('iron',q,True);reduced,basis=task_jacobian(t,j,.12)
  for direction in np.eye(3):
   v,info=differential_step(R,'iron',q,direction*.002,[0,0,0],M);self.assertEqual(info['status'],'ok');np.testing.assert_allclose(j[:3]@v,direction*.002,atol=1e-5);self.assertLess(np.linalg.norm(basis.T@j[3:]@v),1e-4)
  v,_=differential_step(R,'iron',q,[0,0,0],t[:3,0]*.01,M);np.testing.assert_allclose(v,0,atol=1e-12)
 def test_velocity_and_limit_guards(self):
  q=np.zeros(5);v,_=differential_step(R,'iron',q,[1,1,1],[1,1,1],M);self.assertLessEqual(max(abs(v)),.15)
  q[0]=1;v,info=differential_step(R,'iron',q,[.001,0,0],[0,0,0],M);self.assertEqual(info['status'],'out_of_bounds');np.testing.assert_array_equal(v,0)
 def test_mock_and_provisional_metadata(self):
  self.assertEqual(R.root.find('ros2_control/hardware/plugin').text,'mock_components/GenericSystem');self.assertEqual(len(R.active),5);self.assertEqual(M['provisional_servo_supply_voltage_v'],12.6);self.assertFalse(M['calibration_confirmed'])
if __name__=='__main__':unittest.main()
