"""R02 directions, motor-map binding and measurement-fit failure cases."""
import copy
import hashlib
import importlib.util
import unittest
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from joint_effort_iron.calibration import problems
from joint_effort_robot.kinematics import Robot, origin
W=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('r02fit',W/'tools/fit_r02_calibration.py')
fitter=importlib.util.module_from_spec(spec);spec.loader.exec_module(fitter)
TEXT=(W/'src/joint_effort_iron/urdf/iron.urdf').read_text()
R=Robot(W/'src/joint_effort_iron/urdf/iron.urdf')
def calibration():
 return dict(model_revision='R02',urdf_sha256=hashlib.sha256(TEXT.encode()).hexdigest(),
 physical_checks=dict(directions_verified=True,travel_limits_verified=True,tcp_verified=True),
 joints={name:dict(motor_id=i,center_steps=2048,encoder_direction=(-1)**i,min_rad=float(lo)-.05,max_rad=float(hi)+.05) for i,(name,(lo,hi)) in enumerate(zip(R.names('iron'),R.limits('iron')),1)})
class R02Tests(unittest.TestCase):
 def test_reference_axes_and_shaft(self):
  # R02 is a retained reference model; the installed iron model uses draft CAD.
  source=Robot(W/'src/joint_effort_arm_calibration_description/urdf/joint_effort_arm_calibration.urdf')
  frames={'base_link':np.eye(4)}
  for j in source.joints:
   frames[j.find('child').get('link')]=frames[j.find('parent').get('link')]@origin(j.find('origin'))
  names=['base_rotation_joint','shoulder_joint','elbow_joint','wrist_yaw_joint','wrist_pitch_joint']
  for name,axis in zip(names,[[0,0,1],[0,1,0],[0,1,0],[0,0,-1],[0,-1,0]]):
   j=source.active[name];t=frames[j.find('child').get('link')];np.testing.assert_allclose(t[:3,2],axis,atol=1e-9)
  np.testing.assert_allclose(frames['iron_tip'][:3,2],[0,0,-1],atol=1e-9)
  link=source.links['base_link'];np.testing.assert_allclose(origin(link.find('visual/origin')),origin(link.find('collision/origin')))
  self.assertAlmostEqual(float(link.find('visual/origin').get('rpy').split()[2]),-.5295341560844785)
 def test_model_binding_and_physical_flags(self):
  cal=calibration();self.assertEqual(problems(cal,TEXT),[]);self.assertTrue(problems(cal,TEXT+'\n'))
  for key in cal['physical_checks']:
   bad=copy.deepcopy(cal);bad['physical_checks'][key]=False;self.assertTrue(problems(bad,TEXT))
  bad=copy.deepcopy(cal);bad['joints']['joint_1']['center_steps']=50;self.assertTrue(problems(bad,TEXT))
  bad=copy.deepcopy(cal);bad['joints']['joint_2']['motor_id']=1;self.assertTrue(problems(bad,TEXT))
 def test_fit_both_directions_and_wrap(self):
  for center in (2048,50,4050):
   for direction in (-1,1):
    fit=fitter.fit([(a,round(center+direction*a*4096/360)%4096) for a in (-10,0,10)])
    self.assertEqual(fit['center_steps'],center);self.assertEqual(fit['encoder_direction'],direction)
 def test_monitor_count_reconstruction(self):
  spec=importlib.util.spec_from_file_location('monitor',W/'tools/sts_calibration_monitor.py')
  monitor=importlib.util.module_from_spec(spec);spec.loader.exec_module(monitor)
  for direction in (-1,1):
   stub=SimpleNamespace(centers={'joint_1':2048},directions={'joint_1':direction})
   for raw in (1700,2048,2300):
    q=direction*(raw-2048)*2*np.pi/4096
    self.assertEqual(monitor.Monitor.raw(stub,'joint_1',q),raw)
 def test_bad_measurements(self):
  for rows in ([],[(0,2048)],[(0,2048),(5,2105),(10,2162)],[(-10,1900),(0,2048),(10,2162)],[(-10,float('nan')),(0,2048),(10,2162)]):
   with self.assertRaises(ValueError):fitter.fit(rows)
if __name__=='__main__':unittest.main()
