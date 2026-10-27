import unittest
import json
from pathlib import Path
import numpy as np
from joint_effort_robot.kinematics import Robot, task_jacobian, differential_step
P=Path(__file__).resolve().parents[1]/'src/joint_effort_robot'
M=json.loads((P/'config/model.json').read_text())
R=Robot(P/'urdf/joint_effort.urdf')
class KinematicsTests(unittest.TestCase):
    def test_jacobian_independent_finite_difference(self):
        rng=np.random.default_rng(41)
        for arm in ['iron','wire']:
            for _ in range(25):
                q=rng.uniform(R.limits(arm)[:,0]+0.05,R.limits(arm)[:,1]-0.05)
                t,j=R.fk(arm,q,True)
                for k in range(5):
                    h=1e-6; dq=np.eye(5)[k]*h
                    a,b=R.fk(arm,q+dq),R.fk(arm,q-dq)
                    np.testing.assert_allclose((a[:3,3]-b[:3,3])/(2*h),j[:3,k],atol=1e-8)
                    dr=(a[:3,:3]-b[:3,:3])/(2*h)@t[:3,:3].T
                    np.testing.assert_allclose([dr[2,1],dr[0,2],dr[1,0]],j[3:,k],atol=1e-8)
    def test_startup_faces_shared_work_area(self):
        for arm in ['iron','wire']:
            t,j=R.fk(arm,M['initial_positions_by_arm_rad'][arm],True)
            self.assertGreater(t[1,3],0)
            self.assertGreater(np.linalg.svd(task_jacobian(t,j)[0],compute_uv=False)[-1],0.001)
    def test_singular_straight_arm_stops(self):
        v,s=differential_step(R,'iron',np.zeros(5),[.001,0,0],[0,0,0],M)
        self.assertEqual(s['status'],'singularity');np.testing.assert_array_equal(v,0)
    def test_bad_inputs_and_limit_guard(self):
        for dt in [0,-1,float('nan')]:
            with self.assertRaises(ValueError): differential_step(R,'iron',np.zeros(5),[0,0,0],[0,0,0],M,dt)
        with self.assertRaises(ValueError): differential_step(R,'iron',np.zeros(5),[float('nan'),0,0],[0,0,0],M)
        q=np.array(M['initial_positions_by_arm_rad']['iron']);q[0]=5
        v,s=differential_step(R,'iron',q,[.001,0,0],[0,0,0],M)
        self.assertEqual(s['status'],'out_of_bounds');np.testing.assert_array_equal(v,0)
    def test_velocity_cap_and_inputs_unchanged(self):
        linear=np.array([1.,0,0]);angular=np.array([0.,1,0]);q=M['initial_positions_by_arm_rad']['iron']
        v,_=differential_step(R,'iron',q,linear,angular,M)
        self.assertLessEqual(np.max(abs(v)),M['mock_max_joint_velocity_rad_s']+1e-12)
        np.testing.assert_array_equal(linear,[1,0,0]);np.testing.assert_array_equal(angular,[0,1,0])
    def test_free_roll_is_omitted(self):
        q=M['initial_positions_by_arm_rad']['iron'];t=R.fk('iron',q)
        v,_=differential_step(R,'iron',q,[0,0,0],t[:3,0]*.01,M)
        np.testing.assert_allclose(v,0,atol=1e-12)
if __name__=='__main__': unittest.main()
