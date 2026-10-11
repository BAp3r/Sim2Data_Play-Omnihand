import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import numpy as np
    from scipy.spatial.transform import Rotation
    from handoff_planner import box_symmetries
    from plan_contact_trajectory import collision_screen, solve_ik
except ImportError:
    np=None


@unittest.skipIf(np is None,'optional scientific planning environment')
class MeasuredHandoffTests(unittest.TestCase):
    def test_continuation_ik_keeps_nearest_feasible_branch(self):
        class PeriodicModel:
            side='right'; arm_limits=[[-3.,3.]]; arm_names=['joint']
            def fk(self,q):
                matrix=np.eye(4);matrix[0,3]=np.sin(q['joint'])
                return {'right_hand__R_palm':matrix}
        target=np.eye(4);target[0,3]=.5
        result=solve_ik(PeriodicModel(),target,np.eye(4),{},initial=[2.5],prefer_initial=True)
        self.assertLess(result['position_residual_m'],1e-6)
        self.assertAlmostEqual(result['q'][0],5*np.pi/6,places=5)

    def test_symmetries_preserve_cuboid_not_arbitrary_axes(self):
        size=np.array([.084,.06,.06]); matrices=list(box_symmetries(size))
        self.assertEqual(len(matrices),8)
        for matrix in matrices:
            np.testing.assert_allclose(matrix@matrix.T,np.eye(3))
            self.assertAlmostEqual(np.linalg.det(matrix),1)
            np.testing.assert_allclose(np.abs(matrix)@size,size)

    def test_collision_screen_uses_measured_rotation_and_world_table(self):
        class PointModel:
            joints=[]; arm_names=[]
            def fk(self,q):return {}
            def world_samples(self,fk,base):
                yield 'noncontact_link',np.array([[.040,0,1.]])
        model=PointModel();center=np.array([0,0,1.]);size=np.array([.084,.06,.06])
        common=(model,[],[],0,np.eye(4),center,set(),0.,-1.)
        unrotated=collision_screen(*common,box_size=size)
        rotated=collision_screen(*common,box_rotation=Rotation.from_euler('z',90,degrees=True).as_matrix(),box_size=size)
        self.assertFalse(unrotated['passed'])
        self.assertTrue(rotated['passed'])
        self.assertAlmostEqual(rotated['meshes'][0]['thor_collision_plane_clearance_m'],1.)


if __name__=='__main__': unittest.main()
