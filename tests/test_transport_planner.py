import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

if any(importlib.util.find_spec(n) is None for n in ('numpy', 'scipy', 'trimesh')):
    raise unittest.SkipTest('transport tests require existing simulation interpreter')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
import transport_planner as transport


class TransportPlannerTests(unittest.TestCase):
    def test_released_retreat_keeps_box_stationary_and_screens_both_hand_shapes(self):
        class Model:
            side='right'
            joints=[]
            arm_names=[]
            def fk(self,q):return {'right_hand__R_palm':np.eye(4)}
            def world_samples(self,fk,base):return []
        plan={'hand_open':[0.], 'hand_close':[.5], 'box_size':[.084,.06,.06], 'pregrasp':[.2]*6}
        before=copy.deepcopy(plan);centers=[];shapes=[]
        def screen(model,q,active,amount,base,center,tips,*args,**kwargs):
            centers.append(np.array(center));shapes.append(active[0]['open_rad'])
            self.assertEqual(amount,0);self.assertEqual(tips,set())
            return {'passed':True,'failures':[]}
        def ik(*args,**kwargs):
            return {'q':np.asarray(kwargs['initial'])+.001,'position_residual_m':0.,'orientation_residual_rad':0.}
        with patch.object(transport,'positions_for',return_value={}), \
             patch.object(transport,'solve_ik',side_effect=ik), \
             patch.object(transport,'collision_screen',side_effect=screen):
            result=transport.plan_released_retreat(Model(),np.eye(4),[{'name':'active'}],plan,
                np.zeros(6),[.64,.05,-.17],[1,0,0,0],
                {'collision_top_z_m':-.0155,'ground_z_m':-.8},
                dict(inner_size_xyz_m=[.28,.32,.2],wall_thickness_m=.012,
                     bottom_center_xyz_m=[.64,.05,-.2],top_z_m=0.),[.01])
        self.assertTrue(result['passed']);self.assertEqual(len(result['waypoints']),151)
        self.assertEqual(result['stop_waypoint_indices'],[50])
        self.assertEqual(result['waypoints'][-1],plan['pregrasp'])
        self.assertTrue(all(np.array_equal(c,[.64,.05,-.17]) for c in centers))
        self.assertEqual(set(shapes),{0.,.01});self.assertEqual(plan,before)
        self.assertEqual(result['object_pose_writes'],0)

    def test_released_retreat_rejects_colliding_start_before_solving(self):
        class Model:
            side='right'
            joints=[]
            arm_names=[]
            def fk(self,q):return {'right_hand__R_palm':np.eye(4)}
            def world_samples(self,fk,base):return []
        with patch.object(transport,'positions_for',return_value={}), \
             patch.object(transport,'solve_ik') as ik, \
             patch.object(transport,'collision_screen',return_value={'passed':False,'failures':['start overlap']}):
            result=transport.plan_released_retreat(Model(),np.eye(4),[{'name':'active'}],
                {'hand_open':[0.],'box_size':[.084,.06,.06],'pregrasp':[0.]*6},
                np.zeros(6),[.64,.05,-.17],[1,0,0,0],
                {'collision_top_z_m':-.0155,'ground_z_m':-.8},
                dict(inner_size_xyz_m=[.28,.32,.2],wall_thickness_m=.012,
                     bottom_center_xyz_m=[.64,.05,-.2],top_z_m=0.),[0.])
        self.assertFalse(result['passed']);self.assertEqual(result['failures'][0]['stage'],'start')
        ik.assert_not_called()

    def test_measured_hand_keeps_original_command_and_rejects_start_collision(self):
        class Model:
            side='right'
            joints=[]
            arm_names=[]
            def fk(self, q):
                return {'right_hand__R_palm':np.eye(4)}
        plan={'hand_open':[0.], 'hand_close':[.5], 'box_size':[.084,.06,.06]}
        before=copy.deepcopy(plan)
        states=[]
        def positions(joints,active,names,q,amount):
            states.append(active[0]['close_rad'])
            return {}
        with patch.object(transport,'positions_for',side_effect=positions), \
             patch.object(transport,'solve_ik',return_value={'q':np.zeros(6),'position_residual_m':0.,'orientation_residual_rad':0.}), \
             patch.object(transport,'collision_screen',return_value={'passed':False,'failures':['proximal clearance']}) as screen:
            result=transport.plan_transport(Model(),np.eye(4),[{'name':'active'}],plan,
                np.zeros(6),[0,0,0],[1,0,0,0],[.1,0,0],
                {'collision_top_z_m':-.0155,'ground_z_m':-.8},measured_hand=[.4])
        self.assertFalse(result['passed'])
        self.assertAlmostEqual(result['failures'][0]['fraction'],0.)
        self.assertEqual(states,[.4])
        self.assertEqual(plan,before)
        self.assertEqual(result['hand_geometry_q'],[.4])
        screen.assert_called_once()

    def test_bin_bottom_and_walls_leave_declared_inner_volume(self):
        cfg=dict(inner_size_xyz_m=[.28,.32,.2],wall_thickness_m=.012,
                 bottom_center_xyz_m=[.64,.05,-.2],top_z_m=0.)
        solids={name:(np.array(p),np.array(size)) for name,p,size in transport.bin_obstacles(cfg)}
        bottom,size=solids['Bottom']
        self.assertAlmostEqual(bottom[2]+size[2]/2,-.2)
        for name,axis,sign in [('Left',0,-1),('Right',0,1),('Near',1,-1),('Far',1,1)]:
            p,size=solids[name]
            inner_face=p[axis]-sign*size[axis]/2
            self.assertAlmostEqual(inner_face,cfg['bottom_center_xyz_m'][axis]+sign*cfg['inner_size_xyz_m'][axis]/2)


if __name__=='__main__':
    unittest.main()
