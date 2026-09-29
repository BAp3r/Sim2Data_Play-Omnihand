"""Screened Cartesian translation for an already physically held box."""
import numpy as np
from scipy.spatial.transform import Rotation
from plan_contact_trajectory import positions_for,solve_ik,collision_screen


def plan_transport(model,base,active,plan,arm_start,box_position,box_quaternion,delta,table):
    active=[dict(a,open_rad=o,close_rad=c) for a,o,c in zip(active,plan['hand_open'],plan['hand_close'])]
    q=np.asarray(arm_start); hand=positions_for(model.joints,active,model.arm_names,q,1)
    prefix='l' if model.side=='left' else 'R'
    palm=base@model.fk(hand)[f'{model.side}_hand__{prefix}_palm']
    tips={f'{model.side}_hand__{prefix}_{f}_dip_link' for f in
          (('thumb','index','middle') if model.side=='left' else ('thumb','index'))}
    quat=np.asarray(box_quaternion);rotation=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
    delta=np.asarray(delta);center=np.asarray(box_position);waypoints=[q.tolist()];failures=[]
    def screen(q,center):
        return collision_screen(model,q,active,1,base,center,tips,table['collision_top_z_m'],table['ground_z_m'],
                                require_contact=False,box_rotation=rotation,box_size=np.asarray(plan['box_size']))
    for u in np.linspace(0,1,25)[1:]:
        target=palm.copy();target[:3,3]+=u*delta
        ik=solve_ik(model,target,base,hand,initial=q,screen=lambda v:screen(v,center+u*delta),prefer_initial=True)
        candidate=ik['q']
        if ik['position_residual_m']>.002 or ik['orientation_residual_rad']>.02:
            failures.append({'fraction':float(u),'reason':'IK residual'});break
        if np.max(np.abs(candidate-q))>.25:
            failures.append({'fraction':float(u),'reason':'IK branch discontinuity'});break
        for t in np.linspace(0,1,5):
            f=u-1/24+t/24
            gate=screen(q+(candidate-q)*t,center+f*delta)
            if not gate['passed']:
                failures.append({'fraction':float(f),'reason':'collision','details':gate['failures']});break
        if failures:break
        q=candidate;waypoints.append(q.tolist())
    return dict(passed=not failures,failures=failures,waypoints=waypoints,
        box_position=center.tolist(),box_quaternion_wxyz=quat.tolist(),delta_m=delta.tolist(),
        method='constant palm orientation Cartesian translation, 25 IK knots and interpolated OBB checks',
        object_pose_writes=0)
