"""Screened Cartesian translation for an already physically held box."""
import numpy as np
from scipy.spatial.transform import Rotation
from plan_contact_trajectory import positions_for,solve_ik,collision_screen,box_sdf


def bin_obstacles(cfg):
    sx,sy,sz=cfg['inner_size_xyz_m'];t=cfg['wall_thickness_m'];x,y,_=cfg['bottom_center_xyz_m'];z=cfg['top_z_m']
    return [('Bottom',[x,y,z-sz-t/2],[sx+2*t,sy+2*t,t]),
            ('Left',[x-(sx+t)/2,y,z-sz/2],[t,sy+2*t,sz]),
            ('Right',[x+(sx+t)/2,y,z-sz/2],[t,sy+2*t,sz]),
            ('Near',[x,y-(sy+t)/2,z-sz/2],[sx,t,sz]),
            ('Far',[x,y+(sy+t)/2,z-sz/2],[sx,t,sz])]


def plan_transport(model,base,active,plan,arm_start,box_position,box_quaternion,delta,table,bin_config=None,measured_hand=None):
    # A loaded hand does not exactly reach its position targets. Use the same
    # observation for hand geometry and held-box OBB; never alter commands.
    hand_shape=plan['hand_close'] if measured_hand is None else measured_hand
    if len(hand_shape)!=len(active) or not np.isfinite(hand_shape).all():
        raise ValueError('invalid measured active hand state')
    active=[dict(a,open_rad=o,close_rad=c) for a,o,c in zip(active,plan['hand_open'],hand_shape)]
    q=np.asarray(arm_start); hand=positions_for(model.joints,active,model.arm_names,q,1)
    prefix='l' if model.side=='left' else 'R'
    palm=base@model.fk(hand)[f'{model.side}_hand__{prefix}_palm']
    tips={f'{model.side}_hand__{prefix}_{f}_dip_link' for f in
          (('thumb','index','middle') if model.side=='left' else ('thumb','index'))}
    quat=np.asarray(box_quaternion);rotation=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
    delta=np.asarray(delta);center=np.asarray(box_position);waypoints=[q.tolist()];failures=[]
    def screen(q,center):
        result=collision_screen(model,q,active,1,base,center,tips,table['collision_top_z_m'],table['ground_z_m'],
                                require_contact=False,box_rotation=rotation,box_size=np.asarray(plan['box_size']))
        if bin_config:
            fk=model.fk(positions_for(model.joints,active,model.arm_names,q,1))
            for link,points in model.world_samples(fk,base):
                for wall,pos,size in bin_obstacles(bin_config):
                    distance=float(np.min(box_sdf(points,np.asarray(pos),np.asarray(size))))
                    if distance<.002:result['failures'].append(f'bin {wall} clearance: {link} {distance:.6f}')
            result['passed']=not result['failures']
        return result
    knots=25 if model.side=='left' else 50
    for u in np.linspace(0,1,knots)[1:]:
        target=palm.copy();target[:3,3]+=u*delta
        ik=solve_ik(model,target,base,hand,initial=q,screen=lambda v:screen(v,center+u*delta),prefer_initial=True)
        candidate=ik['q']
        if ik['position_residual_m']>.002 or ik['orientation_residual_rad']>.02:
            failures.append({'fraction':float(u),'reason':'IK residual'});break
        if np.max(np.abs(candidate-q))>.25:
            failures.append({'fraction':float(u),'reason':'IK branch discontinuity'});break
        for t in np.linspace(0,1,5):
            f=u-1/(knots-1)+t/(knots-1)
            gate=screen(q+(candidate-q)*t,center+f*delta)
            if not gate['passed']:
                failures.append({'fraction':float(f),'reason':'collision','details':gate['failures']});break
        if failures:break
        q=candidate;waypoints.append(q.tolist())
    return dict(passed=not failures,failures=failures,waypoints=waypoints,
        box_position=center.tolist(),box_quaternion_wxyz=quat.tolist(),delta_m=delta.tolist(),
        method=f'constant palm orientation Cartesian translation, {knots} IK knots and interpolated OBB checks',
        hand_geometry_source='commanded' if measured_hand is None else 'measured active q with URDF mimic',
        hand_geometry_q=list(map(float,hand_shape)),
        object_pose_writes=0)
