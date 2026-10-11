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


def plan_released_retreat(model,base,active,plan,arm_start,box_position,box_quaternion,table,bin_config,measured_hand):
    """Leave a bin vertically before returning, with a stationary released box.

    Reuses the task's FK/IK and collision screen. Both measured and commanded
    open hand shapes are checked; the executor still monitors physical contact.
    """
    if len(measured_hand)!=len(active) or not np.isfinite(measured_hand).all():
        raise ValueError('invalid measured active hand state')
    shapes=[[dict(a,open_rad=v,close_rad=v) for a,v in zip(active,values)]
            for values in (measured_hand,plan['hand_open'])]
    q=np.asarray(arm_start,dtype=float);center=np.asarray(box_position)
    quat=np.asarray(box_quaternion);rotation=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
    hand=positions_for(model.joints,shapes[0],model.arm_names,q,0)
    palm=base@model.fk(hand)[f"{model.side}_hand__{'l' if model.side=='left' else 'R'}_palm"]
    def screen(q):
        failures=[]
        for shape in shapes:
            gate=collision_screen(model,q,shape,0,base,center,set(),table['collision_top_z_m'],table['ground_z_m'],
                                  require_contact=False,box_rotation=rotation,box_size=np.asarray(plan['box_size']))
            failures.extend(gate['failures'])
            fk=model.fk(positions_for(model.joints,shape,model.arm_names,q,0))
            for link,points in model.world_samples(fk,base):
                for wall,pos,size in bin_obstacles(bin_config):
                    distance=float(np.min(box_sdf(points,np.asarray(pos),np.asarray(size))))
                    if distance<.002:failures.append(f'bin {wall} clearance: {link} {distance:.6f}')
        return dict(passed=not failures,failures=failures)
    failures=[];waypoints=[q.tolist()];initial=screen(q)
    if not initial['passed']:failures.append(dict(stage='start',details=initial['failures']))
    if not failures:
        for fraction in np.linspace(0,1,51)[1:]:
            target=palm.copy();target[:3,3]+=[0,0,.25*fraction]
            ik=solve_ik(model,target,base,hand,initial=q,screen=screen,prefer_initial=True)
            candidate=np.asarray(ik['q'])
            if ik['position_residual_m']>.002 or ik['orientation_residual_rad']>.02 or np.max(np.abs(candidate-q))>.25:
                failures.append(dict(stage='vertical_exit',fraction=float(fraction),reason='IK residual/branch discontinuity'));break
            for t in np.linspace(0,1,5):
                gate=screen(q+t*(candidate-q))
                if not gate['passed']:
                    failures.append(dict(stage='vertical_exit',fraction=float(fraction),details=gate['failures']));break
            if failures:break
            q=candidate;waypoints.append(q.tolist())
    if not failures:
        start=q.copy();goal=np.asarray(plan['pregrasp'])
        for fraction in np.linspace(0,1,101)[1:]:
            candidate=start+fraction*(goal-start);gate=screen(candidate)
            if not gate['passed']:
                failures.append(dict(stage='return',fraction=float(fraction),details=gate['failures']));break
            waypoints.append(candidate.tolist())
    return dict(passed=not failures,failures=failures,waypoints=waypoints,
                stop_waypoint_indices=[50] if not failures else [],
                method='open hand: 0.25 m vertical exit then screened return; stationary box OBB',
                box_position=center.tolist(),box_quaternion_wxyz=quat.tolist(),
                hand_geometry_sources=['measured active q with URDF mimic','commanded open with URDF mimic'],
                hand_geometry_q=list(map(float,measured_hand)),object_pose_writes=0)
