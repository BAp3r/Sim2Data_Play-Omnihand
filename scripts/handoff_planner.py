"""Rigid grasp-template reuse at a measured OBB with unchanged collision limits."""
import copy
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
from plan_contact_trajectory import positions_for, solve_ik, collision_screen


def box_symmetries(size):
    """Proper signed permutations which preserve this cuboid's dimensions."""
    size = np.asarray(size)
    for perm in itertools.permutations(range(3)):
        if not np.allclose(size[list(perm)], size, atol=1e-7):
            continue
        for signs in itertools.product((-1, 1), repeat=3):
            matrix = np.eye(3)[:, perm] @ np.diag(signs)
            if np.linalg.det(matrix) > .99:
                yield matrix


def replan_handoff(model, base, active, template, position, quaternion, start, table):
    """Return a fully screened receiver plan, or diagnostics with passed=False.

    No state is written. All active endpoints come from the accepted template;
    passive/mimic joints are expanded by source URDF FK only.
    """
    active = [dict(a, open_rad=o, close_rad=c) for a,o,c in
              zip(active, template['hand_open'], template['hand_close'])]
    pos = np.asarray(position); quat = np.asarray(quaternion)
    rotation = Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
    size = np.asarray(template['box_size']); source = np.asarray(template['box_center'])
    name = f"{model.side}_hand__{'l' if model.side=='left' else 'R'}_palm"
    tips = {f"{model.side}_hand__{'l' if model.side=='left' else 'R'}_{f}_dip_link"
            for f in (('thumb','index','middle') if model.side=='left' else ('thumb','index'))}
    def palm(q):
        return base @ model.fk(positions_for(model.joints,active,model.arm_names,q,1))[name]
    original = palm(template['grasp']); attempts=[]
    def screen(q, amount, center, allowed, require=True):
        return collision_screen(model,q,active,amount,base,center,allowed,
            table['collision_top_z_m'],table['ground_z_m'],require_contact=require,
            box_rotation=rotation,box_size=size)
    symmetries = sorted(box_symmetries(size), key=lambda s:np.linalg.norm(Rotation.from_matrix(rotation@s).as_rotvec()))
    for sym in symmetries:
        delta = rotation @ sym
        grasp = original.copy()
        grasp[:3,:3] = delta @ original[:3,:3]
        grasp[:3,3] = pos + delta @ (original[:3,3]-source)
        # Keep the receiver outside the 2 mm non-contact envelope before
        # descent; the source planner's 50 mm pregrasp can be marginal after
        # the measured handoff yaw/translation.
        pre = grasp.copy(); pre[:3,3] += [0,0,.10]
        lift = grasp.copy(); lift[:3,3] += [0,0,.08]
        qs={}; residuals={}; seed=start; failures=[]
        for key, target, amount, center in [('pregrasp',pre,0,pos),('grasp',grasp,1,pos),('lift',lift,1,pos+[0,0,.08])]:
            hand=positions_for(model.joints,active,model.arm_names,seed,amount)
            allowed=tips if amount else set()
            ik=solve_ik(model,target,base,hand,initial=seed,prefer_initial=True,
                screen=lambda q:screen(q,amount,center,allowed))
            seed=ik['q'];qs[key]=seed
            residuals[key]={k:float(ik[k]) for k in ('position_residual_m','orientation_residual_rad')}
            gate=screen(seed,amount,center,allowed)
            failures.extend(gate['failures'])
            if ik['position_residual_m']>.002 or ik['orientation_residual_rad']>.02:
                failures.append(key+' IK residual')
            if failures:break
        lift_path=None
        if not failures:
            q=np.asarray(qs['grasp']); lift_path=[q.tolist()]
            for fraction in np.linspace(0,1,25)[1:]:
                target=grasp.copy();target[:3,3]+=[0,0,.08*fraction]
                hand=positions_for(model.joints,active,model.arm_names,q,1)
                ik=solve_ik(model,target,base,hand,initial=q,prefer_initial=True)
                candidate=np.asarray(ik['q'])
                if ik['position_residual_m']>.002 or ik['orientation_residual_rad']>.02 or np.max(np.abs(candidate-q))>.25:
                    failures.append('lift Cartesian IK residual/branch discontinuity');break
                for u in np.linspace(0,1,5):
                    f=fraction-1/24+u/24
                    gate=screen(q+u*(candidate-q),1,pos+[0,0,.08*f],tips,False)
                    if not gate['passed']:
                        failures.extend([f'lift fraction={f:.4f}: {reason}' for reason in gate['failures']]);break
                if failures:break
                q=candidate;lift_path.append(q.tolist())
            if not failures:qs['lift']=q
        if not failures:
            # Open descent, closure and lift have the same 25-sample screen
            # and 1 mm contact / 2 mm non-contact bounds as the source planner.
            segments=[(start,qs['pregrasp'],0,0,pos,pos),
                      (qs['pregrasp'],qs['grasp'],0,0,pos,pos),
                      (qs['grasp'],qs['grasp'],0,1,pos,pos)]
            for qa,qb,a,b,ca,cb in segments:
                for u in np.linspace(0,1,25):
                    amount=a+(b-a)*u
                    gate=screen(np.asarray(qa)+(np.asarray(qb)-qa)*u,amount,
                        np.asarray(ca)+(np.asarray(cb)-ca)*u,tips if amount>0 else set(),False)
                    if not gate['passed']:
                        failures.extend(gate['failures']);break
                if failures:break
        attempts.append(dict(symmetry=sym.tolist(),residuals=residuals,failures=failures))
        if not failures:
            plan=copy.deepcopy(template)
            plan.update({key:value.tolist() for key,value in qs.items()})
            plan['lift_waypoints']=lift_path
            plan['box_center']=pos.tolist()
            plan['box_quaternion_wxyz']=quat.tolist()
            return dict(passed=True,plan=plan,attempts=attempts,
                measured_box_position=pos.tolist(),measured_box_quaternion_wxyz=quat.tolist(),
                original_grasp_endpoints_preserved=True,object_pose_writes=0)
    return dict(passed=False,attempts=attempts,measured_box_position=pos.tolist(),
                measured_box_quaternion_wxyz=quat.tolist(),object_pose_writes=0)
