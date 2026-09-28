"""One dynamic CardBox, two real articulations, one continuous synthetic episode.

Inputs are a private JSON recipe containing reviewed USD/URDF, response, plan
and preload paths for both sides, plus profile/manifest/table/cardbox paths.
Only initial active-hand reset writes state. Every later robot motion is an
ArticulationAction. Failed trials retain pre-action RGB/state/action records.
"""
from __future__ import annotations
import argparse
import itertools
import json
from pathlib import Path
import traceback

from convert_physx_commissioning import _start_kit, sha256
from isaac_finger_response import _action, _reset_hand_to_open, _json, _write
from isaac_contact_trial import validate_hand_plan, validate_planned_scene, validate_preload
from relay_recording import PhysicalEpisodeRecorder, REQUIRED_GATES


def footprint_inside(position, quaternion_wxyz, size, center_xy, size_xy, margin=.005):
    import numpy as np
    from scipy.spatial.transform import Rotation
    corners=np.array(list(itertools.product(*[(-v/2,v/2) for v in size])))
    q=np.asarray(quaternion_wxyz)
    world=corners@Rotation.from_quat(q[[1,2,3,0]]).as_matrix().T+position
    return bool(np.all(np.abs(world[:,:2]-center_xy)<=np.asarray(size_xy)/2-margin))


def run(args):
    import numpy as np
    recipe=json.loads(args.spec.read_text(encoding="utf-8"));profile=json.loads(Path(recipe["profile"]).read_text(encoding="utf-8"))
    manifest=json.loads(Path(recipe["manifest"]).read_text(encoding="utf-8"))
    args.out.mkdir(parents=True,exist_ok=False)
    report=dict(scope="synthetic_physx_dual_arm_relay",passed=False,phase="preflight",
                synthetic=True,gates={k:False for k in REQUIRED_GATES},shutdown={"attempted":False,"returned":False},
                episode_object_count=1,object_resets_inside_episode=0,loop_joint_state_writes=0)
    _write(args.out/"result.json",report)
    app=None;recorder=None;trace=None
    try:
        sides=("left","right");plans={};responses={};preloads={}
        for side in sides:
            item=recipe[side];plans[side]=json.loads(Path(item["plan"]).read_text(encoding="utf-8"))
            responses[side]=json.loads(Path(item["response"]).read_text(encoding="utf-8"))
            validate_planned_scene(plans[side],sha256(Path(recipe["profile"])))
            validate_hand_plan(profile,plans[side])
            r=responses[side]
            if not r.get("passed") or not r.get("gates",{}).get("contact_trial_allowed"):
                raise ValueError(f"{side} real response gate failed")
            if r["inputs"]["usd_sha256"]!=sha256(Path(item["usd"])) or r["inputs"]["profile_sha256"]!=sha256(Path(recipe["profile"])):
                raise ValueError("stale response inputs")
            if plans[side]["urdf_sha256"]!=sha256(Path(item["urdf"])):
                raise ValueError("stale plan URDF")
            preloads[side]=json.loads(Path(item["preload"]).read_text(encoding="utf-8"))
        hz=int(recipe.get("physics_hz",1000));fps=int(recipe.get("fps",25));dt=1/hz;stride=hz//fps
        if hz!=1000 or fps!=25: raise ValueError("initial relay uses explicit 1000 Hz physics / 25 Hz command and image clock")
        app,_=_start_kit(args.out,"d3d12")
        import torch,omni.usd,carb
        from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf,Sdf,PhysxSchema
        from isaaclab.sim import SimulationContext,SimulationCfg
        from isaacsim.core.prims import SingleArticulation,RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.utils.semantics import add_update_semantics
        from physx_contact_scene import build_scene,place_robot_base
        from physx_scene_audit import audit_colliders
        from plan_contact_trajectory import RobotModel,pose,solve_ik,positions_for,collision_screen
        from sim2data.relay import RelayStateMachine,RelayPhase
        from scipy.spatial.transform import Rotation
        context=omni.usd.get_context();context.open_stage(str(Path(recipe["left"]["usd"]).resolve()))
        stage=context.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
        stage.DefinePrim("/right_commissioning").GetReferences().AddReference(str(Path(recipe["right"]["usd"]).absolute()))
        stage.Load()
        for side in sides:
            for item in responses[side]["mimic_constraint_override"]:
                prim=stage.GetPrimAtPath(item["prim"])
                for key,value in item["changes"].items():
                    prim.CreateAttribute(f"physxMimicJoint:{item['axis']}:{key}",Sdf.ValueTypeNames.Float).Set(value["after"])
            root=stage.GetPrimAtPath(responses[side]["physics"]["root_path"])
            PhysxSchema.PhysxArticulationAPI.Apply(root).CreateSolverPositionIterationCountAttr(32)
            PhysxSchema.PhysxArticulationAPI(root).CreateSolverVelocityIterationCountAttr(8)
            parent=f"/{side}_commissioning"
            for p in list(Usd.PrimRange(stage.GetPrimAtPath(parent))):
                if p.IsInstance():p.SetInstanceable(False)
            for p in Usd.PrimRange(stage.GetPrimAtPath(parent)):
                if p.IsA(UsdGeom.Mesh):add_update_semantics(p,side+"_robot")
        scene=build_scene(stage,table_path=recipe["table"],cardbox_path=recipe["cardbox"],
            profile=profile,plan=plans["left"],robot_parent_path="/left_commissioning",side="left",manifest=manifest)
        place_robot_base(stage,"/right_commissioning",profile["robots"]["right"])
        UsdLux.DomeLight.Define(stage,"/World/RelayLight").CreateIntensityAttr(900)
        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen",True)
        sim=SimulationContext(SimulationCfg(dt=dt,device="cpu",use_fabric=False))
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing",False)
        rigs={};bodies={}
        for side in sides:
            rigs[side]=SingleArticulation(responses[side]["physics"]["root_path"],name=side+"_robot")
            bodies[side]=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith(f"/{side}_commissioning/")]
        table_filters=scene["thor_colliders"];bin_filters=["/World/Bin/Bottom","/World/Bin/Left","/World/Bin/Right","/World/Bin/Near","/World/Bin/Far"]
        filters=table_filters+[scene["ground_prim_path"]]+bin_filters+bodies["left"]+bodies["right"]
        box=RigidPrim(scene["box_prim_path"],name="relay_box",track_contact_forces=True,
            contact_filter_prim_paths_expr=filters,max_contact_count=8192,reset_xform_properties=False)
        collision_views={}
        for side in sides:
            moving=[p for p in bodies[side] if "base_link" not in p]
            obstacles=table_filters+[scene["ground_prim_path"]]+bin_filters+bodies["right" if side=="left" else "left"]
            collision_views[side]=RigidPrim(moving,name=side+"_illegal_contacts",track_contact_forces=True,
                contact_filter_prim_paths_expr=[list(obstacles) for _ in moving],
                max_contact_count=8192,reset_xform_properties=False)
        cameras={}
        overhead=Camera("/World/TrainingOverhead",resolution=(320,240));cameras["overhead"]=overhead
        m=Gf.Matrix4d().SetLookAt(Gf.Vec3d(.55,-.7,1.1),Gf.Vec3d(.12,.1,0),Gf.Vec3d(0,0,1)).GetInverse();q=m.ExtractRotationQuat()
        overhead.set_world_pose(np.array([.55,-.7,1.1]),np.array([q.GetReal(),*q.GetImaginary()]),camera_axes="usd")
        for side in sides:
            paths=[p.GetPath() for p in stage.Traverse() if p.GetName()==side+"_color_optical"]
            if len(paths)!=1:raise ValueError("missing unambiguous mounted optical camera frame")
            camera=Camera(str(paths[0])+"/TrainingCamera",resolution=(320,240))
            camera.set_local_pose(np.zeros(3),np.array([1.,0,0,0]),camera_axes="ros")
            cameras["wrist_"+side]=camera
        for camera in cameras.values():
            camera.set_focal_length(12);camera.set_horizontal_aperture(24);camera.set_vertical_aperture(18);camera.set_clipping_range(.01,10)
        sim.reset();box.initialize()
        for view in collision_views.values():view.initialize()
        mapping={};models={};base={};targets={};amounts={s:0. for s in sides};reset={}
        for side,robot in rigs.items():
            robot.initialize()
            if not robot.handles_initialized:raise RuntimeError("real articulation failed")
            dofs=list(robot.dof_names);active=profile["gripper_commissioning"][side]["active_joints"]
            names=[f"{side}_arm__joint{i}" for i in range(1,7)]+[a["name"] for a in active]
            indices=[dofs.index(n) for n in names];controller=robot.get_articulation_controller()
            reset[side]=_reset_hand_to_open(robot,dofs,indices[6:],plans[side]["hand_open"],
                np.asarray(robot.get_joint_positions())[indices[:6]].tolist(),names[:6])
            kp,kd=[torch.as_tensor(x).clone() for x in controller.get_gains()]
            kp[indices[:6]]=800;kd[indices[:6]]=50
            kp[indices[6:]]=4;kd[indices[6:]]=.15
            allowed=set(validate_hand_plan(profile,plans[side]))
            fixed=[i for i,n in zip(indices[6:],names[6:]) if n not in allowed]
            kp[fixed]=100;kd[fixed]=10;controller.set_gains(kp,kd)
            efforts=np.asarray(controller.get_max_efforts()).copy();efforts[indices[:6]]=40;efforts[indices[6:]]=1.5
            controller.set_max_efforts(efforts.tolist())
            preload=np.array(validate_preload(preloads[side],plan_sha=sha256(Path(recipe[side]["plan"])),
                profile_sha=sha256(Path(recipe["profile"])),urdf_sha=sha256(Path(recipe[side]["urdf"])),
                active_names=names[6:],effort_limits=efforts[indices[6:]].tolist(),allowed_effort_names=allowed))
            mapping[side]=dict(dofs=dofs,names=names,indices=indices,active=active,fixed=fixed,preload=preload)
            models[side]=RobotModel(recipe[side]["urdf"],side)
            base[side]=pose(profile["robots"][side]["base_xyz_m"],profile["robots"][side]["base_rpy_rad"])
            targets[side]=np.array(plans[side]["start_configuration"]["arm"])
        for camera in cameras.values():camera.initialize()
        for _ in range(20):
            sim.render(); app.update()
        report.update(phase="reset_settle",initialization=reset,scene=scene,real_articulations=True,
            scene_audit=audit_colliders(stage,roots=(scene["box_prim_path"],"/World/RuntimeThor","/World/Bin","/left_commissioning","/right_commissioning")))
        trace=(args.out/"trace.jsonl").open("w");physical_step=0;record_step=0;previous_box=None
        def command():
            actual={};action=[]
            for side,robot in rigs.items():
                mp=mapping[side];plan=plans[side];amount=amounts[side]
                hand=np.array(plan["hand_open"])+amount*(np.array(plan["hand_close"])-plan["hand_open"])
                values=np.r_[targets[side],hand];actual.update(dict(zip(mp["names"],values.tolist())))
                action.extend([*targets[side],amount])
                gravity=np.asarray(robot._articulation_view.get_generalized_gravity_forces()).reshape(-1)
                a=_action(ArticulationAction,mp["indices"],values.tolist())
                a.joint_efforts=torch.tensor(np.r_[np.clip(gravity[mp["indices"][:6]],-40,40),mp["preload"]*amount],dtype=torch.float32)
                robot.apply_action(a)
            return actual,action
        def read():
            nonlocal previous_box
            pos,quat=box.get_world_poses();v=box.get_velocities()[0];matrix=np.asarray(box.get_contact_force_matrix(dt=dt))
            forces=np.linalg.norm(matrix[0],axis=1)
            data=dict(physical_step=physical_step,phase=report["phase"],box_position=_json(pos[0]),box_quaternion_wxyz=_json(quat[0]),box_velocity=_json(v),
                support_force_N=float(sum(forces[:len(table_filters)])),bin_support_force_N=float(forces[len(table_filters)+1]),
                ground_force_N=float(forces[len(table_filters)]),contact_matrix_N=matrix.tolist(),contact_filter_order=filters)
            for side,robot in rigs.items():
                mp=mapping[side];q=np.asarray(robot.get_joint_positions());qd=np.asarray(robot.get_joint_velocities())
                data[side]=dict(q=q.tolist(),qd=qd.tolist(),effort=_json(robot.get_measured_joint_efforts()),
                    hand_force_N=float(sum(forces[i] for i,p in enumerate(filters) if p in bodies[side] and "_hand__" in p)),
                    illegal_contact_N=float(np.linalg.norm(collision_views[side].get_contact_force_matrix(dt=dt))))
            return data
        # Direct open reset is followed by physical, logged mimic settling. No
        # reset-time transient is eligible for a lift or dataset success gate.
        for physical_step in range(500):
            command();sim.step(render=False);trace.write(json.dumps(read())+"\n")
        for side in sides:
            if max(abs(x) for x in read()[side]["qd"])>1:raise RuntimeError("reset velocities failed to settle")
        state_names=[n for side in sides for n in mapping[side]["names"]]
        recorder=PhysicalEpisodeRecorder(args.out/"capture",state_names=state_names,physics_hz=hz,fps=fps,
            provenance={"synthetic":True,"gesture_endpoints":{s:{k:plans[s][k] for k in ("hand_open","hand_close")} for s in sides},
                        "profile_sha256":sha256(Path(recipe["profile"])),"reset_physics_steps":500})
        machine=RelayStateMachine();dwell=0.;previous_box=None
        def control_interval():
            nonlocal physical_step,record_step,previous_box
            sim.render();row=read();state=[row[s]["q"][i] for s in sides for i in mapping[s]["indices"]]
            # Capture the current image/state before the command below is sent.
            images={}
            for n,c in cameras.items():
                rgba=c.get_rgba()
                if rgba is None:
                    sim.render(); app.update(); rgba=c.get_rgba()
                array=np.asarray(rgba) if rgba is not None else None
                if array is None or array.ndim!=3 or array.shape[-1] not in (3,4):
                    raise RuntimeError(f"training camera {n} did not produce RGB")
                images[n]=array[...,:3].astype(np.uint8)
            actual,action=command()
            recorder.record(physics_step=record_step,state=state,action=action,active_commands=actual,cameras=images,
                camera_physics_steps={c:record_step for c in cameras},auxiliary=row)
            for _ in range(stride):
                sim.step(render=False);physical_step+=1;row=read();trace.write(json.dumps(row)+"\n")
                if not np.isfinite(row["box_velocity"]).all() or np.linalg.norm(row["box_velocity"][:3])>2:
                    raise RuntimeError("unbounded object state")
                p=np.array(row["box_position"])
                if previous_box is not None and np.linalg.norm(p-previous_box)>2*dt+.001:raise RuntimeError("object pose discontinuity")
                previous_box=p
                for s in sides:
                    mp=mapping[s];rr=row[s]
                    if max(abs(v) for v in rr["qd"])>50:raise RuntimeError("joint velocity instability")
                    if rr["illegal_contact_N"]>.1:raise RuntimeError(f"{s} illegal scene/robot contact")
                    for i,n in zip(mp["indices"][6:],mp["names"][6:]):
                        if i in mp["fixed"] and abs(rr["q"][i]-actual[n])>.03:raise RuntimeError(f"{s} fixed hand channel drift")
            record_step+=stride;trace.flush();return row
        def phase(name,side,goal,amount,seconds,condition=None):
            if machine.phase.value!=name: raise RuntimeError("relay stage order mismatch")
            report["phase"]=name;_write(args.out/"result.json",report)
            initial=targets[side].copy();initial_amount=amounts[side];steps=round(seconds*fps);stable=0
            for i in range(steps+int(3*fps)):
                t=min(1,(i+1)/steps);blend=t*t*(3-2*t)
                targets[side]=initial+(np.asarray(goal)-initial)*blend;amounts[side]=initial_amount+(amount-initial_amount)*blend
                row=control_interval()
                tracking=max(abs(row[side]["q"][j]-v) for j,v in zip(mapping[side]["indices"][:6],goal))<.03
                ready=t>=1 and tracking and (condition(row) if condition else True)
                stable=stable+1 if ready else 0
                if stable>=fps:
                    machine.tick(1/fps,True);return row
                if machine.tick(1/fps,False)==RelayPhase.FAILED:raise RuntimeError(name+" stage timeout")
            raise RuntimeError(name+" timeout or physical gate failed")
        def palm(side,q):
            model=models[side];mp=mapping[side]
            values=positions_for(model.joints,mp["active"],model.arm_names,q,0)
            return base[side]@model.fk(values)[f"{side}_hand__{'l' if side=='left' else 'R'}_palm"]
        def translated_goal(side,offset):
            matrix=palm(side,plans[side]["grasp"]);matrix[:3,3]+=offset
            model=models[side];hand=dict(zip(mapping[side]["names"][6:],plans[side]["hand_close"]))
            ik=solve_ik(model,matrix,base[side],hand,initial=targets[side])
            if ik["position_residual_m"]>.003 or ik["orientation_residual_rad"]>.03:raise RuntimeError(side+" transfer IK failed")
            return ik["q"]
        weight=plans["left"]["mass_kg"]*9.81
        def stable(row):return np.linalg.norm(row["box_velocity"][:3])<.02 and np.linalg.norm(row["box_velocity"][3:])<.1
        def hand_free(row):return all(row[s]["hand_force_N"]<.01 for s in sides)
        def lifted(side,z):return lambda r:r["box_position"][2]>z+.025 and r[side]["hand_force_N"]>.02 and r["support_force_N"]<.01 and r["ground_force_N"]<.01 and np.linalg.norm(r["box_velocity"][:3])<.05
        source=np.array(scene["box_center_world_m"]);relay=np.array(profile["relay_region"]["center_xyz_m"]);relay[2]=source[2]-.001
        machine.tick(.001,True)
        phase("left_approach","left",plans["left"]["pregrasp"],0,4)
        phase("left_approach_lower","left",plans["left"]["grasp"],0,2)
        phase("left_close","left",plans["left"]["grasp"],1,2)
        phase("left_lift","left",plans["left"]["lift"],1,3,lifted("left",source[2]));report["gates"]["left_contact_lift"]=True
        phase("left_transfer","left",translated_goal("left",relay-source+np.array([0,0,.12])),1,5)
        phase("left_lower","left",translated_goal("left",relay-source),1,3)
        placed=targets["left"].copy()
        support=lambda r:stable(r) and hand_free(r) and .8*weight<r["support_force_N"]<1.2*weight and footprint_inside(r["box_position"],r["box_quaternion_wxyz"],plans["left"]["box_size"],relay[:2],profile["relay_region"]["size_xyz_m"][:2])
        phase("left_release","left",placed,0,2,support);report["gates"]["relay_supported_stable"]=True
        phase("left_retreat","left",plans["left"]["start_configuration"]["arm"],0,4,support)
        report["gates"]["left_released_retreated"]=True
        phase("relay_stable","left",targets["left"].copy(),0,1,support)
        box_now=np.array(read()["box_position"])
        if np.linalg.norm(box_now[:2]-np.array(plans["right"]["box_center"])[:2])>.015:raise RuntimeError("handoff pose outside screened right approach; replanning required")
        phase("right_approach","right",plans["right"]["pregrasp"],0,4)
        phase("right_approach_lower","right",plans["right"]["grasp"],0,2)
        phase("right_close","right",plans["right"]["grasp"],1,2)
        phase("right_lift","right",plans["right"]["lift"],1,3,lifted("right",box_now[2]));report["gates"]["right_contact_lift"]=True
        cfg=profile["bin"];destination=np.array(cfg["bottom_center_xyz_m"]);destination[2]=cfg["top_z_m"]-cfg["inner_size_xyz_m"][2]+plans["right"]["box_size"][2]/2
        offset=destination-np.array(plans["right"]["box_center"])
        over=offset.copy();over[2]=cfg["top_z_m"]+.12-np.array(plans["right"]["box_center"])[2]
        phase("right_transfer","right",translated_goal("right",over),1,5)
        phase("right_lower","right",translated_goal("right",offset),1,4)
        in_bin=lambda r:stable(r) and hand_free(r) and .8*weight<r["bin_support_force_N"]<1.2*weight and footprint_inside(r["box_position"],r["box_quaternion_wxyz"],plans["right"]["box_size"],destination[:2],cfg["inner_size_xyz_m"][:2])
        phase("right_release","right",targets["right"].copy(),0,2,in_bin)
        report["gates"]["bin_footprint"]=True
        phase("right_retreat","right",translated_goal("right",over),0,3,in_bin)
        report["gates"].update(final_released_stable=True,no_illegal_collision=True,no_object_teleport=True)
        report.update(passed=True,phase="complete",physics_steps=physical_step)
    except Exception:
        report.update(passed=False,error=traceback.format_exc())
    finally:
        if trace:trace.close()
        if recorder:
            report["capture"]=recorder.finish(gates=report["gates"],failure_reason=report.get("error"))
        _write(args.out/"result.json",report)
        if app:
            report["shutdown"]["attempted"]=True;_write(args.out/"result.json",report)
            app.close(wait_for_replicator=False)
            report["shutdown"]["returned"]=True;_write(args.out/"result.json",report)
    return 0 if report["passed"] else 3


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--spec",type=Path,required=True);parser.add_argument("--out",type=Path,required=True)
    raise SystemExit(run(parser.parse_args()))
