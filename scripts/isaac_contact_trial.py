"""Bounded synthetic single-arm contact trial; never writes simulated body poses."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import traceback

from convert_physx_commissioning import _start_kit, sha256
from isaac_finger_response import _action, _json, _write


def state_is_bounded(row):
    """Reject incomplete/nonfinite readback before checking the synthetic envelope."""
    try:
        vectors = [row[k] for k in ("q", "qd", "effort", "box_position",
                                   "box_quaternion_wxyz", "box_velocity")]
        if not all(v and all(math.isfinite(x) for x in v) for v in vectors):
            return False
        return (max(abs(v) for v in row["box_velocity"][:3]) <= 2.0
                and max(abs(v) for v in row["box_velocity"][3:]) <= 50.0
                and max(abs(v) for v in row["qd"]) <= 50.0
                and max(abs(v) for v in row["q"]) <= 10.0)
    except (KeyError, TypeError, ValueError):
        return False


def contact_lift_gate(rows, initial_z, dt=1/240):
    """Require uninterrupted, hand-supported clearance; a ballistic toss fails."""
    if not math.isfinite(dt) or not 0 < dt <= 1/30 or not math.isfinite(initial_z):
        raise ValueError("finite initial height and physics dt in (0, 1/30] required")
    longest = current = 0
    for row in rows:
        accepted = (state_is_bounded(row) and row["phase"] == "hold"
                    and row["box_position"][2] > initial_z + .025
                    and 0 <= row["support_force_N"] < .01
                    and 0 <= row["ground_force_N"] < .01
                    and math.isfinite(row["hand_contact_force_N"])
                    and row["hand_contact_force_N"] > .02
                    and sum(v*v for v in row["box_velocity"][:3]) < .05**2)
        current = current + 1 if accepted else 0
        longest = max(longest, current)
    return {"passed": longest*dt >= 1, "continuous_contact_hold_steps": longest,
            "required_hold_seconds": 1, "clearance_m": .025, "max_linear_speed_m_s": .05}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("usd", "profile", "plan", "response", "other-response", "out"):
        parser.add_argument("--" + key, type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise SystemExit("fresh output required")
    args.out = args.out.resolve()
    args.out.mkdir(parents=True)
    report = {"scope": "synthetic_single_arm_contact_trial", "passed": False,
              "production_collection_allowed": False, "direct_object_pose_writes": 0,
              "direct_joint_state_writes": 0, "phase": "preflight", "video": None,
              "shutdown": {"attempted": False, "returned": False}}
    _write(args.out / "result.json", report)
    app = None
    try:
        response = json.loads(args.response.read_text())
        other = json.loads(args.other_response.read_text())
        for gate in (response, other):
            if not gate.get("passed") or not gate.get("gates", {}).get("contact_trial_allowed"):
                raise ValueError("both real hand-response gates must pass")
        if response["inputs"]["side"] == other["inputs"]["side"]:
            raise ValueError("response gates must cover both sides")
        if response["inputs"]["usd_sha256"] != sha256(args.usd):
            raise ValueError("USD identity mismatch")
        if any(r["inputs"]["profile_sha256"] != sha256(args.profile) for r in (response, other)):
            raise ValueError("profile changed after response acceptance")
        plan = json.loads(args.plan.read_text())
        report["inputs"] = {k: {"path": str(getattr(args, k).resolve()), "sha256": sha256(getattr(args, k))}
                            for k in ("usd", "profile", "plan", "response", "other_response")}
        report["synthetic_plan"] = plan
        dt = float(plan.get("physics_dt", 1/240))
        if not math.isfinite(dt) or not 1/2000 <= dt <= 1/240:
            raise ValueError("synthetic physics_dt must be in [1/2000, 1/240]")
        for key in ("mass_kg", "friction"):
            if not math.isfinite(plan[key]) or plan[key] <= 0:
                raise ValueError(f"positive finite {key} required")
        app, nas = _start_kit(args.out, "d3d12")
        report["runtime"] = {"simulation_app_started": True, "graphics_api": "d3d12",
                             "physics_device": "cpu", "nas_profile_loaded": bool(nas)}
        import numpy as np
        import torch
        import omni.usd
        import carb
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, PhysxSchema
        from isaaclab.sim import SimulationContext, SimulationCfg
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        from PIL import Image

        context = omni.usd.get_context()
        context.open_stage(str(args.usd.resolve()))
        stage = context.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        for item in response["mimic_constraint_override"]:
            prim = stage.GetPrimAtPath(item["prim"])
            for key, value in item["changes"].items():
                prim.CreateAttribute(f"physxMimicJoint:{item['axis']}:{key}", Sdf.ValueTypeNames.Float).Set(value["after"])
        root_path = response["physics"]["root_path"]
        root = stage.GetPrimAtPath(root_path)
        PhysxSchema.PhysxArticulationAPI.Apply(root).CreateSolverPositionIterationCountAttr(32)
        PhysxSchema.PhysxArticulationAPI(root).CreateSolverVelocityIterationCountAttr(8)
        material = UsdShade.Material.Define(stage, "/Trial/ContactMaterial")
        mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        mat.CreateStaticFrictionAttr(plan["friction"])
        mat.CreateDynamicFrictionAttr(plan["friction"])
        mat.CreateRestitutionAttr(0)

        def cube(path, center, size, dynamic=False):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
            if dynamic:
                UsdPhysics.RigidBodyAPI.Apply(shape.GetPrim())
                UsdPhysics.MassAPI.Apply(shape.GetPrim()).CreateMassAttr(plan["mass_kg"])
                shape.CreateDisplayColorAttr([(0.65, 0.32, 0.08)])
            return shape

        center = plan["box_center"]
        size = plan["box_size"]
        top = center[2] - size[2] / 2
        cube("/Trial/Support", [center[0], center[1], top - 0.015], [.22, .22, .03])
        cube("/Trial/Box", center, size, True)  # Initial placement only, before simulation.
        ground = UsdGeom.Mesh.Define(stage, "/Trial/Ground")
        ground.CreatePointsAttr([(-2,-2,0),(2,-2,0),(2,2,0),(-2,2,0)])
        ground.CreateFaceVertexCountsAttr([4]); ground.CreateFaceVertexIndicesAttr([0,1,2,3])
        ground.CreateSubdivisionSchemeAttr("none")
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        UsdLux.DomeLight.Define(stage, "/Trial/Light").CreateIntensityAttr(1200)
        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        rgb_stride=max(1,round(1/(30*dt)))
        report["dt"]=dt
        sim = SimulationContext(SimulationCfg(dt=dt, device="cpu", use_fabric=False))
        # Isaac Lab disables CPU contact processing by default; core RigidPrim
        # reporters do not switch this flag back on as Lab ContactSensor does.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        report["cpu_contact_processing_enabled"] = True
        robot = SingleArticulation(root_path, name="contact_robot")
        # Ordered filter groups distinguish support from all hand/arm body contacts.
        robot_parent = root_path.rsplit("/", 1)[0]
        bodies = [str(p.GetPath()) for p in stage.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI)
                  and str(p.GetPath()).startswith(robot_parent + "/")]
        filters = ["/Trial/Support", "/Trial/Ground"] + bodies
        box = RigidPrim("/Trial/Box", name="trial_box", track_contact_forces=True,
                        contact_filter_prim_paths_expr=filters, max_contact_count=4096,
                        disable_stablization=False, reset_xform_properties=False)
        cameras = []
        for name, eye in (("main", [1.05,-.8,.85]), ("close", [center[0]+.26,center[1]-.32,center[2]+.21])):
            camera = Camera("/Trial/Camera_" + name, resolution=(640,480))
            matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0,0,1)).GetInverse()
            q = matrix.ExtractRotationQuat()
            camera.set_world_pose(np.array(eye), np.array([q.GetReal(),*q.GetImaginary()]), camera_axes="usd")
            camera.set_focal_length(18); camera.set_horizontal_aperture(24); camera.set_vertical_aperture(18)
            camera.set_clipping_range(.01,10)
            cameras.append((name,camera))
        sim.reset(); robot.initialize(); box.initialize()
        for _, camera in cameras:
            camera.initialize()
        dofs = list(robot.dof_names)
        active = response["mapping"]["active_hand"]
        arm = response["mapping"]["arm_hold_names"]
        indices = [dofs.index(n) for n in arm] + [dofs.index(x["dof_name"]) for x in active]
        controller = robot.get_articulation_controller()
        kp, kd = controller.get_gains()
        kp = torch.as_tensor(kp).clone(); kd = torch.as_tensor(kd).clone()
        kp[indices[:6]]=800; kd[indices[:6]]=50
        hand_kp=float(plan.get("hand_stiffness", 1.0)); hand_kd=float(plan.get("hand_damping", .05))
        kp[indices[6:]]=hand_kp; kd[indices[6:]]=hand_kd
        controller.set_gains(kp,kd)
        efforts = np.asarray(controller.get_max_efforts()).reshape(-1).copy()
        efforts[indices[:6]]=40; efforts[indices[6:]]=float(plan.get("hand_max_force", .3))
        controller.set_max_efforts(efforts.tolist())
        view=robot._articulation_view
        velocities=torch.as_tensor(view.get_joint_max_velocities()).clone()
        velocities[:,indices]=1
        view.set_max_joint_velocities(velocities)
        report["dof_names"]=dofs; report["contact_filter_order"]=filters
        hand_filter_indices = [i for i,path in enumerate(filters) if "_hand__" in path]
        if not hand_filter_indices:
            raise ValueError("no hand contact filters resolved")
        report["drives"]={"gains":_json(controller.get_gains()),"effort_limits":_json(controller.get_max_efforts()),
                          "velocity_limits":_json(view.get_joint_max_velocities()),"synthetic":True}
        qstart=np.asarray(robot.get_joint_positions()).reshape(-1)[indices]
        hand_open = plan.get("hand_open", [x["open_rad"] for x in active])
        hand_close = plan.get("hand_close", [x["close_rad"] for x in active])
        if len(hand_open) != len(active) or len(hand_close) != len(active):
            raise ValueError("contact hand endpoints must cover active joints only")
        props = robot.dof_properties
        for i, lo, hi in zip(indices[6:], hand_open, hand_close):
            if not all(np.isfinite(v) and props[i]["lower"] <= v <= props[i]["upper"] for v in (lo, hi)):
                raise ValueError("contact hand target outside source limits")
        for arm_target in (plan["pregrasp"], plan["grasp"], plan["lift"]):
            if len(arm_target) != 6 or not all(np.isfinite(v) and props[i]["lower"] <= v <= props[i]["upper"]
                                                for i,v in zip(indices[:6],arm_target)):
                raise ValueError("arm target outside source limits")
        records=[]; images=[]; step=0
        report.update(trace="trace.jsonl", images=images, dt=dt,
                      inertia_override_applied=False)
        rgbroot=args.out/"rgb";rgbroot.mkdir()
        phases=[("approach",plan["pregrasp"],0,4), ("approach_lower",plan["grasp"],0,2),
                ("finger_close",plan["grasp"],1,2), ("lift",plan["lift"],1,3),
                ("hold",plan["lift"],1,2), ("lower",plan["grasp"],1,3),
                ("release",plan["grasp"],0,2), ("retreat",plan["pregrasp"],0,2)]
        with (args.out/"trace.jsonl").open("w") as stream:
            for phase, arm_target, amount, seconds in phases:
                hand=[lo+amount*(hi-lo) for lo,hi in zip(hand_open,hand_close)]
                end=np.array(list(arm_target)+hand)
                count=round(seconds/dt)
                for tick in range(count):
                    u=min(1,(tick+1)/(count*.8)); blend=u*u*(3-2*u)
                    command=qstart+(end-qstart)*blend
                    robot.apply_action(_action(ArticulationAction,indices,command.tolist()))
                    sim.step(render=step%rgb_stride==0)
                    pos,quat=box.get_world_poses()
                    q=robot.get_joint_positions();qd=robot.get_joint_velocities()
                    matrix=box.get_contact_force_matrix(dt=dt)
                    raw=box.get_contact_force_data(dt=dt)
                    forces,points,normals,distances,counts,starts=raw
                    contacts=[]
                    for f in range(len(filters)):
                        for k in range(int(counts[0,f])):
                            ix=int(starts[0,f])+k
                            contacts.append({"other":filters[f],"normal_force_N":_json(forces[ix]),
                                             "point_m":_json(points[ix]),"normal":_json(normals[ix]),
                                             "separation_m":_json(distances[ix])})
                    matrix=np.asarray(matrix).reshape(-1,3)
                    row={"step":step,"time_s":(step+1)*dt,"phase":phase,"commanded":dict(zip([dofs[i] for i in indices],command.tolist())),
                         "q":_json(q),"qd":_json(qd),"effort":_json(robot.get_measured_joint_efforts()),
                         "box_position":_json(pos[0]),"box_quaternion_wxyz":_json(quat[0]),
                         "box_velocity":_json(box.get_velocities()[0]),"contacts":contacts,
                         "support_force_N":float(np.linalg.norm(matrix[0])),
                         "ground_force_N":float(np.linalg.norm(matrix[1])),
                         "hand_contact_force_N":float(np.linalg.norm(matrix[hand_filter_indices],axis=1).sum()),
                         "robot_contact_force_N":float(np.linalg.norm(matrix[2:],axis=1).sum())}
                    stream.write(json.dumps(row)+"\n");records.append(row)
                    if not state_is_bounded(row):
                        report.update(phase="numerical_instability", failed_step=step,
                                      steps=step+1,
                                      contact_observed=any(r["robot_contact_force_N"]>.02 for r in records),
                                      instability_reason="missing/nonfinite readback or state/velocity exceeded bounded commissioning envelope")
                        _write(args.out/"result.json", report)
                        return 3
                    if step%rgb_stride==0:
                        for name,cam in cameras:
                            rgb=cam.get_rgba()
                            if rgb is not None and np.asarray(rgb).shape==(480,640,4):
                                path=rgbroot/f"{name}_{step:06d}.png"
                                Image.fromarray(np.asarray(rgb)[...,:3].astype(np.uint8)).save(path)
                                images.append({"step":step,"camera":name,"file":str(path.relative_to(args.out))})
                    step+=1
                qstart=end
                report["phase"]="completed_"+phase
                _write(args.out/"result.json",report)
        gate=contact_lift_gate(records, center[2], dt=dt)
        report.update(phase="completed", steps=step, trace="trace.jsonl", images=images,
                      max_box_lift_m=max(r["box_position"][2] for r in records)-center[2],
                      contact_lift_gate=gate, passed=gate["passed"],
                      box_final_position=records[-1]["box_position"],
                      contact_observed=any(r["robot_contact_force_N"]>.02 for r in records))
        if report["passed"]:
            import cv2
            for name,_ in cameras:
                if not any(frame["camera"] == name for frame in images):
                    raise RuntimeError(f"missing RGB evidence for {name}")
                writer=cv2.VideoWriter(str(args.out/f"{name}.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),1/(dt*rgb_stride),(640,480))
                if not writer.isOpened():raise RuntimeError("video writer failed")
                for frame in images:
                    if frame["camera"]==name:writer.write(cv2.imread(str(args.out/frame["file"])))
                writer.release()
            report["video"]="main.mp4"
        _write(args.out/"result.json",report)
        return 0 if report["passed"] else 3
    except Exception:
        report.update(phase="failed", passed=False, video=None, error=traceback.format_exc())
        _write(args.out/"result.json",report)
        return 2
    finally:
        if app:
            report["shutdown"]["attempted"]=True
            _write(args.out/"result.json",report)
            app.close(wait_for_replicator=False)
            report["shutdown"]["returned"]=True
            _write(args.out/"result.json",report)


if __name__ == "__main__":
    raise SystemExit(main())
