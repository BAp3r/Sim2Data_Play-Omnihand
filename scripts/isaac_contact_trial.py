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


def validate_planned_scene(plan, profile_sha, scene_only=False):
    """Keep failed or stale global-state plans out of the physics action loop."""
    if plan.get("scene_kind") != "thor_cardbox":
        raise ValueError("contact commissioning requires the Thor/cardbox scene")
    if plan.get("coordinate_frame") != "world" or plan.get("profile_sha256") != profile_sha:
        raise ValueError("Thor plan must bind current profile and world coordinates")
    if not scene_only and plan.get("execution_allowed") is not True:
        raise ValueError("global-state geometry/IK gate failed; action execution blocked")


def camera_content(camera):
    """Record visible labelled pixel counts, not merely non-black backgrounds."""
    import numpy as np
    frame = camera.get_current_frame()
    # Read the same renderer frame as get_rgba(), including paused scene review.
    annotator = getattr(camera, "_custom_annotators", {}).get("semantic_segmentation")
    annotation = annotator.get_data() if annotator is not None else frame.get("semantic_segmentation")
    counts = {"robot": 0, "cardbox": 0}
    if isinstance(annotation, dict) and annotation.get("data") is not None:
        labels = annotation.get("info", {}).get("idToLabels", annotation.get("idToLabels", {}))
        if isinstance(labels, str):
            labels = json.loads(labels)
        pixels = np.asarray(annotation["data"])
        for index, fields in labels.items():
            kind = fields.get("class") if isinstance(fields, dict) else fields
            if kind in counts:
                counts[kind] += int(np.count_nonzero(pixels == int(index)))
    return {"class_pixels": counts, "frame_keys": list(frame),
            "semantic_info": _json(annotation.get("info", {})) if isinstance(annotation, dict) else None,
            "rendering_time": _json(frame.get("rendering_time")),
            "robot_and_box_visible": counts["robot"] > 20 and counts["cardbox"] > 20}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("usd", "profile", "plan", "response", "other-response", "out"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--table", type=Path)
    parser.add_argument("--cardbox", type=Path)
    parser.add_argument("--manifest", type=Path, default=Path("configs/asset_manifest.json"))
    parser.add_argument("--scene-only", action="store_true", help="Scene visibility review with render warmup; no action trajectory or grasp claim")
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
        response = json.loads(args.response.read_text(encoding="utf-8"))
        other = json.loads(args.other_response.read_text(encoding="utf-8"))
        for gate in (response, other):
            if not gate.get("passed") or not gate.get("gates", {}).get("contact_trial_allowed"):
                raise ValueError("both real hand-response gates must pass")
        if response["inputs"]["side"] == other["inputs"]["side"]:
            raise ValueError("response gates must cover both sides")
        if response["inputs"]["usd_sha256"] != sha256(args.usd):
            raise ValueError("USD identity mismatch")
        if any(r["inputs"]["profile_sha256"] != sha256(args.profile) for r in (response, other)):
            raise ValueError("profile changed after response acceptance")
        plan = json.loads(args.plan.read_text(encoding="utf-8"))
        validate_planned_scene(plan, sha256(args.profile), args.scene_only)
        if not args.scene_only:
            if (plan.get("side") != response["inputs"]["side"]
                    or plan.get("urdf_sha256") != response["inputs"]["urdf_sha256"]
                    or plan.get("manifest_sha256") != sha256(args.manifest)):
                raise ValueError("plan side/source URDF/manifest identity mismatch")
        if plan.get("scene_kind") == "thor_cardbox" and not (args.table and args.cardbox):
            raise ValueError("Thor contact requires explicit table and cardbox source files")
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
        from isaacsim.core.utils.semantics import add_update_semantics
        from PIL import Image

        context = omni.usd.get_context()
        context.open_stage(str(args.usd.resolve()))
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("fresh articulation stage did not open")
        for _ in range(10):
            app.update()
        stage.SetEditTarget(stage.GetSessionLayer())
        for item in response["mimic_constraint_override"]:
            prim = stage.GetPrimAtPath(item["prim"])
            for key, value in item["changes"].items():
                prim.CreateAttribute(f"physxMimicJoint:{item['axis']}:{key}", Sdf.ValueTypeNames.Float).Set(value["after"])
        root_path = response["physics"]["root_path"]
        root = stage.GetPrimAtPath(root_path)
        robot_parent = root_path.rsplit("/", 1)[0]
        PhysxSchema.PhysxArticulationAPI.Apply(root).CreateSolverPositionIterationCountAttr(32)
        PhysxSchema.PhysxArticulationAPI(root).CreateSolverVelocityIterationCountAttr(8)
        from physx_contact_scene import build_scene
        scene = build_scene(stage, table_path=args.table, cardbox_path=args.cardbox,
                            profile=json.loads(args.profile.read_text(encoding="utf-8")), plan=plan,
                            robot_parent_path=robot_parent, side=response["inputs"]["side"],
                            manifest=json.loads(args.manifest.read_text(encoding="utf-8")))
        report["scene"] = scene
        box_path = scene["box_prim_path"]
        support_path = scene["support_filter_path"]
        ground_path = scene["ground_prim_path"]
        center = scene["box_center_world_m"]
        UsdLux.DomeLight.Define(stage, "/Trial/Light").CreateIntensityAttr(1200)
        add_update_semantics(stage.GetPrimAtPath(robot_parent), "robot")
        add_update_semantics(stage.GetPrimAtPath(box_path), "cardbox")
        # Label each renderable descendant, including native USD instances.
        from pxr import Usd
        for parent_path, label in ((robot_parent, "robot"), (box_path, "cardbox")):
            for prim in list(Usd.PrimRange(stage.GetPrimAtPath(parent_path))):
                if prim.IsInstance():
                    prim.SetInstanceable(False)
            for prim in Usd.PrimRange(stage.GetPrimAtPath(parent_path)):
                if prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputePurpose() != "guide":
                    add_update_semantics(prim, label)
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
        bodies = [str(p.GetPath()) for p in stage.TraverseAll() if p.HasAPI(UsdPhysics.RigidBodyAPI)
                  and str(p.GetPath()).startswith(robot_parent + "/")]
        filters = [support_path, ground_path] + bodies
        box = RigidPrim(box_path, name="trial_box", track_contact_forces=True,
                        contact_filter_prim_paths_expr=filters, max_contact_count=4096,
                        disable_stablization=False, reset_xform_properties=False)
        cameras = []
        camera_eyes = (("main", [.65,-1.05,.9]), ("close", [center[0]+.26,center[1]-.32,center[2]+.21]))
        for name, eye in camera_eyes:
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
            camera.add_semantic_segmentation_to_frame()
        report["base_pose_after_initialize"] = _json(robot.get_world_pose())
        from pxr import Usd
        robot_prims = [p for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
                       if str(p.GetPath()).startswith(robot_parent + "/") and p.IsA(UsdGeom.Mesh)]
        report["robot_visual_inventory"] = [{"path": str(p.GetPath()),
            "points_count": len(UsdGeom.Mesh(p).GetPointsAttr().Get() or []),
            "world_bounds": _json([list(UsdGeom.BBoxCache(0, ["default", "render"]).ComputeWorldBound(p).ComputeAlignedRange().GetMin()),
                                   list(UsdGeom.BBoxCache(0, ["default", "render"]).ComputeWorldBound(p).ComputeAlignedRange().GetMax())]),
            "visibility": str(UsdGeom.Imageable(p).ComputeVisibility()),
            "purpose": str(UsdGeom.Imageable(p).ComputePurpose())} for p in robot_prims]
        report["usd_layers"] = [{"path": layer.realPath, "sha256": sha256(Path(layer.realPath))}
                                for layer in stage.GetUsedLayers() if layer.realPath and Path(layer.realPath).is_file()]
        if args.scene_only:
            snapshots = {}
            for _ in range(20):
                sim.render()
                app.update()
            for name, camera in cameras:
                rgba = np.asarray(camera.get_rgba())
                if rgba.shape != (480,640,4):
                    raise RuntimeError(f"missing scene-review RGB: {name}")
                path = args.out / f"scene_{name}.png"
                Image.fromarray(rgba[...,:3].astype(np.uint8)).save(path)
                snapshots[name] = {"file": path.name, "std": float(rgba[...,:3].std()),
                                   **camera_content(camera)}
            report.update(phase="scene_review_only", passed=False,
                          explicit_physics_steps=0, physics_steps=None,
                          scene_snapshots=snapshots, trajectory_executed=False,
                          warmup_note="Kit updates may advance PhysX during renderer warmup; no commanded trajectory or contact acceptance")
            _write(args.out / "result.json", report)
            return 0
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
        start = plan.get("start_configuration", {})
        expected_start = np.asarray(list(start.get("arm", [])) + list(start.get("hand", [])), dtype=float)
        if (expected_start.shape != qstart.shape or not np.isfinite(expected_start).all()
                or np.max(np.abs(expected_start-qstart)) > .02):
            raise ValueError("measured reset joints differ from collision-screened trajectory start")
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
                                images.append({"step":step,"phase":phase,"camera":name,
                                               "file":str(path.relative_to(args.out)), **camera_content(cam)})
                    step+=1
                qstart=end
                report["phase"]="completed_"+phase
                _write(args.out/"result.json",report)
        gate=contact_lift_gate(records, center[2], dt=dt)
        visual_gate = any(frame["camera"] == "main" and frame["phase"] == "hold"
                          and frame["robot_and_box_visible"] for frame in images)
        report.update(phase="completed", steps=step, trace="trace.jsonl", images=images,
                      max_box_lift_m=max(r["box_position"][2] for r in records)-center[2],
                      contact_lift_gate=gate, visual_evidence_gate=visual_gate,
                      passed=gate["passed"] and visual_gate,
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
