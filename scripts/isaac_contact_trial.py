"""Bounded synthetic single-arm contact trial; never writes simulated body poses."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import traceback

from convert_physx_commissioning import _start_kit, sha256
from isaac_finger_response import _action, _json, _write
from physx_scene_audit import audit_colliders, validate_contact_matrix


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


def contact_release_gate(rows, mass_kg, dt):
    """Require a continuous supported, stable and hand-free final retreat."""
    if not math.isfinite(mass_kg) or mass_kg <= 0 or not math.isfinite(dt) or not 0 < dt <= 1/30:
        raise ValueError("positive mass and valid physics dt required")
    current = 0
    for row in rows:
        accepted = (state_is_bounded(row) and row["phase"] == "retreat"
                    and 0 <= row["hand_contact_force_N"] < .01
                    and .8*mass_kg*9.81 < row["support_force_N"] < 1.2*mass_kg*9.81
                    and 0 <= row["ground_force_N"] < .01
                    and sum(v*v for v in row["box_velocity"][:3]) < .02**2
                    and sum(v*v for v in row["box_velocity"][3:]) < .1**2)
        current = current + 1 if accepted else 0
    return {"passed": current*dt >= 1., "final_continuous_steps": current,
            "required_dwell_s": 1., "scope": "single-arm return to Thor; not relay footprint acceptance"}


def validate_planned_scene(plan, profile_sha, scene_only=False):
    """Keep failed or stale global-state plans out of the physics action loop."""
    if plan.get("scene_kind") != "thor_cardbox":
        raise ValueError("contact commissioning requires the Thor/cardbox scene")
    if plan.get("coordinate_frame") != "world" or plan.get("profile_sha256") != profile_sha:
        raise ValueError("Thor plan must bind current profile and world coordinates")
    if not scene_only and plan.get("execution_allowed") is not True:
        raise ValueError("global-state geometry/IK gate failed; action execution blocked")


def validate_preload(preload, *, plan_sha, profile_sha, urdf_sha, active_names, effort_limits,
                     allowed_effort_names=None):
    """Reject stale, reordered or unbounded active-only feedforward requests."""
    if (preload.get("plan_sha256") != plan_sha
            or preload.get("profile_sha256") != profile_sha
            or preload.get("urdf_sha256") != urdf_sha
            or preload.get("joint_names") != active_names
            or len(set(active_names)) != len(active_names)
            or preload.get("synthetic") is not True
            or preload.get("requested_force_is_not_measured") is not True):
        raise ValueError("preload input identity or active joint order mismatch")
    values = preload.get("effort_Nm", [])
    if (len(values) != len(active_names) or len(effort_limits) != len(active_names)
            or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
                   or not math.isfinite(limit) or limit < 0 or abs(v) > limit
                   for v, limit in zip(values, effort_limits))):
        raise ValueError("preload exceeds active effort bounds")
    if allowed_effort_names is not None:
        allowed = set(allowed_effort_names)
        if not allowed.issubset(active_names):
            raise ValueError("preload allowed set is not a subset of active joints")
        if any(name not in allowed and abs(float(value)) > 1e-12
               for name, value in zip(active_names, values)):
            raise ValueError("preload allocates effort to a fixed hand channel")
    return values


def validate_hand_plan(profile, plan):
    """Reject variable thumb shaping or a reset different from mapped open."""
    from sim2data.control.gripper import allowed_closing_joint_names, validate_gesture_targets
    side = plan["side"]
    active = profile["gripper_commissioning"][side]["active_joints"]
    opened, closed = plan["hand_open"], plan["hand_close"]
    validate_gesture_targets(side, active, opened, closed)
    expected = [item["open_rad"] for item in active]
    for values in (opened, plan["start_configuration"]["hand"]):
        if len(values) != len(expected) or any(abs(a-b) > 1e-9 for a,b in zip(values, expected)):
            raise ValueError("hand plan must initialize and approach in the mapped open gesture")
    return allowed_closing_joint_names(side)


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
    parser.add_argument("--squeeze-effort", type=float, default=0.0,
                        help="Synthetic active flexion effort preload in Nm, ramped during finger close (0..0.3)")
    parser.add_argument("--preload", type=Path, help="Source FK Jacobian preload bound to this plan")
    parser.add_argument("--scene-only", action="store_true", help="Scene visibility review with render warmup; no action trajectory or grasp claim")
    parser.add_argument("--support-only", action="store_true", help="Gravity/support force validation only; no robot action trajectory")
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
        profile = json.loads(args.profile.read_text(encoding="utf-8"))
        allowed_close = set(validate_hand_plan(profile, plan))
        if not math.isfinite(args.squeeze_effort) or not 0 <= args.squeeze_effort <= .3:
            raise ValueError("squeeze effort must be finite in [0, 0.3] Nm")
        diagnostic_only = args.scene_only or args.support_only
        if args.scene_only and args.support_only:
            raise ValueError("choose scene-only or support-only")
        validate_planned_scene(plan, sha256(args.profile), diagnostic_only)
        if not diagnostic_only:
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
        # Use composed collider paths for contact filters. The Thor asset's
        # namespace regex is useful as a scene description, but the PhysX
        # tensor view may not expand instanceable descendants from that regex.
        # Keeping the resolved paths makes the filter order auditable and lets
        # the matrix report the actual support pair.
        support_filters = list(scene.get("thor_colliders", []))
        if not support_filters:
            raise ValueError("Thor scene resolved no collider filters")
        filters = support_filters + [ground_path] + bodies
        support_filter_indices = list(range(len(support_filters)))
        ground_filter_index = len(support_filters)
        box = RigidPrim(box_path, name="trial_box", track_contact_forces=True,
                        contact_filter_prim_paths_expr=filters, max_contact_count=4096,
                        disable_stablization=False, reset_xform_properties=False)
        # Capture composed collider ownership/material before stepping. This is
        # read-only evidence and helps distinguish a real support failure from a
        # contact-filter/readback failure.
        report["scene_audit"] = audit_colliders(stage, roots=(box_path, "/World/RuntimeThor", ground_path, robot_parent))
        def _view_count(name):
            value = getattr(box._contact_view, name, None)
            return int(value) if value is not None else None
        if args.support_only:
            # The reset arm can overlap the box before any action is sent. Isolate
            # the static support check by disabling only robot collision shapes in
            # this diagnostic session; the contact trial keeps all collisions.
            support_disabled = []
            for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
                if str(prim.GetPath()).startswith(robot_parent + "/") and prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                    support_disabled.append(str(prim.GetPath()))
            report["support_only_collision_override"] = {
                "robot_colliders_disabled": support_disabled,
                "reason": "isolate box gravity/support from reset-pose robot overlap; not used for contact trial",
                "synthetic_diagnostic_only": True,
            }
        cameras = []
        # Training main view is overhead; the wider oblique view is demo only.
        # Include both base locations and the bin in the overhead framing even
        # during this single-arm diagnostic. These are synthetic camera poses.
        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        robot_range = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(robot_parent)).ComputeAlignedRange()
        robot_min = np.asarray(robot_range.GetMin(), dtype=float)
        robot_max = np.asarray(robot_range.GetMax(), dtype=float)
        box_center = np.asarray(center, dtype=float)
        work_min = np.minimum(robot_min, box_center - np.asarray(scene["box_dimensions_m"]) / 2)
        work_max = np.maximum(robot_max, box_center + np.asarray(scene["box_dimensions_m"]) / 2)
        work_target = ((work_min + work_max) / 2).tolist()
        camera_specs = (
            ("overhead", [.12, -.08, 1.20], [.12, -.08, 0.0], [0, 1, 0], 18.0),
            ("demo", [0.62, -1.18, 0.78], work_target, [0, 0, 1], 12.0),
            ("close", [0.28, -0.72, 0.44], [float(box_center[0]), float(box_center[1]), float(box_center[2] + .04)], [0, 0, 1], 16.0),
        )
        report["camera_framing"] = {"work_bounds_min": work_min.tolist(),
                                    "work_bounds_max": work_max.tolist(),
                                    "synthetic": True,
                                    "training_main": "overhead",
                                    "diagnostic_only": ["demo", "close"],
                                    "wrist_streams_present": False,
                                    "poses": [{"name": n, "eye": e, "target": t, "up": u,
                                               "focal_length": f} for n,e,t,u,f in camera_specs]}
        for name, eye, target, up, focal_length in camera_specs:
            camera = Camera("/Trial/Camera_" + name, resolution=(640,480))
            matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse()
            q = matrix.ExtractRotationQuat()
            camera.set_world_pose(np.array(eye), np.array([q.GetReal(),*q.GetImaginary()]), camera_axes="usd")
            camera.set_focal_length(focal_length); camera.set_horizontal_aperture(24); camera.set_vertical_aperture(18)
            camera.set_clipping_range(.01,10)
            cameras.append((name,camera))
        sim.reset(); robot.initialize(); box.initialize()
        # One initialization write of the ten active hand DOFs only. The arm
        # reset and passive/mimic readback remain separate; no loop state writes.
        initial_dofs = list(robot.dof_names)
        initial_active = response["mapping"]["active_hand"]
        initial_arm = response["mapping"]["arm_hold_names"]
        initial_indices = [initial_dofs.index(item["dof_name"]) for item in initial_active]
        from isaac_finger_response import _reset_hand_to_open
        reset_q = np.asarray(robot.get_joint_positions()).reshape(-1)
        report["initialization"] = _reset_hand_to_open(
            robot, initial_dofs, initial_indices, plan["hand_open"],
            [float(reset_q[initial_dofs.index(name)]) for name in initial_arm], initial_arm)
        report["initial_joint_state_writes"] = 1
        # Fail closed if the backend does not explicitly expose initialized
        # articulation handles; a missing flag is not evidence of PhysX.
        report["real_articulation"] = bool(getattr(robot, "handles_initialized", False))
        report["contact_view"] = {
            "num_shapes": _view_count("num_shapes"),
            "num_filters": _view_count("num_filters"),
            "filter_order": filters,
        }
        report["runtime_scene_audit"] = audit_colliders(
            stage, roots=(box_path, "/World/RuntimeThor", ground_path, robot_parent))
        _write(args.out / "result.json", report)
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
        if args.support_only:
            records = []
            for step in range(round(1.0/dt)):
                sim.step(render=False)
                pos, quat = box.get_world_poses()
                matrix = np.asarray(box.get_contact_force_matrix(dt=dt))
                raw = box.get_contact_force_data(dt=dt)
                raw_shapes = [list(np.shape(value)) for value in raw]
                if step == 0:
                    report["contact_buffer_shapes"] = {
                        "matrix": list(matrix.shape), "raw": raw_shapes,
                        "order": ["forces", "points", "normals", "distances", "counts", "starts"],
                    }
                    _write(args.out / "result.json", report)
                matrix_meta = validate_contact_matrix(
                    matrix, filters,
                    sensor_count=_view_count("num_shapes"),
                    filter_count=_view_count("num_filters"),
                )
                records.append({"step": step, "time_s": (step+1)*dt,
                                "box_position": _json(pos[0]), "box_velocity": _json(box.get_velocities()[0]),
                                "support_force_N": float(np.linalg.norm(matrix[0,support_filter_indices],axis=1).sum()),
                                "ground_force_N": float(np.linalg.norm(matrix[0,ground_filter_index])),
                                "robot_force_N": float(np.linalg.norm(matrix[0,ground_filter_index+1:],axis=1).sum()),
                                "net_contact_force_N": _json(box.get_net_contact_forces(dt=dt)),
                                "raw_counts": _json(raw[4]), "raw_shapes": raw_shapes,
                                "contact_matrix": matrix_meta})
            _write(args.out / "support_trace.json", records)
            tail = records[-max(1,round(.2/dt)):]
            expected_force = float(plan["mass_kg"])*9.81
            support_passed = all(
                np.isfinite(row["box_position"]).all() and np.isfinite(row["box_velocity"]).all()
                and abs(row["box_position"][2]-(center[2]-.001)) < .003
                and np.linalg.norm(row["box_velocity"][:3]) < .02
                and .8*expected_force < row["support_force_N"] < 1.2*expected_force
                and row["ground_force_N"] < .01 and row["robot_force_N"] < .01 for row in tail)
            report.update(phase="support_validation", passed=False, support_passed=bool(support_passed),
                          physics_steps=len(records), support_trace="support_trace.json",
                          contact_filter_order=filters, expected_weight_N=expected_force,
                          trajectory_executed=False, grasp_validated=False,
                          final_support_state=records[-1])
            _write(args.out / "result.json", report)
            return 0 if support_passed else 3
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
        fixed_drive_indices=[indices[6+j] for j,item in enumerate(active) if item["dof_name"] not in allowed_close]
        kp[fixed_drive_indices]=40; kd[fixed_drive_indices]=2
        report["fixed_hand_hold_drive"]={"stiffness":40.,"damping":2.,"synthetic":True,
            "joint_names":[dofs[i] for i in fixed_drive_indices],
            "reason":"hold fixed gesture channels against contact; no shaping preload"}
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
        report["arm_gravity_compensation"] = {
            "enabled": True, "source": "PhysX generalized gravity compensation forces",
            "joint_names": arm, "absolute_effort_cap_Nm": 40.0,
            "gravity_remains_enabled": True, "synthetic_controller": True}
        hand_open = plan.get("hand_open", [x["open_rad"] for x in active])
        hand_close = plan.get("hand_close", [x["close_rad"] for x in active])
        if len(hand_open) != len(active) or len(hand_close) != len(active):
            raise ValueError("contact hand endpoints must cover active joints only")
        # Let the action drives settle the direct-open reset and PhysX mimic
        # constraints before recording the trajectory.  These are ordinary
        # ArticulationAction commands during reset; no joint state is written.
        reset_indices = indices
        reset_targets = np.asarray(list(plan["start_configuration"]["arm"]) + list(hand_open), dtype=float)
        reset_settle_steps = max(1, round(0.5 / dt))
        reset_frames = []
        with (args.out/"reset_trace.jsonl").open("w") as reset_stream:
            for reset_step in range(reset_settle_steps):
                gravity = np.asarray(view.get_generalized_gravity_forces()).reshape(-1)
                action = _action(ArticulationAction, reset_indices, reset_targets.tolist())
                action.joint_efforts = torch.tensor(np.r_[np.clip(gravity[indices[:6]],-40,40),np.zeros(10)],dtype=torch.float32)
                robot.apply_action(action)
                sim.step(render=False)
                reset_row = {"step":reset_step,"time_s":(reset_step+1)*dt,
                    "commanded":dict(zip([dofs[i] for i in indices],reset_targets.tolist())),
                    "q":_json(robot.get_joint_positions()),"qd":_json(robot.get_joint_velocities()),
                    "effort":_json(robot.get_measured_joint_efforts()),
                    "box_position":_json(box.get_world_poses()[0][0]),
                    "box_velocity":_json(box.get_velocities()[0])}
                reset_stream.write(json.dumps(reset_row)+"\n")
                reset_frames.append(reset_row)
                if not all(np.isfinite(reset_row[k]).all() for k in ("q","qd","effort","box_position","box_velocity")):
                    raise ValueError("nonfinite reset settle state")
        report["initialization"]["physical_settle"] = {"steps":reset_settle_steps,
            "trace":"reset_trace.jsonl","state_writes":0,
            "max_initial_joint_velocity_rad_s":max(abs(v) for r in reset_frames for v in r["qd"]),
            "final_max_joint_velocity_rad_s":max(abs(v) for v in reset_frames[-1]["qd"])}
        if max(abs(v) for v in reset_frames[-1]["qd"]) > 1:
            raise ValueError("reset settle joint velocities did not converge")
        qstart=np.asarray(robot.get_joint_positions()).reshape(-1)[indices]
        start = plan.get("start_configuration", {})
        expected_start = np.asarray(list(start.get("arm", [])) + list(start.get("hand", [])), dtype=float)
        report["reset_joint_check"] = {"measured": _json(qstart), "expected": _json(expected_start),
                                       "max_abs_error": (float(np.max(np.abs(expected_start-qstart)))
                                                         if expected_start.shape == qstart.shape else None)}
        if (expected_start.shape != qstart.shape or not np.isfinite(expected_start).all()
                or np.max(np.abs(expected_start-qstart)) > .02):
            raise ValueError("measured reset joints differ from collision-screened trajectory start")
        hand_open = plan.get("hand_open", [x["open_rad"] for x in active])
        hand_close = plan.get("hand_close", [x["close_rad"] for x in active])
        if len(hand_open) != len(active) or len(hand_close) != len(active):
            raise ValueError("contact hand endpoints must cover active joints only")
        # Only the three opposed fingers receive bounded flexion preload.
        # Passive/mimic joints remain read-only. This is actuator torque, not a
        # claimed contact force or a change to gravity/collision geometry.
        # The planner records the side-specific gesture.  Fixed active hand
        # channels remain at their open targets and receive zero preload.
        squeeze = np.zeros(len(active))
        for j, item in enumerate(active):
            name = item["dof_name"]
            if name in allowed_close:
                squeeze[j] = args.squeeze_effort * np.sign(hand_close[j]-hand_open[j])
        report["squeeze_preload"] = {"synthetic":True,"effort_Nm":squeeze.tolist(),
            "joint_names":[a["dof_name"] for a in active],"not_measured_contact_force":True}
        if args.preload:
            if args.squeeze_effort:
                raise ValueError("choose scalar or Jacobian preload")
            preload = json.loads(args.preload.read_text(encoding="utf-8"))
            squeeze = np.asarray(validate_preload(
                preload, plan_sha=sha256(args.plan), profile_sha=sha256(args.profile),
                urdf_sha=plan["urdf_sha256"], active_names=[a["dof_name"] for a in active],
                effort_limits=efforts[indices[6:]].tolist(),
                allowed_effort_names=allowed_close), dtype=float)
            report["squeeze_preload"] = preload
        props = robot.dof_properties
        for i, lo, hi in zip(indices[6:], hand_open, hand_close):
            if not all(np.isfinite(v) and props[i]["lower"] - 1e-6 <= v <= props[i]["upper"] + 1e-6 for v in (lo, hi)):
                raise ValueError("contact hand target outside source limits")
        for arm_target in (plan["pregrasp"], plan["grasp"], plan["lift"]):
            if len(arm_target) != 6 or not all(np.isfinite(v) and props[i]["lower"] - 1e-6 <= v <= props[i]["upper"] + 1e-6
                                                for i,v in zip(indices[:6],arm_target)):
                raise ValueError("arm target outside source limits")
        records=[]; images=[]; step=0
        report.update(trace="trace.jsonl", images=images, dt=dt,
                      inertia_override_applied=False)
        rgbroot=args.out/"rgb";rgbroot.mkdir()
        # Resolve the render products before spending a full contact trial on
        # frames with no robot. Rendering alone does not advance the episode.
        for _ in range(20):
            sim.render()
        visibility = {}
        for name, cam in cameras:
            rgba = np.asarray(cam.get_rgba())
            if rgba.shape != (480, 640, 4):
                raise RuntimeError(f"missing initial RGB: {name}")
            Image.fromarray(rgba[..., :3].astype(np.uint8)).save(args.out/f"initial_{name}.png")
            visibility[name] = camera_content(cam)
        report["initial_visibility"] = visibility
        _write(args.out/"result.json", report)
        if not visibility["overhead"]["robot_and_box_visible"]:
            raise ValueError("overhead robot/box visibility failed before approach")
        phases=[("settle_open",plan["start_configuration"]["arm"],0,3),
                ("approach",plan["pregrasp"],0,4), ("approach_lower",plan["grasp"],0,2),
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
                    if phase in ("lift", "lower") and plan.get("lift_waypoints"):
                        waypoints = np.asarray(plan["lift_waypoints"], dtype=float)
                        if waypoints.ndim != 2 or waypoints.shape[1] != 6 or not np.isfinite(waypoints).all():
                            raise ValueError("invalid Cartesian lift joint waypoints")
                        if phase == "lower":
                            waypoints = waypoints[::-1]
                        scaled = blend*(len(waypoints)-1)
                        index = min(int(scaled), len(waypoints)-2)
                        command[:6] = waypoints[index]+(scaled-index)*(waypoints[index+1]-waypoints[index])
                    # Counteract the measured configuration's gravity load with
                    # actuator torque. Keep gravity, contacts and position drives
                    # active; never compensate passive/mimic hand joints.
                    gravity = np.asarray(view.get_generalized_gravity_forces()).reshape(-1)
                    if gravity.shape != (len(dofs),) or not np.isfinite(gravity).all():
                        raise ValueError("invalid PhysX gravity compensation readback")
                    feedforward = np.zeros(len(indices))
                    feedforward[:6] = np.clip(gravity[indices[:6]], -40., 40.)
                    squeeze_scale = (blend if phase == "finger_close" else
                                     1.0 if phase in ("lift", "hold", "lower") else
                                     1.0-blend if phase == "release" else 0.0)
                    feedforward[6:] = squeeze * squeeze_scale
                    action = _action(ArticulationAction,indices,command.tolist())
                    action.joint_efforts = torch.tensor(feedforward, dtype=torch.float32)
                    robot.apply_action(action)
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
                    matrix=np.asarray(matrix)
                    matrix_meta = validate_contact_matrix(
                        matrix, filters,
                        sensor_count=_view_count("num_shapes"),
                        filter_count=_view_count("num_filters"),
                    )
                    row={"step":step,"time_s":(step+1)*dt,"phase":phase,"commanded":dict(zip([dofs[i] for i in indices],command.tolist())),
                         "feedforward_effort_Nm":dict(zip([dofs[i] for i in indices],feedforward.tolist())),
                         "q":_json(q),"qd":_json(qd),"effort":_json(robot.get_measured_joint_efforts()),
                         "box_position":_json(pos[0]),"box_quaternion_wxyz":_json(quat[0]),
                         "box_velocity":_json(box.get_velocities()[0]),"contacts":contacts,
                         "support_force_N":float(np.linalg.norm(matrix[0,support_filter_indices],axis=1).sum()),
                         "ground_force_N":float(np.linalg.norm(matrix[0,ground_filter_index])),
                         "hand_contact_force_N":float(np.linalg.norm(matrix[0,hand_filter_indices],axis=1).sum()),
                         "robot_contact_force_N":float(np.linalg.norm(matrix[0,ground_filter_index+1:],axis=1).sum()),
                         "contact_matrix":matrix_meta}
                    fixed_indices = [j for j,item in enumerate(active) if item["dof_name"] not in allowed_close]
                    fixed_drift = max((abs(float(q[indices[6+j]]) - float(hand_open[j])) for j in fixed_indices), default=0.0)
                    row["fixed_hand_max_drift_rad"] = fixed_drift
                    if fixed_drift > 0.03:
                        report.update(phase="fixed_hand_drift", failed_step=step, steps=step+1,
                                     fixed_hand_max_drift_rad=fixed_drift,
                                     contact_observed=any(r["robot_contact_force_N"]>.02 for r in records))
                        stream.write(json.dumps(row)+"\n"); records.append(row)
                        _write(args.out/"result.json", report)
                        return 3
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
        release_gate=contact_release_gate(records, float(plan["mass_kg"]), dt)
        visual_gate = any(frame["camera"] == "overhead" and frame["phase"] == "hold"
                          and frame["robot_and_box_visible"] for frame in images)
        report.update(phase="completed", steps=step, trace="trace.jsonl", images=images,
                      max_box_lift_m=max(r["box_position"][2] for r in records)-center[2],
                      contact_lift_gate=gate, visual_evidence_gate=visual_gate,
                      contact_release_gate=release_gate,
                      passed=gate["passed"] and release_gate["passed"] and visual_gate,
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
            report["video"]="overhead.mp4"
        _write(args.out/"result.json",report)
        return 0 if report["passed"] else 3
    except Exception:
        report.update(phase="failed", passed=False, video=None, error=traceback.format_exc())
        _write(args.out/"result.json",report)
        return 2
    finally:
        if app:
            if report.get("images"):
                try:
                    from trial_video import encode_trial
                    report["diagnostic_videos"] = encode_trial(args.out, report)
                except Exception as exc:
                    report["diagnostic_video_error"] = repr(exc)
            report["shutdown"]["attempted"]=True
            _write(args.out/"result.json",report)
            app.close(wait_for_replicator=False)
            report["shutdown"]["returned"]=True
            _write(args.out/"result.json",report)


if __name__ == "__main__":
    raise SystemExit(main())
