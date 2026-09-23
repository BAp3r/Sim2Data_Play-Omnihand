"""Headless RTX smoke for the reviewed dual-arm assembly.

The imported assembly is visual-only: this script does not invent robot
articulations or drives. It places a static card-box visual at a synthetic pose
and exercises the three reviewed cameras without advancing physics.
It is not a grasp, calibration, or production collector.
"""
from __future__ import annotations

import argparse
import faulthandler
import hashlib
import gc
import json
import os
import sys
import traceback
from pathlib import Path


CAMERAS = {
    "wrist_left": "/World/left/left_arm__base_link/left_arm__link1/left_arm__link2/left_arm__link3/left_arm__link4/left_arm__link5/left_arm__link6/left_mount_assembly/left_camera_housing/left_color_optical/Camera",
    "wrist_right": "/World/right/right_arm__base_link/right_arm__link1/right_arm__link2/right_arm__link3/right_arm__link4/right_arm__link5/right_arm__link6/right_mount_assembly/right_camera_housing/right_color_optical/Camera",
    "overhead": "/World/overhead",
}


def file_sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate_review_inputs(assembly: Path, profile: Path) -> dict:
    """Reject stale profile/texture bindings before starting Kit."""
    report_path = assembly.parent / "preview_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("profile_sha256") != file_sha256(profile):
        raise ValueError("reviewed assembly/profile hash mismatch")
    if set(report.get("camera_paths", [])) != set(CAMERAS.values()):
        raise ValueError("review report must bind exactly the three reviewed cameras")
    root = assembly.parent.resolve()
    for dependency in report.get("texture_dependencies", []):
        texture = (root / dependency["path"]).resolve()
        if not texture.is_relative_to(root):
            raise ValueError("review texture must stay inside the review directory")
        if not texture.is_file() or file_sha256(texture) != dependency["sha256"]:
            raise ValueError("review texture missing or hash mismatch")
    return {"assembly_sha256": file_sha256(assembly),
            "profile_sha256": file_sha256(profile),
            "report_sha256": file_sha256(report_path),
            "texture_dependencies": report.get("texture_dependencies", [])}



def render_scene(args, result):
    result["phase"] = "opening_assembly"
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    import omni.usd, carb, numpy as np
    from PIL import Image
    import omni.replicator.core as rep
    carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
    carb.settings.get_settings().set_bool("/isaaclab/render/active_viewport", True)
    ctx = omni.usd.get_context(); ctx.open_stage(str(args.assembly.resolve()))
    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("assembly stage failed to open")
    stage.SetEditTarget(stage.GetSessionLayer())
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != "Z":
        raise RuntimeError("assembly must be metre/Z-up")
    result["physics_schema_inventory"] = {
        name: [str(p.GetPath()) for p in stage.TraverseAll() if p.HasAPI(api)]
        for name, api in [("rigid_bodies", UsdPhysics.RigidBodyAPI),
                          ("colliders", UsdPhysics.CollisionAPI),
                          ("articulations", UsdPhysics.ArticulationRootAPI)]}
    result["assembly_prim_count"] = sum(1 for _ in stage.Traverse())
    # Rebind the audited table from its original source on this platform.
    # The flattened review derivative may carry inactive prototype prims.
    old_table = stage.GetPrimAtPath("/World/Table")
    table_transform = UsdGeom.Xformable(old_table).ComputeLocalToWorldTransform(0)
    stage.OverridePrim("/World/Table").SetActive(False)
    table = UsdGeom.Xform.Define(stage, "/World/RuntimeThor")
    table.AddTransformOp().Set(table_transform)
    stage.DefinePrim("/World/RuntimeThor/Asset").GetReferences().AddReference(args.table.absolute().as_posix())
    result["table_rebinding"] = {"source": str(args.table.absolute()), "preserved_world_transform": np.asarray(table_transform).T.tolist(), "physics_validated": False}
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    table_cfg = profile["table"]
    bin_cfg = profile["bin"]
    ground_z = float(table_cfg.get("ground_z_m", -0.7947))
    tabletop_visual_z = 0.0
    # Runtime scene correction: a continuous plane below the Thor table and a
    # synthetic outer-bin rim aligned with tabletop z=0. This is a session-layer
    # override; source USD and the review derivative remain untouched.
    ground = UsdGeom.Mesh.Define(stage, "/World/RuntimeFullFlatGround")
    ground.CreatePointsAttr([(-2.0, -2.0, ground_z), (2.0, -2.0, ground_z),
                             (2.0, 2.0, ground_z), (-2.0, 2.0, ground_z)])
    ground.CreateFaceVertexCountsAttr([4]); ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.14, 0.16)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    result["flat_ground"] = {"prim": "/World/RuntimeFullFlatGround", "prim_type": "Mesh", "z_m": ground_z,
                               "collision_api": True, "full_flat_ground": True, "physics_validated": False}
    bin_root = stage.GetPrimAtPath("/World/Bin")
    if bin_root and bin_root.IsValid():
        # Older preview layers put the 0.2 m walls 6 mm above the target due
        # to wall-thickness centering. Correct only that synthetic offset.
        sz = float(bin_cfg["inner_size_xyz_m"][2])
        t = float(bin_cfg["wall_thickness_m"])
        authored_top = float(bin_cfg["bottom_center_xyz_m"][2]) + sz + t / 2.0
        delta = float(bin_cfg["top_z_m"]) - authored_top
        UsdGeom.Xformable(bin_root).AddTranslateOp().Set(Gf.Vec3d(0, 0, delta))
        collision_paths = []
        for child in bin_root.GetChildren():
            if child.GetName() == "Bottom" or child.GetName() in {"Left", "Right", "Near", "Far"}:
                UsdPhysics.CollisionAPI.Apply(child)
                collision_paths.append(str(child.GetPath()))
        result["bin_rim_alignment"] = {"target_top_z_m": float(bin_cfg["top_z_m"]), "authored_top_z_m": authored_top,
                                        "applied_delta_z_m": delta, "synthetic": True,
                                        "bottom_collision_and_wall_collisions": collision_paths}
    result["scene_geometry"] = {
        "ground_prim_type": "Mesh", "ground_z_m": ground_z,
        "thor_visual_top_z_m": tabletop_visual_z, "thor_collision_top_z_m": -0.0155,
        "thor_visual_collision_delta_m": 0.0155,
        "bin_top_z_m": float(bin_cfg["top_z_m"]),
        "bin_bottom_and_walls_collision": bool(result.get("bin_rim_alignment", {}).get("bottom_collision_and_wall_collisions")),
        "large_cube_ground_prims": [], "synthetic_height_assumptions": True,
        "physics_validated": False,
    }
    camera_paths = dict(CAMERAS)
    result["camera_paths"] = {}
    for name, path in camera_paths.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdGeom.Camera):
            raise RuntimeError(f"missing reviewed camera: {path}")
        result["camera_paths"][name] = path
    if args.three_box_demo:
        # The authored wrist D405 poses are retained for training. For this
        # reviewer-only demo, use two independent, visible camera placements
        # so the three exported streams are auditable rather than blank.
        for name, eye in (("wrist_left", (-0.48, -0.42, 0.48)),
                          ("wrist_right", (0.48, -0.42, 0.48))):
            path = f"/World/Reviewer_{name}"
            camera = UsdGeom.Camera.Define(stage, path)
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
                Gf.Vec3d(*eye), Gf.Vec3d(0.0, 0.10, 0.03), Gf.Vec3d(0, 0, 1)).GetInverse())
            camera.CreateFocalLengthAttr(18.0); camera.CreateHorizontalApertureAttr(24.0)
            camera.CreateVerticalApertureAttr(18.0); camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
            camera_paths[name] = path; result["camera_paths"][name] = path
        result["camera_mode"] = "reviewer_wrist_poses_for_demo_only"

    # Replace only the static box proxy in a session layer. No source asset
    # is edited, no physics is advanced, and no support collider is invented.
    stage.OverridePrim("/World/SyntheticBox").SetActive(False)
    demo_boxes = None
    if args.three_box_demo:
        demo_boxes = json.loads(args.three_box_demo.read_text(encoding="utf-8"))
        if demo_boxes.get("sample_kind") != "synthetic_three_box_domain_randomized_kinematic_demo":
            raise ValueError("unexpected three-box demo plan")
        result["three_box_demo"] = {"plan": str(args.three_box_demo.absolute()), "physics_validated": False,
                                     "contact_validated": False}
    prim = stage.DefinePrim("/World/SyntheticCardBox", "Xform")
    stage.DefinePrim("/World/SyntheticCardBox/Asset").GetReferences().AddReference(args.cardbox.absolute().as_posix())
    x = UsdGeom.Xformable(prim); placement = x.AddTranslateOp(); x.AddScaleOp().Set(Gf.Vec3f(.12))
    bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(prim).ComputeAlignedRange()
    placement.Set(Gf.Vec3d(-.23, .13, -bounds.GetMin()[2]))
    result["static_cardbox"].update(dimensions_m=list(bounds.GetSize()), translation_m=list(placement.Get()))
    if demo_boxes:
        prim.SetActive(False)
        demo_root = UsdGeom.Xform.Define(stage, "/World/ThreeBoxDemo")
        for i, box in enumerate(demo_boxes["boxes"]):
            item = UsdGeom.Xform.Define(stage, f"/World/ThreeBoxDemo/{box['id']}")
            item.AddTranslateOp().Set(Gf.Vec3d(float(box["xy"][0]), float(box["xy"][1]), 0.0))
            item.AddRotateZOp().Set(float(box.get("yaw_rad", 0.0) * 180.0 / 3.141592653589793))
            scales = box["source_scale_xyz"]
            item.AddScaleOp().Set(Gf.Vec3f(float(scales[0]), float(scales[1]), float(scales[2])))
            stage.DefinePrim(f"/World/ThreeBoxDemo/{box['id']}/Asset").GetReferences().AddReference(args.cardbox.absolute().as_posix())
    result["phase"] = "initializing_renderer"
    # Cameras are attached to visual frames and are deliberately sampled as
    # independent RGB streams; no synchronization or calibration is claimed.
    cams = {}
    for name, path in camera_paths.items():
        product = rep.create.render_product(path, (320, 240) if args.trajectory else (640, 480))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product.path])
        cams[name] = (product, annotator)
    rep.orchestrator.set_capture_on_play(False)
    for _ in range(4):
        rep.orchestrator.step(rt_subframes=4, pause_timeline=True, delta_time=0.0)
    if args.trajectory:
        from isaac_motion_capture import capture_trajectory
        capture_trajectory(stage, rep, cams, args, result)
        result.update(passed=True, phase="completed", physics={"steps": 0, "validated": False}, coverage_accepted=False)
        rep.orchestrator.stop()
        return
    images, stats = {}, {}
    for name, (product, annotator) in cams.items():
        rgba = annotator.get_data()
        if rgba is None or np.asarray(rgba).ndim != 3 or np.asarray(rgba).shape[0] < 2:
            raise RuntimeError(f"camera {name} produced no RGB frame: {None if rgba is None else np.asarray(rgba).shape}")
        rgb = np.asarray(rgba)[..., :3].astype(np.uint8)
        fn = name + ".png"; Image.fromarray(rgb).save(args.out / fn)
        camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_paths[name]))
        focal = camera.GetFocalLengthAttr().Get()
        k = [[640*focal/camera.GetHorizontalApertureAttr().Get(), 0, 320], [0,480*focal/camera.GetVerticalApertureAttr().Get(),240], [0,0,1]]
        images[name] = fn; stats[name] = {"shape": list(rgb.shape), "std": float(rgb.std()), "frame_valid": bool(rgb.shape == (240,320,3)), "scene_content_detected": bool(rgb.std() > 2), "K": k, "T_world_usd": np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(camera_paths[name])).ComputeLocalToWorldTransform(0)).T.tolist()}
    result.update({"phase": "completed", "physics": {"steps": 0, "robot_articulation": False, "validated": False}, "rendering": {"backend": "RTX headless", "renderer_setting": carb.settings.get_settings().get("/rtx/rendermode"), "images": images, "stats": stats, "dynamic_synchronization_validated": False}, "passed": all(v["frame_valid"] and v["scene_content_detected"] for v in stats.values()), "coverage_accepted": False})
    if not result["passed"]: raise RuntimeError("one or more RGB streams are empty")
    # Let Kit close the stage and shared SDG graphs as a unit. Detaching each
    # annotator manually can invalidate shared nodes before the next detach.
    rep.orchestrator.stop()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--assembly", type=Path, required=True)
    ap.add_argument("--cardbox", type=Path, required=True)
    ap.add_argument("--table", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--trajectory", type=Path, help="Pinocchio plan for explicitly kinematic RGB capture")
    ap.add_argument("--three-box-demo", type=Path, help="Synthetic three-box visual plan; no physics/contact claim")
    ap.add_argument("--profile", type=Path, default=Path("configs/commissioning.synthetic.json"))
    ap.add_argument("--graphics-api", choices=("d3d12", "vulkan"), default="d3d12")
    args = ap.parse_args()
    if not all(p.is_file() for p in (args.assembly, args.cardbox, args.table)):
        ap.error("assembly, table and cardbox must be existing files")
    identities = validate_review_inputs(args.assembly, args.profile)
    manifest = json.loads(Path("configs/asset_manifest.json").read_text(encoding="utf-8"))
    table_record = next(p for p in manifest["commissioning_model_packages"] if p["id"] == "isaac_5_1_thorlabs_table")
    for path, expected in [(args.table, table_record["sha256"]),
                           (args.cardbox, manifest["assets"]["card_box"]["sha256"])]:
        if file_sha256(path) != expected:
            ap.error("asset identity differs from reviewed manifest")
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=False)
    result = {
        "scope": "synthetic_full_assembly_three_camera_smoke",
        "production_collection_allowed": False,
        "assembly_visual": {"path": str(args.assembly.resolve()), "articulation": False, "drives": False},
        "static_cardbox": {"asset": str(args.cardbox.absolute()), "uniform_scale": 0.12, "physics_validated": False},
        "requested_graphics_api": args.graphics_api, "passed": False, "phase": "starting_kit",
        "input_identity": identities,
        "exit_evidence": "external process exit must be checked separately; Kit may terminate Python during close",
        "shutdown_mode": {"fast_shutdown": False, "skip_cleanup": False},
    }
    (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    app = None
    try:
        profile = Path(sys.executable).parent.parent / "sim2data_nas.json"
        nas = json.loads(profile.read_text()) if profile.is_file() else None
        if nas:
            for key in ("MDL_SYSTEM_PATH", "MDL_USER_PATH"):
                old = os.environ.get(key, "")
                os.environ[key] = os.pathsep.join(nas["mdl_paths"] + ([old] if old else []))
            result["nas_profile"] = nas
        from isaacsim import SimulationApp
        sys.argv = [sys.argv[0], "--portable-root", str(args.out / "kit")]
        if nas:
            sys.argv.append("--/persistent/isaac/asset_root/default=" + nas["asset_root"])
        sys.argv += ["--" + args.graphics_api, "--/app/vulkan=" + ("false" if args.graphics_api == "d3d12" else "true")]
        app = SimulationApp({"headless": True, "width": 640, "height": 480,
                             "fast_shutdown": False,
                             "renderer": "RaytracedLighting", "anti_aliasing": 0,
                             "multi_gpu": False, "active_gpu": 0, "max_gpu_count": 1,
                             "limit_cpu_threads": 4,
                             "extra_args": ["--/app/extensions/registryEnabled=false",
                                             "--/app/extensions/syncRegistryOnStartup=false",
                                             f"--/log/file={args.out / 'kit.log'}"]})
        render_scene(args, result)
    except Exception:
        result["passed"] = False
        result["error"] = traceback.format_exc()
        raise
    finally:
        result["phase"] = "completed" if result.get("passed") else result.get("phase", "failed")
        (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        if app is not None:
            result["shutdown"] = "entering_cleanup"
            (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            with (args.out / "shutdown_stack.txt").open("w") as stack:
                faulthandler.dump_traceback_later(45, repeat=True, file=stack)
                gc.collect()
                try:
                    app.close(wait_for_replicator=False)
                    result["shutdown"] = "app_close_returned"
                except Exception as close_error:
                    result["shutdown"] = "app_close_raised"
                    result["shutdown_error"] = repr(close_error)
                faulthandler.cancel_dump_traceback_later()
            (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__": main()
