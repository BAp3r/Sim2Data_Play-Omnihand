"""RTX capture of approved Pinocchio FK playback; explicitly not dynamics."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


def capture_trajectory(stage, rep, cameras, args, result):
    import numpy as np
    from PIL import Image
    from pxr import Gf, UsdGeom

    plan = json.loads(args.trajectory.read_text(encoding="utf-8"))
    if plan["sample_kind"] != "synthetic_kinematic_planned_motion":
        raise ValueError("unexpected trajectory kind")
    if plan["profile_sha256"] != result["input_identity"]["profile_sha256"]:
        raise ValueError("trajectory/profile identity mismatch")
    if plan["production_collection_allowed"] is not False or plan["task_success"] is not None:
        raise ValueError("kinematic playback cannot become production or relay success")
    if plan["planner"]["library"] != "Pinocchio":
        raise ValueError("this capture requires the reviewed Pinocchio output")
    frames = plan["frames"]
    if len(frames) < 2 or plan["fps"] != 30:
        raise ValueError("expected multi-frame 30 Hz motion plan")
    operations = {}
    base_world = {}
    for side in ("left", "right"):
        path = f"/World/{side}/{side}_arm__base_link"
        base_world[side] = np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T
        for index in range(1, 7):
            path += f"/{side}_arm__link{index}"
            node = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            ops = node.GetOrderedXformOps()
            if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
                raise ValueError(f"unexpected reviewed transform structure: {path}")
            operations[side, index] = (node, ops[0])
            parent = np.eye(4) if index == 1 else np.asarray(frames[0]["T_base_links"][side][f"link{index-1}"])
            child = np.asarray(frames[0]["T_base_links"][side][f"link{index}"])
            expected = np.linalg.inv(parent) @ child
            if not np.allclose(np.asarray(ops[0].Get()).T, expected, atol=1e-8, rtol=0):
                raise ValueError(f"Pinocchio initial FK does not match reviewed USD: {path}")
    for name in cameras:
        (args.out / "images" / name).mkdir(parents=True)
    records = []
    camera_paths = result["camera_paths"]
    camera_intrinsics = {}
    for name, path in camera_paths.items():
        camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
        focal = camera.GetFocalLengthAttr().Get()
        if camera.GetHorizontalApertureOffsetAttr().Get() != 0 or camera.GetVerticalApertureOffsetAttr().Get() != 0:
            raise ValueError("capture assumes reviewed zero-offset pinhole intrinsics")
        camera_intrinsics[name] = [[320*focal/camera.GetHorizontalApertureAttr().Get(),0,160],
                                   [0,240*focal/camera.GetVerticalApertureAttr().Get(),120],[0,0,1]]
    max_fk_error = 0.0
    hashes = {name: set() for name in cameras}
    for index, frame in enumerate(frames):
        if frame["index"] != index or abs(frame["timestamp"]-index/30) > 1e-9:
            raise ValueError("noncontiguous planned timeline")
        for side in ("left", "right"):
            for joint in range(1, 7):
                parent = np.eye(4) if joint == 1 else np.asarray(frame["T_base_links"][side][f"link{joint-1}"])
                child = np.asarray(frame["T_base_links"][side][f"link{joint}"])
                operations[side, joint][1].Set(Gf.Matrix4d((np.linalg.inv(parent) @ child).T.tolist()))
            actual = np.asarray(operations[side, 6][0].ComputeLocalToWorldTransform(0)).T
            error = float(np.max(np.abs(actual - base_world[side] @ np.asarray(frame["T_base_links"][side]["link6"]))))
            max_fk_error = max(max_fk_error, error)
            if error > 1e-8:
                raise RuntimeError("USD FK readback differs from Pinocchio plan")
        # Every rendered triple follows one common FK update. Physics stays
        # paused; the dataset timestamp is this explicit playback clock.
        rep.orchestrator.step(rt_subframes=4, pause_timeline=True, delta_time=0.0)
        images = {}
        for name, (_, annotator) in cameras.items():
            rgb = np.asarray(annotator.get_data())[..., :3]
            if rgb.shape != (240, 320, 3) or rgb.dtype != np.uint8:
                raise RuntimeError(f"invalid captured RGB: {name}: {rgb.shape} {rgb.dtype}")
            relative = f"images/{name}/{index:06d}.png"
            Image.fromarray(rgb).save(args.out / relative)
            images[name] = relative
            hashes[name].add(hashlib.sha256(rgb.tobytes()).hexdigest())
        camera_poses = {name: np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T.tolist()
                        for name,path in camera_paths.items()}
        records.append({"index": index, "timestamp": index/30,
                        "state": frame["q"], "action": frames[min(index+1,len(frames)-1)]["q"],
                        "images": images, "camera_T_world_usd": camera_poses})
        if index % 30 == 0:
            print(f"Captured motion frame {index+1}/{len(frames)}", flush=True)
    if len(hashes["overhead"]) < 2:
        raise RuntimeError("overhead frames did not change during planned motion")
    capture = {
        "sample_kind": "synthetic_kinematic_planned_motion",
        "production_collection_allowed": False, "task_success": None,
        "physics_validated": False, "robot_articulation_validated": False,
        "camera_synchronization_validated": False,
        "capture_alignment": "all streams rendered after the same verified FK update; no dynamic latency claim",
        "fps": 30, "joint_names": plan["joint_names"], "image_shape": [240,320,3],
        "camera_K": camera_intrinsics,
        "camera_calibration": "synthetic pinhole; USD -Z forward,+Y up; not measured D405 calibration",
        "camera_frame_poses": [record["camera_T_world_usd"] for record in records],
        "state_semantics": "planned joint configuration used for USD FK; not physical feedback",
        "action_semantics": "next-frame planned joint configuration; not hardware command",
        "timing_semantics": "explicit kinematic playback clock; no physics ticks",
        "planner": plan["planner"],
        "trajectory_sha256": hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
        "input_identity": result["input_identity"], "frames": records,
        "max_fk_matrix_error": max_fk_error,
        "unique_image_counts": {k: len(v) for k,v in hashes.items()},
        "hands": "reviewed fixed open posture, not actuated",
    }
    with (args.out / "capture.json").open("x", encoding="utf-8") as stream:
        json.dump(capture, stream, indent=2)
    result["motion_capture"] = {"frames": len(records), "manifest": "capture.json",
                                "max_fk_matrix_error": max_fk_error,
                                "unique_image_counts": capture["unique_image_counts"]}
