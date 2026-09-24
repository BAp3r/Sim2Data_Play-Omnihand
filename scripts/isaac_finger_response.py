"""PhysX articulation finger response commissioning probe.

The input USD must be produced by the reviewed Isaac Lab URDF converter.  The
probe initializes a real PhysX articulation and sends position targets through
ArticulationAction.  It never writes joint state during the run; only the
initial simulator reset is used.  No object is added, so this entry point does
not claim contact, grasp, or production collection success.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET
from typing import Any


AMOUNTS = (0.0, 0.5, 1.0, 0.0)
DEFAULT_STEPS = 120
DEFAULT_DT = 1.0 / 240.0
# Synthetic commissioning values.  They are deliberately recorded in result.json.
ARM_KP, ARM_KD, ARM_EFFORT, ARM_VELOCITY = 20.0, 2.0, 40.0, 2.0


def response_passed(stages, movement, mimic, frames, tolerance):
    """Fail closed on missing readback, collapsed targets or invalid coupling."""
    return bool(movement) and all(row["passed"] for row in movement) and len(stages) == 4 and all(
        math.isfinite(stage["hand_max_abs_error_rad"]) and stage["hand_max_abs_error_rad"] <= tolerance
        for stage in stages) and bool(mimic) and all(
        row["max_abs_residual_rad"] is not None and math.isfinite(row["max_abs_residual_rad"])
        and row["max_abs_residual_rad"] <= 0.05 for row in mimic) and bool(frames) and all(
        row["measured_effort"] is not None and all(math.isfinite(v) for v in row["measured_effort"])
        for row in frames)


def _json(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    if hasattr(value, "tolist"):
        return _json(value.tolist())
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def _write(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(_json(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _float_attr(element: ET.Element | None, key: str, default: float | None = None) -> float | None:
    if element is None or element.get(key) is None:
        return default
    try:
        return float(element.get(key, ""))
    except (TypeError, ValueError):
        return default


def _urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    joints: dict[str, dict[str, Any]] = {}
    mimic: list[dict[str, Any]] = []
    for element in root.findall("joint"):
        name = element.get("name")
        if not name:
            continue
        lim = element.find("limit")
        item: dict[str, Any] = {
            "name": name,
            "type": element.get("type"),
            "limit": {key: _float_attr(lim, key) for key in ("lower", "upper", "effort", "velocity")},
        }
        me = element.find("mimic")
        if me is not None:
            item["mimic"] = {
                "joint": me.get("joint"),
                "multiplier": _float_attr(me, "multiplier", 1.0),
                "offset": _float_attr(me, "offset", 0.0),
            }
            mimic.append(item)
        joints[name] = item
    if not joints:
        raise ValueError(f"URDF has no named joints: {path}")
    return {"robot_name": root.get("name"), "joints": joints, "mimic": mimic}


def _variants(name: str, side: str) -> list[str]:
    values: list[str] = []
    for value in (name, name.split("__", 1)[-1]):
        if value and value not in values:
            values.append(value)
    if "_hand__" in name:
        value = name.split("_hand__", 1)[-1]
        if value not in values:
            values.append(value)
    if name.startswith(side + "_arm__"):
        value = name.split("__", 1)[-1]
        if value not in values:
            values.append(value)
    return values


def _resolve(name: str, dofs: list[str], side: str) -> tuple[str | None, str]:
    for value in _variants(name, side):
        exact = [item for item in dofs if item == value]
        if len(exact) == 1:
            return exact[0], "exact"
        suffix = [item for item in dofs if item.endswith(value)]
        if len(suffix) == 1:
            return suffix[0], "suffix"
    return None, "unresolved"


def _roots(stage: Any, side: str, UsdPhysics: Any) -> list[str]:
    paths = [str(prim.GetPath()) for prim in stage.TraverseAll() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(paths) > 1:
        selected = [path for path in paths if side in path.lower()]
        if len(selected) == 1:
            return selected
    return paths


def _action(ArticulationAction: Any, indices: list[int], values: list[float]) -> Any:
    import torch

    return ArticulationAction(
        joint_positions=torch.tensor(values, dtype=torch.float32),
        joint_velocities=None,
        joint_efforts=None,
        joint_indices=torch.tensor(indices, dtype=torch.int64),
    )


def _close_writers(writers: dict[str, Any]) -> dict[str, str]:
    status: dict[str, str] = {}
    for name, writer in writers.items():
        try:
            writer.release()
            status[name] = "closed"
        except Exception as exc:
            status[name] = repr(exc)
    return status


def _camera_setup(stage: Any, Camera: Any, UsdGeom: Any, out: Path) -> tuple[list[tuple[str, Any]], dict[str, Any]]:
    cameras: list[tuple[str, Any]] = []
    inventory = [str(prim.GetPath()) for prim in stage.TraverseAll() if prim.IsA(UsdGeom.Camera)]
    for index, path in enumerate(inventory[:2]):
        name = "main" if index == 0 else "hand_close"
        try:
            camera = Camera(path, resolution=(320, 240), name="finger_response_" + name)
            camera.initialize()
            cameras.append((name, camera))
        except Exception:
            continue
    return cameras, {"camera_prims": inventory, "streams": [name for name, _ in cameras], "png_root": "rgb"}


def _capture(cameras: list[tuple[str, Any]], out: Path, index: int, writers: dict[str, Any], records: list[dict[str, Any]]) -> None:
    if not cameras:
        return
    import numpy as np
    from PIL import Image
    frame: dict[str, Any] = {"index": index, "streams": {}}
    for name, camera in cameras:
        data = camera.get_rgba()
        if data is None:
            continue
        rgb = np.asarray(data)[..., :3].astype(np.uint8)
        if rgb.ndim != 3 or rgb.shape[-1] != 3:
            continue
        directory = out / "rgb" / name
        directory.mkdir(parents=True, exist_ok=True)
        image_path = directory / f"frame_{index:06d}.png"
        Image.fromarray(rgb).save(image_path)
        if name in writers:
            try:
                import cv2
                writers[name].write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            except Exception:
                pass
        frame["streams"][name] = {"png": str(image_path.relative_to(out)), "shape": list(rgb.shape)}
    if frame["streams"]:
        records.append(frame)


def run(args: argparse.Namespace, report: dict[str, Any]) -> int:
    """Start Kit, initialize PhysX, apply targets, and write the response trace."""
    import numpy as np
    import torch
    from isaacsim import SimulationApp

    nas_path = Path(sys.executable).parent.parent / "sim2data_nas.json"
    nas = json.loads(nas_path.read_text(encoding="utf-8")) if nas_path.is_file() else None
    if nas:
        for key in ("MDL_SYSTEM_PATH", "MDL_USER_PATH"):
            old = os.environ.get(key, "")
            os.environ[key] = os.pathsep.join(nas["mdl_paths"] + ([old] if old else []))
        report["runtime"]["nas_profile_loaded"] = True
    sys.argv = [sys.argv[0], "--portable-root", str(args.out / "kit")]
    if nas:
        sys.argv.append("--/persistent/isaac/asset_root/default=" + nas["asset_root"])
    sys.argv.extend(["--" + args.graphics_api, "--/app/vulkan=" + ("false" if args.graphics_api == "d3d12" else "true")])
    app = None
    initialized = False
    try:
        app = SimulationApp({
            "headless": True, "width": 320, "height": 240, "renderer": "RaytracedLighting",
            "anti_aliasing": 0, "multi_gpu": False, "active_gpu": 0, "max_gpu_count": 1,
            "limit_cpu_threads": 4, "fast_shutdown": False,
            "extra_args": ["--/app/extensions/registryEnabled=false", "--/app/extensions/syncRegistryOnStartup=false",
                            f"--/log/file={args.out / 'kit.log'}"],
        })
        report["runtime"].update({"simulation_app_started": True, "graphics_api": args.graphics_api})
        import carb
        import omni.usd
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import UsdGeom, UsdPhysics, Usd

        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        carb.settings.get_settings().set_bool("/isaaclab/render/active_viewport", True)
        context = omni.usd.get_context()
        context.open_stage(str(args.usd.absolute()))
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("USD stage failed to open")
        stage.SetEditTarget(stage.GetSessionLayer())
        report["stage_inventory"] = {
            "colliders": [str(p.GetPath()) for p in stage.TraverseAll()
                          if p.HasAPI(UsdPhysics.CollisionAPI)]}
        root_paths = _roots(stage, args.side, UsdPhysics)
        report["stage"] = {"articulation_root_paths": root_paths, "usd_sha256": _sha(args.usd)}
        if len(root_paths) != 1:
            raise RuntimeError(f"expected one side articulation root, found {root_paths}")
        root_path = root_paths[0]
        articulation = SingleArticulation(root_path, name="finger_response_" + args.side)
        simulation = SimulationContext(SimulationCfg(dt=args.dt, device="cpu", use_fabric=False))
        simulation.reset()
        articulation.initialize()
        if not articulation.handles_initialized:
            raise RuntimeError("SingleArticulation handles did not initialize")
        initialized = True
        dofs = [str(name) for name in articulation.dof_names]
        if not dofs:
            raise RuntimeError("articulation has zero DOFs")
        report["physics"].update({"root_path": root_path, "num_dof": articulation.num_dof,
                                   "num_bodies": articulation.num_bodies, "dof_names": dofs,
                                   "simulation_context": "isaaclab.sim.SimulationContext", "device": "cpu",
                                   "dt": args.dt, "articulation_initialized": True})

        profile = json.loads(args.profile.read_text(encoding="utf-8"))
        hand = profile["gripper_commissioning"][args.side]
        drive = hand["synthetic_drive"]
        active_profile = hand["active_joints"]
        source = _urdf(args.urdf)
        active: list[dict[str, Any]] = []
        unresolved: list[str] = []
        for item in active_profile:
            resolved, strategy = _resolve(str(item["name"]), dofs, args.side)
            if resolved is None:
                unresolved.append(str(item["name"]))
            else:
                active.append({"profile_name": item["name"], "dof_name": resolved, "strategy": strategy,
                               "open_rad": float(item["open_rad"]), "close_rad": float(item["close_rad"])})
        if unresolved:
            raise RuntimeError("unresolved active hand DOFs: " + repr(unresolved))
        active_names = [item["dof_name"] for item in active]
        active_indices = [dofs.index(name) for name in active_names]
        active_set = set(active_names)
        passive: list[dict[str, Any]] = []
        passive_set: set[str] = set()
        for item in source["mimic"]:
            source_name = str(item["name"])
            parent_name = str(item["mimic"]["joint"])
            mimic_name, m_strategy = _resolve(source_name, dofs, args.side)
            parent_dof, p_strategy = _resolve(parent_name, dofs, args.side)
            if mimic_name is None or parent_dof is None:
                continue
            passive_set.add(mimic_name)
            passive.append({"urdf_name": source_name, "dof_name": mimic_name,
                            "source_urdf_name": parent_name, "source_dof_name": parent_dof,
                            "multiplier": float(item["mimic"].get("multiplier", 1.0)),
                            "offset": float(item["mimic"].get("offset", 0.0)),
                            "name_resolution": {"mimic": m_strategy, "source": p_strategy},
                            "commanded": False})
        if active_set & passive_set:
            raise RuntimeError("profile active DOF overlaps URDF mimic DOF")
        # PhysX equation: q_mimic + gearing*q_reference + offset = 0.
        # USD angular offsets are degrees; articulation readback uses radians.
        for item in passive:
            matches = [p for p in stage.TraverseAll() if p.GetName() == item["dof_name"]
                       and p.IsA(UsdPhysics.Joint)]
            if len(matches) != 1:
                raise RuntimeError("ambiguous USD mimic joint: " + item["dof_name"])
            prim = matches[0]
            relationships = [r for r in prim.GetRelationships()
                             if r.GetName().startswith("physxMimicJoint:") and r.GetName().endswith(":referenceJoint")]
            if len(relationships) != 1 or len(relationships[0].GetTargets()) != 1:
                raise RuntimeError("missing/ambiguous USD mimic relationship: " + item["dof_name"])
            rel = relationships[0]
            prefix = rel.GetName().rsplit(":", 1)[0]
            target = rel.GetTargets()[0]
            if str(target).rsplit("/", 1)[-1] != item["source_dof_name"]:
                raise RuntimeError("USD/URDF mimic reference mismatch")
            gearing = prim.GetAttribute(prefix + ":gearing").Get()
            offset = prim.GetAttribute(prefix + ":offset").Get()
            if gearing is None or offset is None:
                raise RuntimeError("USD mimic parameters missing")
            item.update(usd_relationship=str(rel.GetPath()), usd_reference=str(target),
                        usd_gearing=float(gearing), usd_offset_degrees=float(offset),
                        usd_reference_axis=str(prim.GetAttribute(prefix + ":referenceJointAxis").Get()),
                        usd_mimic_axis=prefix.split(":")[-1])
            item["multiplier"] = -float(gearing)
            item["offset"] = -math.radians(float(offset))
        if len(passive) != len(source["mimic"]):
            raise RuntimeError("not all source mimic joints resolved")
        arm_names = [f"{args.side}_arm__joint{i}" for i in range(1, 7)]
        arm_names_resolved: list[str] = []
        for name in arm_names:
            resolved, _ = _resolve(name, dofs, args.side)
            if resolved and resolved not in active_set and resolved not in passive_set and resolved not in arm_names_resolved:
                arm_names_resolved.append(resolved)
        arm_indices = [dofs.index(name) for name in arm_names_resolved]
        report["mapping"] = {"active_hand": active, "active_indices": active_indices,
                              "mimic_passive_read_only": passive, "arm_hold_names": arm_names_resolved,
                              "arm_hold_indices": arm_indices, "unresolved_active": unresolved}
        props = articulation.dof_properties
        report["physics"]["dof_limits_and_drives_before"] = {
            name: {"lower": float(props[i]["lower"]), "upper": float(props[i]["upper"]),
                   "has_limits": bool(props[i]["hasLimits"]), "max_velocity": float(props[i]["maxVelocity"]),
                   "max_effort": float(props[i]["maxEffort"]), "stiffness": float(props[i]["stiffness"]),
                   "damping": float(props[i]["damping"]), "drive_mode": int(props[i]["driveMode"])}
            for i, name in enumerate(dofs)
        }

        controller = articulation.get_articulation_controller()
        try:
            kps, kds = controller.get_gains()
            kps, kds = np.asarray(kps, dtype=float).reshape(-1), np.asarray(kds, dtype=float).reshape(-1)
        except Exception:
            kps, kds = np.zeros(articulation.num_dof), np.zeros(articulation.num_dof)
        if kps.size != articulation.num_dof:
            kps = np.resize(kps, articulation.num_dof)
        if kds.size != articulation.num_dof:
            kds = np.resize(kds, articulation.num_dof)
        for index in arm_indices:
            kps[index], kds[index] = ARM_KP, ARM_KD
        for index in active_indices:
            kps[index], kds[index] = float(drive["stiffness"]), float(drive["damping"])
        controller.set_gains(kps=torch.tensor(kps, dtype=torch.float32), kds=torch.tensor(kds, dtype=torch.float32))
        try:
            efforts = np.asarray(controller.get_max_efforts(), dtype=float).reshape(-1)
        except Exception:
            efforts = np.full(articulation.num_dof, ARM_EFFORT)
        if efforts.size != articulation.num_dof:
            efforts = np.resize(efforts, articulation.num_dof)
        efforts[arm_indices] = ARM_EFFORT
        efforts[active_indices] = float(drive["max_force"])
        controller.set_max_efforts(efforts.tolist())
        velocity_status = "unsupported"
        view = getattr(articulation, "_articulation_view", None)
        if view is not None and hasattr(view, "set_max_joint_velocities"):
            try:
                velocities = np.asarray(view.get_joint_max_velocities(), dtype=float).reshape(-1)
                if velocities.size != articulation.num_dof:
                    velocities = np.resize(velocities, articulation.num_dof)
                velocities[arm_indices] = ARM_VELOCITY
                velocities[active_indices] = float(drive["max_velocity_rad_s"])
                view.set_max_joint_velocities(values=torch.tensor(velocities[None, :], dtype=torch.float32))
                velocity_status = "applied_via_ArticulationView"
            except Exception as exc:
                velocity_status = "failed:" + repr(exc)
        report["synthetic_drive"] = {"hand": {key: drive[key] for key in ("stiffness", "damping", "max_force", "max_velocity_rad_s")},
                                     "arm_hold": {"stiffness": ARM_KP, "damping": ARM_KD, "max_effort": ARM_EFFORT,
                                                   "max_velocity_rad_s": ARM_VELOCITY, "assumption": "hold reset pose"},
                                     "max_velocity_application": velocity_status, "synthetic": True}
        report["physics"]["gains_readback"] = _json(controller.get_gains())
        report["physics"]["max_efforts_readback"] = _json(controller.get_max_efforts())

        q0 = np.asarray(articulation.get_joint_positions(), dtype=float).reshape(-1)
        report["physics"]["initial_q"] = q0.tolist()
        report["physics"]["initial_qd"] = np.asarray(articulation.get_joint_velocities(), dtype=float).reshape(-1).tolist()
        clipped: list[dict[str, Any]] = []
        open_targets: list[float] = []
        close_targets: list[float] = []
        for item, index in zip(active, active_indices):
            low, high = float(props[index]["lower"]), float(props[index]["upper"])
            bounded = bool(props[index]["hasLimits"])
            open_value, close_value = float(item["open_rad"]), float(item["close_rad"])
            if bounded:
                open_target, close_target = np.clip([open_value, close_value], low, high)
            else:
                open_target, close_target = open_value, close_value
            open_targets.append(float(open_target)); close_targets.append(float(close_target))
            clipped.append({"dof_name": item["dof_name"], "open_requested": open_value, "open_target": float(open_target),
                            "close_requested": close_value, "close_target": float(close_target),
                            "clipped": bool(open_target != open_value or close_target != close_value)})
        report["mapping"]["target_clipping"] = clipped

        # A reset pose is read once and then held through targets.  There is no
        # set_joint_positions/set_joint_velocities call in this implementation.
        command_indices = arm_indices + active_indices
        command_log = args.out / "joint_trace.jsonl"
        cameras: list[tuple[str, Any]] = []
        writers: dict[str, Any] = {}
        rgb_records: list[dict[str, Any]] = []
        try:
            from isaacsim.sensors.camera import Camera
            cameras, rgb_report = _camera_setup(stage, Camera, UsdGeom, args.out)
            if cameras:
                try:
                    import cv2
                    for name, _ in cameras:
                        writer = cv2.VideoWriter(str(args.out / f"{name}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                                 30.0, (320, 240))
                        if writer.isOpened():
                            writers[name] = writer
                except Exception:
                    pass
            rgb_report["mp4_streams"] = list(writers)
            report["rgb_capture"] = rgb_report
        except Exception as exc:
            report["rgb_capture"] = {"camera_prims": [], "streams": [], "mp4_streams": [], "error": repr(exc)}

        frames: list[dict[str, Any]] = []
        with command_log.open("w", encoding="utf-8") as stream:
            frame_index = 0
            for phase_index, amount in enumerate(AMOUNTS):
                hand_targets = [open_value + amount * (close_value - open_value)
                                for open_value, close_value in zip(open_targets, close_targets)]
                targets = [float(q0[index]) for index in arm_indices] + hand_targets
                for _ in range(args.steps_per_amount):
                    articulation.apply_action(_action(ArticulationAction, command_indices, targets))
                    simulation.step(render=bool(cameras))
                    q = np.asarray(articulation.get_joint_positions(), dtype=float).reshape(-1)
                    qd = np.asarray(articulation.get_joint_velocities(), dtype=float).reshape(-1)
                    try:
                        effort = np.asarray(articulation.get_measured_joint_efforts(), dtype=float).reshape(-1).tolist()
                    except Exception:
                        effort = None
                    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
                        raise RuntimeError("non-finite joint state")
                    frame = {"frame": frame_index, "phase_index": phase_index, "time_s": (frame_index + 1) * args.dt, "amount": amount,
                             "commanded": {dofs[index]: float(value) for index, value in zip(command_indices, targets)},
                             "q": q.tolist(), "qd": qd.tolist(), "measured_effort": effort,
                             "mimic_q": {item["dof_name"]: float(q[dofs.index(item["dof_name"])]) for item in passive}}
                    stream.write(json.dumps(_json(frame), separators=(",", ":")) + "\n")
                    frames.append(frame)
                    if cameras and frame_index % args.rgb_stride == 0:
                        _capture(cameras, args.out, frame_index, writers, rgb_records)
                    frame_index += 1
        report["rgb_capture"]["frames"] = len(rgb_records)
        report["rgb_capture"]["writer_close"] = _close_writers(writers)

        stages: list[dict[str, Any]] = []
        for phase_index, amount in enumerate(AMOUNTS):
            tail = [item for item in frames if item["phase_index"] == phase_index][-args.settle_window:]
            if not tail:
                continue
            errors: dict[str, float] = {}
            for index, item in zip(active_indices, active):
                pos = active.index(item)
                target = open_targets[pos] + amount * (close_targets[pos] - open_targets[pos])
                errors[item["dof_name"]] = float(max(abs(row["q"][index] - target) for row in tail))
            arm_drift = max((max(abs(row["q"][index] - q0[index]) for row in tail) for index in arm_indices), default=0.0)
            stages.append({"phase_index": phase_index, "amount": amount, "samples": len(tail), "hand_max_abs_error_rad": max(errors.values(), default=0.0),
                           "hand_joint_max_abs_error_rad": errors, "arm_max_abs_drift_rad": float(arm_drift)})
        mimic_summary: list[dict[str, Any]] = []
        for item in passive:
            mi, si = dofs.index(item["dof_name"]), dofs.index(item["source_dof_name"])
            residual = np.asarray([row["q"][mi] - item["multiplier"] * row["q"][si] - item["offset"] for row in frames], dtype=float)
            mimic_summary.append({"dof_name": item["dof_name"], "source_dof_name": item["source_dof_name"],
                                  "multiplier": item["multiplier"], "offset": item["offset"],
                                  "max_abs_residual_rad": float(np.max(np.abs(residual))) if residual.size else None,
                                  "mean_abs_residual_rad": float(np.mean(np.abs(residual))) if residual.size else None,
                                  "samples": int(residual.size)})
        report["response"] = {"amounts": list(AMOUNTS), "stages": stages, "frames": len(frames),
                               "mimic_readback": mimic_summary, "trace": str(command_log.relative_to(args.out)),
                               "direct_joint_state_writes": 0,
                               "targets_sent_via": "ArticulationAction -> SingleArticulation.apply_action"}
        report["physics"].update({"physx_articulation_validated": True, "physics_steps": len(frames), "contact_validated": False})
        report["gates"] = {"real_articulation": True, "finger_target_response": bool(frames), "contact": False,
                           "grasp": False, "production_collection_allowed": False}
        changing = [(index, close_targets[pos] - open_targets[pos]) for pos, index in enumerate(active_indices)
                    if abs(active[pos]["close_rad"] - active[pos]["open_rad"]) > 0.01]
        movement = []
        for index, span in changing:
            means = [float(np.mean([row["q"][index] for row in frames if row["phase_index"] == phase][-args.settle_window:]))
                     for phase in (0, 2, 3)]
            movement.append({"dof_name": dofs[index], "target_span": span, "phase_means": means,
                             "passed": abs(span) >= 0.02 and (means[1]-means[0])*np.sign(span) >= abs(span)*0.5
                             and (means[1]-means[2])*np.sign(span) >= abs(span)*0.5})
        report["response"]["movement"] = movement
        report["passed"] = response_passed(stages, movement, mimic_summary, frames, args.max_response_error)
        report["gates"]["finger_target_response"] = report["passed"]
        report["gates"]["contact_trial_allowed"] = report["passed"] and bool(report["stage_inventory"]["colliders"])
        report["phase"] = "completed" if report["passed"] else "response_outside_tolerance"
        _write(args.out / "result.json", report)
        return 0 if report["passed"] else 3
    except Exception:
        report["phase"] = "failed_runtime"
        report["error"] = traceback.format_exc()
        report["physics"]["physx_articulation_validated"] = initialized
        _write(args.out / "result.json", report)
        return 2
    finally:
        report["shutdown"]["physx_process_phase"] = (
            "response_steps_completed" if report.get("response", {}).get("frames") else
            "initialized_without_response_completion" if initialized else "failed_before_articulation")
        report["shutdown"]["kit_close_attempted"] = app is not None
        _write(args.out / "result.json", report)
        if app is not None:
            try:
                app.close(wait_for_replicator=False)
                report["shutdown"]["kit_close_returned"] = True
            except Exception as exc:
                report["shutdown"]["kit_close_returned"] = False
                report["shutdown"]["kit_close_error"] = repr(exc)
            report["shutdown"]["kit_exit_status"] = "close_returned" if report["shutdown"].get("kit_close_returned") else "close_error"
        _write(args.out / "result.json", report)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--graphics-api", choices=("d3d12", "vulkan"), default="d3d12")
    parser.add_argument("--steps-per-amount", type=int, default=DEFAULT_STEPS)
    parser.add_argument("--settle-window", type=int, default=24)
    parser.add_argument("--rgb-stride", type=int, default=8)
    parser.add_argument("--max-response-error", type=float, default=0.15)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    for path, label in ((args.usd, "--usd"), (args.urdf, "--urdf"), (args.profile, "--profile")):
        if not path.is_file():
            raise SystemExit(f"{label} does not exist: {path}")
    if args.steps_per_amount <= 0 or args.settle_window <= 0 or args.rgb_stride <= 0:
        raise SystemExit("steps/window/stride must be positive")
    args.out = args.out.absolute()
    if args.out.exists() and any(args.out.iterdir()):
        raise SystemExit(f"--out must be fresh or empty: {args.out}")
    args.out.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "scope": "physx_finger_response_commissioning", "production_collection_allowed": False,
        "contact_validated": False, "grasp_validated": False, "passed": False, "phase": "starting_kit",
        "inputs": {"usd": str(args.usd.absolute()), "urdf": str(args.urdf.absolute()),
                   "profile": str(args.profile.absolute()), "side": args.side,
                   "usd_sha256": _sha(args.usd), "urdf_sha256": _sha(args.urdf), "profile_sha256": _sha(args.profile)},
        "runtime": {"python": sys.executable, "simulation_app_started": False},
        "physics": {"physx_articulation_validated": False},
        "shutdown": {"physx_process_phase": "not_started", "kit_close_returned": False},
        "assumptions": ["Synthetic commissioning drives only; no measured hardware parameters are inferred.",
                        "Arm DOFs, when present, are held at the simulator reset pose by position targets.",
                        "No object is added: contact, friction, lift and grasp are not validated.",
                        "Source URDF and USD are read-only; no direct set_joint_positions call is used."],
    }
    _write(args.out / "result.json", report)
    try:
        return run(args, report)
    except Exception:
        report["phase"] = "failed_startup"
        report["error"] = traceback.format_exc()
        _write(args.out / "result.json", report)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
