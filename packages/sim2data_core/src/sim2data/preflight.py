"""Read-only M0 configuration diagnostics. This is not a physics validator."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from sim2data.core import Timing


TABLE_RELAY_STAGES = (
    "left_pick_from_left_table", "left_place_in_table_relay_zone",
    "left_release_and_retreat", "relay_stability_and_clearance_check",
    "right_regrasp_from_relay_zone", "right_place_in_bin_outside_right_table_edge",
    "right_release_and_retreat", "stable_bin_placement_check",
)
RELAY_CONDITIONS = (
    "same_object_identity", "left_hand_object_contact_released", "object_supported_by_table",
    "object_footprint_inside_relay_region", "object_stable_for_dwell", "left_arm_clear_of_right_approach",
)


def required_paths() -> tuple[str, ...]:
    paths = [
        "table.size_xyz_m",
        "adapter.mass_kg", "adapter.inertia_kg_m2", "adapter.collision_mesh",
        "object.verified_library_relative_path", "object.asset_library_version",
        "object.sha256", "object.nominal_size_xyz_m", "object.nominal_mass_kg",
        "object.collision_validation", "object.license_and_redistribution_review",
        "task.source_region_geometry", "task.relay_region_geometry",
        "task.relay_gate.max_linear_speed_m_s", "task.relay_gate.max_angular_speed_rad_s",
        "task.relay_gate.stable_dwell_s", "task.relay_gate.release_dwell_s",
        "task.relay_gate.minimum_left_arm_clearance_m", "task.relay_gate.timeout_s",
        "task.bin_world_pose",
        "task.bin_inner_size_xyz_m", "task.bin_wall_thickness_m",
        "task.bin_support_height_m", "task.right_arm_bin_reachability_validation",
        "dataset.lerobot_package_version_or_commit",
    ]
    for side in ("left", "right"):
        for field in (
            "arm_revision", "arm_asset", "hand_asset", "hand_variant_and_revision",
            "anatomical_hand_side", "T_world_base", "T_flange_mount", "T_mount_hand_root",
            "T_hand_root_grasp_tcp", "state_channel_names", "command_channel_names",
            "actuator_to_joint_mapping", "control_units_and_limits",
        ):
            paths.append(f"robots.{side}.{field}")
    for camera in ("overhead", "wrist_left", "wrist_right"):
        for field in ("resolution_wh", "intrinsics_K", "distortion_model_and_coefficients", "stream_profiles"):
            paths.append(f"camera_candidates.{camera}.{field}")
        transforms = ("T_parent_optical",) if camera == "overhead" else (
            "T_mount_camera_housing", "T_housing_color_optical",
        )
        paths.extend(f"camera_candidates.{camera}.{transform}" for transform in transforms)
    return tuple(paths)


def _lookup(spec: dict[str, Any], path: str) -> Any:
    value: Any = spec
    for part in path.split("."):
        value = value.get(part) if isinstance(value, dict) else None
    return value


def _contract_conflicts(spec: dict[str, Any]) -> list[str]:
    """Check the fixed design declarations, not arbitrary calibration validity."""
    expected = {
        "world_frame.length_unit": "m", "world_frame.angle_unit": "rad",
        "world_frame.origin": "tabletop_center", "world_frame.x": "operator_right",
        "world_frame.y": "away_from_operator", "world_frame.z": "up",
        "world_frame.quaternion_serialization": "wxyz",
        "task.handoff_mode": "tabletop_release_then_regrasp",
        "task.stages": list(TABLE_RELAY_STAGES),
        "task.relay_gate.required_conditions": list(RELAY_CONDITIONS),
        "task.require_real_contact_grasp": True,
        "task.allow_object_teleport_or_attachment_cheat": False,
        "dataset.format": "LeRobotDataset v3.0",
        "dataset.depth_enabled": False,
        "camera_candidates.overhead.parent_frame": "world",
        "camera_candidates.wrist_left.parent_frame": "left_wrist_mount_assembly",
        "camera_candidates.wrist_right.parent_frame": "right_wrist_mount_assembly",
    }
    conflicts = [path for path, correct in expected.items()
                 if type(_lookup(spec, path)) is not type(correct) or _lookup(spec, path) != correct]
    cameras = spec.get("camera_candidates")
    if not isinstance(cameras, dict) or set(cameras) != {"overhead", "wrist_left", "wrist_right"}:
        conflicts.append("camera_candidates")
    timing = spec.get("timing_initial_test_proposal")
    try:
        if not isinstance(timing, dict):
            raise ValueError("missing timing")
        Timing(timing.get("physics_hz"), timing.get("control_hz"))
        if type(timing.get("rgb_hz")) is not int or timing["rgb_hz"] != timing["control_hz"]:
            raise ValueError("RGB and control rates differ")
    except ValueError:
        conflicts.append("timing_initial_test_proposal")
    return conflicts


def _missing(value: Any) -> bool:
    if value is None or (isinstance(value, (str, list, dict)) and not value):
        return True
    if isinstance(value, str):
        return not value.strip()
    if isinstance(value, float) and not math.isfinite(value):
        return True
    if isinstance(value, dict):
        return any(_missing(v) for v in value.values())
    if isinstance(value, list):
        return any(_missing(v) for v in value)
    return False


def assess_scene(spec: dict[str, Any]) -> dict[str, Any]:
    """Find missing or conflicting declarations, without creating validation evidence.

    Depth extrinsics are optional for the RGB-only baseline. Geometry, units,
    transforms and signatures need the asset/scene validators in subsequent gates.
    """
    if not isinstance(spec, dict):
        raise ValueError("scene root must be a JSON object")
    missing = [path for path in required_paths() if _missing(_lookup(spec, path))]
    conflicts = _contract_conflicts(spec)
    blockers = ["M0_FRAMEWORK_HAS_NO_VALIDATED_PHYSICS_COLLECTOR"]
    if spec.get("schema_version") != 1 or type(spec.get("schema_version")) is not int:
        blockers.append("UNSUPPORTED_SCENE_SCHEMA")
    task = spec.get("task")
    if not isinstance(task, dict) or task.get("name") != "dual_arm_table_relay_to_bin":
        blockers.append("TASK_IS_NOT_CONFIRMED_TABLE_RELAY")
    if spec.get("production_collection_enabled") is not False:
        blockers.append("PRODUCTION_FLAG_MUST_REMAIN_FALSE_AT_M0")
    if missing:
        blockers.append("REQUIRED_PARAMETERS_UNRESOLVED")
    if conflicts:
        blockers.append("DESIGN_CONTRACT_CONFLICT")
    return {
        "report_schema_version": 1,
        "check_scope": "declaration completeness and fixed design contracts; not full calibration, physical or dataset validation",
        "required_declarations_present": not missing,
        "production_collection_allowed": False,
        "blockers": blockers,
        "unresolved_parameters": missing,
        "invalid_contracts": conflicts,
        "next_gates": ["M1 asset/runtime binding", "M2 assembly/calibration", "M3 contact relay", "M4 real-episode official SDK roundtrip"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("configs/scene_spec.draft.json"))
    parser.add_argument("--out", type=Path, help="New report path; existing files are never overwritten")
    args = parser.parse_args()
    raw = args.scene.read_bytes()
    spec = json.loads(raw.decode("utf-8-sig"))
    if not isinstance(spec, dict):
        parser.error("scene root must be a JSON object")
    report = assess_scene(spec)
    report["scene_sha256"] = hashlib.sha256(raw).hexdigest()
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as stream:
            stream.write(payload)
    print(payload, end="")
    return 2  # M0 cannot authorize collection, even with a filled-in draft.


if __name__ == "__main__":
    raise SystemExit(main())
