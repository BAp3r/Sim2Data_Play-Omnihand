"""Dependency-free validation for the reviewed asset manifest.

The manifest deliberately contains paths relative to a configured external
asset root.  This module validates the public contract without touching the
NAS or assuming that an external root is mounted.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping


class ManifestError(ValueError):
    """Raised when an asset manifest is structurally unsafe or incomplete."""


def _is_posix_relative(value: Any) -> bool:
    if not isinstance(value, str) or not value or value.startswith("/"):
        return False
    if "\\" in value or value.startswith("~"):
        return False
    parts = value.split("/")
    return ":" not in parts[0] and "" not in parts and ".." not in parts


def _sha256(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return True


def _add(errors: list[str], condition: bool, field: str) -> None:
    if not condition:
        errors.append(field)


def validate_manifest(manifest: Mapping[str, Any]) -> list[str]:
    """Return field names that violate the asset manifest contract.

    Validation is intentionally strict about identity, paths and channel
    counts, while leaving physical acceptance gates explicit.  A ``pending``
    status is valid and is not silently treated as production-ready.
    """

    errors: list[str] = []
    if not isinstance(manifest, Mapping):
        return ["manifest"]

    _add(errors, manifest.get("schema_version") == 1, "schema_version")
    _add(errors, manifest.get("production_collection_allowed") is False,
         "production_collection_allowed")

    assets = manifest.get("assets")
    _add(errors, isinstance(assets, Mapping), "assets")
    card = assets.get("card_box") if isinstance(assets, Mapping) else None
    _add(errors, isinstance(card, Mapping), "assets.card_box")
    if isinstance(card, Mapping):
        _add(errors, _is_posix_relative(card.get("relative_path")),
             "assets.card_box.relative_path")
        _add(errors, _sha256(card.get("sha256")), "assets.card_box.sha256")
        usd = card.get("usd")
        _add(errors, isinstance(usd, Mapping), "assets.card_box.usd")
        if isinstance(usd, Mapping):
            _add(errors, usd.get("meters_per_unit") == 1.0,
                 "assets.card_box.usd.meters_per_unit")
            _add(errors, usd.get("up_axis") == "Z",
                 "assets.card_box.usd.up_axis")
            _add(errors, usd.get("mesh_prim") == "/Root/SM_CardBoxA_01",
                 "assets.card_box.usd.mesh_prim")
            bounds = usd.get("world_bounds_m")
            _add(errors, isinstance(bounds, list) and len(bounds) == 3 and
                 all(isinstance(x, (int, float)) and x > 0 for x in bounds),
                 "assets.card_box.usd.world_bounds_m")
        collision = card.get("collision")
        _add(errors, isinstance(collision, Mapping), "assets.card_box.collision")
        if isinstance(collision, Mapping):
            _add(errors, collision.get("physics_collision_api") is True,
                 "assets.card_box.collision.physics_collision_api")
            _add(errors, collision.get("physics_mesh_collision_api") is True,
                 "assets.card_box.collision.physics_mesh_collision_api")
            _add(errors, bool(collision.get("approximation")),
                 "assets.card_box.collision.approximation")
        rigid = card.get("rigid_body")
        _add(errors, isinstance(rigid, Mapping), "assets.card_box.rigid_body")
        if isinstance(rigid, Mapping):
            _add(errors, rigid.get("mass_kg") is None,
                 "assets.card_box.rigid_body.mass_kg_is_pending")
            _add(errors, rigid.get("inertia_kg_m2") is None,
                 "assets.card_box.rigid_body.inertia_kg_m2_is_pending")
        dependencies = card.get("material_dependencies")
        _add(errors, isinstance(dependencies, list) and bool(dependencies),
             "assets.card_box.material_dependencies")
        if isinstance(dependencies, list):
            for index, dependency in enumerate(dependencies):
                field = f"assets.card_box.material_dependencies[{index}]"
                _add(errors, isinstance(dependency, Mapping), field)
                if isinstance(dependency, Mapping):
                    _add(errors, _is_posix_relative(dependency.get("relative_path")),
                         field + ".relative_path")
                    _add(errors, dependency.get("present_on_audit") is True,
                         field + ".present_on_audit")
                    if dependency.get("sha256") is not None:
                        _add(errors, _sha256(dependency.get("sha256")),
                             field + ".sha256")

    candidates = manifest.get("robot_candidates")
    _add(errors, isinstance(candidates, list) and len(candidates) == 2,
         "robot_candidates")
    if isinstance(candidates, list):
        for index, candidate in enumerate(candidates):
            field = f"robot_candidates[{index}]"
            _add(errors, isinstance(candidate, Mapping), field)
            if not isinstance(candidate, Mapping):
                continue
            source = candidate.get("source")
            urdf = candidate.get("urdf")
            control = candidate.get("arm_control")
            boundary = candidate.get("original_end_effector_boundary")
            for name, value in (("source", source), ("urdf", urdf),
                                ("arm_control", control),
                                ("original_end_effector_boundary", boundary)):
                _add(errors, isinstance(value, Mapping), f"{field}.{name}")
            if isinstance(source, Mapping):
                _add(errors, _sha256(urdf.get("sha256")) if isinstance(urdf, Mapping)
                     else False, field + ".urdf.sha256")
                _add(errors, isinstance(source.get("commit"), str) and
                     len(source["commit"]) == 40, field + ".source.commit")
            if isinstance(control, Mapping):
                names = control.get("active_joint_names")
                limits = control.get("joint_limits")
                _add(errors, isinstance(names, list) and len(names) == 6,
                     field + ".arm_control.active_joint_names")
                _add(errors, isinstance(limits, list) and len(limits) == 6,
                     field + ".arm_control.joint_limits")
            if isinstance(boundary, Mapping):
                _add(errors, boundary.get("parent_link") == "link6",
                     field + ".original_end_effector_boundary.parent_link")

    hands = manifest.get("hand_candidates")
    _add(errors, isinstance(hands, list) and len(hands) >= 2, "hand_candidates")
    o10 = None
    if isinstance(hands, list):
        o10 = next((item for item in hands
                    if isinstance(item, Mapping) and item.get("id") ==
                    "omnihand_2025_o10"), None)
    _add(errors, isinstance(o10, Mapping), "hand_candidates.omnihand_2025_o10")
    if isinstance(o10, Mapping):
        geometry = o10.get("geometry_snapshot")
        motors = o10.get("active_motor_channels")
        passive = o10.get("passive_mimic_joints")
        sdk = o10.get("sdk_control")
        for name, value in (("geometry_snapshot", geometry),
                            ("active_motor_channels", motors),
                            ("passive_mimic_joints", passive),
                            ("sdk_control", sdk)):
            _add(errors, isinstance(value, Mapping),
                 "hand_candidates.omnihand_2025_o10." + name)
        if isinstance(geometry, Mapping):
            _add(errors, geometry.get("active_motor_count") == 10,
                 "hand_candidates.omnihand_2025_o10.geometry_snapshot.active_motor_count")
            _add(errors, geometry.get("passive_mimic_count") == 6,
                 "hand_candidates.omnihand_2025_o10.geometry_snapshot.passive_mimic_count")
            _add(errors, geometry.get("nonfixed_physical_joint_count") == 16,
                 "hand_candidates.omnihand_2025_o10.geometry_snapshot.nonfixed_physical_joint_count")
            _add(errors, _sha256(geometry.get("left_urdf_sha256")),
                 "hand_candidates.omnihand_2025_o10.geometry_snapshot.left_urdf_sha256")
            _add(errors, _sha256(geometry.get("right_urdf_sha256")),
                 "hand_candidates.omnihand_2025_o10.geometry_snapshot.right_urdf_sha256")
        if isinstance(motors, Mapping):
            _add(errors, all(isinstance(motors.get(side), list) and
                             len(motors[side]) == 10 for side in ("left", "right")),
                 "hand_candidates.omnihand_2025_o10.active_motor_channels")
        if isinstance(passive, Mapping):
            _add(errors, all(isinstance(passive.get(side), list) and
                             len(passive[side]) == 6 for side in ("left", "right")),
                 "hand_candidates.omnihand_2025_o10.passive_mimic_joints")
        if isinstance(sdk, Mapping):
            _add(errors, sdk.get("command_channel_count") == 10,
                 "hand_candidates.omnihand_2025_o10.sdk_control.command_channel_count")
            _add(errors, sdk.get("state_channel_count_before_tactile") == 16,
                 "hand_candidates.omnihand_2025_o10.sdk_control.state_channel_count_before_tactile")

    # Absolute machine paths and private host hints do not belong in the public
    # manifest.  URLs are allowed, so only check path-shaped strings here.
    def scan(value: Any, path: str = "manifest") -> None:
        if isinstance(value, Mapping):
            for key, item in value.items():
                scan(item, f"{path}.{key}")
        elif isinstance(value, list):
            for index, item in enumerate(value):
                scan(item, f"{path}[{index}]")
        elif isinstance(value, str):
            lowered = value.lower()
            if (value.startswith("\\\\") or
                    (len(value) >= 3 and value[1] == ":" and
                     value[2] in "\\/") or
                    lowered.startswith(("/mnt/", "/workspace/", "/opt/")) or
                    "192.168." in lowered):
                errors.append(path + ".public_path")

    scan(manifest)
    return sorted(set(errors))


def load_manifest(path: str | Path | None = None) -> dict[str, Any]:
    """Load and validate the repository asset manifest."""

    manifest_path = Path(path) if path is not None else (
        Path(__file__).resolve().parents[2] / "configs" / "asset_manifest.json"
    )
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read asset manifest: {manifest_path}") from exc
    errors = validate_manifest(data)
    if errors:
        raise ManifestError("invalid asset manifest fields: " + ", ".join(errors))
    return data
