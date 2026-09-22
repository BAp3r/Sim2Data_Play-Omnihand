"""Assemble private per-side URDFs from explicit model bindings and debug poses.

Source paths are supplied in a private JSON binding, never inferred or copied
from machine-specific defaults. Generated URDFs contain absolute private mesh
paths and must not be committed. This tool does not import URDF into PhysX.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from sim2data.backends.isaaclab.commissioning import (
    AssemblyTransforms, MeshVisual, SideCommissioningSpec, Transform,
    write_combined_urdfs,
)


def run(profile_path: Path, binding_path: Path, output: Path):
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    binding = json.loads(binding_path.read_text(encoding="utf-8"))
    if profile.get("purpose") != "synthetic_commissioning_only" or profile.get("production_collection_enabled") is not False:
        raise ValueError("Explicit synthetic profile with production disabled required")
    if output.exists():
        raise FileExistsError("Output directory must be new")
    specs = []
    identities = {}
    for side in ("left", "right"):
        source = binding[side]
        design = profile["robots"][side]
        expected_arm = profile.get("arm_model", {}).get("urdf_sha256")
        if expected_arm and hashlib.sha256(Path(source["arm_urdf"]).read_bytes()).hexdigest() != expected_arm:
            raise ValueError(f"{side} arm source does not match the selected profile identity")
        hand_sha = hashlib.sha256(Path(source["hand_urdf"]).read_bytes()).hexdigest()
        expected_hand = design.get("hand_model", {}).get("urdf_sha256")
        if expected_hand and hand_sha != expected_hand:
            raise ValueError(f"{side} hand source does not match the selected side-specific identity")
        identities[side] = {"hand_urdf_sha256": hand_sha, "hand_root_link": source["hand_root_link"],
                            "arm_urdf_sha256": hashlib.sha256(Path(source["arm_urdf"]).read_bytes()).hexdigest()}
        mount = source.get("mount_visual")
        camera_visual = source.get("camera_housing_visual")
        specs.append(SideCommissioningSpec(
            side=side, output_robot_name=f"{side}_commissioning",
            namespace_prefix=f"{side}_", arm_urdf=Path(source["arm_urdf"]),
            hand_urdf=Path(source["hand_urdf"]), flange_link=source["flange_link"],
            hand_root_link=source["hand_root_link"],
            transforms=AssemblyTransforms.from_mapping(design),
            package_roots={name: Path(path) for name, path in source["package_roots"].items()},
            mount_visual=None if mount is None else MeshVisual.from_mapping(mount),
            mount_mesh_base=None if mount is None else Path(source["mount_mesh_base"]),
            camera_housing_size_xyz_m=None if camera_visual else tuple(design["camera_proxy_size_xyz_m"]),
            camera_housing_visual=None if camera_visual is None else MeshVisual.from_mapping(camera_visual),
        ))
    paths = write_combined_urdfs(specs, output)
    record = {"scope": "synthetic_static_assembly", "production_collection_allowed": False,
              "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
              "binding_sha256": hashlib.sha256(binding_path.read_bytes()).hexdigest(),
              "source_identities": identities,
              "outputs": [str(path) for path in paths],
              "physics_validated": False, "dataset_export_allowed": False}
    (output / "input_identity.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    (output / "commissioning_profile.json").write_text(json.dumps(profile, indent=2), encoding="utf-8")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--binding", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    run(args.profile, args.binding, args.out)
