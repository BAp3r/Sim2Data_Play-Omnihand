"""Convert a reviewed URDF into a PhysX-ready USD with Isaac Lab.

This is a conversion entry point only.  It never records a trajectory, creates a
video, teleports an articulation, or synthesizes a rigid body.  The source URDF
must already contain real link collision geometry and URDF mimic relations.  The
conversion output is kept in a fresh directory and is accompanied by a structured
inventory and source identity report.

Example (run with the pinned Isaac Sim environment):
    python scripts/convert_physx_commissioning.py --urdf <mapped.urdf> --out <fresh-dir>

The process is intentionally fail-closed: validation or Kit/importer failures
write ``conversion_report.json`` when it is safe to do so and return a non-zero
exit code.  No generated USD is accepted as a successful physics import until a
separate runtime articulation/contact test inspects it.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET
from typing import Any


# These are synthetic commissioning values from configs/commissioning.synthetic.json.
# They are converter drive overrides and must not be treated as measured hardware
# parameters.  max_force/max_velocity are recorded as commissioning requirements;
# Isaac Lab 2.3's URDF converter API exposes stiffness/damping but does not expose
# those two limits, so they require a later USD/PhysX drive audit.
SYNTHETIC_DRIVE = {
    "drive_type": "force",
    "target_type": "position",
    "stiffness": 4.0,
    "damping": 0.15,
    "max_force": 1.5,
    "max_velocity_rad_s": 1.0,
    "synthetic": True,
    "source": "configs/commissioning.synthetic.json",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _float_attr(element: ET.Element | None, key: str) -> float | None:
    if element is None or element.get(key) is None:
        return None
    try:
        return float(element.get(key, ""))
    except ValueError:
        return None


def inspect_urdf(path: Path) -> dict[str, Any]:
    """Collect deterministic source facts without modifying the URDF."""
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root must be <robot>, got <{root.tag}>")
    links: list[dict[str, Any]] = []
    for link in root.findall("link"):
        visuals = link.findall("visual")
        collisions = link.findall("collision")
        links.append(
            {
                "name": link.get("name"),
                "visual_count": len(visuals),
                "collision_count": len(collisions),
                "collision_geometries": [
                    next(iter((geometry or [])), None).tag
                    if (geometry := collision.find("geometry")) is not None
                    and len(geometry)
                    else None
                    for collision in collisions
                ],
            }
        )
    joints: list[dict[str, Any]] = []
    mimic: list[dict[str, Any]] = []
    active = 0
    fixed = 0
    for joint in root.findall("joint"):
        kind = joint.get("type", "")
        parent = joint.find("parent")
        child = joint.find("child")
        limit = joint.find("limit")
        record: dict[str, Any] = {
            "name": joint.get("name"),
            "type": kind,
            "parent": None if parent is None else parent.get("link"),
            "child": None if child is None else child.get("link"),
            "limit": {
                key: _float_attr(limit, key)
                for key in ("lower", "upper", "effort", "velocity")
            }
            if limit is not None
            else None,
        }
        mimic_element = joint.find("mimic")
        if mimic_element is not None:
            record["mimic"] = {
                "joint": mimic_element.get("joint"),
                "multiplier": _float_attr(mimic_element, "multiplier"),
                "offset": _float_attr(mimic_element, "offset"),
            }
            mimic.append(record.copy())
        joints.append(record)
        if kind == "fixed":
            fixed += 1
        elif kind:
            active += 1
    collision_count = sum(item["collision_count"] for item in links)
    if collision_count == 0:
        raise ValueError(
            "source URDF has no <collision> geometry; refusing visual-only conversion"
        )
    if not joints:
        raise ValueError("source URDF has no joints; a commissioning articulation is required")
    return {
        "robot_name": root.get("name"),
        "link_count": len(links),
        "joint_count": len(joints),
        "active_joint_count": active,
        "fixed_joint_count": fixed,
        "mimic_joint_count": len(mimic),
        "mimic_joints": mimic,
        "collision_geometry_count": collision_count,
        "visual_geometry_count": sum(item["visual_count"] for item in links),
        "links": links,
        "joints": joints,
    }


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _nas_profile() -> dict[str, Any] | None:
    """Load the process-local NAS profile used by the existing Isaac smoke scripts."""
    candidate = Path(sys.executable).parent.parent / "sim2data_nas.json"
    if not candidate.is_file():
        return None
    return json.loads(candidate.read_text(encoding="utf-8"))


def _start_kit(out: Path, graphics_api: str):
    """Start the pinned standalone Kit process using the existing smoke conventions."""
    nas = _nas_profile()
    if nas:
        for key in ("MDL_SYSTEM_PATH", "MDL_USER_PATH"):
            old = os.environ.get(key, "")
            os.environ[key] = os.pathsep.join(nas["mdl_paths"] + ([old] if old else []))
    from isaacsim import SimulationApp  # type: ignore

    sys.argv = [sys.argv[0], "--portable-root", str(out / "kit")]
    if nas:
        sys.argv.append("--/persistent/isaac/asset_root/default=" + nas["asset_root"])
    sys.argv.extend(["--" + graphics_api, "--/app/vulkan=" + ("false" if graphics_api == "d3d12" else "true")])
    app = SimulationApp(
        {
            "headless": True,
            "width": 320,
            "height": 240,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 0,
            "multi_gpu": False,
            "active_gpu": 0,
            "max_gpu_count": 1,
            "limit_cpu_threads": 4,
            "fast_shutdown": False,
            "extra_args": [
                "--/app/extensions/registryEnabled=false",
                "--/app/extensions/syncRegistryOnStartup=false",
                f"--/log/file={out / 'kit.log'}",
            ],
        }
    )
    return app, nas


def _make_cfg(urdf: Path, out: Path):
    """Build the Isaac Lab 2.3 UrdfConverterCfg with explicit safety choices."""
    from isaaclab.sim.converters import UrdfConverterCfg  # type: ignore

    return UrdfConverterCfg(
        asset_path=str(urdf),
        usd_dir=str(out / "usd"),
        fix_base=True,
        merge_fixed_joints=False,
        convert_mimic_joints_to_normal_joints=True,
        collision_from_visuals=False,
        collider_type="convex_hull",
        self_collision=False,
        force_usd_conversion=True,
        make_instanceable=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=SYNTHETIC_DRIVE["stiffness"],
                damping=SYNTHETIC_DRIVE["damping"],
            ),
        ),
    )


def inspect_usd(path: Path) -> dict[str, Any]:
    """Inventory the generated USD without claiming runtime PhysX validation."""
    from pxr import Usd, UsdGeom, UsdPhysics  # type: ignore

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"generated USD could not be opened: {path}")
    rigid: list[str] = []
    colliders: list[str] = []
    articulations: list[str] = []
    joints: list[str] = []
    drives: list[str] = []
    mimic_properties: list[str] = []
    for prim in stage.TraverseAll():
        path_text = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid.append(path_text)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append(path_text)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulations.append(path_text)
        if prim.IsA(UsdPhysics.Joint):
            joints.append(path_text)
        for attr in prim.GetAttributes():
            name = attr.GetName().lower()
            if "drive" in name:
                drives.append(f"{path_text}:{attr.GetName()}")
            if "mimic" in name:
                mimic_properties.append(f"{path_text}:{attr.GetName()}")
    return {
        "usd_path": str(path),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "prim_count": sum(1 for _ in stage.TraverseAll()),
        "rigid_body_paths": rigid,
        "collider_paths": colliders,
        "articulation_root_paths": articulations,
        "joint_paths": joints,
        "drive_properties": sorted(set(drives)),
        "mimic_related_properties": sorted(set(mimic_properties)),
        "runtime_physx_validated": False,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True, help="Mapped, source-collision URDF")
    parser.add_argument("--out", type=Path, required=True, help="Fresh conversion directory")
    parser.add_argument("--graphics-api", choices=("d3d12", "vulkan"), default="d3d12")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    urdf = args.urdf.absolute()
    out = args.out.absolute()
    report: dict[str, Any] = {
        "scope": "physx_commissioning_urdf_conversion",
        "status": "failed",
        "production_collection_allowed": False,
        "runtime_physx_validated": False,
        "video_generated": False,
        "input": {"urdf": str(urdf)},
        "converter": {
            "fix_base": True,
            "merge_fixed_joints": False,
            "convert_mimic_joints_to_normal_joints": True,
            "collision_from_visuals": False,
            "collider_type": "convex_hull",
            "synthetic_drive": SYNTHETIC_DRIVE,
        },
        "shutdown": {"attempted": False, "returned": False, "error": None},
    }
    app = None
    output_is_fresh = not out.exists()
    writable_report = False
    try:
        if not urdf.is_file():
            raise FileNotFoundError(f"URDF does not exist: {urdf}")
        report["input"]["sha256"] = sha256(urdf)
        report["source_inventory"] = inspect_urdf(urdf)
        if not output_is_fresh:
            # Never delete or overwrite a previous conversion. An empty directory
            # is accepted as fresh and populated below; a non-empty directory is not.
            if any(out.iterdir()):
                raise FileExistsError(f"--out must be a fresh empty directory: {out}")
        out.mkdir(parents=True, exist_ok=True)
        _write_json(out / "source_inventory.json", report["source_inventory"])
        _write_json(out / "conversion_report.json", report)
        writable_report = True

        app, nas = _start_kit(out, args.graphics_api)
        report["runtime"] = {
            "graphics_api": args.graphics_api,
            "nas_profile_loaded": nas is not None,
            "simulation_app_started": True,
            "isaaclab_converter": "UrdfConverterCfg",
        }
        cfg = _make_cfg(urdf, out)
        report["converter"]["config_dict"] = cfg.to_dict() if hasattr(cfg, "to_dict") else repr(cfg)
        from isaaclab.sim.converters import UrdfConverter  # type: ignore

        converter = UrdfConverter(cfg)
        usd_path = Path(converter.usd_path)
        if not usd_path.is_file():
            raise RuntimeError(f"URDF converter returned no USD file: {usd_path}")
        report["output"] = {"usd_path": str(usd_path), "sha256": sha256(usd_path)}
        report["usd_inventory"] = inspect_usd(usd_path)
        report["warnings"] = []
        if not report["usd_inventory"]["collider_paths"]:
            report["warnings"].append("No CollisionAPI prims found by composed-stage traversal; contact gate blocked.")
        log_path = out / "kit.log"
        if log_path.is_file():
            report["importer_warning_lines"] = [line for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                                                if any(token in line.lower() for token in
                                                       ("unresolved", "no visual", "no collision", "color_optical"))]
        report["contact_ready"] = False
        report["status"] = "converted_inventory_only"
        report["next_gate"] = (
            "Run a separate runtime test that opens this USD, initializes its articulation, "
            "checks active/mimic DOFs and source colliders, then records measured response "
            "and contact. Conversion alone is not a PhysX or grasp acceptance."
        )
        _write_json(out / "inventory.json", {"source": report["source_inventory"], "usd": report["usd_inventory"]})
        _write_json(out / "conversion_report.json", report)
        return 0
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        if writable_report and out.exists() and out.is_dir():
            _write_json(out / "conversion_report.json", report)
        else:
            print(json.dumps(report, indent=2), file=sys.stderr)
        return 2
    finally:
        if app is not None:
            report["shutdown"]["attempted"] = True
            try:
                app.close(wait_for_replicator=False)
                report["shutdown"]["returned"] = True
            except Exception as exc:  # pragma: no cover - Kit-only path
                report["shutdown"]["error"] = repr(exc)
            if writable_report and out.exists() and out.is_dir():
                _write_json(out / "conversion_report.json", report)


if __name__ == "__main__":
    raise SystemExit(main())
