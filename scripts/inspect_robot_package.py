"""Inspect a robot URDF and its local package without downloading or importing it.

The inspector is intentionally standard-library-only.  It reports the URDF
joint graph, visual/collision mesh references, mimic joints, likely flange and
original end-effector boundaries, and whether referenced files are present in a
directory or ZIP archive.  It does not infer hardware actuator IDs or action
dimensions from URDF names.

Examples::

    python scripts/inspect_robot_package.py \
        --urdf reports/airbot_play.urdf --package-root /private/package \
        --out reports/airbot_package.json

    python scripts/inspect_robot_package.py \
        --archive OmniHand2025_left_urdf.zip \
        --urdf-member OmniHand2025left/urdf/OmniHandleft3.urdf
"""

from __future__ import annotations

import argparse
import hashlib
import json
import posixpath
import re
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
from urllib.parse import urlparse
import xml.etree.ElementTree as ET


MESH_SUFFIXES = {".dae", ".obj", ".ply", ".stl", ".usd", ".usda", ".usdc", ".usdz"}
MOVABLE_TYPES = {"continuous", "prismatic", "revolute", "planar", "floating"
}
SIDE_PREFIXES = {
    "left": ("left", "omnihandleft", "l_"),
    "right": ("right", "omnihandright", "r_"),
}


class InspectionError(ValueError):
    """Raised for an invalid or unsafe package input."""


@dataclass(frozen=True)
class PackageView:
    """Read-only view over a directory or ZIP archive."""

    archive: zipfile.ZipFile | None
    root: Path | None
    members: frozenset[str]
    archive_root: str

    def has(self, relative: str) -> bool:
        normalized = _normal_member(relative)
        if normalized is None:
            return False
        if self.archive is not None:
            return normalized in self.members
        return (self.root / Path(*normalized.split("/"))).is_file()  # type: ignore[union-attr]

    def read(self, relative: str) -> bytes:
        normalized = _normal_member(relative)
        if normalized is None:
            raise InspectionError(f"unsafe package member: {relative}")
        if self.archive is not None:
            return self.archive.read(normalized)
        return (self.root / Path(*normalized.split("/"))).read_bytes()  # type: ignore[union-attr]


def _normal_member(value: str) -> str | None:
    value = value.replace("\\", "/")
    normalized = posixpath.normpath(value)
    if normalized in {"", "."} or normalized.startswith("../") or normalized == "..":
        return None
    return normalized.lstrip("/")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _vector(value: str | None) -> list[float] | None:
    if value is None:
        return None
    values = value.split()
    parsed = [_float(item) for item in values]
    return parsed if all(item is not None for item in parsed) else None  # type: ignore[list-item]


def _attribute_map(element: ET.Element | None, names: Iterable[str]) -> dict[str, Any]:
    if element is None:
        return {}
    return {name: _float(element.get(name)) for name in names if element.get(name) is not None}


def _archive_view(path: Path) -> tuple[PackageView, list[str]]:
    archive = zipfile.ZipFile(path)
    bad = archive.testzip()
    if bad is not None:
        archive.close()
        raise InspectionError(f"corrupt ZIP entry: {bad}")
    members = frozenset(_normal_member(name) for name in archive.namelist()
                        if _normal_member(name) is not None)
    roots = sorted({member.split("/", 1)[0] for member in members if "/" in member})
    archive_root = roots[0] if len(roots) == 1 else ""
    return PackageView(archive=archive, root=None, members=members, archive_root=archive_root), list(members)


def _directory_view(path: Path) -> PackageView:
    return PackageView(archive=None, root=path, members=frozenset(), archive_root="")


def _package_reference(filename: str, urdf_member: str, view: PackageView) -> tuple[str | None, str]:
    """Return a package-relative candidate and a classification."""

    parsed = urlparse(filename)
    if parsed.scheme and parsed.scheme != "package":
        return None, "external_uri"
    if parsed.scheme == "package":
        payload = parsed.path.lstrip("/")
        package_name = parsed.netloc
        if not package_name and "/" in payload:
            package_name, payload = payload.split("/", 1)
        candidates = []
        if view.archive_root:
            candidates.append(posixpath.join(view.archive_root, payload))
        candidates.append(posixpath.join(package_name, payload))
        candidates.append(payload)
        for candidate in candidates:
            normalized = _normal_member(candidate)
            if normalized is not None and view.has(normalized):
                return normalized, "package_uri"
        return _normal_member(candidates[0]), "package_uri_missing"

    base = posixpath.dirname(urdf_member)
    candidate = _normal_member(posixpath.join(base, filename))
    return candidate, "relative"


def _mesh_dependency(filename: str, urdf_member: str, view: PackageView) -> dict[str, Any]:
    candidate, kind = _package_reference(filename, urdf_member, view)
    item: dict[str, Any] = {
        "reference": filename,
        "kind": kind,
        "resolved_member": candidate,
        "present": bool(candidate and view.has(candidate)),
    }
    if item["present"] and candidate:
        data = view.read(candidate)
        item["bytes"] = len(data)
        item["sha256"] = _sha256(data)
    return item


def _infer_side(robot_name: str, source_name: str, link_names: list[str], joint_names: list[str]) -> dict[str, Any]:
    """Return a filename/name hint without treating it as hardware identity.

    The short ``l_``/``r_`` forms are constrained to token starts.  A plain
    substring search would classify a right-hand model as both sides because
    names such as ``thumb_roll_joint`` contain ``l_``.
    """

    identity_text = " ".join([robot_name, source_name]).lower()
    name_text = " ".join([*link_names[:8], *joint_names[:8]]).lower()
    matches = []
    for side, prefixes in SIDE_PREFIXES.items():
        # Plain ``left``/``right`` in a generic gripper link is not an
        # anatomical-side signal (both AIRBOT candidates contain both links).
        explicit = any(prefix in identity_text for prefix in prefixes[:2])
        short = bool(re.search(rf"(?<![a-z0-9]){re.escape(prefixes[2])}(?=[a-z0-9])", name_text))
        if explicit or short:
            matches.append(side)
    return {
        "candidate": matches[0] if len(set(matches)) == 1 else None,
        "evidence": "filename/member and URDF names only",
        "hardware_anatomical_side_confirmed": False,
    }


def inspect(urdf_bytes: bytes, source_name: str, view: PackageView, urdf_member: str) -> dict[str, Any]:
    if urdf_bytes.startswith(b"version https://git-lfs.github.com/spec/v1"):
        return {
            "schema_version": 1,
            "source": {"name": source_name, "sha256": _sha256(urdf_bytes), "bytes": len(urdf_bytes)},
            "checks": {"lfs_pointer": True, "xml_parse_ok": False},
            "error": "URDF is a Git LFS pointer; refusing to treat it as an XML model",
        }
    try:
        root = ET.fromstring(urdf_bytes)
    except ET.ParseError as exc:
        raise InspectionError(f"invalid URDF XML: {exc}") from exc
    if root.tag.rsplit("}", 1)[-1] != "robot":
        raise InspectionError(f"root element is not robot: {root.tag}")

    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [link.get("name", "") for link in links]
    joint_names = [joint.get("name", "") for joint in joints]
    def joint_link(joint: ET.Element, tag: str) -> str | None:
        element = joint.find(tag)
        if element is None:
            return None
        return element.get("link") or (element.text.strip() if element.text else None)

    child_links = {link for joint in joints if (link := joint_link(joint, "child"))}
    root_links = [name for name in link_names if name not in child_links]

    mesh_refs: list[dict[str, Any]] = []
    link_records: list[dict[str, Any]] = []
    camera_links: list[str] = []
    for link in links:
        name = link.get("name", "")
        lower = name.lower()
        if "camera" in lower or "optical" in lower:
            camera_links.append(name)
        inertial = link.find("inertial")
        mass = inertial.find("mass") if inertial is not None else None
        visual_count = 0
        collision_count = 0
        for visual in link.findall("visual"):
            mesh = visual.find("./geometry/mesh")
            if mesh is not None and mesh.get("filename"):
                visual_count += 1
                mesh_refs.append({"link": name, "role": "visual", **_mesh_dependency(
                    mesh.get("filename", ""), urdf_member, view), "scale": _vector(mesh.get("scale"))})
        for collision in link.findall("collision"):
            mesh = collision.find("./geometry/mesh")
            if mesh is not None and mesh.get("filename"):
                collision_count += 1
                mesh_refs.append({"link": name, "role": "collision", **_mesh_dependency(
                    mesh.get("filename", ""), urdf_member, view), "scale": _vector(mesh.get("scale"))})
        link_records.append({
            "name": name,
            "mass_kg": _float(mass.get("value")) if mass is not None else None,
            "visual_mesh_count": visual_count,
            "collision_mesh_count": collision_count,
        })

    joint_records: list[dict[str, Any]] = []
    mimic_names: set[str] = set()
    movable_names: list[str] = []
    for joint in joints:
        name = joint.get("name", "")
        joint_type = joint.get("type", "")
        mimic = joint.find("mimic")
        mimic_record = None
        if mimic is not None:
            mimic_names.add(name)
            mimic_record = {
                "joint": mimic.get("joint"),
                "multiplier": _float(mimic.get("multiplier")),
                "offset": _float(mimic.get("offset")),
            }
        if joint_type in MOVABLE_TYPES:
            movable_names.append(name)
        limit = joint.find("limit")
        joint_records.append({
            "name": name,
            "type": joint_type,
            "parent": joint_link(joint, "parent"),
            "child": joint_link(joint, "child"),
            "origin_xyz_m": _vector(joint.find("origin").get("xyz") if joint.find("origin") is not None else None),
            "origin_rpy_rad": _vector(joint.find("origin").get("rpy") if joint.find("origin") is not None else None),
            "axis": _vector(joint.find("axis").get("xyz") if joint.find("axis") is not None else None),
            "limit": _attribute_map(limit, ("lower", "upper", "effort", "velocity")),
            "mimic": mimic_record,
        })

    link6_children = [record for record in joint_records if record["parent"] == "link6"]
    named_end = [record["name"] for record in joint_records
                 if re.search(r"gripper|finger|endleft|endright|custom.end", record["name"], re.I)]
    named_camera_joints = [record["name"] for record in joint_records
                           if re.search(r"camera|optical", record["name"], re.I)]
    flange_candidates = [name for name in link_names
                         if re.search(r"(?:^|_)(?:link6|flange|tool0|ee|end)(?:$|_)", name, re.I)]
    hand_root_candidates = [name for name in link_names
                            if re.search(r"(?:palm|hand[_-]?root|base[_-]?link)$", name, re.I)]
    package_meshes = [item for item in mesh_refs if item.get("present")]
    missing_meshes = [item for item in mesh_refs if not item.get("present")]
    robot_name = root.get("name", "")
    return {
        "schema_version": 1,
        "source": {"name": source_name, "sha256": _sha256(urdf_bytes), "bytes": len(urdf_bytes)},
        "robot": {"name": robot_name, "root_links": root_links, "link_count": len(links), "joint_count": len(joints)},
        "side": _infer_side(robot_name, source_name, link_names, joint_names),
        "links": link_records,
        "joints": joint_records,
        "joint_summary": {
            "nonfixed_joint_count": sum(record["type"] != "fixed" for record in joint_records),
            "movable_joint_count": len(movable_names),
            "mimic_joint_count": len(mimic_names),
            "fixed_joint_count": sum(record["type"] == "fixed" for record in joint_records),
            "movable_joint_names": movable_names,
            "mimic_joint_names": sorted(mimic_names),
            "hardware_action_mapping": None,
        },
        "mesh_dependencies": {
            "references": mesh_refs,
            "unique_reference_count": len({item["reference"] for item in mesh_refs}),
            "present_reference_count": len(package_meshes),
            "missing_reference_count": len(missing_meshes),
        },
        "boundary": {
            "flange_candidates": flange_candidates,
            "hand_root_candidates": hand_root_candidates,
            "link6_parent_children": link6_children,
            "original_end_effector_joint_candidates": named_end,
            "original_camera_links": camera_links,
            "original_camera_joint_candidates": named_camera_joints,
            "assembly_cut_status": "candidate_only; validate against approved hardware datum",
        },
        "physical": {
            "urdf_inertial_elements_present": any(item["mass_kg"] is not None for item in link_records),
            "hardware_mass_properties_confirmed": False,
            "collision_runtime_validated": False,
        },
        "checks": {
            "lfs_pointer": False,
            "xml_parse_ok": True,
            "all_mesh_dependencies_resolved": not missing_meshes,
            "mesh_package_is_complete": not missing_meshes,
        },
    }


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--urdf", type=Path, help="URDF file on disk")
    group.add_argument("--archive", type=Path, help="ZIP containing a URDF and meshes")
    parser.add_argument("--urdf-member", help="URDF member path when --archive is used")
    parser.add_argument("--package-root", type=Path, help="directory containing package:// resources")
    parser.add_argument("--out", type=Path, help="write JSON here instead of stdout")
    return parser.parse_args()


def main() -> int:
    args = _args()
    view: PackageView | None = None
    try:
        if args.archive:
            if not args.archive.is_file():
                raise InspectionError(f"archive does not exist: {args.archive}")
            if not args.urdf_member:
                raise InspectionError("--urdf-member is required with --archive")
            view, _ = _archive_view(args.archive)
            member = _normal_member(args.urdf_member)
            if member is None or not view.has(member):
                raise InspectionError(f"URDF member is absent: {args.urdf_member}")
            data = view.read(member)
            source_name = f"{args.archive}!{member}"
        else:
            if args.urdf_member:
                raise InspectionError("--urdf-member is only valid with --archive")
            if not args.urdf or not args.urdf.is_file():
                raise InspectionError(f"URDF does not exist: {args.urdf}")
            package_root = args.package_root or args.urdf.parent
            if not package_root.is_dir():
                raise InspectionError(f"package root does not exist: {package_root}")
            view = _directory_view(package_root)
            member = args.urdf.name
            data = args.urdf.read_bytes()
            source_name = str(args.urdf)
        result = inspect(data, source_name, view, member)
    except (OSError, zipfile.BadZipFile, InspectionError) as exc:
        print(f"inspect_robot_package: {exc}", file=sys.stderr)
        return 2
    finally:
        if view is not None and view.archive is not None:
            view.archive.close()

    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0 if result.get("checks", {}).get("xml_parse_ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
