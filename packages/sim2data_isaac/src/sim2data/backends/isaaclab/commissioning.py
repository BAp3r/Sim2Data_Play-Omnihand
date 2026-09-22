"""Deterministic, XML-only commissioning of a synthetic dual-arm URDF.

This module is deliberately independent of Isaac Lab and robot SDKs.  It
combines one explicitly selected arm URDF and one explicitly selected hand
URDF, removes the arm subtree below a named flange, and adds fixed synthetic
mount/camera links.  It validates the resulting link graph and resource
references, but it does not validate dynamics, meshes, collision geometry,
joint limits, or physics behavior.

All source model names, flange links, hand-root links, namespace prefixes and
output robot names are caller supplied.  No anatomical hand side or model
identity is inferred from a filename or link name.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path, PurePosixPath
from typing import Mapping, Sequence
from xml.etree import ElementTree as ET


class CommissioningError(ValueError):
    """Raised when a source URDF or synthetic assembly input is unsafe."""


def _require_text(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CommissioningError(f"{field_name} must be a nonempty string")
    return value.strip()


def _require_xml_name(value: object, field_name: str) -> str:
    text = _require_text(value, field_name)
    if any(character.isspace() or character in "/\\<>" for character in text):
        raise CommissioningError(f"{field_name} must be a safe XML/URDF name")
    if text in {".", ".."}:
        raise CommissioningError(f"{field_name} must be a safe XML/URDF name")
    return text


def _finite(value: object, field_name: str) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise CommissioningError(f"{field_name} must contain numeric values")
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as exc:
        raise CommissioningError(f"{field_name} must contain numeric values") from exc
    if not math.isfinite(number):
        raise CommissioningError(f"{field_name} must contain finite values")
    return number


def _vector3(values: Sequence[object], field_name: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)) or len(values) != 3:
        raise CommissioningError(f"{field_name} must contain exactly 3 values")
    return tuple(_finite(value, field_name) for value in values)  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class Transform:
    """A synthetic URDF origin expressed in metres and radians."""

    xyz_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "xyz_m", _vector3(self.xyz_m, "xyz_m"))
        object.__setattr__(self, "rpy_rad", _vector3(self.rpy_rad, "rpy_rad"))

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], field_name: str = "transform") -> "Transform":
        if not isinstance(value, Mapping):
            raise CommissioningError(f"{field_name} must be an object")
        if "xyz_m" not in value or "rpy_rad" not in value:
            raise CommissioningError(f"{field_name} requires xyz_m and rpy_rad")
        return cls(
            _vector3(value["xyz_m"], f"{field_name}.xyz_m"),  # type: ignore[arg-type]
            _vector3(value["rpy_rad"], f"{field_name}.rpy_rad"),  # type: ignore[arg-type]
        )

    def as_origin_attributes(self) -> dict[str, str]:
        return {
            "xyz": " ".join(_format_number(value) for value in self.xyz_m),
            "rpy": " ".join(_format_number(value) for value in self.rpy_rad),
        }


@dataclass(frozen=True, slots=True)
class AssemblyTransforms:
    """The four explicit synthetic edges used by the generated fixed chain."""

    T_flange_mount: Transform
    T_mount_hand_root: Transform
    T_mount_camera_housing: Transform
    T_housing_color_optical: Transform

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "AssemblyTransforms":
        if not isinstance(value, Mapping):
            raise CommissioningError("transforms must be an object")
        fields = (
            "T_flange_mount",
            "T_mount_hand_root",
            "T_mount_camera_housing",
            "T_housing_color_optical",
        )
        missing = tuple(field_name for field_name in fields if field_name not in value)
        if missing:
            raise CommissioningError("missing transforms: " + ", ".join(missing))
        return cls(
            *(Transform.from_mapping(value[field_name], field_name) for field_name in fields)  # type: ignore[arg-type]
        )


@dataclass(frozen=True, slots=True)
class MeshVisual:
    """An explicitly supplied visual mesh and its local m/rad origin."""

    filename: str
    origin: Transform = field(
        default_factory=lambda: Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    )

    def __post_init__(self) -> None:
        _require_text(self.filename, "mesh visual filename")
        if not isinstance(self.origin, Transform):
            raise CommissioningError("mesh visual origin must be Transform")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "MeshVisual":
        if not isinstance(value, Mapping) or "filename" not in value:
            raise CommissioningError("mesh visual requires filename")
        origin_value = value.get(
            "origin",
            {"xyz_m": (0.0, 0.0, 0.0), "rpy_rad": (0.0, 0.0, 0.0)},
        )
        return cls(
            _require_text(value["filename"], "mesh visual filename"),
            Transform.from_mapping(origin_value, "mesh visual origin"),  # type: ignore[arg-type]
        )


@dataclass(frozen=True, slots=True)
class SideCommissioningSpec:
    """Explicit source and naming inputs for one scene side."""

    side: str
    output_robot_name: str
    namespace_prefix: str
    arm_urdf: Path
    hand_urdf: Path
    flange_link: str
    hand_root_link: str
    transforms: AssemblyTransforms
    package_roots: Mapping[str, Path] = field(default_factory=dict)
    mount_visual: MeshVisual | None = None
    camera_housing_size_xyz_m: tuple[float, float, float] | None = None
    mount_mesh_base: Path | None = None

    def __post_init__(self) -> None:
        _require_text(self.side, "side")
        for field_name in ("output_robot_name", "namespace_prefix", "flange_link", "hand_root_link"):
            _require_xml_name(getattr(self, field_name), field_name)
        if not isinstance(self.transforms, AssemblyTransforms):
            raise CommissioningError("transforms must be AssemblyTransforms")
        if not isinstance(self.arm_urdf, Path) or not isinstance(self.hand_urdf, Path):
            raise CommissioningError("arm_urdf and hand_urdf must be pathlib.Path values")
        if not isinstance(self.package_roots, Mapping):
            raise CommissioningError("package_roots must be a mapping")
        for package, root in self.package_roots.items():
            _require_text(package, "package_roots key")
            if not isinstance(root, Path):
                raise CommissioningError("package_roots values must be pathlib.Path values")
        if self.mount_visual is not None and not isinstance(self.mount_visual, MeshVisual):
            raise CommissioningError("mount_visual must be MeshVisual or None")
        if self.camera_housing_size_xyz_m is not None:
            size = _vector3(self.camera_housing_size_xyz_m, "camera_housing_size_xyz_m")
            if any(value <= 0.0 for value in size):
                raise CommissioningError("camera_housing_size_xyz_m must be positive")
            object.__setattr__(self, "camera_housing_size_xyz_m", size)
        if self.mount_mesh_base is not None and not isinstance(self.mount_mesh_base, Path):
            raise CommissioningError("mount_mesh_base must be pathlib.Path or None")

    @property
    def mount_link(self) -> str:
        return f"{self.namespace_prefix}mount_assembly"

    @property
    def camera_housing_link(self) -> str:
        return f"{self.namespace_prefix}camera_housing"

    @property
    def color_optical_link(self) -> str:
        return f"{self.namespace_prefix}color_optical"


@dataclass(frozen=True, slots=True)
class CommissioningResult:
    """In-memory output and path-free summary for one combined URDF."""

    side: str
    output_robot_name: str
    xml_text: str
    resolved_meshes: tuple[str, ...]
    retained_arm_links: tuple[str, ...]
    retained_arm_joints: tuple[str, ...]
    hand_links: tuple[str, ...]
    hand_joints: tuple[str, ...]

    @property
    def summary(self) -> dict[str, object]:
        return {
            "status": "synthetic_commissioning_only",
            "physics_validated": False,
            "production": False,
            "production_collection_enabled": False,
            "side": self.side,
            "output_robot_name": self.output_robot_name,
            "retained_arm_link_count": len(self.retained_arm_links),
            "retained_arm_joint_count": len(self.retained_arm_joints),
            "hand_link_count": len(self.hand_links),
            "hand_joint_count": len(self.hand_joints),
            "resolved_mesh_count": len(self.resolved_meshes),
        }


@dataclass(frozen=True, slots=True)
class _ParsedUrdf:
    path: Path
    root: ET.Element
    links: Mapping[str, ET.Element]
    joints: Mapping[str, ET.Element]
    link_order: tuple[str, ...]
    joint_order: tuple[str, ...]
    root_link: str


def _format_number(value: float) -> str:
    return format(value, ".12g")


def _tag(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _parse_urdf(path: Path, role: str) -> _ParsedUrdf:
    try:
        resolved_path = path.expanduser().resolve(strict=True)
    except OSError as exc:
        raise CommissioningError(f"{role} URDF does not exist: {path}") from exc
    if not resolved_path.is_file():
        raise CommissioningError(f"{role} URDF is not a file: {path}")
    try:
        root = ET.parse(resolved_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise CommissioningError(f"cannot parse {role} URDF: {path}") from exc
    if _tag(root) != "robot":
        raise CommissioningError(f"{role} URDF root must be <robot>")
    _require_text(root.get("name"), f"{role} robot name")

    links: dict[str, ET.Element] = {}
    joints: dict[str, ET.Element] = {}
    link_order: list[str] = []
    joint_order: list[str] = []
    for child in root:
        child_tag = _tag(child)
        if child_tag == "link":
            name = _require_text(child.get("name"), f"{role} link name")
            if name in links:
                raise CommissioningError(f"duplicate {role} link: {name}")
            links[name] = child
            link_order.append(name)
        elif child_tag == "joint":
            name = _require_text(child.get("name"), f"{role} joint name")
            if name in joints:
                raise CommissioningError(f"duplicate {role} joint: {name}")
            joints[name] = child
            joint_order.append(name)

    if not links:
        raise CommissioningError(f"{role} URDF has no links")
    _validate_graph(links, joints, role)
    return _ParsedUrdf(
        path=resolved_path,
        root=root,
        links=links,
        joints=joints,
        link_order=tuple(link_order),
        joint_order=tuple(joint_order),
        root_link=_graph_root(links, joints),
    )


def _joint_endpoints(joint: ET.Element, role: str, joint_name: str) -> tuple[str, str]:
    parent = joint.find("parent")
    child = joint.find("child")
    if parent is None or child is None:
        raise CommissioningError(f"{role} joint {joint_name} must have parent and child")
    parent_link = _require_text(parent.get("link"), f"{role} joint {joint_name} parent link")
    child_link = _require_text(child.get("link"), f"{role} joint {joint_name} child link")
    return parent_link, child_link


def _graph_root(links: Mapping[str, ET.Element], joints: Mapping[str, ET.Element]) -> str:
    children: set[str] = set()
    for joint_name, joint in joints.items():
        _, child = _joint_endpoints(joint, "URDF", joint_name)
        children.add(child)
    roots = tuple(name for name in links if name not in children)
    if len(roots) != 1:
        raise CommissioningError(f"URDF must have exactly one root link, got {roots!r}")
    return roots[0]


def _validate_graph(
    links: Mapping[str, ET.Element],
    joints: Mapping[str, ET.Element],
    role: str,
) -> None:
    parent_for_child: dict[str, str] = {}
    children: dict[str, list[str]] = {name: [] for name in links}
    for joint_name, joint in joints.items():
        parent, child = _joint_endpoints(joint, role, joint_name)
        if parent not in links or child not in links:
            missing = parent if parent not in links else child
            raise CommissioningError(f"{role} joint {joint_name} references missing link {missing}")
        if child in parent_for_child:
            raise CommissioningError(f"{role} link {child} has multiple parent joints")
        parent_for_child[child] = joint_name
        children[parent].append(child)
        mimic = joint.find("mimic")
        if mimic is not None:
            target = _require_text(mimic.get("joint"), f"{role} joint {joint_name} mimic target")
            if target not in joints:
                raise CommissioningError(f"{role} joint {joint_name} mimics missing joint {target}")

    root = _graph_root(links, joints)
    state: dict[str, int] = {}

    def visit(link: str) -> None:
        status = state.get(link, 0)
        if status == 1:
            raise CommissioningError(f"{role} link graph contains a cycle at {link}")
        if status == 2:
            return
        state[link] = 1
        for child in children[link]:
            visit(child)
        state[link] = 2

    visit(root)
    if len(state) != len(links):
        missing = tuple(name for name in links if name not in state)
        raise CommissioningError(f"{role} URDF contains disconnected links: {missing!r}")


def _ancestor_chain(parsed: _ParsedUrdf, flange_link: str) -> tuple[set[str], set[str]]:
    if flange_link not in parsed.links:
        raise CommissioningError(f"flange link is not present in arm URDF: {flange_link}")
    parent_link: dict[str, tuple[str, str]] = {}
    for joint_name, joint in parsed.joints.items():
        parent, child = _joint_endpoints(joint, "arm", joint_name)
        parent_link[child] = (parent, joint_name)
    keep_links = {flange_link}
    keep_joints: set[str] = set()
    current = flange_link
    while current in parent_link:
        parent, joint_name = parent_link[current]
        keep_links.add(parent)
        keep_joints.add(joint_name)
        current = parent
    return keep_links, keep_joints


def _validate_retained_mimics(
    joints: Mapping[str, ET.Element], retained_joints: set[str], role: str
) -> None:
    targets: dict[str, str] = {}
    for joint_name in retained_joints:
        mimic = joints[joint_name].find("mimic")
        if mimic is None:
            continue
        target = _require_text(mimic.get("joint"), f"{role} joint {joint_name} mimic target")
        if target not in retained_joints:
            raise CommissioningError(
                f"retained {role} joint {joint_name} mimics deleted joint {target}"
            )
        targets[joint_name] = target

    state: dict[str, int] = {}

    def visit(joint_name: str) -> None:
        status = state.get(joint_name, 0)
        if status == 1:
            raise CommissioningError(f"{role} mimic graph contains a cycle at {joint_name}")
        if status == 2:
            return
        state[joint_name] = 1
        target = targets.get(joint_name)
        if target is not None:
            visit(target)
        state[joint_name] = 2

    for joint_name in targets:
        visit(joint_name)


def _namespace_map(prefix: str, role: str, names: Sequence[str]) -> dict[str, str]:
    return {name: _require_xml_name(f"{prefix}{role}__{name}", "namespaced URDF name") for name in names}


def _named_maps(
    root: ET.Element,
    prefix: str,
    role: str,
    link_names: Sequence[str],
    joint_names: Sequence[str],
) -> dict[str, dict[str, str]]:
    maps = {
        "link": _namespace_map(prefix, role, link_names),
        "joint": _namespace_map(prefix, role, joint_names),
        "material": {},
        "transmission": {},
        "actuator": {},
        "ros2_control": {},
    }
    for element in root.iter():
        element_tag = _tag(element)
        name = element.get("name")
        if name and element_tag in ("material", "transmission", "actuator", "ros2_control"):
            maps[element_tag].setdefault(
                name,
                _require_xml_name(
                    f"{prefix}{role}__{element_tag}__{name}",
                    "namespaced URDF name",
                ),
            )
    return maps


def _copy_element(
    element: ET.Element,
    maps: Mapping[str, Mapping[str, str]],
) -> ET.Element:
    element_tag = _tag(element)
    attrs: dict[str, str] = {}
    for key, value in element.attrib.items():
        mapped = value
        if key == "link":
            mapped = maps["link"].get(value, value)
        elif key == "joint":
            mapped = maps["joint"].get(value, value)
        elif key == "reference":
            mapped = maps["link"].get(value, maps["joint"].get(value, value))
        elif key == "name":
            if element_tag in maps and value in maps[element_tag]:
                mapped = maps[element_tag][value]
            elif element_tag == "material" and value in maps["material"]:
                mapped = maps["material"][value]
        attrs[key] = mapped
    copied = ET.Element(element.tag, attrs)
    copied.text = element.text
    copied.tail = element.tail
    for child in element:
        copied.append(_copy_element(child, maps))
    return copied


def _root_extra_is_retained(
    element: ET.Element,
    link_map: Mapping[str, str],
    joint_map: Mapping[str, str],
) -> bool:
    element_tag = _tag(element)
    if element_tag == "gazebo":
        reference = element.get("reference")
        if reference and reference not in link_map and reference not in joint_map:
            return False
    if element_tag == "transmission":
        for nested in element.iter():
            if _tag(nested) == "joint":
                reference = nested.get("name")
                if reference and reference not in joint_map:
                    return False
    return True


def _validate_meshes(
    root: ET.Element,
    source_path: Path,
    package_roots: Mapping[str, Path],
) -> tuple[str, ...]:
    resolved: list[str] = []
    for element in root.iter():
        if _tag(element) != "mesh":
            continue
        filename = _require_text(element.get("filename"), "mesh filename")
        resolved_path = _resolve_mesh(filename, source_path.parent, package_roots)
        # Output lives away from both source URDFs. Preserve the actual binding
        # in this private artifact instead of leaving now-broken relative URIs.
        element.set("filename", resolved_path.as_posix())
        resolved.append(str(resolved_path))
    return tuple(resolved)


def _resolve_mesh(
    filename: str,
    relative_base: Path,
    package_roots: Mapping[str, Path],
) -> Path:
    if filename.startswith("package://"):
        remainder = filename[len("package://") :]
        package, separator, relative = remainder.partition("/")
        if not separator or not package or not relative:
            raise CommissioningError(f"invalid package mesh URI: {filename}")
        if package not in package_roots:
            raise CommissioningError(f"mesh package is not mapped: {package}")
        root = _resolve_directory(package_roots[package], f"package {package}")
        relative_path = PurePosixPath(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts or "\\" in relative:
            raise CommissioningError(f"mesh package path escapes package root: {filename}")
        candidate = root.joinpath(*relative_path.parts)
        return _resolve_existing_file(candidate, root, filename)

    if "://" in filename:
        raise CommissioningError(f"unsupported mesh URI: {filename}")
    if "\\" in filename:
        raise CommissioningError(f"mesh path must use POSIX separators: {filename}")
    relative_path = PurePosixPath(filename)
    if relative_path.is_absolute() or Path(filename).drive:
        raise CommissioningError(f"absolute mesh paths are forbidden: {filename}")
    root = _resolve_directory(relative_base, "mesh relative base")
    candidate = root.joinpath(*relative_path.parts)
    return _resolve_existing_file(candidate, root, filename)


def _resolve_directory(path: Path, label: str) -> Path:
    try:
        resolved = path.expanduser().resolve(strict=True)
    except OSError as exc:
        raise CommissioningError(f"{label} does not exist: {path}") from exc
    if not resolved.is_dir():
        raise CommissioningError(f"{label} is not a directory: {path}")
    return resolved


def _resolve_existing_file(candidate: Path, root: Path, original: str) -> Path:
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise CommissioningError(f"mesh does not exist: {original}") from exc
    if not resolved.is_relative_to(root):
        raise CommissioningError(f"mesh path escapes root: {original}")
    if not resolved.is_file():
        raise CommissioningError(f"mesh is not a file: {original}")
    return resolved


def _fixed_joint(
    name: str,
    parent: str,
    child: str,
    transform: Transform,
) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", transform.as_origin_attributes())
    return joint


def _box_geometry_link(
    name: str,
    size_xyz_m: tuple[float, float, float] | None,
    visual_mesh: MeshVisual | None,
    mesh_base: Path,
    package_roots: Mapping[str, Path],
) -> tuple[ET.Element, tuple[str, ...]]:
    """Build a generated link and validate any explicitly supplied geometry."""
    link = ET.Element("link", {"name": name})
    resolved_meshes: list[str] = []
    if visual_mesh is not None:
        resolved_meshes.append(
            str(_resolve_mesh(visual_mesh.filename, mesh_base, package_roots))
        )
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", visual_mesh.origin.as_origin_attributes())
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": Path(resolved_meshes[-1]).as_posix()})
    if size_xyz_m is not None:
        size = " ".join(_format_number(value) for value in size_xyz_m)
        for element_name in ("visual", "collision"):
            element = ET.SubElement(link, element_name)
            geometry = ET.SubElement(element, "geometry")
            ET.SubElement(geometry, "box", {"size": size})
    return link, tuple(resolved_meshes)


def _validate_output_graph(root: ET.Element) -> None:
    links = {
        child.get("name"): child
        for child in root
        if _tag(child) == "link" and child.get("name")
    }
    joints = {
        child.get("name"): child
        for child in root
        if _tag(child) == "joint" and child.get("name")
    }
    if len(links) != sum(1 for child in root if _tag(child) == "link"):
        raise CommissioningError("combined URDF has duplicate or nameless links")
    if len(joints) != sum(1 for child in root if _tag(child) == "joint"):
        raise CommissioningError("combined URDF has duplicate or nameless joints")
    _validate_graph(links, joints, "combined")
    _validate_retained_mimics(joints, set(joints), "combined")


def build_combined_urdf(spec: SideCommissioningSpec) -> CommissioningResult:
    """Build one namespaced synthetic combined URDF without writing files."""
    if not isinstance(spec, SideCommissioningSpec):
        raise CommissioningError("spec must be SideCommissioningSpec")
    arm = _parse_urdf(spec.arm_urdf, "arm")
    hand = _parse_urdf(spec.hand_urdf, "hand")
    if spec.hand_root_link != hand.root_link:
        raise CommissioningError(
            f"hand_root_link {spec.hand_root_link!r} must be the hand URDF root {hand.root_link!r}"
        )
    keep_arm_links, keep_arm_joints = _ancestor_chain(arm, spec.flange_link)
    _validate_retained_mimics(arm.joints, keep_arm_joints, "arm")

    arm_links = tuple(name for name in arm.link_order if name in keep_arm_links)
    arm_joints = tuple(name for name in arm.joint_order if name in keep_arm_joints)
    hand_links = hand.link_order
    hand_joints = hand.joint_order
    arm_maps = _named_maps(arm.root, spec.namespace_prefix, "arm", arm_links, arm_joints)
    hand_maps = _named_maps(hand.root, spec.namespace_prefix, "hand", hand_links, hand_joints)

    output_root = ET.Element("robot", {"name": spec.output_robot_name})
    resolved_meshes: list[str] = []

    for source, maps, selected_links, selected_joints in (
        (arm, arm_maps, arm_links, arm_joints),
        (hand, hand_maps, hand_links, hand_joints),
    ):
        for child in source.root:
            child_tag = _tag(child)
            if child_tag in ("link", "joint"):
                continue
            if not _root_extra_is_retained(child, maps["link"], maps["joint"]):
                continue
            copied = _copy_element(child, maps)
            resolved_meshes.extend(_validate_meshes(copied, source.path, spec.package_roots))
            output_root.append(copied)
        for link_name in selected_links:
            copied = _copy_element(source.links[link_name], maps)
            resolved_meshes.extend(_validate_meshes(copied, source.path, spec.package_roots))
            output_root.append(copied)
        for joint_name in selected_joints:
            output_root.append(_copy_element(source.joints[joint_name], maps))

    flange_output = arm_maps["link"][spec.flange_link]
    hand_root_output = hand_maps["link"][spec.hand_root_link]
    mount = spec.mount_link
    housing = spec.camera_housing_link
    optical = spec.color_optical_link
    mount_element, mount_meshes = _box_geometry_link(
        mount,
        None,
        spec.mount_visual,
        spec.mount_mesh_base or arm.path.parent,
        spec.package_roots,
    )
    housing_element, housing_meshes = _box_geometry_link(
        housing,
        spec.camera_housing_size_xyz_m,
        None,
        arm.path.parent,
        spec.package_roots,
    )
    optical_element = ET.Element("link", {"name": optical})
    output_root.extend((mount_element, housing_element, optical_element))
    resolved_meshes.extend(mount_meshes)
    resolved_meshes.extend(housing_meshes)
    output_root.extend(
        (
            _fixed_joint(
                f"{spec.namespace_prefix}flange_mount_fixed",
                flange_output,
                mount,
                spec.transforms.T_flange_mount,
            ),
            _fixed_joint(
                f"{spec.namespace_prefix}mount_hand_root_fixed",
                mount,
                hand_root_output,
                spec.transforms.T_mount_hand_root,
            ),
            _fixed_joint(
                f"{spec.namespace_prefix}mount_camera_housing_fixed",
                mount,
                housing,
                spec.transforms.T_mount_camera_housing,
            ),
            _fixed_joint(
                f"{spec.namespace_prefix}camera_housing_color_optical_fixed",
                housing,
                optical,
                spec.transforms.T_housing_color_optical,
            ),
        )
    )
    _validate_output_graph(output_root)
    xml_text = "<?xml version=\"1.0\"?>\n" + ET.tostring(output_root, encoding="unicode") + "\n"
    return CommissioningResult(
        side=spec.side,
        output_robot_name=spec.output_robot_name,
        xml_text=xml_text,
        resolved_meshes=tuple(dict.fromkeys(resolved_meshes)),
        retained_arm_links=tuple(arm_maps["link"][name] for name in arm_links),
        retained_arm_joints=tuple(arm_maps["joint"][name] for name in arm_joints),
        hand_links=tuple(hand_maps["link"][name] for name in hand_links),
        hand_joints=tuple(hand_maps["joint"][name] for name in hand_joints),
    )


commission_side = build_combined_urdf


def write_combined_urdfs(
    specs: Sequence[SideCommissioningSpec],
    output_dir: Path,
) -> tuple[Path, ...]:
    """Write multiple side outputs and one summary without overwriting files."""
    if isinstance(specs, (str, bytes)) or not specs:
        raise CommissioningError("specs must be a nonempty sequence")
    results = tuple(build_combined_urdf(spec) for spec in specs)
    sides = [result.side for result in results]
    if len(sides) != len(set(sides)):
        raise CommissioningError("commissioning sides must be unique")
    target_dir = output_dir.expanduser()
    if target_dir.exists() and not target_dir.is_dir():
        raise CommissioningError(f"output path is not a directory: {output_dir}")
    targets = tuple(target_dir / f"{result.output_robot_name}.urdf" for result in results)
    summary_path = target_dir / "commissioning_summary.json"
    existing = tuple(path for path in (*targets, summary_path) if path.exists())
    if existing:
        raise CommissioningError("refusing to overwrite output: " + ", ".join(map(str, existing)))
    target_dir.mkdir(parents=True, exist_ok=True)
    for target, result in zip(targets, results):
        with target.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(result.xml_text)
    summary = {
        "status": "synthetic_commissioning_only",
        "physics_validated": False,
        "production": False,
        "production_collection_enabled": False,
        "sides": [result.summary for result in results],
    }
    with summary_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return targets + (summary_path,)


def write_combined_urdf(spec: SideCommissioningSpec, output_dir: Path) -> Path:
    """Write one side and its summary, refusing existing output files."""
    paths = write_combined_urdfs((spec,), output_dir)
    return paths[0]
