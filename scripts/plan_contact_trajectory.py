"""Solve a synthetic Thor/cardbox grasp candidate from source URDF geometry."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

SCENE_KIND = "thor_cardbox"
BOX_CENTER = np.array([-0.23, 0.13, 0.0145], dtype=float)
BOX_SIZE = np.array([0.084, 0.06, 0.06], dtype=float)
FACE_PATCH_MARGIN_M = 0.002


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def vec(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return np.asarray(default, dtype=float)
    result = np.asarray([float(value) for value in text.split()], dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"expected finite 3-vector, got {text!r}")
    return result


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def origin(node):
    value = node.find("origin")
    return np.eye(4) if value is None else pose(vec(value.get("xyz")), vec(value.get("rpy")))


def motion(joint, value):
    result = np.eye(4)
    kind = joint.get("type")
    if kind in ("revolute", "continuous", "prismatic"):
        axis_node = joint.find("axis")
        axis = vec(None if axis_node is None else axis_node.get("xyz"), (1, 0, 0))
        norm = np.linalg.norm(axis)
        if norm <= 1e-12:
            raise ValueError(f"zero axis: {joint.get('name')}")
        if kind == "prismatic":
            result[:3, 3] = axis / norm * value
        else:
            result[:3, :3] = Rotation.from_rotvec(axis / norm * value).as_matrix()
    return result


def box_sdf(points, center, size):
    half = np.asarray(size) / 2
    offset = np.abs(np.asarray(points) - center)
    outside = np.maximum(offset - half, 0)
    outside_distance = np.linalg.norm(outside, axis=-1)
    inside_depth = np.min(half - offset, axis=-1)
    return np.where(outside_distance > 0, outside_distance, -inside_depth)


def validate_hand_targets(joints, active, side):
    by_name = {joint.get("name"): joint for joint in joints}
    prefix = f"{side}_hand__"
    expected = {name for name, joint in by_name.items()
                if name.startswith(prefix) and joint.get("type") != "fixed"
                and joint.find("mimic") is None and joint.find("limit") is not None
                and float(joint.find("limit").get("upper")) > float(joint.find("limit").get("lower"))}
    limits = {}
    for item in active:
        name = item["name"]
        joint = by_name.get(name)
        if joint is None or joint.find("mimic") is not None or joint.get("type") == "fixed":
            raise ValueError(f"unknown, fixed, or mimic joint cannot be commanded: {name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"active joint has no source limits: {name}")
        low, high = float(limit.get("lower")), float(limit.get("upper"))
        opened, closed = float(item["open_rad"]), float(item["close_rad"])
        if not all(math.isfinite(value) and low <= value <= high for value in (opened, closed)):
            raise ValueError(f"active hand target outside source limits: {name}")
        if name in limits:
            raise ValueError(f"duplicate active hand target: {name}")
        limits[name] = {"lower_rad": low, "upper_rad": high,
                        "open_rad": opened, "close_rad": closed}
    if set(limits) != expected:
        raise ValueError(f"profile active list mismatch: missing={sorted(expected-set(limits))}, extra={sorted(set(limits)-expected)}")
    return limits


def nearest_zero_configuration(limits):
    """Return the source-limit-bounded arm configuration nearest zero."""
    bounds = np.asarray(limits, dtype=float)
    if bounds.ndim != 2 or bounds.shape[1] != 2 or not np.isfinite(bounds).all():
        raise ValueError("arm limits must be a finite Nx2 array")
    if np.any(bounds[:, 0] > bounds[:, 1]):
        raise ValueError("arm lower limits must not exceed upper limits")
    return np.clip(np.zeros(bounds.shape[0], dtype=float), bounds[:, 0], bounds[:, 1])


def positions_for(joints, active, arm_names, arm_q, amount):
    values = {item["name"]: float(item["open_rad"] + amount *
                                  (item["close_rad"] - item["open_rad"])) for item in active}
    values.update({name: float(value) for name, value in zip(arm_names, arm_q)})
    by_name = {joint.get("name"): joint for joint in joints}

    def resolve(name, seen):
        if name in values:
            return values[name]
        if name in seen:
            raise ValueError(f"cyclic mimic relation: {name}")
        joint = by_name[name]
        mimic = joint.find("mimic")
        if mimic is not None:
            source = mimic.get("joint")
            value = (float(mimic.get("multiplier", "1")) * resolve(source, seen | {name})
                     + float(mimic.get("offset", "0")))
            limit = joint.find("limit")
            if limit is None or not float(limit.get("lower", "nan")) <= value <= float(limit.get("upper", "nan")):
                raise ValueError(f"source mimic value violates limits: {name}={value}")
        else:
            limit = joint.find("limit")
            value = 0.0
            if joint.get("type") in ("revolute", "continuous", "prismatic") and limit is not None:
                value = min(float(limit.get("upper", 0)), max(float(limit.get("lower", 0)), value))
        values[name] = value
        return value

    for name in by_name:
        resolve(name, set())
    return values


class RobotModel:
    def __init__(self, urdf_path, side):
        self.path = Path(urdf_path).resolve()
        self.side = side
        self.xml = ET.parse(self.path).getroot()
        self.links = {link.get("name"): link for link in self.xml.findall("link")}
        self.joints = self.xml.findall("joint")
        children = {joint.find("child").get("link") for joint in self.joints}
        roots = sorted(set(self.links) - children)
        if self.xml.tag != "robot" or len(roots) != 1:
            raise ValueError(f"URDF must have one root link, found {roots}")
        self.root_link = roots[0]
        self.arm_names = [f"{side}_arm__joint{i}" for i in range(1, 7)]
        by_joint = {joint.get("name"): joint for joint in self.joints}
        self.arm_limits = []
        for name in self.arm_names:
            joint = by_joint.get(name)
            limit = None if joint is None else joint.find("limit")
            if joint is None or joint.get("type") != "revolute" or limit is None:
                raise ValueError(f"missing bounded source arm joint: {name}")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            if not math.isfinite(lower + upper) or lower >= upper:
                raise ValueError(f"invalid source limits: {name}")
            self.arm_limits.append((lower, upper))
        self.meshes, self.mesh_inventory = self._collision_meshes()

    def _collision_meshes(self):
        shapes, inventory = [], []
        for link_name, link in self.links.items():
            for index, collision in enumerate(link.findall("collision")):
                mesh_node = collision.find("geometry/mesh")
                if mesh_node is None:
                    raise ValueError(f"non-mesh collision geometry needs explicit support: {link_name}")
                filename = mesh_node.get("filename")
                if not filename:
                    raise ValueError(f"missing source collision path: {link_name}")
                path = Path(filename)
                if not path.is_absolute():
                    path = (self.path.parent / path).resolve()
                if not path.is_file():
                    raise FileNotFoundError(f"source collision mesh missing: {link_name}: {path}")
                scale = vec(mesh_node.get("scale"), (1, 1, 1))
                if np.any(np.abs(scale) < 1e-12):
                    raise ValueError(f"zero collision mesh scale: {link_name}")
                mesh = trimesh.load_mesh(str(path), process=False)
                if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
                    raise ValueError(f"empty/unsupported source collision mesh: {path}")
                count = min(max(int(len(mesh.faces) * 2), 256), 2048)
                samples, _ = trimesh.sample.sample_surface(mesh, count, seed=17 + len(shapes))
                samples = np.asarray(samples) * scale
                shapes.append({"link": link_name, "points": samples, "origin": origin(collision)})
                inventory.append({"link": link_name, "sha256": sha256(path),
                                  "sampled_surface_points": len(samples),
                                  "mesh_vertex_count": len(mesh.vertices), "mesh_face_count": len(mesh.faces)})
        if not shapes:
            raise ValueError("no source collision meshes in URDF")
        return shapes, inventory

    def fk(self, q):
        result = {self.root_link: np.eye(4)}
        pending = list(self.joints)
        while pending:
            remaining, changed = [], False
            for joint in pending:
                parent = joint.find("parent").get("link")
                child = joint.find("child").get("link")
                if parent not in result:
                    remaining.append(joint)
                    continue
                result[child] = result[parent] @ origin(joint) @ motion(joint, q.get(joint.get("name"), 0.0))
                changed = True
            if not changed:
                raise ValueError("URDF joint graph is disconnected or cyclic")
            pending = remaining
        return result

    def world_samples(self, link_frames, base_world):
        for shape in self.meshes:
            matrix = base_world @ link_frames[shape["link"]] @ shape["origin"]
            yield shape["link"], shape["points"] @ matrix[:3, :3].T + matrix[:3, 3]


def hand_geometry(model, q, side, max_points_per_link=None):
    """Return collision-surface samples for every hand link in palm coordinates."""
    frames = model.fk(q)
    palm_name = f"{side}_hand__{'l' if side == 'left' else 'R'}_palm"
    inv_palm = np.linalg.inv(frames[palm_name])
    prefix = f"{side}_hand__"
    by_link = {}
    for shape in model.meshes:
        link = shape["link"]
        if not link.startswith(prefix):
            continue
        local = inv_palm @ frames[link] @ shape["origin"]
        points = shape["points"]
        if max_points_per_link is not None and len(points) > max_points_per_link:
            indices = np.linspace(0, len(points) - 1, max_points_per_link, dtype=int)
            points = points[indices]
        by_link.setdefault(link, []).append(points @ local[:3, :3].T + local[:3, 3])
    return {link: np.concatenate(parts) for link, parts in by_link.items()}


def hand_clouds(model, q, side):
    geometry = hand_geometry(model, q, side)
    suffix = "l" if side == "left" else "R"
    clouds = {}
    for role in ("thumb", "index", "middle"):
        link = f"{side}_hand__{suffix}_{role}_dip_link"
        if link not in geometry:
            raise ValueError(f"missing source distal collision geometry: {link}")
        clouds[role] = geometry[link]
    return clouds


def nearest_surface_distances(points, target, count=8):
    """Return actual point-to-target distances; never average signed vectors."""
    points = np.asarray(points, dtype=float)
    target = np.asarray(target, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("surface contact cloud must be a non-empty Nx3 array")
    distances = np.linalg.norm(points - target, axis=1)
    take = min(max(int(count), 1), len(distances))
    return np.sort(np.partition(distances, take - 1)[:take])


def project_to_box_face_patch(points, center, size, axis, face_coordinate,
                              margin_m=FACE_PATCH_MARGIN_M):
    """Project points onto the inset rectangular patch of one AABB face."""
    points = np.asarray(points, dtype=float)
    center = np.asarray(center, dtype=float)
    half = np.asarray(size, dtype=float) / 2
    projected = points.copy()
    projected[:, axis] = float(face_coordinate)
    for tangent in range(3):
        if tangent == axis:
            continue
        low, high = center[tangent] - half[tangent] + margin_m, center[tangent] + half[tangent] - margin_m
        if low > high:
            raise ValueError("face patch margin exceeds box half extent")
        projected[:, tangent] = np.clip(projected[:, tangent], low, high)
    return projected


def box_face_patch_distances(points, center, size, axis, face_coordinate,
                             margin_m=FACE_PATCH_MARGIN_M):
    points = np.asarray(points, dtype=float)
    projected = project_to_box_face_patch(points, center, size, axis, face_coordinate, margin_m)
    return np.linalg.norm(points - projected, axis=1), projected


def hand_clearance_residual(clearances, *, minimum_clearance_m=0.002, count=24, weight=10.0):
    """Fixed-length least-squares penalty for insufficient non-contact clearance."""
    clearances = np.asarray(clearances, dtype=float).reshape(-1)
    if clearances.size == 0:
        return np.zeros(count, dtype=float)
    take = min(count, clearances.size)
    worst = np.partition(clearances, take - 1)[:take]
    result = np.minimum(worst - minimum_clearance_m, 0.0) * weight
    if take < count:
        result = np.pad(result, (0, count - take))
    return result


def hand_box_clearance(geometry, position, rotation, box_center, box_size):
    rows = {}
    matrix = rotation.as_matrix()
    for link, points in geometry.items():
        world = points @ matrix.T + position
        distances = box_sdf(world, box_center, box_size)
        rows[link] = {"minimum_signed_clearance_m": float(np.min(distances)),
                      "penetrating_samples_beyond_1mm": int(np.count_nonzero(distances < -0.001))}
    minimum = min((row["minimum_signed_clearance_m"] for row in rows.values()), default=math.inf)
    return {"minimum_signed_clearance_m": minimum,
            "penetrating_samples_beyond_1mm": sum(row["penetrating_samples_beyond_1mm"] for row in rows.values()),
            "links": rows}

def palm_goal(approach, close_hint):
    approach = approach / np.linalg.norm(approach)
    close = close_hint - np.dot(close_hint, approach) * approach
    close /= np.linalg.norm(close)
    local_x = np.cross(close, approach)
    return Rotation.from_matrix(np.column_stack((local_x, close, approach))), close


def fit_contacts(clouds, approach, close_hint, collision_clouds=None, allowed_links=()):
    rotation0, close_axis = palm_goal(approach, close_hint)
    half = BOX_SIZE / 2
    face_axis = int(np.argmax(np.abs(close_axis[:2])))
    face_direction = 1.0 if close_axis[face_axis] >= 0 else -1.0
    face_coordinates = {"thumb": BOX_CENTER[face_axis] - face_direction * half[face_axis],
                        "index": BOX_CENTER[face_axis] + face_direction * half[face_axis],
                        "middle": BOX_CENTER[face_axis] + face_direction * half[face_axis]}
    target = {role: BOX_CENTER.copy() for role in face_coordinates}
    for role, coordinate in face_coordinates.items():
        target[role][face_axis] = coordinate
    initial_position = BOX_CENTER - approach * 0.075
    allowed_links = set(allowed_links)
    penalty_clouds = ({name: points for name, points in collision_clouds.items()
                       if name not in allowed_links} if collision_clouds else {})
    collision_points = (np.concatenate([penalty_clouds[name] for name in sorted(penalty_clouds)])
                        if penalty_clouds else np.empty((0, 3), dtype=float))

    def residual(value):
        position = value[:3]
        rotation = rotation0 * Rotation.from_rotvec(value[3:])
        matrix = rotation.as_matrix()
        parts = []
        for role in ("thumb", "index", "middle"):
            world = clouds[role] @ matrix.T + position
            distances, _ = box_face_patch_distances(world, BOX_CENTER, BOX_SIZE,
                                                     face_axis, face_coordinates[role])
            nearest_count = min(8, len(distances))
            nearest = np.sort(np.partition(distances, nearest_count - 1)[:nearest_count])
            # Distances to the intended surface patch cannot cancel by direction.
            parts.extend(nearest)
        if len(collision_points):
            hand_world = collision_points @ matrix.T + position
            signed_clearance = box_sdf(hand_world, BOX_CENTER, BOX_SIZE)
            parts.extend(hand_clearance_residual(signed_clearance))
        parts.extend(value[3:] * 0.018)
        parts.extend((position - initial_position) * 0.025)
        parts.append(max(float(np.dot(position - BOX_CENTER, approach)) + 0.015, 0.0) * 8)
        return np.asarray(parts)

    solution = least_squares(residual, np.r_[initial_position, np.zeros(3)],
                             bounds=(np.r_[BOX_CENTER - 0.16, [-1.0] * 3],
                                     np.r_[BOX_CENTER + 0.16, [1.0] * 3]),
                             max_nfev=180, ftol=1e-8, xtol=1e-8, gtol=1e-8)
    position = solution.x[:3]
    rotation = rotation0 * Rotation.from_rotvec(solution.x[3:])
    contact_residuals = {}
    for role, point in target.items():
        world = clouds[role] @ rotation.as_matrix().T + position
        distances, projected = box_face_patch_distances(world, BOX_CENTER, BOX_SIZE,
                                                        face_axis, face_coordinates[role])
        nearest_count = min(8, len(distances))
        nearest_indices = np.argpartition(distances, nearest_count - 1)[:nearest_count]
        nearest = distances[nearest_indices]
        closest_index = int(nearest_indices[np.argmin(nearest)])
        contact_residuals[role] = {"target_world_m": point.tolist(),
                                   "face_axis": face_axis,
                                   "face_coordinate_m": float(face_coordinates[role]),
                                   "face_patch_margin_m": FACE_PATCH_MARGIN_M,
                                   "nearest_min_m": float(np.min(nearest)),
                                   "nearest_mean_m": float(np.mean(nearest)),
                                   "nearest_max_m": float(np.max(nearest)),
                                   "nearest_surface_point_world_m": world[closest_index].tolist(),
                                   "nearest_box_patch_point_world_m": projected[closest_index].tolist()}
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation.as_matrix(), position
    closed_hand = (hand_box_clearance(collision_clouds, position, rotation, BOX_CENTER, BOX_SIZE)
                   if collision_clouds else {"minimum_signed_clearance_m": None,
                                             "penetrating_samples_beyond_1mm": None, "links": {}})
    noncontact_minimum = min((row["minimum_signed_clearance_m"]
                              for link, row in closed_hand["links"].items()
                              if link not in allowed_links), default=math.inf)
    closed_hand["noncontact_minimum_clearance_m"] = (noncontact_minimum
                                                      if math.isfinite(noncontact_minimum) else None)
    return {"position": position, "rotation": rotation, "matrix": matrix,
            "approach": approach, "close_axis": close_axis,
            "contacts": contact_residuals,
            "closed_hand_box_clearance": closed_hand,
            "position_residual_m": max(value["nearest_mean_m"] for value in contact_residuals.values()),
            "orientation_residual_rad": float(np.linalg.norm(solution.x[3:])),
            "optimizer_success": bool(solution.success),
            "objective_norm": float(np.linalg.norm(solution.fun))}

def rot_error(target, actual):
    return Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()


def solve_ik(model, target, base_world, hand_q, initial=None, preview=None, screen=None):
    bounds = np.asarray(model.arm_limits)
    lower, upper = bounds[:, 0], bounds[:, 1]
    starts = []
    if initial is not None:
        starts.append(np.clip(initial, lower + 1e-8, upper - 1e-8))
    if preview is not None:
        starts.append(np.clip(preview, lower + 1e-8, upper - 1e-8))
    starts.append((lower + upper) / 2)
    for seed in range(3):
        starts.append(np.random.default_rng(141 + seed).uniform(lower * 0.8 + upper * 0.2,
                                                                 lower * 0.2 + upper * 0.8))
    best = None
    palm = f"{model.side}_hand__{'l' if model.side == 'left' else 'R'}_palm"
    for start in starts:
        def residual(arm_q):
            q = dict(hand_q)
            q.update(zip(model.arm_names, arm_q))
            actual = base_world @ model.fk(q)[palm]
            return np.r_[(actual[:3, 3] - target[:3, 3]) / 0.001,
                         rot_error(target, actual) / 0.01]
        solution = least_squares(residual, start, bounds=(lower, upper), max_nfev=220,
                                 ftol=1e-9, xtol=1e-9, gtol=1e-9)
        raw = residual(solution.x)
        q = dict(hand_q)
        q.update(zip(model.arm_names, solution.x))
        actual = base_world @ model.fk(q)[palm]
        candidate = {"q": solution.x, "score": float(np.linalg.norm(raw)),
                     "position_residual_m": float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
                     "orientation_residual_rad": float(np.linalg.norm(rot_error(target, actual))),
                     "optimizer_success": bool(solution.success)}
        candidate["screen_failures"] = 0 if screen is None else len(screen(solution.x)["failures"])
        candidate["selection_rank"] = (candidate["score"] > 3,
                                        candidate["screen_failures"], candidate["score"])
        if best is None or candidate["selection_rank"] < best["selection_rank"]:
            best = candidate
    return best


def collision_screen(model, arm_q, active, amount, base_world, box_center, allowed, table_z, ground_z):
    q = positions_for(model.joints, active, model.arm_names, arm_q, amount)
    rows, failures = [], []
    min_table = math.inf
    for link, points in model.world_samples(model.fk(q), base_world):
        clearance = float(np.min(box_sdf(points, box_center, BOX_SIZE)))
        # Audited Thor footprint, unlike an infinite horizontal half-space.
        footprint = (np.abs(points[:, 0]) <= .45) & (np.abs(points[:, 1]) <= .375)
        table_clearance = float(np.min(points[footprint, 2]) - table_z) if footprint.any() else None
        ground_clearance = float(np.min(points[:, 2]) - ground_z)
        if table_clearance is not None:
            min_table = min(min_table, table_clearance)
        rows.append({"link": link, "cardbox_signed_clearance_m": clearance,
                     "thor_collision_plane_clearance_m": table_clearance,
                     "full_flat_ground_clearance_m": ground_clearance,
                     "surface_samples": int(len(points))})
        if link in allowed:
            if clearance < -0.001 or clearance > 0.008:
                failures.append(f"intended distal contact clearance outside [-0.001, 0.008] m: {link} {clearance:.6f}")
        elif clearance < 0.002:
            failures.append(f"unintended cardbox clearance/collision: {link} {clearance:.6f}")
        if ground_clearance < -0.002:
            failures.append(f"full flat ground penetration: {link} {ground_clearance:.6f}")
        if table_clearance is not None and table_clearance < -0.002:
            failures.append(f"Thor collision-top plane penetration: {link} {table_clearance:.6f}")
    return {"passed": not failures, "failures": failures,
            "minimum_table_plane_clearance_m": min_table if math.isfinite(min_table) else None, "meshes": rows,
            "method": "sampled source collision surfaces versus cardbox AABB, bounded Thor footprint/top, and ground; not exhaustive triangle collision"}


def create_plan(urdf_path, profile_path, manifest_path, side):
    profile = json.loads(Path(profile_path).read_text(encoding="utf-8"))
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    if profile.get("purpose") != "synthetic_commissioning_only" or profile.get("production_collection_enabled") is not False:
        raise ValueError("explicit production-disabled synthetic profile required")
    table = profile["table"]
    if not math.isclose(float(table["collision_top_z_m"]), -0.0155, abs_tol=1e-8):
        raise ValueError("Thor collision top must match audited -0.0155 m datum")
    card = manifest["assets"]["card_box"]
    dims = np.asarray(card["usd"]["world_bounds_m"], dtype=float) * 0.12
    if not np.allclose(dims, BOX_SIZE, atol=1e-7) or card["collision"]["approximation"] != "boundingCube":
        raise ValueError(f"manifest scaled cardbox mismatch or collision changed: {dims.tolist()}")
    active = profile["gripper_commissioning"][side]["active_joints"]
    robot_profile = profile["robots"][side]
    model = RobotModel(urdf_path, side)
    active_limits = validate_hand_targets(model.joints, active, side)
    close_q = positions_for(model.joints, active, model.arm_names, np.zeros(6), 1.0)
    open_q = positions_for(model.joints, active, model.arm_names, np.zeros(6), 0.0)
    clouds = hand_clouds(model, close_q, side)
    closed_hand_geometry = hand_geometry(model, close_q, side, max_points_per_link=384)
    distal_links = {f"{side}_hand__{'l' if side == 'left' else 'R'}_{name}_dip_link"
                    for name in ("thumb", "index", "middle")}
    base_world = pose(robot_profile["base_xyz_m"], robot_profile["base_rpy_rad"])
    approach_start_q = nearest_zero_configuration(model.arm_limits)
    preview_map = robot_profile.get("preview_joint_positions", {})
    preview_q = np.asarray([float(preview_map.get(name, start))
                             for name, start in zip(model.arm_names, approach_start_q)])
    approach_directions = [
        (np.array([0., 1., 0.]), np.array([1., 0., 0.]), "south_to_north"),
        (np.array([0., -1., 0.]), np.array([1., 0., 0.]), "north_to_south"),
        (np.array([1., 0., 0.]), np.array([0., 1., 0.]), "west_to_east"),
        (np.array([-1., 0., 0.]), np.array([0., 1., 0.]), "east_to_west"),
        (np.array([0., 0., -1.]), np.array([1., 0., 0.]), "top_down_x"),
        (np.array([0., 0., -1.]), np.array([0., 1., 0.]), "top_down_y"),
    ]
    candidates = [(approach, closing * roll,
                   f"{name}_roll_{'positive' if roll > 0 else 'negative'}")
                  for approach, closing, name in approach_directions for roll in (1.0, -1.0)]
    results = []
    for approach, close_hint, label in candidates:
        fit = fit_contacts(clouds, approach, close_hint, closed_hand_geometry,
                           allowed_links=distal_links)
        pre, grasp, lift = fit["matrix"].copy(), fit["matrix"].copy(), fit["matrix"].copy()
        pre[:3, 3] -= approach * 0.05
        lift[:3, 3] += [0, 0, 0.08]
        tips = {f"{side}_hand__{'l' if side == 'left' else 'R'}_{name}_dip_link"
                for name in ("thumb", "index", "middle")}
        def screen(arm_q, amount, box_center, allowed):
            return collision_screen(model, arm_q, active, amount, base_world,
                                    box_center, allowed, table["collision_top_z_m"], table["ground_z_m"])
        ik_pre = solve_ik(model, pre, base_world, open_q, initial=preview_q,
                          screen=lambda q: screen(q, 0, BOX_CENTER, set()))
        ik_grasp = solve_ik(model, grasp, base_world, close_q, ik_pre["q"],
                            screen=lambda q: screen(q, 1, BOX_CENTER, tips))
        ik_lift = solve_ik(model, lift, base_world, close_q, ik_grasp["q"],
                           screen=lambda q: screen(q, 1, BOX_CENTER + [0,0,.08], tips))
        ik = {"pregrasp": ik_pre, "grasp": ik_grasp, "lift": ik_lift}
        ik_ok = all(item["position_residual_m"] <= 0.002 and item["orientation_residual_rad"] <= 0.02
                    and np.all(item["q"] >= np.asarray(model.arm_limits)[:, 0] - 1e-8)
                    and np.all(item["q"] <= np.asarray(model.arm_limits)[:, 1] + 1e-8)
                    for item in ik.values())
        contact_ok = fit["optimizer_success"] and all(item["nearest_mean_m"] <= 0.008
                                                       for item in fit["contacts"].values())
        collision = None
        if ik_ok:
            tips = {f"{side}_hand__{'l' if side == 'left' else 'R'}_{name}_dip_link"
                    for name in ("thumb", "index", "middle")}
            pre_gate = collision_screen(model, ik_pre["q"], active, 0.0, base_world,
                                        BOX_CENTER, set(), table["collision_top_z_m"], table["ground_z_m"])
            open_grasp_gate = collision_screen(model, ik_grasp["q"], active, 0.0, base_world,
                                               BOX_CENTER, set(), table["collision_top_z_m"], table["ground_z_m"])
            grasp_gate = collision_screen(model, ik_grasp["q"], active, 1.0, base_world,
                                          BOX_CENTER, tips, table["collision_top_z_m"], table["ground_z_m"])
            lift_gate = collision_screen(model, ik_lift["q"], active, 1.0, base_world,
                                         BOX_CENTER + [0, 0, 0.08], tips, table["collision_top_z_m"], table["ground_z_m"])
            collision = {"pregrasp": pre_gate, "open_grasp": open_grasp_gate,
                         "closed_grasp": grasp_gate, "lift": lift_gate}
        closed_hand_clearance = fit["closed_hand_box_clearance"]
        closed_hand_clearance_ok = (closed_hand_clearance["noncontact_minimum_clearance_m"] is None
                                    or closed_hand_clearance["noncontact_minimum_clearance_m"] >= 0.002)
        collision_ok = (collision is not None and all(item["passed"] for item in collision.values())
                        and closed_hand_clearance_ok)
        path_gate = {"passed": False, "samples_per_segment": 25,
                     "reason": "endpoint gate failed", "failures": []}
        if ik_ok:
            # Screen the actual joint-interpolated phases, including finger
            # closing; endpoint-only IK is not a collision-free trajectory.
            for segment, start, end, a0, a1 in (
                ("approach", approach_start_q, ik_pre["q"], 0, 0),
                ("approach_lower", ik_pre["q"], ik_grasp["q"], 0, 0),
                ("finger_close", ik_grasp["q"], ik_grasp["q"], 0, 1),
                ("lift", ik_grasp["q"], ik_lift["q"], 1, 1)):
                for u in np.linspace(0, 1, 25):
                    box_target = BOX_CENTER + ([0,0,.08*u] if segment == "lift" else np.zeros(3))
                    allowed = tips if segment in ("finger_close", "lift") else set()
                    checked = screen(start+(end-start)*u, a0+(a1-a0)*u, box_target, allowed)
                    if not checked["passed"]:
                        path_gate["failures"].append({"segment": segment, "fraction": float(u),
                                                       "reasons": checked["failures"]})
            path_gate.update(passed=not path_gate["failures"], reason="sampled joint-interpolated path")
        collision_ok = collision_ok and path_gate["passed"]
        fit.update({"name": label, "ik": ik, "ik_feasible": ik_ok,
                    "contact_feasible": contact_ok, "collision": collision,
                    "path_gate": path_gate,
                    "closed_hand_clearance_feasible": closed_hand_clearance_ok,
                    "collision_feasible": collision_ok,
                    "candidate_passed": bool(ik_ok and contact_ok and collision_ok)})
        results.append(fit)
    results.sort(key=lambda item: (not item["candidate_passed"],
                                  not item["contact_feasible"], not item["ik_feasible"],
                                  not item["collision_feasible"], item["position_residual_m"],
                                  max(value["position_residual_m"] for value in item["ik"].values())))
    chosen = results[0]
    passed = chosen["candidate_passed"]
    gate_candidates = []
    for item in results:
        gate_candidates.append({
                                "approach": item["name"], "passed": item["candidate_passed"],
                                "diagnostic_only": not item["candidate_passed"],
                                "execution_allowed": bool(item["candidate_passed"]),
                                "contact_feasible": item["contact_feasible"],
                                "ik_feasible": item["ik_feasible"],
                                "collision_feasible": item["collision_feasible"],
                                "closed_hand_clearance_feasible": item["closed_hand_clearance_feasible"],
                                "closed_hand_box_clearance": item["closed_hand_box_clearance"],
                                "finger_contact_residuals": item["contacts"],
                                "palm_position_residual_m": item["position_residual_m"],
                                "palm_orientation_residual_rad": item["orientation_residual_rad"],
                                "ik": {name: {
                                              "position_residual_m": value["position_residual_m"],
                                              "orientation_residual_rad": value["orientation_residual_rad"],
                                              "optimizer_success": value["optimizer_success"]}
                                       for name, value in item["ik"].items()},
                                "collision": item["collision"]})
        gate_candidates[-1]["path_gate"] = item["path_gate"]
    plan = {"purpose": "synthetic geometric candidate only; planning does not validate physics",
            "scene_kind": SCENE_KIND, "coordinate_frame": "world", "side": side,
            "ground": {"kind": "full_flat_mesh", "z_m": float(table["ground_z_m"])},
            "thor_table": {"asset_id": table["asset_id"], "visual_top_z_m": float(table["visual_top_z_m"]),
                           "collision_top_z_m": float(table["collision_top_z_m"])},
            "profile_sha256": sha256(Path(profile_path)),
            "urdf_sha256": sha256(Path(urdf_path)), "manifest_sha256": sha256(Path(manifest_path)),
            "passed": bool(passed), "execution_allowed": bool(passed), "physics_validated": False,
            "production_collection_allowed": False,
            "start_configuration": {"arm": approach_start_q.tolist(),
                                     "hand": [float(item["open_rad"]) for item in active]},
            "start_configuration_joint_names": {"arm": list(model.arm_names),
                                                 "hand": [item["name"] for item in active]},
            "start_configuration_sources": {"arm": "source URDF zero configuration clamped to arm joint limits",
                                              "hand": "synthetic profile open targets in active-joint order",
                                              "runtime_readback_required": True},
            "box_center": BOX_CENTER.tolist(), "box_size": BOX_SIZE.tolist(),
            "mass_kg": 0.08, "friction": 0.8, "physics_dt": 0.001,
            "planner": {"algorithm": "source URDF FK+mimic, collision STL sampling, scipy least_squares contact fitting and bounded six-axis IK",
                        "mimic_relations_are_source_urdf": True,
                        "mimic_relations": [{"name": joint.get("name"),
                                             "source": joint.find("mimic").get("joint"),
                                             "multiplier": float(joint.find("mimic").get("multiplier", "1")),
                                             "offset": float(joint.find("mimic").get("offset", "0"))}
                                            for joint in model.joints if joint.find("mimic") is not None],
                        "profile_base_xyz_m": robot_profile["base_xyz_m"],
                        "profile_base_rpy_rad": robot_profile["base_rpy_rad"],
                        "thor_visual_top_z_m": float(table["visual_top_z_m"]),
                        "thor_collision_top_z_m": float(table["collision_top_z_m"]),
                        "thor_visual_collision_delta_m": float(table["visual_top_z_m"] - table["collision_top_z_m"]),
                        "cardbox_scale_uniform": 0.12, "cardbox_manifest_asset_id": card["id"],
                        "active_hand_limits_rad": {name: {"lower_rad": limits["lower_rad"], "upper_rad": limits["upper_rad"]}
                                                   for name, limits in active_limits.items()},
                        "arm_joint_limits_rad": {name: {"lower": bounds[0], "upper": bounds[1]}
                                                for name, bounds in zip(model.arm_names, model.arm_limits)},
                        "source_collision_mesh_sha256": model.mesh_inventory,
                        "preview_joint_positions_are_ik_seeds_only": True,
                        "geometry_proxy_limitations": [
                            "each source STL is hashed and deterministically surface-sampled up to 2048 points; continuous triangle intersection is not exhaustively solved",
                            "cardbox collision is the manifest boundingCube; cardboard deformation and true contact response are absent",
                            "hand fit uses inset box-face patches for intended distal contacts and penalizes sampled non-contact hand links; continuous triangle clearance is not proven",
                            "approach start arm state is URDF nearest-zero clamped to limits; runtime must read back and compare it before motion",
                            "Thor uses its audited finite XY footprint and collision top, not full mesh triangles",
                            "base/adapter pose, mass, friction and drive assumptions remain synthetic; no dynamic stability or friction-cone proof",
                            "no self-collision or camera bracket collision gate is included"],
                        "geometry_gate": {"passed": bool(passed), "execution_allowed": bool(passed),
                                          "selected_candidate": chosen["name"],
                                          "failure_reasons": [] if passed else [key for key, ok in
                                              (("opposed_finger_contact_fit", chosen["contact_feasible"]),
                                               ("bounded_six_axis_ik", chosen["ik_feasible"]),
                                               ("sampled_collision_clearance", chosen["collision_feasible"])) if not ok],
                                          "candidates": gate_candidates}}}
    if passed:
        plan.update({"hand_open": [float(item["open_rad"]) for item in active],
                     "hand_close": [float(item["close_rad"]) for item in active],
                     "hand_stiffness": float(profile["gripper_commissioning"][side]["synthetic_drive"]["stiffness"]),
                     "hand_damping": float(profile["gripper_commissioning"][side]["synthetic_drive"]["damping"]),
                     "hand_max_force": float(profile["gripper_commissioning"][side]["synthetic_drive"]["max_force"]),
                     "pregrasp": chosen["ik"]["pregrasp"]["q"].tolist(),
                     "grasp": chosen["ik"]["grasp"]["q"].tolist(),
                     "lift": chosen["ik"]["lift"]["q"].tolist(),
                     "position_residual_m": chosen["position_residual_m"],
                     "orientation_residual_rad": chosen["orientation_residual_rad"],
                     "finger_contact_residuals": chosen["contacts"],
                     "poses_world": {"pregrasp_position_m": (chosen["position"] - chosen["approach"] * 0.05).tolist(),
                                     "grasp_position_m": chosen["position"].tolist(),
                                     "lift_position_m": (chosen["position"] + [0, 0, 0.08]).tolist(),
                                     "quaternion_wxyz": chosen["rotation"].as_quat()[[3, 0, 1, 2]].tolist(),
                                     "approach_unit": chosen["approach"].tolist(),
                                     "closing_axis_unit": chosen["close_axis"].tolist()}})
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "profile", "manifest", "out"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise SystemExit("fresh private output path required")
    args.out.mkdir(parents=True)
    result_path = args.out / "plan.json"
    try:
        result = create_plan(args.urdf, args.profile, args.manifest, args.side)
        result["input_paths_private"] = {name: str(getattr(args, name).resolve())
                                         for name in ("urdf", "profile", "manifest")}
        result_path.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({"plan": str(result_path.resolve()), "passed": result["passed"],
                          "execution_allowed": result["execution_allowed"],
                          "selected": result["planner"]["geometry_gate"]["selected_candidate"]}, indent=2))
        return 0 if result["passed"] else 3
    except Exception:
        result = {"scene_kind": SCENE_KIND, "coordinate_frame": "world", "side": args.side,
                  "passed": False, "execution_allowed": False, "physics_validated": False,
                  "production_collection_allowed": False, "error": traceback.format_exc(),
                  "profile_sha256": sha256(args.profile) if args.profile.is_file() else None,
                  "input_paths_private": {name: str(getattr(args, name).resolve())
                                          for name in ("urdf", "profile", "manifest")}}
        result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"plan": str(result_path.resolve()), "passed": False,
                          "execution_allowed": False, "error": result["error"]}, indent=2), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
