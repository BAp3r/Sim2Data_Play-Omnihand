"""Multi-start geometric search for an OmniHand/cardbox grasp candidate.

The search uses source URDF FK and collision meshes only. It does not start Kit,
write USD, drive an articulation, or claim a physical grasp. The returned
candidate is diagnostic input for the arm IK and PhysX contact gates.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

try:
    from scripts.plan_contact_trajectory import (
        BOX_CENTER, BOX_SIZE, FACE_PATCH_MARGIN_M, RobotModel, box_face_patch_distances,
        box_sdf, hand_geometry, palm_goal, validate_hand_targets,
    )
except ModuleNotFoundError:  # direct ``python scripts/optimize_contact_grasp.py``
    from plan_contact_trajectory import (
        BOX_CENTER, BOX_SIZE, FACE_PATCH_MARGIN_M, RobotModel, box_face_patch_distances,
        box_sdf, hand_geometry, palm_goal, validate_hand_targets,
    )

SCENE_KIND = "thor_cardbox"
THOR_COLLISION_TOP_M = -0.0155
FULL_FLAT_GROUND_M = -0.7947
THOR_HALF_X_M, THOR_HALF_Y_M = 0.45, 0.375
NONCONTACT_M = 0.002
CONTACT_PENETRATION_M = 0.001
PREGRASP_M = 0.05


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _json(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, dict):
        return {str(k): _json(x) for k, x in v.items()}
    if isinstance(v, (tuple, list)):
        return [_json(x) for x in v]
    return v


def expand_q(model, active, values):
    """Expand 10 commanded values using only source URDF mimic relations."""
    out = {item["name"]: float(value) for item, value in zip(active, values)}
    out.update({name: 0.0 for name in model.arm_names})
    by_name = {joint.get("name"): joint for joint in model.joints}

    def resolve(name, seen):
        if name in out:
            return out[name]
        if name in seen:
            raise ValueError("cyclic mimic relation: " + name)
        joint = by_name[name]
        mimic = joint.find("mimic")
        if mimic is None:
            limit = joint.find("limit")
            value = 0.0 if limit is None else float(np.clip(
                0.0, float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0))))
        else:
            value = float(mimic.get("multiplier", "1")) * resolve(
                mimic.get("joint"), seen | {name}) + float(mimic.get("offset", "0"))
            limit = joint.find("limit")
            if limit is not None and not (float(limit.get("lower")) - 1e-8 <= value <= float(limit.get("upper")) + 1e-8):
                raise ValueError(f"mimic limit violated: {name}={value}")
        out[name] = value
        return value

    for name in by_name:
        resolve(name, set())
    return out


def distal_links(side):
    suffix = "l" if side == "left" else "R"
    return {role: f"{side}_hand__{suffix}_{role}_dip_link"
            for role in ("thumb", "index", "middle")}


def world_geometry(geometry, position, rotation):
    return {link: points @ rotation.as_matrix().T + position
            for link, points in geometry.items()}


def layout(closing_axis):
    axis = np.asarray(closing_axis, dtype=float).copy()
    axis[2] = 0.0
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        raise ValueError("closing axis must have horizontal component")
    axis /= norm
    face_axis = int(np.argmax(np.abs(axis[:2])))
    direction = 1.0 if axis[face_axis] >= 0 else -1.0
    half = BOX_SIZE / 2.0
    coords = {
        "thumb": float(BOX_CENTER[face_axis] - direction * half[face_axis]),
        "index": float(BOX_CENTER[face_axis] + direction * half[face_axis]),
        "middle": float(BOX_CENTER[face_axis] + direction * half[face_axis]),
    }
    return face_axis, direction, coords


def surface_gate(world, contacts, allow_contact):
    rows, failures = {}, []
    for link, points in world.items():
        sdf = box_sdf(points, BOX_CENTER, BOX_SIZE)
        minimum = float(np.min(sdf))
        threshold = -CONTACT_PENETRATION_M if allow_contact and link in contacts else NONCONTACT_M
        rows[link] = {"minimum_signed_clearance_m": minimum,
                      "contact_link": link in contacts,
                      "penetrating_samples": int(np.count_nonzero(sdf < 0.0))}
        if minimum < threshold:
            failures.append({"link": link, "clearance_m": minimum,
                             "required_m": threshold})
    return rows, failures


def table_gate(world):
    rows, failures = [], []
    for link, points in world.items():
        ground = float(np.min(points[:, 2]) - FULL_FLAT_GROUND_M)
        mask = ((np.abs(points[:, 0]) <= THOR_HALF_X_M) &
                (np.abs(points[:, 1]) <= THOR_HALF_Y_M))
        table = float(np.min(points[mask, 2]) - THOR_COLLISION_TOP_M) if mask.any() else None
        rows.append({"link": link, "thor_clearance_m": table, "ground_clearance_m": ground})
        if table is not None and table < -CONTACT_PENETRATION_M:
            failures.append({"link": link, "surface": "thor", "clearance_m": table})
        if ground < -CONTACT_PENETRATION_M:
            failures.append({"link": link, "surface": "full_flat_ground", "clearance_m": ground})
    return rows, failures


def contact_rows(world, contact_map, face_axis, face_coordinates):
    result = {}
    for role, link in contact_map.items():
        if link not in world:
            result[role] = {"link": link, "available": False}
            continue
        distances, projected = box_face_patch_distances(
            world[link], BOX_CENTER, BOX_SIZE, face_axis,
            face_coordinates[role], FACE_PATCH_MARGIN_M)
        n = min(8, len(distances))
        idx = np.argpartition(distances, n - 1)[:n]
        near = distances[idx]
        j = int(idx[np.argmin(near)])
        result[role] = {
            "link": link, "available": True,
            "nearest_min_m": float(np.min(near)),
            "nearest_mean_m": float(np.mean(near)),
            "nearest_max_m": float(np.max(near)),
            "face_axis": face_axis,
            "face_coordinate_m": float(face_coordinates[role]),
            "nearest_surface_point_world_m": world[link][j].tolist(),
            "nearest_box_patch_point_world_m": projected[j].tolist(),
        }
    return result


def evaluate_candidate(model, side, active, hand_open, hand_close,
                       position, rotation, approach, closing_axis,
                       path_samples=7, max_points=128):
    contacts = distal_links(side)
    face_axis, direction, face_coordinates = layout(closing_axis)
    open_geometry = hand_geometry(model, expand_q(model, active, hand_open), side,
                                  max_points_per_link=max_points)
    close_geometry = hand_geometry(model, expand_q(model, active, hand_close), side,
                                   max_points_per_link=max_points)
    open_world = world_geometry(open_geometry, position - approach * PREGRASP_M, rotation)
    open_grasp_world = world_geometry(open_geometry, position, rotation)
    close_world = world_geometry(close_geometry, position, rotation)
    open_rows, open_fail = surface_gate(open_world, set(contacts.values()), False)
    open_grasp_rows, open_grasp_fail = surface_gate(open_grasp_world, set(contacts.values()), False)
    close_rows, close_fail = surface_gate(close_world, set(contacts.values()), True)
    open_surfaces, open_surface_fail = table_gate(open_world)
    open_grasp_surfaces, open_grasp_surface_fail = table_gate(open_grasp_world)
    close_surfaces, close_surface_fail = table_gate(close_world)
    rows = contact_rows(close_world, contacts, face_axis, face_coordinates)
    contact_fail = []
    for role, row in rows.items():
        if not row.get("available"):
            contact_fail.append({"role": role, "reason": "missing distal collision link"})
        elif row["nearest_mean_m"] > 0.008:
            contact_fail.append({"role": role, "reason": "nearest mean > 8 mm",
                                 "nearest_mean_m": row["nearest_mean_m"]})
    closure = []
    closure_fail = []
    approach_path = []
    approach_fail = []
    pregrasp_position = position - approach * PREGRASP_M
    for fraction in np.linspace(0.0, 1.0, max(2, int(path_samples))):
        palm_position = pregrasp_position + fraction * (position - pregrasp_position)
        world = world_geometry(open_geometry, palm_position, rotation)
        _, bad = surface_gate(world, set(contacts.values()), False)
        surfaces, bad_surface = table_gate(world)
        approach_fail.extend([dict(x, fraction=float(fraction)) for x in bad])
        approach_fail.extend([dict(x, fraction=float(fraction)) for x in bad_surface])
        table_min = min((x["thor_clearance_m"] for x in surfaces if x["thor_clearance_m"] is not None), default=None)
        approach_path.append({"fraction": float(fraction),
                              "minimum_thor_clearance_m": table_min,
                              "failures": len(bad) + len(bad_surface)})
    for fraction in np.linspace(0.0, 1.0, max(2, int(path_samples))):
        values = hand_open + float(fraction) * (np.asarray(hand_close) - hand_open)
        geometry = hand_geometry(model, expand_q(model, active, values), side,
                                 max_points_per_link=max_points)
        world = world_geometry(geometry, position, rotation)
        _, bad = surface_gate(world, set(contacts.values()), True)
        surfaces, bad_surface = table_gate(world)
        closure_fail.extend([dict(x, fraction=float(fraction)) for x in bad])
        closure_fail.extend([dict(x, fraction=float(fraction)) for x in bad_surface])
        table_min = min((x["thor_clearance_m"] for x in surfaces if x["thor_clearance_m"] is not None), default=None)
        closure.append({"fraction": float(fraction),
                        "minimum_box_clearance_m": min(x["minimum_signed_clearance_m"] for x in surface_gate(world, set(contacts.values()), True)[0].values()),
                        "minimum_thor_clearance_m": table_min,
                        "failures": len(bad) + len(bad_surface)})
    failures = []
    failures.extend({"stage": "open_pregrasp", **x} for x in open_fail + open_surface_fail)
    failures.extend({"stage": "open_grasp", **x} for x in open_grasp_fail + open_grasp_surface_fail)
    failures.extend({"stage": "closed_endpoint", **x} for x in close_fail + close_surface_fail)
    failures.extend({"stage": "contacts", **x} for x in contact_fail)
    failures.extend({"stage": "open_approach_path", **x} for x in approach_fail)
    failures.extend({"stage": "closure_path", **x} for x in closure_fail)
    return {
        "palm_pose_world": {
            "matrix_4x4": _pose_matrix(position, rotation).tolist(),
            "position_m": np.asarray(position).tolist(),
            "quaternion_wxyz": rotation.as_quat()[[3, 0, 1, 2]].tolist(),
        },
        "approach_unit_world": (approach / np.linalg.norm(approach)).tolist(),
        "closing_axis_unit_world": (closing_axis / np.linalg.norm(closing_axis)).tolist(),
        "contact_layout": {"face_axis": face_axis, "face_direction": direction,
                            "face_coordinates_m": face_coordinates,
                            "patch_margin_m": FACE_PATCH_MARGIN_M},
        "contacts": rows,
        "clearance": {"open_pregrasp": {"links": open_rows, "surfaces": open_surfaces},
                       "open_at_grasp": {"links": open_grasp_rows, "surfaces": open_grasp_surfaces},
                       "open_approach_path": approach_path,
                       "closed_endpoint": {"links": close_rows, "surfaces": close_surfaces},
                       "closure_path": closure},
        "gate_failures": failures,
        "execution_allowed": False,
        "physics_validated": False,
        "production_collection_allowed": False,
    }


def _pose_matrix(position, rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = position
    return matrix


def _directions():
    approaches = [(np.array([0., 1., 0.]), "south_to_north"),
                  (np.array([0., -1., 0.]), "north_to_south"),
                  (np.array([1., 0., 0.]), "west_to_east"),
                  (np.array([-1., 0., 0.]), "east_to_west"),
                  (np.array([0., 0., -1.]), "top_down")]
    for approach, name in approaches:
        for close_hint, axis_name in ((np.array([1., 0., 0.]), "x"),
                                      (np.array([0., 1., 0.]), "y")):
            if abs(np.dot(approach, close_hint)) > 0.98:
                continue
            for sign in (1., -1.):
                hint = close_hint * sign
                rotation, _ = palm_goal(approach, hint)
                yield approach, hint, f"{name}_close_{axis_name}_{'positive' if sign > 0 else 'negative'}", rotation


def _seed_position(model, side, active, close, rotation, face_axis, direction):
    geometry = hand_geometry(model, expand_q(model, active, close), side, max_points_per_link=128)
    links = distal_links(side)
    half = BOX_SIZE / 2.
    targets = []
    for role, link in links.items():
        target = BOX_CENTER.copy()
        target[face_axis] = BOX_CENTER[face_axis] + (direction if role != "thumb" else -direction) * half[face_axis]
        targets.append(target - np.mean(geometry[link], axis=0) @ rotation.as_matrix().T)
    return np.mean(targets, axis=0)


def _objective(model, side, active, opened, profile_close, position0,
               rotation0, approach, closing_axis, max_points, closure_penalty=False):
    links = distal_links(side)
    face_axis, _, face_coordinates = layout(closing_axis)
    n = len(active)
    open_geometry = hand_geometry(model, expand_q(model, active, opened), side,
                                  max_points_per_link=max_points)

    def residual(x):
        close = x[:n]
        position = x[n:n + 3]
        rotation = rotation0 * Rotation.from_rotvec(x[n + 3:n + 6])
        close_geometry = hand_geometry(model, expand_q(model, active, close), side,
                                       max_points_per_link=max_points)
        opened_world = world_geometry(open_geometry, position - approach * PREGRASP_M, rotation)
        opened_grasp_world = world_geometry(open_geometry, position, rotation)
        closed_world = world_geometry(close_geometry, position, rotation)
        terms = []
        for role, link in links.items():
            points = closed_world[link]
            distance, _ = box_face_patch_distances(points, BOX_CENTER, BOX_SIZE,
                                                   face_axis, face_coordinates[role],
                                                   FACE_PATCH_MARGIN_M)
            contact_count = min(6, len(distance))
            terms.extend(np.sort(distance)[:contact_count] * 22.)
            signed = box_sdf(points, BOX_CENTER, BOX_SIZE)
            worst = np.partition(signed, contact_count - 1)[:contact_count]
            terms.extend(np.minimum(worst + 0.0002, 0.) * 120.)
        worlds = [(closed_world, 100.), (opened_world, 100.), (opened_grasp_world, 100.)]
        if closure_penalty:
            for fraction in (.25, .5, .75):
                intermediate = hand_geometry(model, expand_q(model, active, opened + fraction*(close-opened)),
                                             side, max_points_per_link=max_points)
                worlds.append((world_geometry(intermediate, position, rotation), 100.))
        for world, weight in worlds:
            for link, points in world.items():
                sdf = box_sdf(points, BOX_CENTER, BOX_SIZE)
                intended_contact = world is closed_world and link in links.values()
                if not intended_contact:
                    count = min(8, len(sdf))
                    worst = np.partition(sdf, count - 1)[:count]
                    margin = -0.0002 if link in links.values() and world is not opened_world and world is not opened_grasp_world else 0.003
                    terms.extend(np.minimum(worst - margin, 0.) * weight)
                ground = points[:, 2] - FULL_FLAT_GROUND_M
                ground_count = min(8, len(ground))
                terms.extend(np.minimum(np.partition(ground, ground_count - 1)[:ground_count], 0.) * weight)
                mask = ((np.abs(points[:, 0]) <= THOR_HALF_X_M) & (np.abs(points[:, 1]) <= THOR_HALF_Y_M))
                # Keep the residual vector length fixed even when a sampled
                # link moves outside the finite Thor footprint.
                table_terms = np.zeros(8, dtype=float)
                if mask.any():
                    table = points[mask, 2] - THOR_COLLISION_TOP_M
                    table_count = min(8, len(table))
                    table_values = np.minimum(np.partition(table, table_count - 1)[:table_count] - 0.002, 0.) * weight
                    table_terms[:len(table_values)] = table_values
                terms.extend(table_terms)
        terms.extend((close - profile_close) * .04)
        terms.extend(x[n + 3:n + 6] * .04)
        return np.asarray(terms, dtype=float)
    return residual


def search_candidates(urdf_path, profile_path, manifest_path, side,
                      starts=3, max_nfev=80, max_points=48, seed=141):
    profile = json.loads(Path(profile_path).read_text(encoding="utf-8"))
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    if profile.get("purpose") != "synthetic_commissioning_only" or profile.get("production_collection_enabled") is not False:
        raise ValueError("production-disabled synthetic profile required")
    if not math.isclose(float(profile["table"]["collision_top_z_m"]), THOR_COLLISION_TOP_M, abs_tol=1e-8):
        raise ValueError("Thor collision top datum mismatch")
    card = manifest["assets"]["card_box"]
    dimensions = np.asarray(card["usd"]["world_bounds_m"], dtype=float) * 0.12
    if card["collision"]["approximation"] != "boundingCube" or not np.allclose(dimensions, BOX_SIZE, atol=1e-7):
        raise ValueError("scaled cardbox manifest mismatch")
    model = RobotModel(urdf_path, side)
    active = profile["gripper_commissioning"][side]["active_joints"]
    names = [x["name"] for x in active]
    opened = np.asarray([x["open_rad"] for x in active], dtype=float)
    profile_close = np.asarray([x["close_rad"] for x in active], dtype=float)
    limits = validate_hand_targets(model.joints, active, side)
    lower = np.asarray([limits[name]["lower_rad"] for name in names])
    upper = np.asarray([limits[name]["upper_rad"] for name in names])
    rng = np.random.default_rng(seed)
    candidates = []
    for approach, closing, label, rotation0 in _directions():
        face_axis, direction, _ = layout(closing)
        position0 = _seed_position(model, side, active, profile_close, rotation0, face_axis, direction)
        bounds = (np.r_[lower, BOX_CENTER - .25, [-1.4] * 3],
                  np.r_[upper, BOX_CENTER + .25, [1.4] * 3])
        initial = [np.r_[profile_close, position0, np.zeros(3)]]
        for _ in range(max(1, int(starts) - 1)):
            initial.append(np.r_[np.clip(profile_close + rng.normal(0, .3, len(active)), lower + 1e-7, upper - 1e-7),
                                 position0 + rng.normal(0, .02, 3), rng.normal(0, .3, 3)])
        residual = _objective(model, side, active, opened, profile_close, position0,
                             rotation0, approach, closing, max_points, closure_penalty=True)
        best = None
        for x0 in initial:
            try:
                solution = least_squares(residual, x0, bounds=bounds, max_nfev=max_nfev,
                                         ftol=1e-7, xtol=1e-7, gtol=1e-7)
            except (ValueError, FloatingPointError):
                continue
            score = float(np.linalg.norm(solution.fun))
            if best is None or score < best[0]:
                best = (score, solution)
        if best is None:
            candidates.append({"name": label, "execution_allowed": False,
                               "gate_failures": [{"stage": "optimizer", "reason": "no finite solution"}]})
            continue
        score, solution = best
        n = len(active)
        close = solution.x[:n]
        position = solution.x[n:n + 3]
        rotation = rotation0 * Rotation.from_rotvec(solution.x[n + 3:n + 6])
        candidate = evaluate_candidate(model, side, active, opened, close, position,
                                       rotation, approach, closing, max_points=max_points * 2)
        candidate.update({"name": label,
                          "optimization": {"starts": len(initial), "nfev": int(solution.nfev),
                                           "objective_norm": score, "optimizer_success": bool(solution.success),
                                           "variables": "10 active hand joints + palm position + palm rotation"},
                          "hand_open": {"joint_names": names, "values_rad": opened.tolist()},
                          "hand_close": {"joint_names": names, "values_rad": close.tolist()},
                          "source_active_limits_rad": limits,
                          "mimic_relations_source_urdf": [
                              {"name": j.get("name"), "source": j.find("mimic").get("joint"),
                               "multiplier": float(j.find("mimic").get("multiplier", "1")),
                               "offset": float(j.find("mimic").get("offset", "0"))}
                              for j in model.joints if j.find("mimic") is not None]})
        # This search output is never executable by itself. Root must run arm IK,
        # source limit readback, full approach/lift path and PhysX contact gates.
        candidate["execution_allowed"] = False
        candidates.append(candidate)
    candidates.sort(key=lambda x: (len(x.get("gate_failures", [])),
                                   float(x.get("optimization", {}).get("objective_norm", math.inf))))
    return {"purpose": "synthetic geometric grasp search; diagnostic only",
            "scene_kind": SCENE_KIND, "side": side, "coordinate_frame": "world",
            "box_center_m": BOX_CENTER.tolist(), "box_size_m": BOX_SIZE.tolist(),
            "thor_collision_top_z_m": THOR_COLLISION_TOP_M,
            "full_flat_ground_z_m": FULL_FLAT_GROUND_M,
            "physics_validated": False, "production_collection_allowed": False,
            "execution_allowed": False, "selected_candidate": candidates[0]["name"] if candidates else None,
            "candidate_count": len(candidates), "candidates": candidates,
            "profile_sha256": sha256(profile_path), "manifest_sha256": sha256(manifest_path),
            "urdf_sha256": sha256(urdf_path)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "profile", "manifest", "out"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--starts", type=int, default=3)
    parser.add_argument("--max-nfev", type=int, default=80)
    parser.add_argument("--max-points", type=int, default=48)
    parser.add_argument("--seed", type=int, default=141)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise SystemExit("fresh private output path required")
    args.out.mkdir(parents=True)
    result_path = args.out / "search.json"
    try:
        result = search_candidates(args.urdf, args.profile, args.manifest, args.side,
                                   starts=args.starts, max_nfev=args.max_nfev,
                                   max_points=args.max_points, seed=args.seed)
        result["input_paths_private"] = {x: str(getattr(args, x).resolve()) for x in ("urdf", "profile", "manifest")}
        result_path.write_text(json.dumps(_json(result), indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({"search": str(result_path.resolve()), "selected": result["selected_candidate"],
                          "execution_allowed": False}, indent=2))
        return 3
    except Exception:
        result = {"scene_kind": SCENE_KIND, "side": args.side, "execution_allowed": False,
                  "physics_validated": False, "production_collection_allowed": False,
                  "error": traceback.format_exc()}
        result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
