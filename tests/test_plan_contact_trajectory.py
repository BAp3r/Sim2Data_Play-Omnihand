import importlib.util
import json
import sys
import tempfile
from contextlib import ExitStack
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

if any(importlib.util.find_spec(name) is None for name in ("numpy", "scipy", "trimesh")):
    raise unittest.SkipTest("planner tests require the existing simulation interpreter")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
import scripts.plan_contact_trajectory as planner

from scripts.plan_contact_trajectory import (
    BOX_CENTER,
    BOX_SIZE,
    fit_contacts,
    positions_for,
    palm_goal,
    validate_hand_targets,
    box_sdf,
    collision_screen,
    nearest_surface_distances,
    hand_clearance_residual,
    nearest_zero_configuration,
)


class ContactTrajectoryPlannerTests(unittest.TestCase):
    def test_closing_path_allows_separated_tip_but_never_deep_penetration(self):
        class Model:
            joints = []
            arm_names = []
            points = np.asarray([BOX_CENTER + [0.10, 0, 0.10]])

            def fk(self, q):
                return {}

            def world_samples(self, frames, base):
                yield "tip", self.points

        model = Model()
        args = (model, [], [], 0, np.eye(4), BOX_CENTER, {"tip"}, -.0155, -.7947)
        self.assertFalse(collision_screen(*args)["passed"])
        self.assertTrue(collision_screen(*args, require_contact=False)["passed"])
        model.points = BOX_CENTER[None, :]
        self.assertFalse(collision_screen(*args, require_contact=False)["passed"])

    def setUp(self):
        self.joints = ET.fromstring("""<robot>
          <joint name="left_hand__l_thumb_joint" type="revolute"><limit lower="0" upper="1"/><parent link="p"/><child link="t"/></joint>
          <joint name="left_hand__l_thumb_mimic" type="revolute"><limit lower="-1" upper="0"/><mimic joint="left_hand__l_thumb_joint" multiplier="-0.5" offset="0"/><parent link="t"/><child link="tm"/></joint>
          <joint name="left_hand__l_index_joint" type="revolute"><limit lower="-1" upper="1"/><parent link="p"/><child link="i"/></joint>
          <joint name="left_hand__l_fixed_range" type="revolute"><limit lower="0" upper="0"/><parent link="p"/><child link="z"/></joint>
        </robot>""").findall("joint")
        self.active = [
            {"name": "left_hand__l_thumb_joint", "open_rad": 0.0, "close_rad": 0.8},
            {"name": "left_hand__l_index_joint", "open_rad": -0.5, "close_rad": 0.5},
        ]

    def test_active_targets_cover_only_limited_non_mimic_dofs(self):
        limits = validate_hand_targets(self.joints, self.active, "left")
        self.assertEqual(set(limits), {item["name"] for item in self.active})
        with self.assertRaisesRegex(ValueError, "outside source limits"):
            validate_hand_targets(self.joints, [dict(self.active[0], close_rad=1.01), self.active[1]], "left")

    def test_approach_start_is_nearest_zero_clamped_to_source_limits(self):
        result = nearest_zero_configuration([[-2.0, 2.0], [0.2, 1.0], [-1.0, -0.3]])
        self.assertTrue(np.allclose(result, [0.0, 0.2, -0.3]))

    def test_mimic_position_comes_from_source_relationship(self):
        values = positions_for(self.joints, self.active, [], np.asarray([]), 0.5)
        self.assertAlmostEqual(values["left_hand__l_thumb_joint"], 0.4)
        self.assertAlmostEqual(values["left_hand__l_thumb_mimic"], -0.2)
        self.assertNotIn("left_hand__l_thumb_mimic", {item["name"] for item in self.active})

    def test_box_signed_distance_distinguishes_clearance_and_penetration(self):
        points = np.asarray([BOX_CENTER, BOX_CENTER + [0.06, 0.0, 0.0]])
        distances = box_sdf(points, BOX_CENTER, BOX_SIZE)
        self.assertLess(distances[0], 0)
        self.assertAlmostEqual(float(distances[1]), 0.018)

    def test_surface_distance_residual_cannot_cancel_symmetric_samples(self):
        target = np.zeros(3)
        points = np.asarray([[0.01, 0, 0], [-0.01, 0, 0],
                             [0, 0.01, 0], [0, -0.01, 0],
                             [0, 0, 0.01], [0, 0, -0.01],
                             [0.006, 0.008, 0], [-0.006, -0.008, 0]])
        self.assertTrue(np.allclose(points.mean(axis=0), target))
        distances = nearest_surface_distances(points, target, count=8)
        self.assertAlmostEqual(float(distances.mean()), 0.01)
        self.assertGreater(float(np.linalg.norm(distances)), 0.0)

    def test_closed_hand_penetration_residual_detects_palm_and_allows_contact_tolerance(self):
        inside_clearance = box_sdf(np.asarray([BOX_CENTER]), BOX_CENTER, BOX_SIZE)
        surface_clearance = box_sdf(np.asarray([BOX_CENTER + [BOX_SIZE[0] / 2, 0, 0]]),
                                    BOX_CENTER, BOX_SIZE)
        self.assertLess(float(hand_clearance_residual(inside_clearance)[0]), 0.0)
        self.assertLess(float(hand_clearance_residual(surface_clearance)[0]), 0.0)
        two_mm_clear = hand_clearance_residual(np.asarray([0.002]))
        self.assertTrue(np.allclose(two_mm_clear, 0))

    def test_collision_aware_fit_keeps_distal_contact_and_moves_palm_clear(self):
        approach = np.asarray([0.0, 0.0, 1.0])
        close_hint = np.asarray([1.0, 0.0, 0.0])
        rotation, close_axis = palm_goal(approach, close_hint)
        initial_position = BOX_CENTER - approach * 0.075
        extent = abs(close_axis[0]) * (BOX_SIZE[0] / 2) + abs(close_axis[1]) * (BOX_SIZE[1] / 2)
        targets = {"thumb": BOX_CENTER - close_axis * extent,
                   "index": BOX_CENTER + close_axis * extent,
                   "middle": BOX_CENTER + close_axis * extent}
        for target in targets.values():
            target[1] = BOX_CENTER[1] - BOX_SIZE[1] / 2 + 0.002
        clouds = {role: np.repeat(((target - initial_position) @ rotation.as_matrix())[None, :], 8, axis=0)
                  for role, target in targets.items()}
        distal_links = {"thumb_dip", "index_dip", "middle_dip"}
        palm_center_local = (BOX_CENTER + [0.0, 0.0, 0.02] - initial_position) @ rotation.as_matrix()
        collision_clouds = {f"{role}_dip": cloud[:1] for role, cloud in clouds.items()}
        collision_clouds["palm"] = palm_center_local[None, :]

        fitted = fit_contacts(clouds, approach, close_hint, collision_clouds,
                              allowed_links=distal_links)

        self.assertLess(max(item["nearest_mean_m"] for item in fitted["contacts"].values()), 0.002)
        palm_clearance = fitted["closed_hand_box_clearance"]["links"]["palm"]["minimum_signed_clearance_m"]
        self.assertGreaterEqual(palm_clearance, 0.00199)
        self.assertTrue(all(fitted["closed_hand_box_clearance"]["links"][link]["minimum_signed_clearance_m"]
                            >= -0.001 for link in distal_links))

    def test_contact_fit_recovers_known_opposed_finger_targets(self):
        approach = np.asarray([0.0, 1.0, 0.0])
        goal, closing = palm_goal(approach, np.asarray([1.0, 0.0, 0.0]))
        half = BOX_SIZE / 2
        origin = BOX_CENTER - approach * 0.075
        target_points = {
            "thumb": BOX_CENTER - [half[0], 0, 0],
            "index": BOX_CENTER + [half[0], 0, 0],
            "middle": BOX_CENTER + [half[0], 0, 0],
        }
        forward = goal.as_matrix()
        clouds = {role: np.repeat(((point - origin) @ forward)[None, :], 16, axis=0)
                  for role, point in target_points.items()}
        fitted = fit_contacts(clouds, approach, np.asarray([1.0, 0.0, 0.0]))
        self.assertLess(fitted["position_residual_m"], 0.002)
        self.assertLess(fitted["orientation_residual_rad"], 0.05)
        self.assertLess(np.linalg.norm(fitted["position"] - origin), 0.01)
        self.assertTrue(np.allclose(fitted["rotation"].as_matrix(), goal.as_matrix(), atol=0.05))

    def test_contact_fit_uses_each_explicit_center_without_global_leak(self):
        approach = np.asarray([0.0, 1.0, 0.0])
        closing = np.asarray([1.0, 0.0, 0.0])
        goal, _ = palm_goal(approach, closing)

        def fit_at(center):
            initial = center - approach * 0.075
            half = BOX_SIZE / 2
            targets = {
                "thumb": center - [half[0], 0, 0],
                "index": center + [half[0], 0, 0],
                "middle": center + [half[0], 0, 0],
            }
            clouds = {role: np.repeat(((point - initial) @ goal.as_matrix())[None, :], 16, axis=0)
                      for role, point in targets.items()}
            return fit_contacts(clouds, approach, closing, box_center=center)

        default_before = BOX_CENTER.copy()
        first_center = np.asarray([0.0, 0.13, 0.0145])
        second_center = np.asarray([0.11, -0.07, 0.0945])
        first = fit_at(first_center)
        second = fit_at(second_center)
        default = fit_at(BOX_CENTER)

        self.assertLess(np.linalg.norm(first["position"] - (first_center - approach * 0.075)), 0.01)
        self.assertLess(np.linalg.norm(second["position"] - (second_center - approach * 0.075)), 0.01)
        self.assertLess(np.linalg.norm(default["position"] - (BOX_CENTER - approach * 0.075)), 0.01)
        self.assertTrue(np.array_equal(planner.BOX_CENTER, default_before))

    def test_contact_fit_validates_and_copies_explicit_center(self):
        approach = np.asarray([0.0, 1.0, 0.0])
        closing = np.asarray([1.0, 0.0, 0.0])
        center = np.asarray([0.1, 0.2, 0.3])
        original_center = center.copy()
        captured = {}

        def optimizer(_residual, x0, bounds, **kwargs):
            captured["bounds"] = tuple(np.array(bound, copy=True) for bound in bounds)
            center[:] = [9.0, 9.0, 9.0]
            return SimpleNamespace(x=np.r_[original_center - approach * 0.075, np.zeros(3)],
                                   success=True, fun=np.zeros(1))

        clouds = {"thumb": np.zeros((8, 3))}
        with patch.object(planner, "least_squares", side_effect=optimizer):
            fitted = fit_contacts(clouds, approach, closing, box_center=center)

        self.assertTrue(np.array_equal(center, [9.0, 9.0, 9.0]))
        self.assertTrue(np.allclose(captured["bounds"][0][:3], original_center - 0.16))
        self.assertTrue(np.allclose(captured["bounds"][1][:3], original_center + 0.16))
        target = fitted["contacts"]["thumb"]["target_world_m"]
        self.assertAlmostEqual(target[0], original_center[0] - BOX_SIZE[0] / 2)
        self.assertAlmostEqual(target[1], original_center[1])
        self.assertAlmostEqual(target[2], original_center[2])
        for invalid in ([0.0, 0.1], [0.0, float("nan"), 0.2], [0.0, True, 0.2]):
            with self.subTest(center=invalid), self.assertRaises(ValueError):
                fit_contacts(clouds, approach, closing, box_center=invalid)

    def test_grasp_search_requires_world_frame_finite_numeric_center_and_audited_size(self):
        base = {"scene_kind": "thor_cardbox", "coordinate_frame": "world",
                "box_center_m": [0.0, 0.13, 0.0145], "box_size_m": BOX_SIZE.tolist()}
        self.assertTrue(np.array_equal(planner._grasp_search_box_center(base), base["box_center_m"]))
        malformed_centers = ([0.0, 0.1], [0.0, 0.1, 0.2, 0.3],
                             [0.0, float("nan"), 0.2], [0.0, float("inf"), 0.2],
                             [0.0, True, 0.2], [0.0, "0.1", 0.2])
        for center in malformed_centers:
            with self.subTest(center=center), self.assertRaises(ValueError):
                planner._grasp_search_box_center(dict(base, box_center_m=center))
        with self.assertRaisesRegex(ValueError, "box size mismatch"):
            planner._grasp_search_box_center(dict(base, box_size_m=[0.084, 0.061, 0.06]))
        with self.assertRaisesRegex(ValueError, "scene/frame mismatch"):
            planner._grasp_search_box_center(dict(base, coordinate_frame="robot"))

    def test_create_plan_binds_search_center_through_contacts_ik_collision_and_output(self):
        class Model:
            joints = []
            arm_names = [f"left_arm__joint{i}" for i in range(1, 7)]
            arm_limits = [(-1.0, 1.0)] * 6
            mesh_inventory = []

        def solve_ik(*args, **kwargs):
            ik_targets_seen.append(np.array(args[1][:3, 3], copy=True))
            return {"q": np.zeros(6), "position_residual_m": 0.0,
                    "orientation_residual_rad": 0.0, "optimizer_success": True}

        ik_targets_seen = []
        centers_seen_by_clearance = []
        centers_seen_by_screen = []
        centers_seen_by_patch = []

        def geometry(*args, **kwargs):
            roles = planner.contact_roles("left")
            result = {f"left_hand__l_{role}_dip_link": np.zeros((8, 3)) for role in roles}
            result["left_hand__l_palm"] = np.ones((8, 3))
            return result

        def face_patch(points, center, size, axis, coordinate, *args, **kwargs):
            centers_seen_by_patch.append(np.array(center, copy=True))
            return np.zeros(len(points)), np.zeros((len(points), 3))

        def clearance(geometry, position, rotation, center, size):
            centers_seen_by_clearance.append(np.array(center, copy=True))
            return {"links": {"left_hand__l_palm": {"minimum_signed_clearance_m": 0.01}},
                    "noncontact_minimum_clearance_m": 0.01,
                    "minimum_signed_clearance_m": 0.01,
                    "penetrating_samples_beyond_1mm": 0}

        def screen(*args, **kwargs):
            centers_seen_by_screen.append(np.array(args[5], copy=True))
            return {"passed": True, "failures": []}

        def is_lift_of(center, expected):
            return (np.allclose(center[:2], expected[:2]) and
                    expected[2] - 1e-9 <= center[2] <= expected[2] + 0.08 + 1e-9)

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            urdf, profile_path, manifest_path, search_path = (root / name for name in
                                                               ("robot.urdf", "profile.json", "manifest.json", "search.json"))
            urdf.write_text("synthetic test input", encoding="utf-8")
            profile = {
                "purpose": "synthetic_commissioning_only", "production_collection_enabled": False,
                "table": {"collision_top_z_m": -0.0155, "visual_top_z_m": -0.01,
                          "ground_z_m": -0.1, "asset_id": "thor-test"},
                "gripper_commissioning": {"left": {"active_joints": [],
                                                      "synthetic_drive": {"stiffness": 1,
                                                                          "damping": 1,
                                                                          "max_force": 1}}},
                "robots": {"left": {"base_xyz_m": [0, 0, 0], "base_rpy_rad": [0, 0, 0],
                                     "preview_joint_positions": {}}},
            }
            manifest = {"assets": {"card_box": {"id": "cardbox-test",
                                                  "usd": {"world_bounds_m": [0.7, 0.5, 0.5]},
                                                  "collision": {"approximation": "boundingCube"}}}}
            profile_path.write_text(json.dumps(profile), encoding="utf-8")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            center_a = [0.0, 0.13, 0.0145]
            center_b = [0.12, -0.08, 0.1145]

            def write_search(center):
                matrix = np.eye(4)
                matrix[:3, 3] = center
                search = {"scene_kind": "thor_cardbox", "coordinate_frame": "world", "side": "left",
                          "box_center_m": center, "box_size_m": BOX_SIZE.tolist(), "candidates": [{
                              "name": "test_candidate", "approach_unit_world": [0, 1, 0],
                              "closing_axis_unit_world": [1, 0, 0],
                              "palm_pose_world": {"matrix_4x4": matrix.tolist()},
                              "hand_open": {"joint_names": [], "values_rad": []},
                              "hand_close": {"joint_names": [], "values_rad": []},
                              "optimization": {"objective_norm": 0.0}}],
                          "urdf_sha256": planner.sha256(urdf),
                          "profile_sha256": planner.sha256(profile_path),
                          "manifest_sha256": planner.sha256(manifest_path)}
                search_path.write_text(json.dumps(search), encoding="utf-8")

            patches = [
                patch.object(planner, "RobotModel", return_value=Model()),
                patch.object(planner, "validate_hand_targets", return_value={}),
                patch.object(planner, "validate_gesture_channels", return_value={}),
                patch.object(planner, "closing_joint_names", return_value=[]),
                patch.object(planner, "positions_for", return_value={}),
                patch.object(planner, "hand_clouds", return_value={}),
                patch.object(planner, "hand_geometry", side_effect=geometry),
                patch.object(planner, "box_face_patch_distances", side_effect=face_patch),
                patch.object(planner, "hand_box_clearance", side_effect=clearance),
                patch.object(planner, "collision_screen", side_effect=screen),
                patch.object(planner, "solve_ik", side_effect=solve_ik),
            ]
            before = planner.BOX_CENTER.copy()
            with ExitStack() as stack:
                for context in patches:
                    stack.enter_context(context)
                results = []
                for center in (center_a, center_b):
                    write_search(center)
                    patch_start = len(centers_seen_by_patch)
                    clearance_start = len(centers_seen_by_clearance)
                    screen_start = len(centers_seen_by_screen)
                    ik_start = len(ik_targets_seen)
                    results.append(planner.create_plan(urdf, profile_path, manifest_path, "left", search_path))
                    expected = np.asarray(center)
                    self.assertTrue(all(np.allclose(item, expected)
                                        for item in centers_seen_by_patch[patch_start:]))
                    self.assertTrue(all(np.allclose(item, expected)
                                        for item in centers_seen_by_clearance[clearance_start:]))
                    self.assertTrue(all(np.allclose(item, expected) or
                                        is_lift_of(item, expected)
                                        for item in centers_seen_by_screen[screen_start:]))
                    self.assertTrue(all(np.isclose(item[0], expected[0]) and
                                        expected[1] - 0.05 - 1e-9 <= item[1] <= expected[1] + 1e-9 and
                                        expected[2] - 1e-9 <= item[2] <= expected[2] + 0.08 + 1e-9
                                        for item in ik_targets_seen[ik_start:]))

        for expected, result in zip((center_a, center_b), results):
            self.assertTrue(np.allclose(result["box_center"], expected))
            self.assertTrue(np.allclose(result["poses_world"]["grasp_position_m"], expected))
            thumb_contact = result["planner"]["geometry_gate"]["candidates"][0]["finger_contact_residuals"]["thumb"]
            self.assertAlmostEqual(thumb_contact["face_coordinate_m"], expected[0] - BOX_SIZE[0] / 2)
        for center in centers_seen_by_patch + centers_seen_by_clearance:
            self.assertTrue(any(np.allclose(center, expected) for expected in (center_a, center_b)))
        self.assertTrue(np.array_equal(planner.BOX_CENTER, before))

    def test_create_plan_without_search_retains_nominal_center(self):
        class Model:
            joints = []
            arm_names = [f"left_arm__joint{i}" for i in range(1, 7)]
            arm_limits = [(-1.0, 1.0)] * 6
            mesh_inventory = []

        fit_centers = []
        screen_centers = []

        def fit(*args, **kwargs):
            center = np.asarray(kwargs["box_center"])
            fit_centers.append(center.copy())
            approach, close_hint = args[1], args[2]
            rotation, close_axis = palm_goal(approach, close_hint)
            position = center - approach * 0.075
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = rotation.as_matrix(), position
            contacts = {role: {"nearest_mean_m": 0.0} for role in planner.contact_roles("left")}
            return {"position": position, "rotation": rotation, "matrix": matrix,
                    "approach": approach, "close_axis": close_axis, "contacts": contacts,
                    "closed_hand_box_clearance": {"noncontact_minimum_clearance_m": 0.01, "links": {}},
                    "position_residual_m": 0.0, "orientation_residual_rad": 0.0,
                    "optimizer_success": True, "objective_norm": 0.0}

        def solve_ik(*args, **kwargs):
            return {"q": np.zeros(6), "position_residual_m": 0.0,
                    "orientation_residual_rad": 0.0, "optimizer_success": True}

        def screen(*args, **kwargs):
            screen_centers.append(np.asarray(args[5]).copy())
            return {"passed": True, "failures": []}

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            urdf, profile_path, manifest_path = (root / name for name in
                                                  ("robot.urdf", "profile.json", "manifest.json"))
            urdf.write_text("synthetic test input", encoding="utf-8")
            profile = {
                "purpose": "synthetic_commissioning_only", "production_collection_enabled": False,
                "table": {"collision_top_z_m": -0.0155, "visual_top_z_m": -0.01,
                          "ground_z_m": -0.1, "asset_id": "thor-test"},
                "gripper_commissioning": {"left": {"active_joints": [],
                                                      "synthetic_drive": {"stiffness": 1,
                                                                          "damping": 1,
                                                                          "max_force": 1}}},
                "robots": {"left": {"base_xyz_m": [0, 0, 0], "base_rpy_rad": [0, 0, 0],
                                     "preview_joint_positions": {}}},
            }
            manifest = {"assets": {"card_box": {"id": "cardbox-test",
                                                  "usd": {"world_bounds_m": [0.7, 0.5, 0.5]},
                                                  "collision": {"approximation": "boundingCube"}}}}
            profile_path.write_text(json.dumps(profile), encoding="utf-8")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with patch.object(planner, "RobotModel", return_value=Model()), \
                    patch.object(planner, "validate_hand_targets", return_value={}), \
                    patch.object(planner, "validate_gesture_channels", return_value={}), \
                    patch.object(planner, "closing_joint_names", return_value=[]), \
                    patch.object(planner, "positions_for", return_value={}), \
                    patch.object(planner, "hand_clouds", return_value={}), \
                    patch.object(planner, "hand_geometry", return_value={}), \
                    patch.object(planner, "fit_contacts", side_effect=fit), \
                    patch.object(planner, "collision_screen", side_effect=screen), \
                    patch.object(planner, "solve_ik", side_effect=solve_ik):
                result = planner.create_plan(urdf, profile_path, manifest_path, "left")

        self.assertTrue(np.array_equal(result["box_center"], BOX_CENTER))
        self.assertEqual(len(fit_centers), 12)
        self.assertTrue(all(np.array_equal(center, BOX_CENTER) for center in fit_centers))
        self.assertTrue(all(np.allclose(center, BOX_CENTER) or
                            np.allclose(center[:2], BOX_CENTER[:2]) and
                            BOX_CENTER[2] - 1e-9 <= center[2] <= BOX_CENTER[2] + 0.08 + 1e-9
                            for center in screen_centers))


if __name__ == "__main__":
    unittest.main()
