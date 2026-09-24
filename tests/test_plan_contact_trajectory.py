import importlib.util
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

if any(importlib.util.find_spec(name) is None for name in ("numpy", "scipy", "trimesh")):
    raise unittest.SkipTest("planner tests require the existing simulation interpreter")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.plan_contact_trajectory import (
    BOX_CENTER,
    BOX_SIZE,
    fit_contacts,
    positions_for,
    palm_goal,
    validate_hand_targets,
    box_sdf,
    nearest_surface_distances,
    hand_clearance_residual,
    nearest_zero_configuration,
)


class ContactTrajectoryPlannerTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()