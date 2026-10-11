import unittest
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import patch

if any(importlib.util.find_spec(name) is None for name in ("numpy", "scipy", "trimesh")):
    raise unittest.SkipTest("grasp search tests require the existing simulation interpreter")

import numpy as np

from scripts import optimize_contact_grasp as search


class SearchHelpersTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = json.loads(Path("configs/commissioning.synthetic.json").read_text(encoding="utf-8"))
        cls.manifest_path = Path("configs/asset_manifest.json")

    def test_layout_uses_opposed_box_faces(self):
        axis, direction, coordinates = search.layout(np.array([1.0, 0.0, 0.0]))
        self.assertEqual(axis, 0)
        self.assertEqual(direction, 1.0)
        self.assertAlmostEqual(coordinates["thumb"], search.BOX_CENTER[0] - search.BOX_SIZE[0] / 2)
        self.assertAlmostEqual(coordinates["index"], search.BOX_CENTER[0] + search.BOX_SIZE[0] / 2)
        self.assertAlmostEqual(coordinates["middle"], coordinates["index"])

    def test_expand_q_applies_source_mimic_relationship(self):
        root = ET.fromstring("""<robot>
          <joint name="active" type="revolute"><parent link="p"/><child link="c"/>
            <limit lower="-1" upper="1"/></joint>
          <joint name="mimic" type="revolute"><parent link="c"/><child link="d"/>
            <limit lower="-2" upper="2"/><mimic joint="active" multiplier="1.5" offset="0.1"/></joint>
        </robot>""")
        model = type("Model", (), {"joints": root.findall("joint"), "arm_names": []})()
        values = search.expand_q(model, [{"name": "active"}], [0.4])
        self.assertAlmostEqual(values["active"], 0.4)
        self.assertAlmostEqual(values["mimic"], 0.7)

    def test_closure_joint_sets_are_side_specific(self):
        left = self.profile["gripper_commissioning"]["left"]["active_joints"]
        right = self.profile["gripper_commissioning"]["right"]["active_joints"]
        left_allowed, left_fixed = search.closure_indices("left", left)
        right_allowed, right_fixed = search.closure_indices("right", right)
        self.assertEqual(
            {left[i]["name"] for i in left_allowed},
            {"left_hand__l_thumb_mcp_joint", "left_hand__l_index_pip_joint",
             "left_hand__l_middle_pip_joint"},
        )
        self.assertEqual(
            {right[i]["name"] for i in right_allowed},
            {"right_hand__R_thumb_mcp_joint", "right_hand__R_index_pip_joint"},
        )
        self.assertEqual(len(left_allowed), 3)
        self.assertEqual(len(right_allowed), 2)
        self.assertIn("left_hand__l_thumb_abad_joint", {left[i]["name"] for i in left_fixed})
        self.assertIn("right_hand__R_thumb_abad_joint", {right[i]["name"] for i in right_fixed})
        self.assertEqual(set(search.distal_links("right")), {"thumb", "index"})

    def test_fixed_hand_channels_remain_at_open_through_closure(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                active = self.profile["gripper_commissioning"][side]["active_joints"]
                allowed, fixed = search.closure_indices(side, active)
                opened = np.asarray([item["open_rad"] for item in active])
                closed_allowed = np.asarray([active[i]["close_rad"] for i in allowed])
                for fraction in np.linspace(0.0, 1.0, 9):
                    close = search.close_from_allowed(
                        side, active, opened,
                        opened[allowed] + fraction * (closed_allowed - opened[allowed]),
                    )
                    np.testing.assert_array_equal(close[fixed], opened[fixed])

    def test_search_optimizer_uses_only_gesture_closure_variables(self):
        manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        self.assertIn("assets", manifest)
        for side in ("left", "right"):
            with self.subTest(side=side):
                active = self.profile["gripper_commissioning"][side]["active_joints"]
                allowed, fixed = search.closure_indices(side, active)
                names = [item["name"] for item in active]
                opened = np.asarray([item["open_rad"] for item in active])
                active_links = search.distal_links(side)
                points = np.asarray([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0],
                                     [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
                seen_q = []

                def geometry(_model, q, _side, **_kwargs):
                    seen_q.append(dict(q))
                    return {link: points.copy() for link in active_links.values()} | {
                        f"{side}_hand__palm": points.copy()
                    }

                def limits(_joints, joints, _side):
                    return {item["name"]: {
                        "lower_rad": min(item["open_rad"], item["close_rad"]) - 1.0,
                        "upper_rad": max(item["open_rad"], item["close_rad"]) + 1.0,
                    } for item in joints}

                captured = {}

                def fake_least_squares(fun, x0, bounds, **_kwargs):
                    x0 = np.asarray(x0, dtype=float)
                    lower, upper = (np.asarray(v, dtype=float) for v in bounds)
                    expected_size = len(allowed) + 6
                    self.assertEqual(x0.shape, (expected_size,))
                    self.assertEqual(lower.shape, (expected_size,))
                    self.assertEqual(upper.shape, (expected_size,))
                    captured["x0"] = x0.copy()
                    solved = x0.copy()
                    solved[0] += 0.01
                    residual = fun(solved)
                    return SimpleNamespace(x=solved, fun=residual, success=True, nfev=1)

                fake_model = SimpleNamespace(joints=[], arm_names=[])
                with (patch.object(search, "RobotModel", return_value=fake_model),
                      patch.object(search, "validate_hand_targets", side_effect=limits),
                      patch.object(search, "_directions", return_value=[
                          (np.asarray([0.0, 1.0, 0.0]), np.asarray([1.0, 0.0, 0.0]),
                           "test_direction", search.Rotation.identity())]),
                      patch.object(search, "_seed_position", return_value=search.BOX_CENTER.copy()),
                      patch.object(search, "hand_geometry", side_effect=geometry),
                      patch.object(search, "evaluate_candidate", return_value={"gate_failures": []}),
                      patch.object(search, "least_squares", side_effect=fake_least_squares)):
                    result = search.search_candidates(
                        Path("configs/commissioning.synthetic.json"),
                        Path("configs/commissioning.synthetic.json"),
                        self.manifest_path, side, starts=1,
                    )

                candidate = result["candidates"][0]
                closed = np.asarray(candidate["hand_close"]["values_rad"])
                self.assertEqual(candidate["optimization"]["variables"][:len(allowed)],
                                 [names[i] for i in allowed])
                self.assertEqual(candidate["closure_joint_names"], [names[i] for i in allowed])
                self.assertEqual(candidate["fixed_joint_names"], [names[i] for i in fixed])
                np.testing.assert_array_equal(closed[fixed], opened[fixed])
                self.assertGreater(abs(closed[allowed[0]] - opened[allowed[0]]), 0.0)
                self.assertTrue(seen_q)
                for q in seen_q:
                    for index in fixed:
                        self.assertEqual(q[names[index]], opened[index])

    def test_surface_gate_requires_noncontact_clearance(self):
        world = {
            "palm": np.asarray([[search.BOX_CENTER[0], search.BOX_CENTER[1], search.BOX_CENTER[2]]]),
            "tip": np.asarray([[search.BOX_CENTER[0] + search.BOX_SIZE[0] / 2, search.BOX_CENTER[1], search.BOX_CENTER[2]]]),
        }
        rows, failures = search.surface_gate(world, {"tip"}, allow_contact=True)
        self.assertLess(rows["palm"]["minimum_signed_clearance_m"], 0.0)
        self.assertTrue(any(item["link"] == "palm" for item in failures))
        self.assertFalse(any(item["link"] == "tip" for item in failures))

    def test_evaluate_candidate_is_diagnostic_only_and_contains_required_fields(self):
        model = type("Model", (), {"joints": [], "arm_names": []})()
        active = [{"name": "active", "open_rad": 0.0, "close_rad": 0.5}]
        links = search.distal_links("left")
        geometry = {
            links["thumb"]: np.asarray([[-search.BOX_SIZE[0] / 2, 0.0, 0.0]]),
            links["index"]: np.asarray([[search.BOX_SIZE[0] / 2, 0.0, 0.0]]),
            links["middle"]: np.asarray([[search.BOX_SIZE[0] / 2, 0.0, 0.0]]),
            "left_hand__l_palm": np.asarray([[0.0, 0.0, 0.08]]),
        }
        with patch.object(search, "hand_geometry", return_value=geometry):
            candidate = search.evaluate_candidate(
                model, "left", active, np.asarray([0.0]), np.asarray([0.5]),
                search.BOX_CENTER, search.Rotation.identity(),
                np.asarray([0.0, 1.0, 0.0]), np.asarray([1.0, 0.0, 0.0]),
                path_samples=3, max_points=8)
        self.assertFalse(candidate["execution_allowed"])
        self.assertFalse(candidate["physics_validated"])
        self.assertEqual(set(candidate["contacts"]), {"thumb", "index", "middle"})
        self.assertEqual(len(candidate["clearance"]["closure_path"]), 3)
        self.assertEqual(len(candidate["clearance"]["open_approach_path"]), 3)
        self.assertIn("open_at_grasp", candidate["clearance"])
        self.assertIn("matrix_4x4", candidate["palm_pose_world"])


if __name__ == "__main__":
    unittest.main()
