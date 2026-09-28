import unittest
import importlib.util
import xml.etree.ElementTree as ET
from unittest.mock import patch

if any(importlib.util.find_spec(name) is None for name in ("numpy", "scipy", "trimesh")):
    raise unittest.SkipTest("grasp search tests require the existing simulation interpreter")

import numpy as np

from scripts import optimize_contact_grasp as search


class SearchHelpersTests(unittest.TestCase):
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
