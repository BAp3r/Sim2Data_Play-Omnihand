import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from sim2data.control.gripper import load_gripper_map


ROOT = Path(__file__).resolve().parents[1]
URDFS = ROOT / ".local/evidence/m5/astra/inputs01"
PROFILE = ROOT / "configs/commissioning.synthetic.json"


@unittest.skipUnless(all((URDFS / f"{side}.urdf").is_file() for side in ("left", "right")),
                     "private commissioning URDFs unavailable")
class GripperControlTests(unittest.TestCase):
    def test_each_side_expands_scalar_to_ten_distinct_active_joints(self):
        left = load_gripper_map("left", URDFS / "left.urdf", PROFILE)
        right = load_gripper_map("right", URDFS / "right.urdf", PROFILE)
        self.assertEqual(len(left.expand(0.0)), 10)
        self.assertEqual(len(right.expand(1.0)), 10)
        self.assertNotEqual(left.expand(1.0), right.expand(1.0))

    def test_limits_are_respected_and_mimics_are_not_commanded(self):
        for side in ("left", "right"):
            mapping = load_gripper_map(side, URDFS / f"{side}.urdf", PROFILE)
            joints = {j.attrib["name"]: j for j in ET.parse(URDFS / f"{side}.urdf").getroot().findall("joint")}
            profile = json.loads(PROFILE.read_text())["gripper_commissioning"][side]
            for endpoint in profile["active_joints"]:
                joint = joints[endpoint["name"]]
                self.assertIsNone(joint.find("mimic"))
                limit = joint.find("limit")
                for key in ("open_rad", "close_rad"):
                    self.assertGreaterEqual(endpoint[key], float(limit.attrib["lower"]))
                    self.assertLessEqual(endpoint[key], float(limit.attrib["upper"]))
            for amount in (0, 0.5, 1):
                for name, value in mapping.expand(amount).items():
                    limit = joints[name].find("limit")
                    self.assertGreaterEqual(value, float(limit.attrib["lower"]))
                    self.assertLessEqual(value, float(limit.attrib["upper"]))
            self.assertFalse(any("dip" in name for name in mapping.expand(0.5)))

    def test_profile_keeps_production_disabled(self):
        profile = json.loads(PROFILE.read_text())
        self.assertFalse(profile["production_collection_enabled"])
        self.assertEqual(profile["gripper_commissioning"]["left"]["mapping_status"],
                         "synthetic_assumption_pending_physx_response")
