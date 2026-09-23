import json
import unittest
from pathlib import Path

from sim2data.control.gripper import load_gripper_map


ROOT = Path(__file__).resolve().parents[1]
URDFS = ROOT / ".local/evidence/m5/astra/inputs01"
PROFILE = ROOT / "configs/commissioning.synthetic.json"


@unittest.skipUnless((URDFS / "left.urdf").is_file(), "private commissioning URDFs unavailable")
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
            for value in mapping.expand(0.5).values():
                self.assertTrue(-3.2 <= value <= 3.2)
            self.assertFalse(any("dip" in name for name in mapping.expand(0.5)))

    def test_profile_keeps_production_disabled(self):
        profile = json.loads(PROFILE.read_text())
        self.assertFalse(profile["production_collection_enabled"])
        self.assertEqual(profile["gripper_commissioning"]["left"]["mapping_status"],
                         "synthetic_assumption_pending_physx_response")
