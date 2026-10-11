import json
import copy
from pathlib import Path
import unittest

from sim2data.assets.manifest import load_manifest, validate_manifest


class AssetManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]
        cls.path = cls.root / "configs" / "asset_manifest.json"
        cls.manifest = json.loads(cls.path.read_text(encoding="utf-8"))

    def test_manifest_loads_with_contract_validator(self):
        self.assertEqual(validate_manifest(self.manifest), [])
        self.assertEqual(load_manifest(self.path)["schema_version"], 1)

    def test_card_box_is_relative_and_static_only(self):
        card = self.manifest["assets"]["card_box"]
        self.assertTrue(card["relative_path"].startswith("5.1.0/"))
        self.assertEqual(card["usd"]["world_bounds_m"], [0.7, 0.5, 0.5])
        self.assertEqual(card["collision"]["approximation"], "boundingCube")
        self.assertIsNone(card["rigid_body"]["mass_kg"])
        self.assertIsNone(card["rigid_body"]["inertia_kg_m2"])

    def test_robot_candidates_do_not_turn_joint_count_into_action_dim(self):
        for candidate in self.manifest["robot_candidates"]:
            self.assertEqual(candidate["arm_control"]["active_joint_count"], 6)
            self.assertNotIn("action_dim", candidate["arm_control"])
            self.assertEqual(candidate["original_end_effector_boundary"]["parent_link"], "link6")
            self.assertIn("PHYSICAL_PENDING", candidate["status"])

    def test_o10_has_ten_commands_and_six_mimic_joints_per_side(self):
        hand = next(item for item in self.manifest["hand_candidates"]
                    if item["id"] == "omnihand_2025_o10")
        self.assertEqual(hand["geometry_snapshot"]["active_motor_count"], 10)
        self.assertEqual(hand["geometry_snapshot"]["passive_mimic_count"], 6)
        for side in ("left", "right"):
            self.assertEqual(len(hand["active_motor_channels"][side]), 10)
            self.assertEqual(len(hand["passive_mimic_joints"][side]), 6)
        self.assertEqual(hand["sdk_control"]["command_channel_count"], 10)
        self.assertFalse(hand["urdf_sdk_reconciliation"]["matches_exactly"])

    def test_public_manifest_contains_no_machine_path(self):
        text = self.path.read_text(encoding="utf-8").lower()
        for marker in ("192.168.", "\\\\", "/mnt/", "/workspace/", "c:\\"):
            self.assertNotIn(marker, text)

    def test_invalid_numeric_units_dimensions_and_hashes_are_rejected(self):
        for value in (True, float("inf"), float("nan"), -1.0):
            with self.subTest(dimension=value):
                changed = copy.deepcopy(self.manifest)
                changed["assets"]["card_box"]["usd"]["world_bounds_m"][0] = value
                self.assertIn("assets.card_box.usd.world_bounds_m", validate_manifest(changed))
        changed = copy.deepcopy(self.manifest)
        changed["schema_version"] = True
        changed["assets"]["card_box"]["usd"]["meters_per_unit"] = True
        changed["assets"]["card_box"]["sha256"] = "+" + "1" * 63
        errors = validate_manifest(changed)
        self.assertIn("schema_version", errors)
        self.assertIn("assets.card_box.usd.meters_per_unit", errors)
        self.assertIn("assets.card_box.sha256", errors)

    def test_selected_asset_cannot_escape_the_external_root(self):
        for value in ("../other.usd", "/private/other.usd", "C:/private/other.usd", "safe/../../other.usd"):
            with self.subTest(path=value):
                changed = copy.deepcopy(self.manifest)
                changed["assets"]["card_box"]["relative_path"] = value
                self.assertIn("assets.card_box.relative_path", validate_manifest(changed))


if __name__ == "__main__":
    unittest.main()
