import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from sim2data.control.gripper import (
    allowed_closing_joint_names,
    load_gripper_map,
    validate_gesture_targets,
)


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


class GestureSemanticsTests(unittest.TestCase):
    """Regression checks that do not depend on ignored private commissioning URDFs."""

    _LIMITS = {
        "left": {
            "thumb_roll_joint": (0.034, 1.274),
            "thumb_abad_joint": (-1.642, 0.045),
            "thumb_mcp_joint": (0.0, 0.842),
            "index_abad_joint": (-0.087, 0.105),
            "index_pip_joint": (-1.571, 0.0),
            "middle_pip_joint": (-1.571, 0.0),
            "ring_abad_joint": (-0.105, 0.070),
            "ring_pip_joint": (-1.571, 0.0),
            "pinky_abad_joint": (-0.052, 0.140),
            "pinky_pip_joint": (-1.571, 0.0),
        },
        "right": {
            "thumb_roll_joint": (-0.524, 0.698),
            "thumb_abad_joint": (-1.464, 0.045),
            "thumb_mcp_joint": (0.0, 0.842),
            "index_abad_joint": (-0.227, 0.0),
            "index_pip_joint": (0.0, 1.571),
            "middle_pip_joint": (0.0, 1.571),
            "ring_abad_joint": (0.0, 0.169),
            "ring_pip_joint": (0.0, 1.571),
            "pinky_abad_joint": (0.0, 0.185),
            "pinky_pip_joint": (0.0, 1.571),
        },
    }

    def _write_urdf(self, root: Path, side: str) -> Path:
        robot = ET.Element("robot", name=f"{side}_fixture")
        hand_prefix = f"{side}_hand__{'l' if side == 'left' else 'R'}_"
        for suffix, (lower, upper) in self._LIMITS[side].items():
            joint = ET.SubElement(robot, "joint", name=hand_prefix + suffix, type="revolute")
            ET.SubElement(joint, "limit", lower=str(lower), upper=str(upper))
        # These are deliberately present to prove that mimic joints remain
        # read-only and cannot leak into the command dictionary.
        for suffix, source in (
            ("thumb_pip_joint", "thumb_mcp_joint"),
            ("thumb_dip_joint", "thumb_mcp_joint"),
            ("index_dip_joint", "index_pip_joint"),
            ("middle_dip_joint", "middle_pip_joint"),
            ("ring_dip_joint", "ring_pip_joint"),
            ("pinky_dip_joint", "pinky_pip_joint"),
        ):
            joint = ET.SubElement(robot, "joint", name=hand_prefix + suffix, type="revolute")
            ET.SubElement(joint, "limit", lower="0", upper="2")
            ET.SubElement(joint, "mimic", joint=hand_prefix + source, multiplier="1", offset="0")
        path = root / f"{side}.urdf"
        ET.ElementTree(robot).write(path, encoding="utf-8", xml_declaration=True)
        return path

    def _maps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = {side: self._write_urdf(root, side) for side in ("left", "right")}
            yield root, paths

    def test_prepare_reset_keep_fixed_gesture_channels(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            maps = {
                side: load_gripper_map(side, self._write_urdf(root, side), PROFILE)
                for side in ("left", "right")
            }
            for side, mapping in maps.items():
                self.assertEqual(mapping.prepare_targets(), mapping.open_targets())
                self.assertEqual(mapping.reset_targets(), mapping.open_targets())
                mid = mapping.expand(0.5)
                for name in mapping.fixed_joint_names:
                    self.assertEqual(mid[name], mapping.open_targets()[name])
                self.assertEqual(set(mapping.command_joint_names), set(mapping.open_targets()))

    def test_tripod_and_pinch_closing_sets_are_different(self):
        self.assertEqual(
            set(allowed_closing_joint_names("left")),
            {
                "left_hand__l_thumb_mcp_joint",
                "left_hand__l_index_pip_joint",
                "left_hand__l_middle_pip_joint",
            },
        )
        self.assertEqual(
            set(allowed_closing_joint_names("right")),
            {"right_hand__R_thumb_mcp_joint", "right_hand__R_index_pip_joint"},
        )
        self.assertNotEqual(set(allowed_closing_joint_names("left")), set(allowed_closing_joint_names("right")))

    def test_open_jaw_keeps_unused_fingers_straight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for side in ("left", "right"):
                mapping = load_gripper_map(side, self._write_urdf(root, side), PROFILE)
                for joint in mapping.joints:
                    if "pip_joint" in joint.name or "thumb_mcp" in joint.name:
                        self.assertEqual(joint.open_rad, 0.0)
                        if not joint.allow_close:
                            for amount in (0, 0.5, 1, 0):
                                self.assertEqual(mapping.expand(amount)[joint.name], 0.0)
                    if "thumb_roll" in joint.name or "thumb_abad" in joint.name:
                        self.assertFalse(joint.allow_close)
                        self.assertLessEqual(abs(joint.open_rad), 0.05)

    def test_gesture_adjustment_is_explicit_and_checked(self):
        profile = json.loads(PROFILE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            urdf = self._write_urdf(root, "left")
            active = profile["gripper_commissioning"]["left"]["active_joints"]
            target = next(item for item in active if item["name"].endswith("thumb_mcp_joint"))
            self.assertEqual(target["legacy_open_rad"], -0.5)
            self.assertEqual(target["gesture_open_adjustment_rad"], -0.5)
            del target["gesture_open_adjustment_rad"]
            bad = root / "bad.json"
            bad.write_text(json.dumps(profile), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "explicit legacy mapping"):
                load_gripper_map("left", urdf, bad)

    def test_mimics_are_not_commands_and_endpoints_are_limited(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for side in ("left", "right"):
                urdf = self._write_urdf(root, side)
                mapping = load_gripper_map(side, urdf, PROFILE)
                joints = {
                    node.attrib["name"]: node
                    for node in ET.parse(urdf).getroot().findall("joint")
                }
                self.assertEqual(len(mapping.command_joint_names), 10)
                self.assertTrue(all(joints[name].find("mimic") is None for name in mapping.command_joint_names))
                for name, target in ((name, target) for name, target in mapping.mapping_table.items()):
                    limit = joints[name].find("limit")
                    self.assertGreaterEqual(target["open_rad"], float(limit.attrib["lower"]))
                    self.assertLessEqual(target["open_rad"], float(limit.attrib["upper"]))
                    self.assertGreaterEqual(target["close_rad"], float(limit.attrib["lower"]))
                    self.assertLessEqual(target["close_rad"], float(limit.attrib["upper"]))
                mimic_names = {
                    node.attrib["name"]
                    for node in joints.values()
                    if node.find("mimic") is not None
                }
                self.assertTrue(mimic_names.isdisjoint(mapping.command_joint_names))

    def test_validate_gesture_targets_rejects_wrong_channel(self):
        profile = json.loads(PROFILE.read_text(encoding="utf-8"))
        active = profile["gripper_commissioning"]["right"]["active_joints"]
        opened = [entry["open_rad"] for entry in active]
        closed = [entry["close_rad"] for entry in active]
        self.assertEqual(validate_gesture_targets("right", active, opened, closed), tuple(allowed_closing_joint_names("right")))
        wrong = list(closed)
        wrong[5] += 0.01
        with self.assertRaises(ValueError):
            validate_gesture_targets("right", active, opened, wrong)
