import unittest
from xml.etree import ElementTree as ET

from sim2data.backends.isaaclab.preview_pose import resolve_preview_positions


class PreviewPoseTests(unittest.TestCase):
    def model(self):
        return ET.fromstring('''<robot><joint name="thumb" type="revolute">
        <limit lower="0.035" upper="1"/></joint>
        <joint name="coupled" type="revolute"><limit lower="0" upper="0.5"/>
        <mimic joint="thumb" multiplier="0.5"/></joint></robot>''')

    def test_default_respects_positive_lower_and_mimic(self):
        self.assertEqual(resolve_preview_positions(self.model(), {}), {"thumb": .035, "coupled": .0175})

    def test_invalid_override_and_unknown_rejected(self):
        for pose in ({"thumb": 0}, {"thumb": float("nan")}, {"missing": 0}, {"coupled": .1}):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                resolve_preview_positions(self.model(), pose)

    def test_mimic_cannot_violate_limits(self):
        model = self.model()
        model.findall("joint")[1].find("limit").set("upper", "0.01")
        with self.assertRaises(ValueError):
            resolve_preview_positions(model, {})
