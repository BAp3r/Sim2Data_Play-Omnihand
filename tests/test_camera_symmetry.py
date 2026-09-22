"""Guard the selected synthetic stereo-camera mounting contract, not calibration."""
import json
import math
from pathlib import Path
import unittest

from sim2data.backends.isaaclab.scene_frames import SE3


def transform(spec):
    r, p, y = (v / 2 for v in spec["rpy_rad"])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return SE3(spec["xyz_m"], (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy))


class CameraSymmetryTests(unittest.TestCase):
    def test_real_camera_centers_symmetric_and_optical_roll_explicit(self):
        profile = json.loads((Path(__file__).resolve().parents[1] / "configs/commissioning.synthetic.json").read_text())
        housing, optical = [], []
        for side in ("left", "right"):
            spec = profile["robots"][side]
            h = transform(spec["T_mount_camera_housing"])
            housing.append(h.apply(profile["camera_symmetry"]["nominal_body_center_in_housing_m"]))
            optical.append(h @ transform(spec["T_housing_color_optical"]))
        for pair in (housing, [item.translation for item in optical]):
            for axis, sign in enumerate((1, -1, 1)):
                self.assertAlmostEqual(pair[1][axis], sign * pair[0][axis], places=9)
        left, right = (item.rotation_matrix() for item in optical)
        for row in range(3):
            for col, sign in enumerate((-1, -1, 1)):
                self.assertAlmostEqual(right[row][col], sign * left[row][col], places=9)
        self.assertFalse(profile["camera_symmetry"]["image_pixel_flip_applied"])
