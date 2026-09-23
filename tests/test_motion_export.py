"""CPU-only validation for the frozen synthetic motion export input."""
from __future__ import annotations
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib
from scripts.export_motion_sample import (
    ACTION_SEMANTICS,
    IMAGE_SHAPE,
    JOINT_NAMES,
    SAMPLE_KIND,
    STATE_SEMANTICS,
    load_capture,
)

def _png(width=IMAGE_SHAPE[1], height=IMAGE_SHAPE[0]):
    def chunk(kind, data):
        value = kind + data
        return struct.pack(">I", len(data)) + value + struct.pack(">I", zlib.crc32(value))
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress((b"\0" + bytes(width * 3)) * height))
        + chunk(b"IEND", b"")
    )

class MotionExportInputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.manifest = {
            "sample_kind": SAMPLE_KIND,
            "production_collection_allowed": False,
            "task_success": None,
            "fps": 30,
            "joint_names": list(JOINT_NAMES),
            "state_semantics": STATE_SEMANTICS,
            "action_semantics": ACTION_SEMANTICS,
            "frames": [self.frame(0), self.frame(1)],
            "image_shape": list(IMAGE_SHAPE),
            "physics_validated": False,
            "capture_alignment": "fixture provenance",
        }
        self.write()

    def tearDown(self):
        self.temp.cleanup()

    def frame(self, index):
        images = {}
        for camera in ("overhead", "wrist_left", "wrist_right"):
            relative = f"images/{camera}/{index:06d}.png"
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(_png())
            images[camera] = relative
        return {
            "index": index,
            "timestamp": index / 30,
            "state": [float(index)] * 12,
            "action": [float(index + 1)] * 12,
            "images": images,
        }

    def write(self):
        (self.root / "capture.json").write_text(json.dumps(self.manifest), encoding="utf-8")

    def test_accepts_capture(self):
        result = load_capture(self.root)
        self.assertEqual(len(result.frames), 2)
        self.assertEqual(len(result.frames[0].state), 12)
        self.assertEqual(result.provenance["capture_alignment"], "fixture provenance")

    def test_rejects_forged_claims(self):
        forged_claims = (
            ("task_success", True),
            ("production_collection_allowed", True),
            ("physics_validated", True),
        )
        for field, value in forged_claims:
            with self.subTest(field=field):
                self.manifest[field] = value
                self.write()
                with self.assertRaises(ValueError):
                    load_capture(self.root)
                self.manifest[field] = None if field == "task_success" else False

    def test_rejects_index_nonfinite_and_path_escape(self):
        self.manifest["frames"][1]["index"] = 3
        self.write()
        with self.assertRaisesRegex(ValueError, "contiguous"):
            load_capture(self.root)
        self.manifest["frames"][1]["index"] = 1
        self.manifest["frames"][0]["state"][0] = float("nan")
        self.write()
        with self.assertRaises(ValueError):
            load_capture(self.root)
        self.manifest["frames"][0]["state"][0] = 0
        self.manifest["frames"][0]["action"][0] = float("inf")
        self.write()
        with self.assertRaises(ValueError):
            load_capture(self.root)
        self.manifest["frames"][0]["action"][0] = 1
        self.manifest["frames"][0]["images"]["overhead"] = "../../outside.png"
        self.write()
        with self.assertRaisesRegex(ValueError, "image path"):
            load_capture(self.root)

    def test_rejects_wrong_png_shape(self):
        (self.root / "images/overhead/000000.png").write_bytes(_png(width=319))
        with self.assertRaisesRegex(ValueError, "RGB shape"):
            load_capture(self.root)

    def test_rejects_empty_capture(self):
        self.manifest["frames"] = []
        self.write()
        with self.assertRaisesRegex(ValueError, "at least one frame"):
            load_capture(self.root)

if __name__ == "__main__":
    unittest.main()
