"""Input identity failures must be detected without importing Kit."""
import json
from pathlib import Path
import tempfile
import unittest

from scripts.isaac_scene_smoke import CAMERAS, file_sha256, validate_review_inputs


class SceneSmokeInputs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.assembly = self.root / "assembly.usdc"
        self.assembly.write_bytes(b"private fixture; USD parsing belongs to runtime")
        self.profile = self.root / "profile.json"
        self.profile.write_text("{}", encoding="utf-8")
        self.texture = self.root / "texture.png"
        self.texture.write_bytes(b"texture fixture")
        self.report = {
            "profile_sha256": file_sha256(self.profile),
            "camera_paths": list(CAMERAS.values()),
            "texture_dependencies": [{"path": "texture.png", "sha256": file_sha256(self.texture)}],
        }
        self.write_report()

    def write_report(self):
        (self.root / "preview_report.json").write_text(json.dumps(self.report), encoding="utf-8")

    def test_reviewed_binding(self):
        result = validate_review_inputs(self.assembly, self.profile)
        self.assertEqual(result["assembly_sha256"], file_sha256(self.assembly))

    def test_profile_drift_rejected(self):
        self.profile.write_text('{"changed": true}', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "profile hash mismatch"):
            validate_review_inputs(self.assembly, self.profile)

    def test_texture_tamper_and_absence_rejected(self):
        self.texture.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "texture missing or hash mismatch"):
            validate_review_inputs(self.assembly, self.profile)
        self.texture.unlink()
        with self.assertRaisesRegex(ValueError, "texture missing or hash mismatch"):
            validate_review_inputs(self.assembly, self.profile)

    def test_missing_wrist_rejected(self):
        self.report["camera_paths"].pop()
        self.write_report()
        with self.assertRaisesRegex(ValueError, "three reviewed cameras"):
            validate_review_inputs(self.assembly, self.profile)

    def test_texture_escape_rejected(self):
        self.report["texture_dependencies"][0]["path"] = "../outside.png"
        self.write_report()
        with self.assertRaisesRegex(ValueError, "inside the review directory"):
            validate_review_inputs(self.assembly, self.profile)


if __name__ == "__main__":
    unittest.main()
