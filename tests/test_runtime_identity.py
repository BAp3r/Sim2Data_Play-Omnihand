"""Source identity must not inherit the commit of an enclosing project."""
import importlib.metadata
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.runtime_identity import inspect_runtime


class RuntimeIdentityTests(unittest.TestCase):
    def test_unversioned_lab_copy_does_not_claim_parent_git_identity(self):
        with TemporaryDirectory() as directory:
            parent = Path(directory)
            (parent / ".git").mkdir()
            root = parent / "vendor" / "IsaacLab"
            source = root / "source" / "isaaclab" / "isaaclab" / "__init__.py"
            source.parent.mkdir(parents=True)
            source.write_text("# selected source\n")
            (root / "VERSION").write_text("2.3.0\n")
            (root / "isaaclab.sh").write_text("# launcher\n")
            dist = SimpleNamespace(version="0.47.3", locate_file=lambda _: root,
                                   read_text=lambda _: None)

            def distribution(name):
                if name == "isaaclab":
                    return dist
                raise importlib.metadata.PackageNotFoundError(name)

            with patch("scripts.runtime_identity.metadata.distribution", side_effect=distribution), \
                 patch("scripts.runtime_identity.importlib.util.find_spec", return_value=SimpleNamespace(origin=str(source))), \
                 patch("scripts.runtime_identity.subprocess.run") as git:
                result = inspect_runtime()
            lab = result["packages"]["isaaclab"]
            self.assertEqual(lab["source_identity_status"], "unversioned_copy")
            self.assertIsNone(lab["upstream_commit"])
            self.assertEqual(lab["release_file"], "2.3.0")
            self.assertIn("source/isaaclab/isaaclab/__init__.py", lab["selected_source_sha256"])
            self.assertFalse(result["production_binding_approved"])
            git.assert_not_called()


if __name__ == "__main__":
    unittest.main()
