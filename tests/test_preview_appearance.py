"""Optional CPU-USD checks for authored appearance, never physical validation."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
AVAILABLE = all(importlib.util.find_spec(name) is not None for name in ("numpy", "trimesh", "pxr"))


@unittest.skipUnless(AVAILABLE, "requires the existing isolated CPU USD environment")
class PreviewAppearanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        import trimesh
        from pxr import Usd, UsdGeom, UsdShade
        spec = importlib.util.spec_from_file_location("preview_appearance_under_test", ROOT / "scripts/build_commissioning_preview.py")
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)
        cls.np, cls.trimesh = np, trimesh
        cls.Usd, cls.UsdGeom, cls.UsdShade = Usd, UsdGeom, UsdShade
        cls.appearance = json.loads((ROOT / "configs/appearance.reference.json").read_text())

    def test_smooth_corner_normals_preserve_original_geometry(self):
        np = self.np
        # Duplicated STL-like corners at a gently folded shared edge.
        geometry = self.trimesh.Trimesh(vertices=[[0,0,0],[1,0,0],[0,1,0],
                                                [0,0,0],[0,-1,.2],[1,0,0]], faces=[[0,1,2],[3,4,5]], process=False)
        before_vertices, before_faces = geometry.vertices.copy(), geometry.faces.copy()
        normals = self.module.corner_shading_normals(geometry, self.appearance["surface_shading"])
        np.testing.assert_allclose(normals[0,0], normals[1,0], atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(normals, axis=2), 1)
        np.testing.assert_array_equal(geometry.vertices, before_vertices)
        np.testing.assert_array_equal(geometry.faces, before_faces)

    def test_sharp_corner_keeps_separate_shading_normals(self):
        geometry = self.trimesh.creation.box()
        normals = self.module.corner_shading_normals(geometry, self.appearance["surface_shading"])
        self.np.testing.assert_allclose(normals, self.np.repeat(geometry.face_normals[:,None,:], 3, axis=1), atol=1e-12)

    def test_palm_is_one_silver_shell_material(self):
        geometry = self.trimesh.creation.box()
        for name in ("left_hand__l_palm", "right_hand__R_palm"):
            style = self.module.reference_style(name, geometry, self.appearance)
            self.assertEqual(style["primary"], self.appearance["palette"]["hand_shell"])
            self.assertIsNone(style["accent"])
            self.assertIsNone(style["weights"])

    def test_panel_texture_uv_and_normals_are_portable(self):
        with tempfile.TemporaryDirectory() as directory:
            stage = self.Usd.Stage.CreateNew(str(Path(directory) / "test.usda"))
            geometry = self.trimesh.creation.icosphere(subdivisions=1)
            style = self.module.reference_style("left_arm__link2", geometry, self.appearance)
            self.assertTrue(((style["weights"] > 0) & (style["weights"] < 1)).any())
            self.module.mesh(stage, "/World/Panel", geometry, (.5,.5,.5), style)
            mesh = self.UsdGeom.Mesh(stage.GetPrimAtPath("/World/Panel"))
            uv = self.UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st")
            self.assertEqual(uv.GetInterpolation(), "faceVarying")
            self.assertEqual(len(uv.Get()), 3 * len(geometry.faces))
            self.assertEqual(mesh.GetNormalsInterpolation(), "faceVarying")
            self.assertEqual(len(mesh.GetNormalsAttr().Get()), 3 * len(geometry.faces))
            self.assertFalse(self.UsdGeom.Subset.GetGeomSubsets(self.UsdGeom.Imageable(mesh)))
            textures = [self.UsdShade.Shader(p) for p in stage.Traverse() if p.IsA(self.UsdShade.Shader)
                        and self.UsdShade.Shader(p).GetIdAttr().Get() == "UsdUVTexture"]
            self.assertEqual(len(textures), 1)
            relative = textures[0].GetInput("file").Get().path
            self.assertFalse(Path(relative).is_absolute())
            self.assertTrue((Path(directory) / relative).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(textures[0].GetInput("sourceColorSpace").Get(), "raw")


if __name__ == "__main__":
    unittest.main()
