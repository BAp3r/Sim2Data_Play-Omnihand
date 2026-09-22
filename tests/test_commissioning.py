import json
from pathlib import Path
import tempfile
import unittest
from xml.etree import ElementTree as ET

from sim2data.backends.isaaclab.commissioning import (
    AssemblyTransforms,
    CommissioningError,
    MeshVisual,
    SideCommissioningSpec,
    Transform,
    build_combined_urdf,
    write_combined_urdf,
)


ARM_URDF = """\
<robot name="arm_candidate">
  <link name="base"><inertial><mass value="1"/></inertial></link>
  <link name="flange"><visual><geometry><mesh filename="meshes/flange.stl"/></geometry></visual></link>
  <link name="old_gripper"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="old_tip"/>
  <joint name="base_to_flange" type="fixed"><parent link="base"/><child link="flange"/></joint>
  <joint name="old_gripper_joint" type="fixed"><parent link="flange"/><child link="old_gripper"/></joint>
  <joint name="old_tip_joint" type="fixed"><parent link="old_gripper"/><child link="old_tip"/></joint>
  <material name="dark"><color rgba="0 0 0 1"/></material>
</robot>
"""

HAND_URDF = """\
<robot name="hand_candidate">
  <link name="hand_root"><inertial><mass value="2"/></inertial></link>
  <link name="finger"><visual><geometry><mesh filename="package://hand_pkg/meshes/finger.stl"/></geometry></visual></link>
  <link name="passive"/>
  <joint name="hand_finger" type="revolute"><parent link="hand_root"/><child link="finger"/></joint>
  <joint name="hand_passive" type="revolute"><parent link="finger"/><child link="passive"/><mimic joint="hand_finger" multiplier="1" offset="0"/></joint>
</robot>
"""


def _transform(value=(0.0, 0.0, 0.0)):
    return Transform(value, (0.0, 0.0, 0.0))


def _spec(arm: Path, hand: Path, package_root: Path) -> SideCommissioningSpec:
    transforms = AssemblyTransforms(
        _transform(),
        _transform((0.0, 0.0, 0.0261)),
        _transform((0.0, -0.055, 0.012)),
        _transform(),
    )
    return SideCommissioningSpec(
        side="left",
        output_robot_name="synthetic_left_combined",
        namespace_prefix="left__",
        arm_urdf=arm,
        hand_urdf=hand,
        flange_link="flange",
        hand_root_link="hand_root",
        transforms=transforms,
        package_roots={"hand_pkg": package_root},
    )


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / "arm" / "meshes").mkdir(parents=True)
        (self.root / "hand" / "meshes").mkdir(parents=True)
        (self.root / "arm" / "arm.urdf").write_text(ARM_URDF, encoding="utf-8")
        (self.root / "hand" / "hand.urdf").write_text(HAND_URDF, encoding="utf-8")
        (self.root / "arm" / "meshes" / "flange.stl").write_text("fixture", encoding="utf-8")
        (self.root / "hand" / "meshes" / "finger.stl").write_text("fixture", encoding="utf-8")
        (self.root / "hand" / "meshes" / "mount.stl").write_text("fixture", encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def test_mapping_transforms_require_explicit_metric_units(self):
        transforms = AssemblyTransforms.from_mapping(
            {
                "T_flange_mount": {"xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
                "T_mount_hand_root": {"xyz_m": [0, 0, 0.0261], "rpy_rad": [0, 0, 0]},
                "T_mount_camera_housing": {"xyz_m": [0, -0.055, 0.012], "rpy_rad": [0, 0, 0]},
                "T_housing_color_optical": {"xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
            }
        )
        self.assertEqual(transforms.T_mount_hand_root.xyz_m, (0.0, 0.0, 0.0261))
        with self.assertRaisesRegex(CommissioningError, "finite"):
            Transform((0.0, float("nan"), 0.0), (0.0, 0.0, 0.0))

    def test_combines_namespaced_sources_trims_old_gripper_and_rewrites_mimic(self):
        spec = _spec(self.root / "arm" / "arm.urdf", self.root / "hand" / "hand.urdf", self.root / "hand")
        result = build_combined_urdf(spec)
        root = ET.fromstring(result.xml_text)
        link_names = {link.get("name") for link in root.findall("link")}
        joint_names = {joint.get("name") for joint in root.findall("joint")}
        self.assertIn("left__arm__flange", link_names)
        self.assertIn("left__hand__hand_root", link_names)
        self.assertIn("left__mount_assembly", link_names)
        self.assertIn("left__camera_housing", link_names)
        self.assertIn("left__color_optical", link_names)
        self.assertNotIn("left__arm__old_gripper", link_names)
        self.assertIn("left__flange_mount_fixed", joint_names)
        self.assertIn("left__mount_hand_root_fixed", joint_names)
        self.assertIn("left__mount_camera_housing_fixed", joint_names)
        self.assertIn("left__camera_housing_color_optical_fixed", joint_names)
        passive = root.find("joint[@name='left__hand__hand_passive']")
        self.assertIsNotNone(passive)
        self.assertEqual(passive.find("mimic").get("joint"), "left__hand__hand_finger")
        self.assertEqual(result.summary["status"], "synthetic_commissioning_only")
        self.assertFalse(result.summary["physics_validated"])
        self.assertFalse(result.summary["production"])

    def test_retained_mimic_to_deleted_arm_joint_fails(self):
        arm = self.root / "arm" / "arm.urdf"
        arm.write_text(
            ARM_URDF.replace(
                '<joint name="base_to_flange" type="fixed"><parent link="base"/><child link="flange"/></joint>',
                '<joint name="base_to_flange" type="fixed"><parent link="base"/><child link="flange"/><mimic joint="old_gripper_joint"/></joint>',
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(CommissioningError, "mimics deleted joint"):
            build_combined_urdf(_spec(arm, self.root / "hand" / "hand.urdf", self.root / "hand"))

    def test_mesh_package_escape_and_missing_file_are_rejected(self):
        hand = self.root / "hand" / "hand.urdf"
        hand.write_text(HAND_URDF.replace("package://hand_pkg/meshes/finger.stl", "package://hand_pkg/../outside.stl"), encoding="utf-8")
        with self.assertRaisesRegex(CommissioningError, "escapes package root"):
            build_combined_urdf(_spec(self.root / "arm" / "arm.urdf", hand, self.root / "hand"))
        hand.write_text(HAND_URDF.replace("package://hand_pkg/meshes/finger.stl", "package://hand_pkg/meshes/missing.stl"), encoding="utf-8")
        with self.assertRaisesRegex(CommissioningError, "does not exist"):
            build_combined_urdf(_spec(self.root / "arm" / "arm.urdf", hand, self.root / "hand"))

    def test_output_refuses_overwrite_and_writes_summary(self):
        spec = _spec(self.root / "arm" / "arm.urdf", self.root / "hand" / "hand.urdf", self.root / "hand")
        output = self.root / "commissioned"
        path = write_combined_urdf(spec, output)
        summary = json.loads((output / "commissioning_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(path.name, "synthetic_left_combined.urdf")
        self.assertEqual(summary["status"], "synthetic_commissioning_only")
        with self.assertRaisesRegex(CommissioningError, "overwrite"):
            write_combined_urdf(spec, output)

    def test_optional_mount_mesh_origin_and_housing_box_are_explicit(self):
        spec = _spec(self.root / "arm" / "arm.urdf", self.root / "hand" / "hand.urdf", self.root / "hand")
        spec = SideCommissioningSpec(
            side=spec.side,
            output_robot_name=spec.output_robot_name,
            namespace_prefix=spec.namespace_prefix,
            arm_urdf=spec.arm_urdf,
            hand_urdf=spec.hand_urdf,
            flange_link=spec.flange_link,
            hand_root_link=spec.hand_root_link,
            transforms=spec.transforms,
            package_roots=spec.package_roots,
            mount_visual=MeshVisual(
                "package://hand_pkg/meshes/mount.stl",
                Transform((0.11184514335, -0.038218873357, -0.117407201924), (1.57079632679, 0.0, 0.0)),
            ),
            camera_housing_size_xyz_m=(0.045, 0.045, 0.025),
        )
        result = build_combined_urdf(spec)
        root = ET.fromstring(result.xml_text)
        mount = root.find("link[@name='left__mount_assembly']")
        mesh = mount.find("visual/geometry/mesh")
        origin = mount.find("visual/origin")
        self.assertEqual(mesh.get("filename"), (self.root / "hand" / "meshes" / "mount.stl").resolve().as_posix())
        self.assertEqual(origin.get("rpy"), "1.57079632679 0 0")
        housing = root.find("link[@name='left__camera_housing']")
        self.assertEqual(housing.find("visual/geometry/box").get("size"), "0.045 0.045 0.025")
        self.assertEqual(housing.find("collision/geometry/box").get("size"), "0.045 0.045 0.025")
        self.assertEqual(result.summary["resolved_mesh_count"], 3)

    def test_hand_root_and_flange_must_be_explicit_existing_links(self):
        spec = _spec(self.root / "arm" / "arm.urdf", self.root / "hand" / "hand.urdf", self.root / "hand")
        bad = SideCommissioningSpec(
            side=spec.side,
            output_robot_name=spec.output_robot_name,
            namespace_prefix=spec.namespace_prefix,
            arm_urdf=spec.arm_urdf,
            hand_urdf=spec.hand_urdf,
            flange_link="guessed_flange",
            hand_root_link=spec.hand_root_link,
            transforms=spec.transforms,
            package_roots=spec.package_roots,
        )
        with self.assertRaisesRegex(CommissioningError, "flange link"):
            build_combined_urdf(bad)


if __name__ == "__main__":
    unittest.main()
