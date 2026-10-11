import math
import unittest

from sim2data.backends.isaaclab.scene_frames import (
    FrameTransform,
    MissingCalibrationError,
    OpticalConvention,
    SE3,
    adapt_optical_transform,
    compose_camera_chain,
    compose_frame_chain,
    compose_hand_chain,
    compose_mount_optical_chain,
    optical_basis_transform,
    require_complete_transforms,
)
from sim2data.backends.isaaclab.scene_requirements import (
    APPROVED_OBSERVATION_ID,
    missing_parameters,
    requirements,
)


class SE3Tests(unittest.TestCase):
    def assertVectorAlmostEqual(self, first, second):
        self.assertEqual(len(first), len(second))
        for left, right in zip(first, second):
            self.assertTrue(math.isclose(left, right, abs_tol=1e-9), (first, second))

    def test_composition_uses_parent_child_column_vector_order(self):
        world_from_arm = SE3.from_translation(1.0, 0.0, 0.0)
        arm_from_tool = SE3.from_translation(0.0, 2.0, 0.0)
        world_from_tool = world_from_arm @ arm_from_tool
        self.assertVectorAlmostEqual(world_from_tool.apply((0.0, 0.0, 0.0)), (1.0, 2.0, 0.0))

    def test_inverse_round_trip(self):
        transform = SE3((0.4, -0.2, 0.7), (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
        point = (0.3, 0.1, -0.8)
        self.assertVectorAlmostEqual(transform.inverse().apply(transform.apply(point)), point)

    def test_mount_offset_rotates_with_parent(self):
        parent = SE3(rotation_wxyz=(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
        mount = SE3.from_translation(1.0, 0.0, 0.0)
        self.assertVectorAlmostEqual((parent @ mount).translation, (0.0, 1.0, 0.0))

    def test_rejects_non_unit_quaternion(self):
        with self.assertRaisesRegex(ValueError, "unit quaternion"):
            SE3(rotation_wxyz=(2.0, 0.0, 0.0, 0.0))

    def test_labelled_chain_rejects_disconnected_edges(self):
        with self.assertRaisesRegex(ValueError, "frame discontinuity"):
            compose_frame_chain(
                FrameTransform("world", "flange", SE3.identity()),
                FrameTransform("wrong_parent", "mount", SE3.identity()),
            )

    def test_hand_chain_composes_flange_mount_and_hand_root_edges(self):
        result = compose_hand_chain(
            FrameTransform("world", "flange", SE3.from_translation(1.0, 0.0, 0.0)),
            FrameTransform("flange", "mount", SE3.from_translation(0.0, 2.0, 0.0)),
            FrameTransform("mount", "hand_root", SE3.from_translation(0.0, 0.0, 3.0)),
        )
        self.assertEqual((result.parent_frame, result.child_frame), ("world", "hand_root"))
        self.assertVectorAlmostEqual(result.apply((0.0, 0.0, 0.0)), (1.0, 2.0, 3.0))

    def test_hand_chain_rejects_a_finger_instead_of_mount_edge(self):
        with self.assertRaisesRegex(ValueError, "must identify mount"):
            compose_hand_chain(
                FrameTransform("world", "flange", SE3.identity()),
                FrameTransform("flange", "finger_link", SE3.identity()),
                FrameTransform("finger_link", "hand_root", SE3.identity()),
            )

    def test_complete_transform_gate_rejects_none(self):
        with self.assertRaises(MissingCalibrationError) as context:
            require_complete_transforms({"T_world_flange": SE3.identity(), "T_flange_mount": None})
        self.assertEqual(context.exception.missing, ("T_flange_mount",))


class OpticalTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, first, second):
        self.assertEqual(len(first), len(second))
        for left, right in zip(first, second):
            self.assertTrue(math.isclose(left, right, abs_tol=1e-9), (first, second))

    def test_opencv_to_usd_basis(self):
        basis = optical_basis_transform(OpticalConvention.OPENCV, OpticalConvention.USD_CAMERA)
        self.assertVectorAlmostEqual(basis.apply((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))

    def test_optical_adaptation_changes_only_basis(self):
        parent_from_cv = SE3((0.5, 0.0, 0.0))
        parent_from_usd = adapt_optical_transform(parent_from_cv, "opencv", "usd")
        self.assertVectorAlmostEqual(parent_from_usd.translation, parent_from_cv.translation)
        self.assertVectorAlmostEqual(parent_from_usd.apply((1.0, 2.0, 3.0)), (1.5, -2.0, -3.0))

    def test_camera_chain_matches_design_formula(self):
        edge_world_flange = FrameTransform("world", "flange", SE3.from_translation(1.0, 0.0, 0.0))
        edge_flange_mount = FrameTransform("flange", "mount", SE3.from_translation(0.0, 2.0, 0.0))
        edge_mount_housing = FrameTransform("mount", "housing", SE3.from_translation(0.0, 0.0, 3.0))
        edge_housing_optical = FrameTransform("housing", "color_optical", SE3.identity())
        result = compose_camera_chain(
            edge_world_flange,
            edge_flange_mount,
            edge_mount_housing,
            edge_housing_optical,
        )
        self.assertEqual((result.parent_frame, result.child_frame), ("world", "color_optical"))
        self.assertVectorAlmostEqual(result.apply((0.0, 0.0, 0.0)), (1.0, 2.0, 3.0))

    def test_mount_optical_chain_keeps_camera_on_mount_assembly(self):
        result = compose_mount_optical_chain(
            FrameTransform("mount", "camera_housing", SE3.from_translation(0.0, 2.0, 0.0)),
            FrameTransform("camera_housing", "color_optical", SE3.from_translation(0.0, 0.0, 3.0)),
        )
        self.assertEqual((result.parent_frame, result.child_frame), ("mount", "color_optical"))
        self.assertVectorAlmostEqual(result.apply((0.0, 0.0, 0.0)), (0.0, 2.0, 3.0))

    def test_mount_optical_chain_rejects_disconnected_finger_attachment(self):
        with self.assertRaisesRegex(ValueError, "frame discontinuity"):
            compose_mount_optical_chain(
                FrameTransform("mount", "camera_housing", SE3.identity()),
                FrameTransform("finger_link", "color_optical", SE3.identity()),
            )

    def test_camera_chain_adapts_only_optical_basis_after_mount_chain(self):
        result = compose_camera_chain(
            FrameTransform("world", "flange", SE3.from_translation(1.0, 0.0, 0.0)),
            FrameTransform("flange", "mount", SE3.from_translation(0.0, 2.0, 0.0)),
            FrameTransform("mount", "camera_housing", SE3.from_translation(0.0, 0.0, 3.0)),
            FrameTransform("camera_housing", "color_optical", SE3.identity()),
            target_convention=OpticalConvention.USD_CAMERA,
        )
        self.assertEqual((result.parent_frame, result.child_frame), ("world", "color_optical"))
        self.assertVectorAlmostEqual(result.transform.translation, (1.0, 2.0, 3.0))
        self.assertVectorAlmostEqual(result.apply((1.0, 2.0, 3.0)), (2.0, 0.0, 0.0))


class RequirementTests(unittest.TestCase):
    def test_m0_requirements_include_updated_task_inputs(self):
        keys = set(missing_parameters())
        self.assertIn("task.shared_transfer_region.pose_size_support_height", keys)
        self.assertIn("task.right_outer_bin.pose_size_support_height", keys)
        self.assertIn("task.right_arm_reachability_to_outer_bin", keys)
        self.assertIn("camera_candidates.wrist_left.T_housing_color_optical", keys)
        self.assertIn("camera_candidates.wrist_right.T_housing_color_optical", keys)
        self.assertNotIn("camera_candidates.wrist_right.T_housing_depth_optical", keys)
        self.assertIn("adapter.inertia_kg_m2", keys)
        self.assertNotIn("adapter.nominal_length_m", keys)
        self.assertTrue(keys)

    def test_approved_observation_is_topology_only(self):
        observation = next(item for item in requirements() if item.key == "scene.approved_observation_package")
        self.assertEqual(observation.source, APPROVED_OBSERVATION_ID)
        self.assertFalse(observation.blocking)
        self.assertFalse(observation.resolved)

    def test_synthetic_camera_still_requires_an_explicit_pose(self):
        pose = next(item for item in requirements() if item.key == "camera_candidates.overhead.T_parent_optical")
        self.assertTrue(pose.blocking)
        self.assertFalse(pose.resolved)
        profile = next(item for item in requirements() if item.key == "camera_candidates.overhead.rgb_profile_and_intrinsics")
        self.assertEqual(profile.status, 'synthetic_design')


if __name__ == "__main__":
    unittest.main()
