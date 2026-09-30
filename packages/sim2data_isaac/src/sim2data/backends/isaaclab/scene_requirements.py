"""B_SCENE parameter list derived from approved design inputs and observations.

This is a public, path-free requirements list.  A ``user_specified`` model
identity or a layout observation does not resolve a metric calibration value.
Only measured values, an approved asset/config manifest, or runtime discovery
can clear a blocking item.  The gate intentionally remains closed at M0.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

ParameterStatus = Literal[
    "user_specified",
    "layout_observed",
    "approved_manifest",
    "measured",
    "runtime_discovery",
    "missing",
    "optional_design",
    "synthetic_design",
]


@dataclass(frozen=True, slots=True)
class SceneParameterRequirement:
    key: str
    status: ParameterStatus
    blocking: bool
    source: str
    note: str

    @property
    def resolved(self) -> bool:
        return self.status in {"approved_manifest", "measured", "runtime_discovery", "synthetic_design"}


# The key names mirror the public draft and frame names in DESIGN.md.  They
# deliberately do not contain guessed dimensions, poses, serial numbers or
# stream profiles.  The approved observation contributes topology only; it
# never resolves a metric transform, camera calibration, or physical model.
APPROVED_OBSERVATION_ID = "OBS-M1-20260922-ROOT-01"


B_SCENE_REQUIREMENTS: tuple[SceneParameterRequirement, ...] = (
    SceneParameterRequirement(
        "table.size_xyz_m", "missing", True, "not provided", "Measure tabletop extents and floor-to-surface height; world z=0 remains the tabletop surface."
    ),
    SceneParameterRequirement(
        "robots.left.T_world_base", "missing", True, "not provided", "Survey left base pose in the tabletop world frame."
    ),
    SceneParameterRequirement(
        "robots.right.T_world_base", "missing", True, "not provided", "Survey right base pose in the tabletop world frame."
    ),
    SceneParameterRequirement(
        "robots.left.arm_revision_and_asset", "missing", True, "approved manifest absent", "Bind the exact AIRBOT Play revision and official asset path."
    ),
    SceneParameterRequirement(
        "robots.right.arm_revision_and_asset", "missing", True, "approved manifest absent", "Bind the exact AIRBOT Play revision and official asset path."
    ),
    SceneParameterRequirement(
        "robots.left.hand_variant_revision_and_side", "missing", True, "not provided", "Confirm OmniHand 2025 variant, hardware revision and anatomical side."
    ),
    SceneParameterRequirement(
        "robots.right.hand_variant_revision_and_side", "missing", True, "not provided", "Confirm OmniHand 2025 variant, hardware revision and anatomical side."
    ),
    SceneParameterRequirement(
        "robots.left.T_flange_mount", "missing", True, "not measured", "Measure the complete flange-to-mount rigid transform."
    ),
    SceneParameterRequirement(
        "robots.right.T_flange_mount", "missing", True, "not measured", "Measure the complete flange-to-mount rigid transform."
    ),
    SceneParameterRequirement(
        "robots.left.T_mount_hand_root", "missing", True, "not measured", "Measure the mount-to-hand-root transform."
    ),
    SceneParameterRequirement(
        "robots.right.T_mount_hand_root", "missing", True, "not measured", "Measure the mount-to-hand-root transform."
    ),
    SceneParameterRequirement(
        "robots.left.T_hand_root_grasp_tcp", "missing", True, "not measured", "Define and measure the grasp TCP relative to hand_root."
    ),
    SceneParameterRequirement(
        "robots.right.T_hand_root_grasp_tcp", "missing", True, "not measured", "Define and measure the grasp TCP relative to hand_root."
    ),
    SceneParameterRequirement(
        "adapter.nominal_length_m", "user_specified", False, "user approximate description", "The approximately 0.04 m length is a nominal description only; it must not be expanded into a translation or rotation."
    ),
    SceneParameterRequirement(
        "adapter.mass_kg", "missing", True, "not measured", "Measure the complete adapter and camera-mount mass carried by each arm."
    ),
    SceneParameterRequirement(
        "adapter.inertia_kg_m2", "missing", True, "not measured", "Measure or derive the installation inertia from an approved CAD/physical model."
    ),
    SceneParameterRequirement(
        "adapter.collision_mesh", "missing", True, "not supplied", "Provide collision geometry for the adapter and camera mount; do not replace it with a visual-only approximation."
    ),
    SceneParameterRequirement(
        "scene.approved_observation_package", "layout_observed", False, APPROVED_OBSERVATION_ID, "Approved topology only: same-side dual arms, separate adapter, underside wrist camera housing and elevated independent camera support."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_left.model", "user_specified", False, "user update", "RealSense D405 identity supplied; image evidence alone is not used to infer it."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_right.model", "user_specified", False, "user update", "RealSense D405 identity supplied; image evidence alone is not used to infer it."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_left.T_mount_camera_housing", "missing", True, "not measured", "Measure the complete left wrist mount-assembly to camera-housing transform."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_right.T_mount_camera_housing", "missing", True, "not measured", "Measure the complete right wrist mount-assembly to camera-housing transform."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_left.T_housing_color_optical", "missing", True, "not measured", "Measure the left D405 housing to color-optical transform."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_left.T_housing_depth_optical", "missing", False, "not measured", "Required only if depth is enabled; RGB baseline keeps depth disabled."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_right.T_housing_color_optical", "missing", True, "not measured", "Measure the right D405 housing to color-optical transform."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_right.T_housing_depth_optical", "missing", False, "not measured", "Required only if depth is enabled; RGB baseline keeps depth disabled."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_left.rgb_profile_and_intrinsics", "missing", True, "camera config absent", "Read the enabled left D405 RGB resolution, profile, K, distortion and crop/resize contract."
    ),
    SceneParameterRequirement(
        "camera_candidates.wrist_right.rgb_profile_and_intrinsics", "missing", True, "camera config absent", "Read the enabled right D405 RGB resolution, profile, K, distortion and crop/resize contract."
    ),
    SceneParameterRequirement(
        "camera_candidates.stream_count_and_timestamps", "missing", True, "runtime not connected", "Freeze actual stream count and timestamp alignment before collection."
    ),
    SceneParameterRequirement(
        "camera_candidates.overhead.T_parent_optical", "missing", True, "authorized synthetic design; pose not selected", "A generic camera is allowed but its explicit pose and coverage must still be configured."
    ),
    SceneParameterRequirement(
        "camera_candidates.overhead.rgb_profile_and_intrinsics", "synthetic_design", False, "scene draft design values", "640x480, 30 Hz and declared ideal-pinhole K are design values, not measured calibration."
    ),
    SceneParameterRequirement(
        "camera_candidates.overhead.coverage_validation", "missing", True, "not checked", "Validate that the selected overhead pose covers the source, relay and external-bin regions without relying on the phone reference view."
    ),
    SceneParameterRequirement(
        "task.shared_transfer_region.pose_size_support_height", "missing", True, "not measured", "Define the surface region reachable by both arms for tabletop transfer."
    ),
    SceneParameterRequirement(
        "task.right_outer_bin.pose_size_support_height", "missing", True, "not measured", "Measure the right-side off-table bin pose, dimensions and support height."
    ),
    SceneParameterRequirement(
        "task.right_arm_reachability_to_outer_bin", "missing", True, "not checked", "Run reachability and collision checks after right arm/base calibration."
    ),
    SceneParameterRequirement(
        "task.left_source_object_pose_size_mass", "missing", True, "not measured", "Measure source placement and object geometry/mass before contact planning."
    ),
    SceneParameterRequirement(
        "task.source_object_rigidity_model", "missing", True, "not confirmed", "Confirm whether the wrinkled cuboid packaging can be modeled as a rigid body before contact validation."
    ),
    SceneParameterRequirement(
        "scene.initial_penetration_and_occlusion_check", "missing", True, "Isaac scene not opened", "Verify no initial penetration and inspect real camera occlusion after asset binding."
    ),
)


def requirements(*, blocking_only: bool = False) -> tuple[SceneParameterRequirement, ...]:
    """Return an immutable snapshot of the B_SCENE requirements."""
    if blocking_only:
        return tuple(item for item in B_SCENE_REQUIREMENTS if item.blocking)
    return B_SCENE_REQUIREMENTS


def missing_parameters() -> tuple[str, ...]:
    """Return blocking keys that are unresolved at the current M0 evidence level."""
    return tuple(item.key for item in B_SCENE_REQUIREMENTS if item.blocking and not item.resolved)
