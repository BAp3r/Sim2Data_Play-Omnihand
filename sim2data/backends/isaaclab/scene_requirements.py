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


# The key names mirror the frame names in DESIGN.md and deliberately do not
# contain guessed dimensions, poses, serial numbers or stream profiles.
B_SCENE_REQUIREMENTS: tuple[SceneParameterRequirement, ...] = (
    SceneParameterRequirement(
        "table.size_xyz_m", "missing", True, "not provided", "Measure tabletop extents and surface height."
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
        "adapter.mass_inertia_collision", "missing", True, "only nominal length supplied", "The approximately 4 cm description is not a 6D transform or inertial model; transforms are specified per robot."
    ),
    SceneParameterRequirement(
        "cameras.wrist_left.model", "user_specified", False, "user update", "RealSense D405 identity supplied; image evidence alone is not used to infer it."
    ),
    SceneParameterRequirement(
        "cameras.wrist_right.model", "user_specified", False, "user update", "RealSense D405 identity supplied; image evidence alone is not used to infer it."
    ),
    SceneParameterRequirement(
        "cameras.wrist_left.T_mount_housing", "missing", True, "not measured", "Measure the complete left wrist camera housing pose."
    ),
    SceneParameterRequirement(
        "cameras.wrist_right.T_mount_housing", "missing", True, "not measured", "Measure the complete right wrist camera housing pose."
    ),
    SceneParameterRequirement(
        "cameras.wrist_left.T_housing_color_optical", "missing", True, "not measured", "Measure the D405 color optical frame pose."
    ),
    SceneParameterRequirement(
        "cameras.wrist_left.T_housing_depth_optical", "missing", False, "not measured", "Required only if depth is enabled; RGB baseline does not use depth."
    ),
    SceneParameterRequirement(
        "cameras.wrist_right.T_housing_color_optical", "missing", True, "not measured", "Measure the D405 color optical frame pose."
    ),
    SceneParameterRequirement(
        "cameras.wrist_right.T_housing_depth_optical", "missing", False, "not measured", "Required only if depth is enabled; RGB baseline does not use depth."
    ),
    SceneParameterRequirement(
        "cameras.wrist_left_and_right.rgb_profiles_intrinsics_distortion", "missing", True, "camera config absent", "Read enabled RGB resolutions, profiles, K and distortion; depth requires a separate schema extension when enabled."
    ),
    SceneParameterRequirement(
        "cameras.stream_count_and_timestamps", "missing", True, "runtime not connected", "Freeze actual stream count and timestamp alignment before collection."
    ),
    SceneParameterRequirement(
        "camera_candidates.overhead.T_parent_optical", "missing", True, "authorized synthetic design; pose not selected", "A generic camera is allowed but its explicit pose and coverage must still be configured."
    ),
    SceneParameterRequirement(
        "camera_candidates.overhead.rgb_profile_and_intrinsics", "synthetic_design", False, "scene draft design values", "640x480, 30 Hz and declared ideal-pinhole K are design values, not measured calibration."
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
