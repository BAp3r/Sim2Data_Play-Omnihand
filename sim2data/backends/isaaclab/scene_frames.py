"""Backend-independent frame and camera convention utilities.

The project convention is a right-handed world frame in metres, with column
vectors and ``T_A_B`` mapping a vector expressed in frame B into frame A.
Quaternions in this module are serialized as ``(w, x, y, z)``.  These helpers
only check and compose supplied transforms; they do not estimate calibration,
load Isaac assets, or claim that a scene can be opened.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]
QuaternionWXYZ = tuple[float, float, float, float]


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise ValueError(f"{name} must contain numeric values")
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must contain finite values")
    return number


def _vector3(values: Sequence[float], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return tuple(_finite_number(value, name) for value in values)  # type: ignore[return-value]


def _quaternion(values: Sequence[float], name: str) -> QuaternionWXYZ:
    if len(values) != 4:
        raise ValueError(f"{name} must contain exactly 4 values in wxyz order")
    quaternion = tuple(_finite_number(value, name) for value in values)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0 or not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{name} must be a unit quaternion in wxyz order")
    return quaternion  # type: ignore[return-value]


def _quaternion_multiply(first: QuaternionWXYZ, second: QuaternionWXYZ) -> QuaternionWXYZ:
    """Return the Hamilton product ``first * second`` in wxyz order."""
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quaternion_conjugate(quaternion: QuaternionWXYZ) -> QuaternionWXYZ:
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _rotate(quaternion: QuaternionWXYZ, vector: Vector3) -> Vector3:
    """Rotate a vector without allocating a temporary quaternion."""
    _, qx, qy, qz = quaternion
    vx, vy, vz = vector
    # This is equivalent to q * (0, v) * q_conjugate and assumes a unit q.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + quaternion[0] * tx + qy * tz - qz * ty,
        vy + quaternion[0] * ty + qz * tx - qx * tz,
        vz + quaternion[0] * tz + qx * ty - qy * tx,
    )


def _add(first: Vector3, second: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(first, second))  # type: ignore[return-value]


def _negate(vector: Vector3) -> Vector3:
    return tuple(-value for value in vector)  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class SE3:
    """A rigid transform ``T_parent_child``.

    ``translation_m`` is the child origin expressed in the parent frame and
    ``rotation_wxyz`` rotates child-frame vectors into the parent frame.  The
    class deliberately rejects non-unit quaternions instead of normalizing an
    unreviewed calibration value silently.
    """

    translation_m: Sequence[float] = (0.0, 0.0, 0.0)
    rotation_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "translation_m", _vector3(self.translation_m, "translation_m"))
        object.__setattr__(self, "rotation_wxyz", _quaternion(self.rotation_wxyz, "rotation_wxyz"))

    @classmethod
    def identity(cls) -> "SE3":
        return cls()

    @classmethod
    def from_translation(cls, x: float, y: float, z: float) -> "SE3":
        return cls((x, y, z))

    @property
    def translation(self) -> Vector3:
        """Short alias retained for geometry code that uses ``t`` terminology."""
        return self.translation_m  # type: ignore[return-value]

    @property
    def quaternion_wxyz(self) -> QuaternionWXYZ:
        return self.rotation_wxyz  # type: ignore[return-value]

    def apply(self, point_child: Sequence[float]) -> Vector3:
        """Map a point from the child frame into the parent frame."""
        point = _vector3(point_child, "point_child")
        return _add(_rotate(self.quaternion_wxyz, point), self.translation)

    def compose(self, child: "SE3") -> "SE3":
        """Return ``self * child`` using the ``T_A_B * T_B_C`` rule."""
        if not isinstance(child, SE3):
            raise TypeError("child must be an SE3")
        return SE3(
            _add(_rotate(self.quaternion_wxyz, child.translation), self.translation),
            _quaternion_multiply(self.quaternion_wxyz, child.quaternion_wxyz),
        )

    def __matmul__(self, child: "SE3") -> "SE3":
        return self.compose(child)

    def inverse(self) -> "SE3":
        """Return ``T_child_parent``."""
        inverse_rotation = _quaternion_conjugate(self.quaternion_wxyz)
        return SE3(_negate(_rotate(inverse_rotation, self.translation)), inverse_rotation)

    def rotation_matrix(self) -> tuple[tuple[float, float, float], ...]:
        """Return the child-to-parent 3x3 rotation matrix."""
        w, x, y, z = self.quaternion_wxyz
        return (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        )

    def matrix4(self) -> tuple[tuple[float, float, float, float], ...]:
        rotation = self.rotation_matrix()
        return tuple(
            tuple(row) + (translation,)
            for row, translation in zip(rotation, self.translation)
        ) + ((0.0, 0.0, 0.0, 1.0),)


@dataclass(frozen=True, slots=True)
class FrameTransform:
    """A labelled edge whose value is ``T_parent_frame_child_frame``."""

    parent_frame: str
    child_frame: str
    transform: SE3

    def __post_init__(self) -> None:
        for name, value in (("parent_frame", self.parent_frame), ("child_frame", self.child_frame)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a nonempty frame name")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent_frame and child_frame must differ")
        if not isinstance(self.transform, SE3):
            raise TypeError("transform must be an SE3")

    @property
    def parent(self) -> str:
        return self.parent_frame

    @property
    def child(self) -> str:
        return self.child_frame

    def then(self, next_edge: "FrameTransform") -> "FrameTransform":
        """Append an edge from this child frame to the next child frame."""
        if not isinstance(next_edge, FrameTransform):
            raise TypeError("next_edge must be a FrameTransform")
        if self.child_frame != next_edge.parent_frame:
            raise ValueError(
                f"frame discontinuity: {self.child_frame!r} != {next_edge.parent_frame!r}"
            )
        return FrameTransform(
            self.parent_frame,
            next_edge.child_frame,
            self.transform @ next_edge.transform,
        )

    def inverse(self) -> "FrameTransform":
        return FrameTransform(self.child_frame, self.parent_frame, self.transform.inverse())

    def apply(self, point_child: Sequence[float]) -> Vector3:
        return self.transform.apply(point_child)


def compose_frame_chain(*edges: FrameTransform) -> FrameTransform:
    """Compose a nonempty, contiguous sequence of labelled frame edges."""
    if not edges:
        raise ValueError("at least one frame edge is required")
    result = edges[0]
    for edge in edges[1:]:
        result = result.then(edge)
    return result


class OpticalConvention(str, Enum):
    """Supported camera local axes.

    OpenCV optical uses +X right, +Y down, +Z forward.  USD cameras use +X
    right, +Y up and look along -Z.  Both are right-handed, so their basis
    change is a 180 degree rotation about X.
    """

    OPENCV = "opencv_optical"
    USD_CAMERA = "usd_camera"


def _convention(value: OpticalConvention | str) -> OpticalConvention:
    if isinstance(value, OpticalConvention):
        return value
    aliases = {
        "opencv": OpticalConvention.OPENCV,
        "opencv_optical": OpticalConvention.OPENCV,
        "usd": OpticalConvention.USD_CAMERA,
        "usd_camera": OpticalConvention.USD_CAMERA,
    }
    try:
        return aliases[value.lower()]
    except (AttributeError, KeyError) as exc:
        raise ValueError(f"unsupported optical convention: {value!r}") from exc


def optical_basis_transform(
    source: OpticalConvention | str,
    target: OpticalConvention | str,
) -> SE3:
    """Return ``T_source_target`` for a camera basis conversion.

    The returned transform has zero translation.  For example, when source is
    OpenCV and target is USD, ``p_source = T_source_target * p_target`` and
    ``T_source_target`` maps ``(x, y, z)`` to ``(x, -y, -z)``.
    """
    source_convention = _convention(source)
    target_convention = _convention(target)
    if source_convention is target_convention:
        return SE3.identity()
    # 180 degrees about +X: quaternion (w, x, y, z) = (0, 1, 0, 0).
    return SE3(rotation_wxyz=(0.0, 1.0, 0.0, 0.0))


def adapt_optical_transform(
    parent_from_source: SE3,
    source: OpticalConvention | str,
    target: OpticalConvention | str,
) -> SE3:
    """Express ``T_parent_optical`` using another optical basis."""
    if not isinstance(parent_from_source, SE3):
        raise TypeError("parent_from_source must be an SE3")
    return parent_from_source @ optical_basis_transform(source, target)


def compose_camera_chain(
    world_from_flange: FrameTransform,
    flange_from_mount: FrameTransform,
    mount_from_housing: FrameTransform,
    housing_from_optical: FrameTransform,
    *,
    target_convention: OpticalConvention | str | None = None,
    source_convention: OpticalConvention | str = OpticalConvention.OPENCV,
) -> FrameTransform:
    """Build the explicit world-to-optical chain from DESIGN.md.

    The final edge is interpreted in ``source_convention``.  When a target
    convention is supplied, only the final optical basis is adapted; the
    physical housing and mounting transforms remain unchanged.
    """
    composed = compose_frame_chain(
        world_from_flange,
        flange_from_mount,
        mount_from_housing,
        housing_from_optical,
    )
    if target_convention is None:
        return composed
    converted = adapt_optical_transform(
        composed.transform,
        source_convention,
        target_convention,
    )
    return FrameTransform(composed.parent_frame, composed.child_frame, converted)


class MissingCalibrationError(ValueError):
    """Raised when a production chain is requested with unresolved inputs."""

    def __init__(self, missing: Iterable[str]):
        values = tuple(dict.fromkeys(str(item) for item in missing))
        if not values:
            raise ValueError("MissingCalibrationError requires at least one item")
        self.missing = values
        super().__init__("missing calibration parameters: " + ", ".join(values))


def require_complete_transforms(values: dict[str, SE3 | None]) -> dict[str, SE3]:
    """Validate a named transform set before building a physical scene chain."""
    missing = tuple(name for name, value in values.items() if value is None)
    if missing:
        raise MissingCalibrationError(missing)
    invalid = tuple(name for name, value in values.items() if not isinstance(value, SE3))
    if invalid:
        raise TypeError("transform values must be SE3 or None: " + ", ".join(invalid))
    return {name: value for name, value in values.items() if value is not None}
