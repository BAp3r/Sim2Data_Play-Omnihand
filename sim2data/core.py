"""Small, backend-independent checks. These do NOT establish sim-to-real fidelity."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path, PurePosixPath
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Timing:
    """Initial contract: all policy camera streams use the control tick.

    At physics_step=k*ticks_per_frame, observe s[k], render I[k], choose and
    record a[k], then simulate ticks_per_frame ticks. No post-step image is
    allowed to be mislabeled as I[k]. Validate actual renderer timing separately.
    """
    physics_hz: int = 240
    control_hz: int = 30

    def __post_init__(self) -> None:
        for name, value in (("physics_hz", self.physics_hz), ("control_hz", self.control_hz)):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.physics_hz % self.control_hz:
            raise ValueError("physics_hz must be an integer multiple of control_hz")

    @property
    def ticks_per_frame(self) -> int:
        return self.physics_hz // self.control_hz

    def timestamp(self, frame_index: int) -> float:
        if type(frame_index) is not int or frame_index < 0:
            raise ValueError("frame_index must be a nonnegative integer")
        return frame_index / self.control_hz


def component_seed(dataset_seed: int, episode_id: int, component: str) -> int:
    """Stable seed independent of worker ID, launch order and Python hash seed.

    This does not promise bitwise deterministic GPU contact physics.
    """
    for value in (dataset_seed, episode_id):
        if type(value) is not int or value < 0:
            raise ValueError("seed and episode_id must be nonnegative integers")
    if not isinstance(component, str) or not component.strip():
        raise ValueError("component must be a nonempty string")
    payload = f"sim2data-seed-v1\0{dataset_seed}\0{episode_id}\0{component}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def resolve_local_asset(root: Path, relative_path: str) -> Path:
    """Resolve a manifest POSIX-relative path, refusing traversal and LFS pointers.

    The configured root itself may be a symlink to a mounted, read-only library.
    A child symlink escaping that root is rejected. No network downloader runs.
    """
    if not relative_path or "\\" in relative_path or ":" in relative_path:
        raise ValueError("use a nonempty POSIX-relative asset path, not a URI or drive path")
    rel = PurePosixPath(relative_path)
    if rel.is_absolute() or ".." in rel.parts:
        raise ValueError("absolute paths and parent traversal are forbidden")
    root = Path(root).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("asset root is not a directory")
    path = root.joinpath(*rel.parts).resolve(strict=True)
    if not path.is_relative_to(root):
        raise ValueError("asset resolves outside the configured library")
    if not path.is_file():
        raise ValueError("asset must be a file")
    with path.open("rb") as stream:
        if stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError("asset is an unresolved Git LFS pointer; fetch its LFS object")
    return path


def _finite_vector(values: Sequence[float], expected: int, name: str) -> None:
    if len(values) != expected:
        raise ValueError(f"{name}: expected {expected} values, got {len(values)}")
    for value in values:
        if isinstance(value, (str, bytes, bool)):
            raise ValueError(f"{name}: expected numeric values, not strings or booleans")
        try:
            valid = math.isfinite(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{name}: nonnumeric value") from exc
        if not valid:
            raise ValueError(f"{name}: NaN or infinity")


@dataclass(frozen=True)
class ChannelLayout:
    """Named state and COMMAND channels, not an assumed URDF DOF count.

    Actuator mapping, units and limits still require a reviewed robot manifest.
    Keeping separate names allows state and command dimensions to differ.
    """
    state_names: tuple[str, ...]
    action_names: tuple[str, ...]

    def __post_init__(self) -> None:
        for field in ("state_names", "action_names"):
            names = getattr(self, field)
            if isinstance(names, (str, bytes)) or not isinstance(names, Sequence):
                raise ValueError("channel names must be a sequence of names, not a single string")
            if not names or any(not isinstance(n, str) or not n.strip() for n in names):
                raise ValueError("channel names must be nonempty strings")
            if len(names) != len(set(names)):
                raise ValueError("channel names must be unique within each vector")
            object.__setattr__(self, field, tuple(names))

    def validate(self, state: Sequence[float], action: Sequence[float]) -> None:
        _finite_vector(state, len(self.state_names), "observation.state")
        _finite_vector(action, len(self.action_names), "action")


@dataclass(frozen=True)
class FrameClock:
    frame_index: int
    physics_step: int
    timestamp: float
    camera_physics_steps: tuple[int, ...]


def validate_episode_clocks(frames: Iterable[FrameClock], timing: Timing, camera_count: int) -> int:
    """Validate declared synchronization without buffering frames or image arrays.

    Call only on an already-approved, fixed camera schema. A missing physical
    camera must change the schema; never manufacture a black image stream.
    """
    if type(camera_count) is not int or camera_count < 1:
        raise ValueError("camera_count must be a positive integer")
    count = 0
    for expected_index, frame in enumerate(frames):
        expected_step = expected_index * timing.ticks_per_frame
        if type(frame.frame_index) is not int or frame.frame_index != expected_index:
            raise ValueError("noncontiguous episode frame indices")
        if type(frame.physics_step) is not int or frame.physics_step != expected_step:
            raise ValueError("incorrect physics step for observation")
        _finite_vector((frame.timestamp,), 1, "timestamp")
        if not math.isclose(
            frame.timestamp, timing.timestamp(expected_index), rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("timestamp is off the simulation-time grid")
        if len(frame.camera_physics_steps) != camera_count:
            raise ValueError("camera count changed or a stream is missing")
        if any(type(s) is not int or s != expected_step for s in frame.camera_physics_steps):
            raise ValueError("camera and proprioception refer to different simulation steps")
        count += 1
    if count == 0:
        raise ValueError("empty episode")
    return count
