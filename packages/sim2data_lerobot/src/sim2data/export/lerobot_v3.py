"""Strict adapter for the official LeRobot v3 writer and loader.

The adapter accepts only a reviewed schema.  ``ExportSchema.smoke_test`` is a
synthetic schema for format checks and must not be used as a robot manifest.
Episodes are validated in memory before the official SDK sees them.  A small
exclusive marker beside each output root prevents two writers from targeting
the same dataset.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
import importlib.metadata
import inspect
import json
import math
import os
from pathlib import Path
import time
from types import MappingProxyType
from typing import Any, Mapping, Sequence
import uuid

from sim2data.core import FrameClock, Timing, validate_episode_clocks

try:  # Optional in the dependency-free foundation environment.
    import numpy as _np
except ImportError:  # pragma: no cover - exercised by the root environment
    _np = None


class SDKUnavailableError(RuntimeError):
    """The installed environment cannot provide the required official API."""


@dataclass(frozen=True)
class LeRobotSDKInfo:
    """Evidence collected before creating a dataset root."""

    package_version: str
    codebase_version: str
    module_file: str


def _import_lerobot() -> tuple[Any, LeRobotSDKInfo]:
    """Import and check the exact v3 writer surface required by D_EXPORT.

    Older v3 builds may expose additional legacy writer methods or a different
    encoder configuration surface.  The adapter checks the required methods
    directly and adapts only the documented codec argument where the official
    release lacks ``VideoEncoderConfig``.
    """

    try:
        package_version = importlib.metadata.version("lerobot")
    except importlib.metadata.PackageNotFoundError as exc:
        raise SDKUnavailableError("the official 'lerobot' package is not installed") from exc

    try:
        module = importlib.import_module("lerobot.datasets.lerobot_dataset")
        dataset_cls = module.LeRobotDataset
    except (ImportError, AttributeError) as exc:
        raise SDKUnavailableError(
            "installed lerobot does not expose lerobot.datasets.LeRobotDataset"
        ) from exc

    # CODEBASE_VERSION moved between releases.  Reading it from the metadata
    # module first supports current official releases while the module fallback
    # keeps the check useful when a compatible release inlines the constant.
    try:
        metadata_module = importlib.import_module("lerobot.datasets.dataset_metadata")
        codebase_version = getattr(metadata_module, "CODEBASE_VERSION", None)
    except ImportError:
        codebase_version = None
    if codebase_version is None:
        codebase_version = getattr(module, "CODEBASE_VERSION", None)

    required = ("create", "add_frame", "save_episode", "finalize")
    if codebase_version != "v3.0" or any(
        not callable(getattr(dataset_cls, name, None)) for name in required
    ):
        missing = [name for name in required if not callable(getattr(dataset_cls, name, None))]
        raise SDKUnavailableError(
            f"lerobot {package_version} is not the required v3.0 writer API "
            f"(codebase_version={codebase_version!r}, missing={missing})"
        )

    return dataset_cls, LeRobotSDKInfo(
        package_version=package_version,
        codebase_version=str(codebase_version),
        module_file=str(Path(module.__file__).resolve()),
    )


def require_lerobot_v3() -> LeRobotSDKInfo:
    """Return checked SDK metadata without creating an output directory."""

    _, info = _import_lerobot()
    return info


def _require_numpy() -> Any:
    if _np is None:
        raise SDKUnavailableError("NumPy is required by the LeRobot export environment")
    return _np


def _nonempty_names(values: Sequence[str], field_name: str) -> tuple[str, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{field_name} must be a sequence of names")
    try:
        names = tuple(values)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be a sequence of names") from exc
    if not names or any(not isinstance(value, str) or not value.strip() for value in names):
        raise ValueError(f"{field_name} must contain nonempty strings")
    if len(names) != len(set(names)):
        raise ValueError(f"{field_name} must be unique")
    return names


def _shape3(value: Sequence[int], field_name: str) -> tuple[int, int, int]:
    try:
        shape = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be a positive (height, width, channels) tuple") from exc
    if len(shape) != 3 or any(type(item) is not int or item <= 0 for item in shape):
        raise ValueError(f"{field_name} must be a positive (height, width, channels) tuple")
    if shape[2] != 3:
        raise ValueError(f"{field_name} must have three RGB channels")
    return shape


@dataclass(frozen=True)
class ExportSchema:
    """Manifest-derived channel names and HWC RGB camera shapes."""

    name: str
    state_names: tuple[str, ...]
    action_names: tuple[str, ...]
    camera_shapes: Mapping[str, tuple[int, int, int]]

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("schema name must be a nonempty string")
        state_names = _nonempty_names(self.state_names, "state_names")
        action_names = _nonempty_names(self.action_names, "action_names")
        if not isinstance(self.camera_shapes, Mapping) or not self.camera_shapes:
            raise ValueError("camera_shapes must contain at least one confirmed camera")
        camera_shapes: dict[str, tuple[int, int, int]] = {}
        for key, shape in self.camera_shapes.items():
            if not isinstance(key, str) or not key.startswith("observation.images."):
                raise ValueError("camera feature keys must start with observation.images.")
            camera_shapes[key] = _shape3(shape, f"camera_shapes[{key!r}]")
        object.__setattr__(self, "state_names", state_names)
        object.__setattr__(self, "action_names", action_names)
        object.__setattr__(self, "camera_shapes", MappingProxyType(camera_shapes))

    @classmethod
    def smoke_test(
        cls,
        *,
        height: int = 16,
        width: int = 16,
        state_names: Sequence[str] = ("smoke_state_0", "smoke_state_1"),
        action_names: Sequence[str] = ("smoke_action_0", "smoke_action_1"),
    ) -> "ExportSchema":
        """Build the virtual three-camera schema used only by format smoke tests."""

        shape = _shape3((height, width, 3), "smoke_test image shape")
        return cls(
            name="smoke_test",
            state_names=tuple(state_names),
            action_names=tuple(action_names),
            camera_shapes={
                "observation.images.smoke_test_overhead": shape,
                "observation.images.smoke_test_wrist_left": shape,
                "observation.images.smoke_test_wrist_right": shape,
            },
        )

    @property
    def camera_keys(self) -> tuple[str, ...]:
        return tuple(self.camera_shapes)

    @property
    def features(self) -> dict[str, dict[str, Any]]:
        """Return the official feature declaration consumed by ``create``."""

        return {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(self.state_names),),
                "names": {"axes": list(self.state_names)},
            },
            "action": {
                "dtype": "float32",
                "shape": (len(self.action_names),),
                "names": {"axes": list(self.action_names)},
            },
            **{
                key: {
                    "dtype": "video",
                    "shape": shape,
                    "names": ["height", "width", "channels"],
                }
                for key, shape in self.camera_shapes.items()
            },
        }


def _finite_vector(value: Any, expected: int, name: str) -> Any:
    np = _require_numpy()
    if isinstance(value, (str, bytes, bool)):
        raise ValueError(f"{name} must be a numeric vector")
    try:
        raw = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a numeric vector") from exc
    if raw.ndim != 1 or raw.shape != (expected,):
        raise ValueError(f"{name} must have shape ({expected},), got {raw.shape}")
    if raw.dtype.kind not in "iuf" or raw.dtype.kind == "b":
        raise ValueError(f"{name} must contain real numeric values")
    try:
        converted = np.asarray(raw, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain float32-compatible values") from exc
    if not np.isfinite(converted).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return np.ascontiguousarray(converted)


def _rgb_image(value: Any, expected: tuple[int, int, int], name: str) -> Any:
    np = _require_numpy()
    try:
        image = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an RGB image array") from exc
    height, width, channels = expected
    if image.shape == (channels, height, width):
        image = np.transpose(image, (1, 2, 0))
    if image.shape != expected:
        raise ValueError(f"{name} must have HWC shape {expected}, got {image.shape}")
    if image.dtype.kind not in "iuf" or image.dtype.kind == "b":
        raise ValueError(f"{name} must contain real numeric pixels")
    if image.dtype == np.uint8:
        result = image
    else:
        if not np.isfinite(image).all():
            raise ValueError(f"{name} contains NaN or infinity")
        if image.dtype.kind == "f" and float(image.min()) >= 0.0 and float(image.max()) <= 1.0:
            result = np.rint(image * 255.0).astype(np.uint8)
        elif image.dtype.kind in "iu" and int(image.min()) >= 0 and int(image.max()) <= 255:
            result = image.astype(np.uint8)
        else:
            raise ValueError(f"{name} pixels must be uint8 or finite floats in [0, 1]")
    return np.ascontiguousarray(result)


@dataclass(frozen=True)
class FrameSample:
    """One observation/action sample plus optional simulation clock evidence."""

    state: Any
    action: Any
    cameras: Mapping[str, Any]
    task: str
    frame_index: int | None = None
    physics_step: int | None = None
    timestamp: float | None = None
    camera_physics_steps: tuple[int, ...] | None = None


@dataclass
class _EpisodeStage:
    task: str
    frames: list[FrameSample] = field(default_factory=list)
    privileged: Mapping[str, Any] | None = None


@dataclass(frozen=True)
class DatasetReadback:
    """Loader evidence used by smoke reports and integration tests."""

    root: Path
    package_version: str
    codebase_version: str
    num_frames: int
    num_episodes: int
    feature_keys: tuple[str, ...]
    stats_keys: tuple[str, ...]
    video_keys: tuple[str, ...]


class LeRobotV3Writer:
    """One-owner staged writer backed by the official v3 SDK."""

    def __init__(
        self,
        *,
        root: str | Path,
        schema: ExportSchema,
        fps: int = 30,
        timing: Timing | None = None,
        repo_id: str = "sim2data/smoke_test",
        robot_type: str | None = None,
        camera_encoder: Any | None = None,
        video_files_size_in_mb: float | None = None,
        data_files_size_in_mb: float | None = None,
    ) -> None:
        if type(fps) is not int or fps <= 0:
            raise ValueError("fps must be a positive integer")
        if not isinstance(schema, ExportSchema):
            raise TypeError("schema must be an ExportSchema built from an approved manifest")
        if not isinstance(repo_id, str) or not repo_id.strip():
            raise ValueError("repo_id must be a nonempty string")
        if robot_type is not None and (not isinstance(robot_type, str) or not robot_type.strip()):
            raise ValueError("robot_type must be None or a nonempty reviewed-manifest value")
        if timing is not None and timing.control_hz != fps:
            raise ValueError("timing.control_hz must match the LeRobot fps")
        self.timing = timing or Timing(physics_hz=fps * 8, control_hz=fps)
        self.fps = fps
        self.schema = schema
        self.repo_id = repo_id
        self.root = Path(root).expanduser().resolve()
        self._stage: _EpisodeStage | None = None
        self.aborted_reasons: list[str] = []
        self._finalized = False
        self._fatal_error: BaseException | None = None
        self._lock_path = self.root.parent / f".{self.root.name}.sim2data.writer.lock"
        self._lock_owned = False

        dataset_cls, self.sdk_info = _import_lerobot()
        self._acquire_root()
        try:
            create_kwargs: dict[str, Any] = {
                "repo_id": repo_id,
                "fps": fps,
                "features": schema.features,
                "root": self.root,
                "robot_type": robot_type or schema.name,
                "use_videos": True,
                "image_writer_processes": 0,
                "image_writer_threads": 0,
                "video_backend": "pyav",
                "batch_encoding_size": 1,
            }
            create_params = inspect.signature(dataset_cls.create).parameters
            if camera_encoder is not None:
                if "rgb_encoder" in create_params:
                    # Current official v3 releases name the RGB video encoder
                    # ``rgb_encoder``.  Keep accepting the adapter's neutral
                    # ``camera_encoder`` argument so callers do not depend on
                    # a particular SDK release's spelling.
                    create_kwargs["rgb_encoder"] = camera_encoder
                elif "camera_encoder" in create_params:
                    create_kwargs["camera_encoder"] = camera_encoder
                elif "vcodec" in create_params:
                    # LeRobot 0.4.x exposes the same v3 writer surface but
                    # accepts a codec name instead of VideoEncoderConfig.
                    codec = getattr(camera_encoder, "vcodec", None)
                    if isinstance(codec, str) and codec:
                        create_kwargs["vcodec"] = codec
            for key, value in (
                ("video_files_size_in_mb", video_files_size_in_mb),
                ("data_files_size_in_mb", data_files_size_in_mb),
            ):
                if value is not None:
                    if (
                        not isinstance(value, (int, float))
                        or isinstance(value, bool)
                        or not math.isfinite(value)
                        or value <= 0
                    ):
                        raise ValueError(f"{key} must be a positive finite number")
                    if key in create_params:
                        create_kwargs[key] = value
            self.dataset = dataset_cls.create(**create_kwargs)
        except BaseException:
            self._release_root()
            raise

    def _acquire_root(self) -> None:
        self.root.parent.mkdir(parents=True, exist_ok=True)
        if self.root.exists():
            raise RuntimeError(f"LeRobot writer root already exists: {self.root}")
        payload = {
            "owner": "sim2data.export.LeRobotV3Writer",
            "pid": os.getpid(),
            "created_unix": time.time(),
            "token": uuid.uuid4().hex,
        }
        try:
            fd = os.open(self._lock_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL)
        except FileExistsError as exc:
            raise RuntimeError(f"LeRobot root is already owned by another writer: {self.root}") from exc
        try:
            os.write(fd, json.dumps(payload, sort_keys=True).encode("utf-8"))
        finally:
            os.close(fd)
        self._lock_owned = True

    def _release_root(self) -> None:
        if self._lock_owned:
            try:
                self._lock_path.unlink()
            except FileNotFoundError:
                pass
            finally:
                self._lock_owned = False

    @property
    def pending_frames(self) -> int:
        return 0 if self._stage is None else len(self._stage.frames)

    @property
    def num_episodes(self) -> int:
        return int(self.dataset.meta.total_episodes)

    def begin_episode(self, task: str, *, privileged: Mapping[str, Any] | None = None) -> None:
        self._ensure_open()
        if self._stage is not None:
            raise RuntimeError("an episode is already staged; save or abort it first")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task must be a nonempty string")
        if privileged is not None and not isinstance(privileged, Mapping):
            raise ValueError("privileged sidecar data must be a mapping")
        self._stage = _EpisodeStage(task=task, privileged=privileged)

    def add_frame(self, frame: FrameSample | None = None, **values: Any) -> None:
        """Validate and stage one frame without calling the official writer."""

        self._ensure_open()
        if self._stage is None:
            raise RuntimeError("call begin_episode() before add_frame()")
        if frame is not None and values:
            raise TypeError("pass either a FrameSample or keyword frame values")
        if frame is None:
            try:
                frame = FrameSample(**values)
            except TypeError as exc:
                raise TypeError(
                    "keyword frame values require state, action, cameras, and task"
                ) from exc
        if not isinstance(frame, FrameSample):
            raise TypeError("frame must be a FrameSample")
        self._stage.frames.append(self._validate_sample(frame, len(self._stage.frames), self._stage.task))

    def _validate_sample(self, frame: FrameSample, expected_index: int, task: str) -> FrameSample:
        if frame.task != task:
            raise ValueError("all frames in an episode must use its declared task")
        if frame.frame_index is not None and (
            type(frame.frame_index) is not int or frame.frame_index != expected_index
        ):
            raise ValueError("frame_index is not contiguous")
        expected_step = expected_index * self.timing.ticks_per_frame
        if frame.physics_step is not None and (
            type(frame.physics_step) is not int or frame.physics_step != expected_step
        ):
            raise ValueError("physics_step does not match the simulation-time grid")
        expected_timestamp = self.timing.timestamp(expected_index)
        if frame.timestamp is not None:
            if (
                isinstance(frame.timestamp, bool)
                or not isinstance(frame.timestamp, (int, float))
                or not math.isfinite(frame.timestamp)
            ):
                raise ValueError("timestamp must be a finite real number")
            if not math.isclose(frame.timestamp, expected_timestamp, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError("timestamp does not match the simulation-time grid")
        expected_camera_steps = tuple(expected_step for _ in self.schema.camera_keys)
        if frame.camera_physics_steps is not None:
            actual_steps = tuple(frame.camera_physics_steps)
            if any(type(step) is not int for step in actual_steps) or actual_steps != expected_camera_steps:
                raise ValueError("camera frame is delayed, missing, or from a different physics step")
        if not isinstance(frame.cameras, Mapping):
            raise ValueError("cameras must be a mapping")
        if set(frame.cameras) != set(self.schema.camera_keys):
            missing = set(self.schema.camera_keys) - set(frame.cameras)
            extra = set(frame.cameras) - set(self.schema.camera_keys)
            raise ValueError(f"camera schema mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
        # Copy at the staging boundary.  A simulator commonly reuses one
        # NumPy buffer for the next tick; retaining a view here would let a
        # later tick mutate an already validated frame before commit.
        return FrameSample(
            state=_finite_vector(frame.state, len(self.schema.state_names), "observation.state").copy(),
            action=_finite_vector(frame.action, len(self.schema.action_names), "action").copy(),
            cameras={
                key: _rgb_image(frame.cameras[key], self.schema.camera_shapes[key], key).copy()
                for key in self.schema.camera_keys
            },
            task=task,
            frame_index=expected_index,
            physics_step=expected_step,
            timestamp=expected_timestamp,
            camera_physics_steps=expected_camera_steps,
        )

    def _validate_stage(self) -> list[FrameSample]:
        if self._stage is None:
            raise RuntimeError("no episode is staged")
        if not self._stage.frames:
            raise ValueError("empty episode is not allowed")
        clocks = [
            FrameClock(
                frame_index=frame.frame_index if frame.frame_index is not None else -1,
                physics_step=frame.physics_step if frame.physics_step is not None else -1,
                timestamp=frame.timestamp if frame.timestamp is not None else float("nan"),
                camera_physics_steps=frame.camera_physics_steps or (),
            )
            for frame in self._stage.frames
        ]
        validate_episode_clocks(clocks, self.timing, len(self.schema.camera_keys))
        return list(self._stage.frames)

    def save_episode(self, *, sidecar: Mapping[str, Any] | None = None, parallel_encoding: bool = False) -> int:
        """Commit one validated stage through official ``add_frame/save_episode``."""

        self._ensure_open()
        frames = self._validate_stage()
        stage = self._stage
        assert stage is not None
        episode_index = self.num_episodes
        sidecar_data = sidecar if sidecar is not None else stage.privileged
        serialized_sidecar: str | None = None
        if sidecar_data is not None:
            if not isinstance(sidecar_data, Mapping):
                raise ValueError("sidecar data must be a mapping")
            # Validate before touching the official writer.  If this fails,
            # the caller can repair the staged sidecar and retry safely.
            serialized_sidecar = _serialize_sidecar(sidecar_data)
        try:
            for frame in frames:
                self.dataset.add_frame(
                    {
                        "observation.state": frame.state.copy(),
                        "action": frame.action.copy(),
                        **{key: value.copy() for key, value in frame.cameras.items()},
                        "task": frame.task,
                    }
                )
            self.dataset.save_episode(parallel_encoding=parallel_encoding)
        except BaseException as exc:
            self._fatal_error = exc
            clear_buffer = getattr(self.dataset, "clear_episode_buffer", None)
            if callable(clear_buffer):
                try:
                    clear_buffer(delete_images=True)
                except BaseException:
                    pass
            raise
        self._stage = None
        if sidecar_data is not None:
            try:
                self._write_sidecar(episode_index, serialized_sidecar or "{}")
            except BaseException as exc:
                # The episode is already in the official index, but without
                # its required sidecar the root is not safe to continue.
                self._fatal_error = exc
                raise
        return episode_index

    def _write_sidecar(self, episode_index: int, serialized_sidecar: str) -> Path:
        sidecar_dir = self.root / "sidecar"
        sidecar_dir.mkdir(parents=True, exist_ok=True)
        path = sidecar_dir / f"episode-{episode_index:06d}.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(serialized_sidecar, encoding="utf-8")
        temporary.replace(path)
        return path

    def abort_episode(self, reason: str = "aborted") -> None:
        """Drop an in-memory stage; it never reaches the official index."""

        self._ensure_open()
        if self._stage is None:
            raise RuntimeError("no episode is staged")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a nonempty string")
        self.aborted_reasons.append(reason)
        self._stage = None

    def finalize(self) -> None:
        """Flush official metadata/video indexes and release the root lock."""

        if self._finalized:
            return
        if self._stage is not None:
            raise RuntimeError("cannot finalize with a staged episode; save or abort it first")
        if self._fatal_error is not None:
            raise RuntimeError("cannot finalize after an official writer failure") from self._fatal_error
        self.dataset.finalize()
        self._finalized = True
        self._release_root()

    def _ensure_open(self) -> None:
        if self._finalized:
            raise RuntimeError("writer is finalized")
        if self._fatal_error is not None:
            raise RuntimeError("writer is unusable after an official writer failure") from self._fatal_error

    def readback(self, *, delta_timestamps: Mapping[str, Sequence[float]] | None = None) -> Any:
        if not self._finalized:
            raise RuntimeError("finalize the writer before loading it")
        return load_lerobot_dataset(root=self.root, repo_id=self.repo_id, delta_timestamps=delta_timestamps)

    def __enter__(self) -> "LeRobotV3Writer":
        self._ensure_open()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        if exc_type is not None:
            self._stage = None
            self._release_root()
            return
        if not self._finalized:
            if self._stage is not None:
                raise RuntimeError("context exited with an unsaved episode")
            self.finalize()

    def __del__(self) -> None:  # pragma: no cover - interpreter shutdown safety net
        try:
            self._release_root()
        except Exception:
            pass


def _json_default(value: Any) -> Any:
    np = _np
    if np is not None and isinstance(value, np.ndarray):
        return value.tolist()
    if np is not None and isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"value of type {type(value).__name__} is not JSON serializable")


def _serialize_sidecar(sidecar: Mapping[str, Any]) -> str:
    try:
        return json.dumps(sidecar, sort_keys=True, indent=2, allow_nan=False, default=_json_default)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("sidecar data must be finite JSON-serializable values") from exc


def load_lerobot_dataset(
    *,
    root: str | Path,
    repo_id: str = "sim2data/smoke_test",
    delta_timestamps: Mapping[str, Sequence[float]] | None = None,
    video_backend: str = "pyav",
) -> Any:
    """Open a finalized local dataset through the official loader."""

    dataset_cls, _ = _import_lerobot()
    if delta_timestamps is not None:
        for key, values in delta_timestamps.items():
            if not isinstance(key, str):
                raise ValueError("delta timestamp feature keys must be strings")
            for value in values:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"delta timestamp for {key!r} must be finite")
    return dataset_cls(
        repo_id=repo_id,
        root=Path(root).expanduser().resolve(),
        delta_timestamps={key: list(values) for key, values in delta_timestamps.items()}
        if delta_timestamps is not None
        else None,
        video_backend=video_backend,
        download_videos=False,
    )


def read_training_batch(dataset: Any, batch_size: int = 2) -> Mapping[str, Any]:
    """Read one batch through the official Torch Dataset/DataLoader path."""

    if type(batch_size) is not int or batch_size <= 0:
        raise ValueError("batch_size must be a positive integer")
    try:
        import torch
    except ImportError as exc:  # pragma: no cover - only export envs have torch
        raise SDKUnavailableError("Torch is required for the batch smoke") from exc
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    try:
        return next(iter(loader))
    except StopIteration as exc:
        raise ValueError("cannot read a batch from an empty dataset") from exc


def summarize_readback(dataset: Any) -> DatasetReadback:
    """Collect loader metadata without making robot-fidelity claims."""

    info = dataset.meta
    stats = getattr(info, "stats", None) or {}
    features = getattr(info, "features", {}) or {}
    video_keys = getattr(info, "video_keys", ()) or ()
    codebase_version = str(getattr(info, "codebase_version", getattr(info, "_version", "v3.0")))
    # Some official releases expose the same format as ``3.0`` on the loaded
    # metadata object while the public constant is ``v3.0``.  Keep the report
    # stable and compare the normalized value with the preflight contract.
    if codebase_version == "3.0":
        codebase_version = "v3.0"
    return DatasetReadback(
        root=Path(dataset.root),
        package_version=importlib.metadata.version("lerobot"),
        codebase_version=codebase_version,
        num_frames=int(getattr(dataset, "num_frames", len(dataset))),
        num_episodes=int(getattr(dataset, "num_episodes", info.total_episodes)),
        feature_keys=tuple(features),
        stats_keys=tuple(stats),
        video_keys=tuple(video_keys),
    )


__all__ = [
    "DatasetReadback",
    "ExportSchema",
    "FrameSample",
    "LeRobotSDKInfo",
    "LeRobotV3Writer",
    "SDKUnavailableError",
    "load_lerobot_dataset",
    "read_training_batch",
    "require_lerobot_v3",
    "summarize_readback",
]
