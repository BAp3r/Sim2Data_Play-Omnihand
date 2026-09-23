"""Export one approved synthetic planned-motion capture through LeRobot v3."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path, PurePosixPath
import struct
import zlib
from typing import Any

FPS = 30
IMAGE_SHAPE = (240, 320, 3)
SAMPLE_KIND = "synthetic_kinematic_planned_motion"
CAMERAS = ("overhead", "wrist_left", "wrist_right")
JOINT_NAMES = tuple([f"left.joint{i}" for i in range(1, 7)] + [f"right.joint{i}" for i in range(1, 7)])
STATE_SEMANTICS = "planned joint configuration used for USD FK; not physical feedback"
ACTION_SEMANTICS = "next-frame planned joint configuration; not hardware command"
_CAPTURE_FIELDS = {
    "sample_kind", "production_collection_allowed", "task_success", "fps",
    "joint_names", "state_semantics", "action_semantics", "frames",
    "image_shape", "physics_validated",
}


@dataclass(frozen=True)
class CaptureFrame:
    index: int
    timestamp: float
    state: tuple[float, ...]
    action: tuple[float, ...]
    images: dict[str, Path]


@dataclass(frozen=True)
class MotionCapture:
    root: Path
    frames: tuple[CaptureFrame, ...]
    provenance: dict[str, Any]


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"capture.json contains duplicate key {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"capture.json contains invalid numeric constant {value}")


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        converted = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite number")
    try:
        struct.pack("f", converted)
    except OverflowError as exc:
        raise ValueError(f"{name} must fit in float32") from exc
    return converted


def _vector(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != len(JOINT_NAMES):
        raise ValueError(f"{name} must contain exactly {len(JOINT_NAMES)} joint values")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def _validate_png_shape(path: Path) -> None:
    if not 33 <= path.stat().st_size <= 32 * 1024 * 1024:
        raise ValueError(f"{path.name} has an invalid or oversized PNG file")
    with path.open("rb") as stream:
        header = stream.read(33)
    if header[:8] != b"\x89PNG\r\n\x1a\n" or header[8:16] != b"\x00\x00\x00\rIHDR":
        raise ValueError(f"{path.name} has no valid PNG IHDR chunk")
    if zlib.crc32(header[12:29]) & 0xFFFFFFFF != struct.unpack(">I", header[29:33])[0]:
        raise ValueError(f"{path.name} has an invalid PNG IHDR checksum")
    width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", header[16:29])
    if (height, width, 3) != IMAGE_SHAPE:
        raise ValueError(f"{path.name} must have RGB shape {IMAGE_SHAPE}")
    if (depth, color, compression, filtering, interlace) != (8, 2, 0, 0, 0):
        raise ValueError(f"{path.name} must be a non-interlaced 8-bit RGB PNG")


def _image_path(root: Path, relative: Any, camera: str, index: int) -> Path:
    expected = f"images/{camera}/{index:06d}.png"
    if not isinstance(relative, str) or relative != expected:
        raise ValueError(f"frame {index} {camera} image path must be {expected!r}")
    posix = PurePosixPath(relative)
    if posix.is_absolute() or ".." in posix.parts or "\\" in relative or ":" in relative:
        raise ValueError("image path must remain inside the capture directory")
    try:
        path = root.joinpath(*posix.parts).resolve(strict=True)
    except FileNotFoundError as exc:
        raise ValueError(f"frame {index} {camera} image is missing: {relative}") from exc
    if not path.is_relative_to(root):
        raise ValueError(f"frame {index} {camera} image resolves outside the capture directory")
    if not path.is_file():
        raise ValueError(f"frame {index} {camera} image is not a file")
    _validate_png_shape(path)
    return path


def load_capture(capture_dir: str | Path) -> MotionCapture:
    """Validate the frozen capture and PNG inputs without SDK imports."""
    root = Path(capture_dir).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("capture directory must be a directory")
    try:
        manifest = json.loads(
            (root / "capture.json").read_text(encoding="utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_json_constant,
        )
    except OSError as exc:
        raise ValueError("capture.json is missing or unreadable") from exc
    except json.JSONDecodeError as exc:
        raise ValueError("capture.json is not valid JSON") from exc
    if not isinstance(manifest, dict) or not _CAPTURE_FIELDS <= set(manifest):
        raise ValueError("capture.json is missing required approved motion-capture fields")
    expected = {
        "sample_kind": SAMPLE_KIND,
        "production_collection_allowed": False,
        "task_success": None,
        "fps": FPS,
        "joint_names": list(JOINT_NAMES),
        "state_semantics": STATE_SEMANTICS,
        "action_semantics": ACTION_SEMANTICS,
        "image_shape": list(IMAGE_SHAPE),
        "physics_validated": False,
    }
    for field, value in expected.items():
        if manifest[field] != value or type(manifest[field]) is not type(value):
            raise ValueError(f"capture.json {field} does not match the approved manifest")
    items = manifest["frames"]
    if not isinstance(items, list) or not items:
        raise ValueError("capture.json frames must contain at least one frame")
    frames = []
    for index, item in enumerate(items):
        allowed_frame_fields = {"index", "timestamp", "state", "action", "images", "camera_T_world_usd"}
        if not isinstance(item, dict) or not {"index", "timestamp", "state", "action", "images"} <= set(item) or not set(item) <= allowed_frame_fields:
            raise ValueError(f"frame {index} fields do not match the approved manifest")
        if type(item["index"]) is not int or item["index"] != index:
            raise ValueError("frame indices must be contiguous and start at zero")
        timestamp = _number(item["timestamp"], f"frame {index} timestamp")
        if not math.isclose(timestamp, index / FPS, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"frame {index} timestamp must equal replay-relative index / {FPS}")
        images = item["images"]
        if not isinstance(images, dict) or set(images) != set(CAMERAS):
            raise ValueError(f"frame {index} must contain the three approved camera streams")
        frames.append(CaptureFrame(
            index=index,
            timestamp=timestamp,
            state=_vector(item["state"], f"frame {index} state"),
            action=_vector(item["action"], f"frame {index} action"),
            images={camera: _image_path(root, images[camera], camera, index) for camera in CAMERAS},
        ))
    provenance = {key: value for key, value in manifest.items() if key not in _CAPTURE_FIELDS}
    if any("camera_T_world_usd" in item for item in items):
        poses = []
        for item in items:
            pose = item.get("camera_T_world_usd")
            if not isinstance(pose, dict) or set(pose) != set(CAMERAS):
                raise ValueError("every frame must provide all camera poses when enabled")
            for matrix in pose.values():
                if not isinstance(matrix, list) or len(matrix) != 4:
                    raise ValueError("camera poses must be 4x4 matrices")
                for row in matrix:
                    if not isinstance(row, list) or len(row) != 4:
                        raise ValueError("camera poses must be 4x4 matrices")
                    for value in row:
                        _number(value, "camera pose")
            poses.append(pose)
        if "camera_frame_poses" in provenance and provenance["camera_frame_poses"] != poses:
            raise ValueError("per-frame camera poses disagree with provenance")
        provenance["camera_frame_poses"] = poses
    return MotionCapture(root=root, frames=tuple(frames), provenance=provenance)


def _read_rgb(path: Path, np: Any) -> Any:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow is required to decode approved RGB PNG captures") from exc
    try:
        with Image.open(path) as image:
            if image.format != "PNG" or image.mode != "RGB" or image.size != (IMAGE_SHAPE[1], IMAGE_SHAPE[0]):
                raise ValueError(f"{path.name} does not decode to the approved RGB image shape")
            image.load()
            pixels = np.asarray(image, dtype=np.uint8)
    except OSError as exc:
        raise ValueError(f"{path.name} cannot be decoded as a valid PNG") from exc
    if pixels.shape != IMAGE_SHAPE:
        raise ValueError(f"{path.name} decoded to {pixels.shape}, expected {IMAGE_SHAPE}")
    return np.ascontiguousarray(pixels)


def export_capture(capture_dir: str | Path, output_root: str | Path) -> dict[str, Any]:
    """Write one official LeRobot episode, finalize, load, and validate a batch."""
    capture = load_capture(capture_dir)
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("NumPy is required in the approved LeRobot export environment") from exc
    try:
        from sim2data.core import Timing
        from sim2data.export import (
            ExportSchema, FrameSample, LeRobotV3Writer, load_lerobot_dataset,
            read_training_batch, require_lerobot_v3, summarize_readback,
        )
    except ImportError as exc:
        raise RuntimeError("sim2data LeRobot adapter is unavailable in this environment") from exc
    output = Path(output_root).expanduser().resolve()
    if output == capture.root or output.is_relative_to(capture.root) or capture.root.is_relative_to(output):
        raise ValueError("output root must be separate from the private capture directory")
    sdk = require_lerobot_v3()
    camera_keys = {camera: f"observation.images.{camera}" for camera in CAMERAS}
    schema = ExportSchema(
        name=SAMPLE_KIND,
        state_names=JOINT_NAMES,
        action_names=JOINT_NAMES,
        camera_shapes={key: IMAGE_SHAPE for key in camera_keys.values()},
    )
    writer = LeRobotV3Writer(
        root=output,
        schema=schema,
        fps=FPS,
        timing=Timing(physics_hz=FPS * 8, control_hz=FPS),
        repo_id="sim2data/synthetic_kinematic_planned_motion",
    )
    task = "synthetic planned motion; task success unknown"
    writer.begin_episode(task, privileged={
        "sample_kind": SAMPLE_KIND,
        "production_collection_allowed": False,
        "task_success": None,
        "physics_validated": False,
        "capture_provenance": capture.provenance,
        "state_semantics": STATE_SEMANTICS,
        "action_semantics": ACTION_SEMANTICS,
        "capture_time_basis": "replay-relative timestamps; not physics-step time",
        "adapter_timeline": {
            "timing_label": "synthetic adapter timeline only; not measured physics",
            "synthetic_timeline_index": "i*8",
            "synthetic_timeline_indices": [frame.index * 8 for frame in capture.frames],
            "physics_hz": FPS * 8,
            "control_hz": FPS,
        },
    })
    for frame in capture.frames:
        timeline_index = frame.index * 8
        writer.add_frame(FrameSample(
            state=np.asarray(frame.state, dtype=np.float32),
            action=np.asarray(frame.action, dtype=np.float32),
            cameras={camera_keys[camera]: _read_rgb(frame.images[camera], np) for camera in CAMERAS},
            task=task,
            frame_index=frame.index,
            physics_step=timeline_index,
            timestamp=frame.timestamp,
            camera_physics_steps=(timeline_index,) * len(CAMERAS),
        ))
    if writer.save_episode(parallel_encoding=False) != 0 or writer.num_episodes != 1:
        raise RuntimeError("official writer did not create exactly one episode")
    writer.finalize()
    dataset = load_lerobot_dataset(root=output)
    if len(dataset) != len(capture.frames) or dataset.num_episodes != 1:
        raise RuntimeError("official loader did not read back the single expected episode")
    batch_size = min(2, len(capture.frames))
    batch = read_training_batch(dataset, batch_size=batch_size)
    vector_shape = (batch_size, len(JOINT_NAMES))
    for key in ("observation.state", "action"):
        if tuple(batch[key].shape) != vector_shape:
            raise RuntimeError(f"training batch {key} has unexpected shape {tuple(batch[key].shape)}")
    image_shape = (batch_size, IMAGE_SHAPE[2], IMAGE_SHAPE[0], IMAGE_SHAPE[1])
    for key in camera_keys.values():
        if tuple(batch[key].shape) != image_shape:
            raise RuntimeError(f"training batch {key} has unexpected shape {tuple(batch[key].shape)}")
    summary = summarize_readback(dataset)
    if summary.num_frames != len(capture.frames) or summary.num_episodes != 1:
        raise RuntimeError("official readback summary does not match the exported episode")
    return {
        "sdk": {"package_version": sdk.package_version, "codebase_version": sdk.codebase_version,
                "module_file": sdk.module_file},
        "dataset": {"root": str(output), "num_frames": summary.num_frames,
                    "num_episodes": summary.num_episodes, "feature_keys": list(summary.feature_keys),
                    "stats_keys": list(summary.stats_keys), "video_keys": list(summary.video_keys),
                    "training_batch_shapes": {key: list(batch[key].shape)
                                               for key in ("observation.state", "action", *camera_keys.values())}},
        "claims": {"sample_kind": SAMPLE_KIND, "task_success": None,
                   "production_collection_allowed": False, "physics_validated": False,
                   "timing_label": "synthetic adapter timeline only; not measured physics"},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = export_capture(args.capture_dir, args.output_root)
    serialized = json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)


if __name__ == "__main__":
    main()
