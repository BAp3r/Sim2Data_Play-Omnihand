"""Run a tiny official LeRobot v3 format smoke test.

The generated data is synthetic and must not be used as robot data.  The
script requires an explicit output root so each invocation owns a separate
dataset directory.  No Hub upload or network access is performed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace

import numpy as np

from sim2data.core import Timing
from sim2data.export import (
    ExportSchema,
    FrameSample,
    LeRobotV3Writer,
    load_lerobot_dataset,
    read_training_batch,
    require_lerobot_v3,
    summarize_readback,
)


def _frame(schema: ExportSchema, episode: int, index: int, fps: int) -> FrameSample:
    image = {
        key: np.full(shape, (episode * 31 + index + camera) % 255, dtype=np.uint8)
        for camera, (key, shape) in enumerate(schema.camera_shapes.items())
    }
    return FrameSample(
        state=np.asarray([episode + index / 10, index], dtype=np.float32),
        action=np.asarray([index, episode], dtype=np.float32),
        cameras=image,
        task="synthetic format smoke",
        frame_index=index,
        physics_step=index * 8,
        timestamp=index / fps,
        camera_physics_steps=(index * 8,) * len(schema.camera_keys),
    )


def run(root: Path, *, episodes: int, frames: int, height: int, width: int, fps: int) -> dict:
    if episodes < 1 or frames < 1:
        raise ValueError("episodes and frames must be positive")
    schema = ExportSchema.smoke_test(height=height, width=width)
    try:
        from lerobot.configs import video as video_config
        encoder_config = getattr(video_config, "RGBEncoderConfig", None) or getattr(
            video_config, "VideoEncoderConfig", None
        )
    except ImportError:
        # LeRobot 0.4.x has the v3 writer methods but no encoder config class;
        # the adapter maps the SDK's default codec for that API.
        encoder_config = None

    writer = LeRobotV3Writer(
        root=root,
        schema=schema,
        fps=fps,
        timing=Timing(physics_hz=fps * 8, control_hz=fps),
        video_files_size_in_mb=0.0001,
        data_files_size_in_mb=0.0001,
        camera_encoder=(
            encoder_config(vcodec="h264", preset="ultrafast", g=1)
            if encoder_config is not None
            else SimpleNamespace(vcodec="h264")
        ),
    )
    for episode in range(episodes):
        writer.begin_episode("synthetic format smoke", privileged={"synthetic": True, "episode": episode})
        for index in range(frames):
            writer.add_frame(_frame(schema, episode, index, fps))
        writer.save_episode(parallel_encoding=False)
    writer.finalize()

    window = {
        "observation.state": [-1 / fps, 0.0],
        "action": [0.0, 1 / fps],
        schema.camera_keys[0]: [0.0, 1 / fps],
    }
    dataset = load_lerobot_dataset(root=root, delta_timestamps=window)
    batch = read_training_batch(dataset, batch_size=min(2, len(dataset)))
    summary = summarize_readback(dataset)
    return {
        "sdk": {
            "package_version": require_lerobot_v3().package_version,
            "codebase_version": require_lerobot_v3().codebase_version,
            "module_file": require_lerobot_v3().module_file,
        },
        "root": str(root),
        "synthetic_only": True,
        "frames": len(dataset),
        "episodes": dataset.num_episodes,
        "feature_keys": list(summary.feature_keys),
        "stats_keys": list(summary.stats_keys),
        "video_keys": list(summary.video_keys),
        "batch_shapes": {key: list(value.shape) for key, value in batch.items() if hasattr(value, "shape")},
        "reopened_frames": len(load_lerobot_dataset(root=root)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, help="new private output directory (default: temporary directory)")
    parser.add_argument("--report", type=Path, help="optional JSON report path")
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--height", type=int, default=8)
    parser.add_argument("--width", type=int, default=8)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    if args.root is None:
        with tempfile.TemporaryDirectory(prefix="sim2data-lerobot-smoke-") as temporary:
            result = run(Path(temporary) / "dataset", episodes=args.episodes, frames=args.frames, height=args.height, width=args.width, fps=args.fps)
    else:
        result = run(args.root, episodes=args.episodes, frames=args.frames, height=args.height, width=args.width, fps=args.fps)
    rendered = json.dumps(result, sort_keys=True, indent=2)
    print(rendered)
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        # Evidence reports belong to one writer/run.  Refuse an existing path
        # so a rerun cannot silently replace the prior SDK result.
        with args.report.open("x", encoding="utf-8") as report:
            report.write(rendered + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
