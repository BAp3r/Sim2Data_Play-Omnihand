"""Synthetic CPU checks against the installed official LeRobot v3 SDK.

These tests exercise format creation and loader behavior only.  They do not
establish robot dimensions, calibration, physics, camera latency, or task
success.  The tests use virtual channels and tiny RGB arrays.
"""

from __future__ import annotations

import tempfile
from types import SimpleNamespace
import inspect
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import numpy as np
except ImportError:  # pragma: no cover - dependency-free foundation checkout
    np = None

from sim2data.core import Timing
from sim2data.export import (
    ExportSchema,
    FrameSample,
    LeRobotV3Writer,
    SDKUnavailableError,
    load_lerobot_dataset,
    read_training_batch,
    require_lerobot_v3,
    summarize_readback,
)


def _sdk_available() -> bool:
    if np is None:
        return False
    try:
        require_lerobot_v3()
    except SDKUnavailableError:
        return False
    return True


@unittest.skipUnless(_sdk_available(), "official LeRobot v3 export environment is unavailable")
class LeRobotV3SmokeTests(unittest.TestCase):
    def _frame(self, schema: ExportSchema, episode: int, index: int) -> FrameSample:
        image = {
            key: np.full(shape, (episode * 31 + index + camera) % 255, dtype=np.uint8)
            for camera, (key, shape) in enumerate(schema.camera_shapes.items())
        }
        return FrameSample(
            state=np.asarray([episode + index / 10, index], dtype=np.float32),
            action=np.asarray([index, episode], dtype=np.float32),
            cameras=image,
            task="synthetic format roundtrip",
            frame_index=index,
            physics_step=index * 8,
            timestamp=index / 30,
            camera_physics_steps=(index * 8,) * len(schema.camera_keys),
        )

    def _writer(self, root: Path, schema: ExportSchema) -> LeRobotV3Writer:
        # The encoder object is created by the installed official SDK.  h264
        # is selected explicitly so this test reports missing codec support
        # instead of silently choosing a machine-dependent hardware encoder.
        try:
            from lerobot.configs.video import VideoEncoderConfig
        except ModuleNotFoundError:
            # LeRobot 0.4.x has the required v3 writer methods but predates the
            # VideoEncoderConfig facade; the adapter maps its default codec.
            VideoEncoderConfig = None

        return LeRobotV3Writer(
            root=root,
            schema=schema,
            fps=30,
            timing=Timing(physics_hz=240, control_hz=30),
            video_files_size_in_mb=0.0001,
            data_files_size_in_mb=0.0001,
            camera_encoder=(
                VideoEncoderConfig(vcodec="h264", preset="ultrafast", g=1)
                if VideoEncoderConfig is not None
                else SimpleNamespace(vcodec="h264")
            ),
        )

    def test_multi_episode_boundaries_windows_stats_batch_and_reopen(self) -> None:
        schema = ExportSchema.smoke_test(height=8, width=8)
        with tempfile.TemporaryDirectory(prefix="sim2data-lerobot-smoke-") as temp:
            root = Path(temp) / "dataset"
            writer = self._writer(root, schema)
            for episode in range(2):
                writer.begin_episode(
                    "synthetic format roundtrip",
                    privileged={"synthetic_privileged": {"episode": episode}},
                )
                for index in range(3):
                    writer.add_frame(self._frame(schema, episode, index))
                self.assertEqual(writer.save_episode(), episode)

            self.assertTrue((root / "sidecar" / "episode-000000.json").is_file())
            self.assertNotIn("synthetic_privileged", schema.features)

            # A second owner cannot open a live or already-created root.
            with self.assertRaises(RuntimeError):
                LeRobotV3Writer(root=root, schema=schema, fps=30)

            # A rejected/aborted stage must not become a third episode.
            writer.begin_episode("synthetic format roundtrip")
            writer.add_frame(self._frame(schema, 2, 0))
            writer.abort_episode("synthetic staging rejection")
            self.assertEqual(writer.num_episodes, 2)
            writer.finalize()

            info = require_lerobot_v3()
            dataset = load_lerobot_dataset(
                root=root,
                delta_timestamps={
                    "observation.state": [-1 / 30, 0.0],
                    "action": [0.0, 1 / 30],
                    schema.camera_keys[0]: [0.0, 1 / 30],
                },
            )
            self.assertEqual(len(dataset), 6)
            self.assertEqual(dataset.num_episodes, 2)
            sample = dataset[2]
            self.assertEqual(tuple(sample["observation.state"].shape), (2, 2))
            self.assertEqual(tuple(sample["action"].shape), (2, 2))
            self.assertEqual(tuple(sample[schema.camera_keys[0]].shape), (2, 3, 8, 8))
            self.assertTrue(
                {"observation.state", "action"} | set(schema.camera_keys)
                <= set(dataset.meta.stats)
            )
            stats = dataset.meta.stats
            np.testing.assert_allclose(
                np.asarray(stats["observation.state"]["min"]), [0.0, 0.0], rtol=1e-6, atol=1e-6
            )
            np.testing.assert_allclose(
                np.asarray(stats["observation.state"]["max"]), [1.2, 2.0], rtol=1e-6, atol=1e-6
            )
            np.testing.assert_allclose(
                np.asarray(stats["observation.state"]["mean"]), [0.6, 1.0], rtol=1e-6, atol=1e-6
            )
            np.testing.assert_allclose(
                np.asarray(stats["action"]["min"]), [0.0, 0.0], rtol=1e-6, atol=1e-6
            )
            np.testing.assert_allclose(
                np.asarray(stats["action"]["max"]), [2.0, 1.0], rtol=1e-6, atol=1e-6
            )
            np.testing.assert_allclose(
                np.asarray(stats["action"]["mean"]), [1.0, 0.5], rtol=1e-6, atol=1e-6
            )

            # Tiny limits force each episode to its own video file.  Verify
            # offsets for all cameras at the episode boundary.
            supports_video_size_limit = "video_files_size_in_mb" in inspect.signature(
                type(writer.dataset).create
            ).parameters
            for key in schema.camera_keys:
                file_indices = [episode[f"videos/{key}/file_index"] for episode in dataset.meta.episodes]
                if supports_video_size_limit:
                    self.assertEqual(file_indices, [0, 1])
                else:
                    # LeRobot 0.4.x has the required v3 writer surface but
                    # does not expose file-size controls.  Its official
                    # writer is still checked for valid offsets and the
                    # boundary check runs in the 0.5.x smoke environment.
                    self.assertEqual(file_indices, [0, 0])
                from_timestamps = [episode[f"videos/{key}/from_timestamp"] for episode in dataset.meta.episodes]
                self.assertEqual(from_timestamps, [0.0, 0.0] if supports_video_size_limit else [0.0, 3 / 30])

            batch = read_training_batch(dataset, batch_size=2)
            self.assertEqual(tuple(batch["observation.state"].shape), (2, 2, 2))
            self.assertEqual(tuple(batch[schema.camera_keys[0]].shape), (2, 2, 3, 8, 8))
            summary = summarize_readback(dataset)
            self.assertEqual(summary.package_version, info.package_version)
            self.assertEqual(summary.codebase_version, "v3.0")
            self.assertEqual(summary.num_frames, 6)
            self.assertEqual(summary.num_episodes, 2)
            self.assertEqual(set(summary.video_keys), set(schema.camera_keys))

            reopened = load_lerobot_dataset(root=root)
            self.assertEqual(len(reopened), 6)
            self.assertEqual(tuple(reopened[0]["action"].shape), (2,))
            # Decode both sides of the episode boundary.  Constant synthetic
            # colors make accidental stream/episode concatenation visible even
            # with a lossy H.264 codec.
            for global_index, episode, index in ((2, 0, 2), (3, 1, 0)):
                decoded = reopened[global_index]
                np.testing.assert_allclose(
                    np.asarray(decoded["observation.state"]), [episode + index / 10, index], atol=1e-5
                )
                for camera, key in enumerate(schema.camera_keys):
                    expected_pixel = episode * 31 + index + camera
                    decoded_mean = float(np.asarray(decoded[key], dtype=np.float32).mean()) * 255.0
                    self.assertLess(
                        abs(decoded_mean - expected_pixel), 8.0,
                        msg=f"decoded {key} crossed episode boundary",
                    )

            # Window padding must stay inside an episode.  At the first frame
            # of episode 1, the previous state is padded rather than frame 2
            # from episode 0; the analogous future action is padded at the
            # last frame of episode 0.
            first_episode_one = dataset[3]
            state_pad = np.asarray(first_episode_one["observation.state_is_pad"])
            self.assertEqual(state_pad.tolist(), [True, False])
            self.assertFalse(
                np.allclose(
                    np.asarray(first_episode_one["observation.state"])[0],
                    np.asarray(reopened[2]["observation.state"]),
                )
            )
            last_episode_zero = dataset[2]
            action_pad = np.asarray(last_episode_zero["action_is_pad"])
            self.assertEqual(action_pad.tolist(), [False, True])
            self.assertFalse(
                np.allclose(
                    np.asarray(last_episode_zero["action"])[1],
                    np.asarray(reopened[3]["action"]),
                )
            )

    def test_rejects_empty_nan_missing_and_delayed_frames(self) -> None:
        schema = ExportSchema.smoke_test(height=4, width=4)
        with tempfile.TemporaryDirectory(prefix="sim2data-lerobot-reject-") as temp:
            writer = self._writer(Path(temp) / "dataset", schema)
            writer.begin_episode("reject checks")
            with self.assertRaises(ValueError):
                writer.save_episode()
            writer.abort_episode("empty stage")

            writer.begin_episode("reject checks")
            good = self._frame(schema, 0, 0)
            with self.assertRaises(ValueError):
                writer.add_frame(
                    FrameSample(
                        state=np.asarray([np.nan, 0], dtype=np.float32),
                        action=good.action,
                        cameras=good.cameras,
                        task=good.task,
                    )
                )
            with self.assertRaises(ValueError):
                writer.add_frame(
                    FrameSample(
                        state=good.state,
                        action=good.action,
                        cameras={key: value for key, value in list(good.cameras.items())[:-1]},
                        task=good.task,
                    )
                )
            with self.assertRaises(ValueError):
                writer.add_frame(
                    FrameSample(
                        state=good.state,
                        action=good.action,
                        cameras=good.cameras,
                        task=good.task,
                        camera_physics_steps=(1,) * len(schema.camera_keys),
                    )
                )
            writer.abort_episode("invalid frame checks")

            writer.begin_episode("reject checks")
            good = self._frame(schema, 0, 0)
            writer.add_frame(
                FrameSample(
                    state=good.state,
                    action=good.action,
                    cameras=good.cameras,
                    task="reject checks",
                )
            )
            with self.assertRaises(ValueError):
                writer.save_episode(sidecar={"invalid": float("nan")})
            self.assertEqual(writer.num_episodes, 0)
            writer.abort_episode("invalid sidecar")

            # Reusing simulator buffers after add_frame must not rewrite the
            # staged copy or bypass the finite-value checks.
            writer.begin_episode("buffer ownership")
            state_buffer = np.asarray([0.25, 0.75], dtype=np.float32)
            action_buffer = np.asarray([0.5, 0.25], dtype=np.float32)
            image_buffers = {
                key: np.full(shape, 17, dtype=np.uint8)
                for key, shape in schema.camera_shapes.items()
            }
            writer.add_frame(
                FrameSample(
                    state=state_buffer,
                    action=action_buffer,
                    cameras=image_buffers,
                    task="buffer ownership",
                )
            )
            state_buffer[:] = np.nan
            action_buffer[:] = -99
            for image in image_buffers.values():
                image[:] = 0
            writer.save_episode()
            writer.finalize()
            reopened = load_lerobot_dataset(root=Path(temp) / "dataset")
            np.testing.assert_allclose(reopened[0]["observation.state"], [0.25, 0.75], atol=1e-5)
            np.testing.assert_allclose(reopened[0]["action"], [0.5, 0.25], atol=1e-5)
            for key in schema.camera_keys:
                decoded_mean = float(np.asarray(reopened[0][key]).mean()) * 255.0
                self.assertLess(abs(decoded_mean - 17), 8.0)

            failed_sidecar_writer = self._writer(Path(temp) / "sidecar-failure", schema)
            failed_sidecar_writer.begin_episode("sidecar failure")
            failed_sidecar_writer.add_frame(
                FrameSample(
                    state=np.asarray([0.0, 0.0], dtype=np.float32),
                    action=np.asarray([0.0, 0.0], dtype=np.float32),
                    cameras={
                        key: np.zeros(shape, dtype=np.uint8)
                        for key, shape in schema.camera_shapes.items()
                    },
                    task="sidecar failure",
                )
            )
            with patch.object(failed_sidecar_writer, "_write_sidecar", side_effect=OSError("synthetic disk failure")):
                with self.assertRaises(OSError):
                    failed_sidecar_writer.save_episode(sidecar={"valid": True})
            with self.assertRaises(RuntimeError):
                failed_sidecar_writer.finalize()
            failed_sidecar_writer._release_root()


if __name__ == "__main__":
    unittest.main()
