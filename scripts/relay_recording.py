"""Synchronous pre-action recording for a synthetic PhysX relay episode."""
from __future__ import annotations

import json
import math
from pathlib import Path

CAMERAS = ("overhead", "wrist_left", "wrist_right")
ACTION_NAMES = tuple(name for side in ("left", "right")
                     for name in [*(f"{side}.arm_joint{i}.target_rad" for i in range(1, 7)),
                                  f"{side}.gripper.amount"])
REQUIRED_GATES = ("left_contact_lift", "relay_supported_stable", "left_released_retreated",
                  "right_contact_lift", "bin_footprint", "final_released_stable",
                  "no_illegal_collision", "no_object_teleport")


class PhysicalEpisodeRecorder:
    """Persist every attempted episode; only accepted episodes may be exported.

    Call ``record`` after rendering the current state, immediately before
    applying the action held for the following control interval. All camera
    samples must refer to that same physics step. Passive states remain in the
    diagnostic trace and are never inserted into observation.state.
    """

    def __init__(self, root, *, state_names, physics_hz, fps, provenance):
        self.root = Path(root)
        self.root.mkdir(exist_ok=False, parents=True)
        self.state_names = tuple(state_names)
        if len(self.state_names) != 32 or len(set(self.state_names)) != 32:
            raise ValueError("expected 12 arm and 20 independent active hand state names")
        if type(physics_hz) is not int or type(fps) is not int or physics_hz % fps:
            raise ValueError("physics frequency must be an integer multiple of recording frequency")
        self.physics_hz, self.fps = physics_hz, fps
        self.stride = physics_hz // fps
        self.provenance = provenance
        self.frames = []
        self.stream = (self.root/"frames.jsonl").open("w", encoding="utf-8")
        self.aux = (self.root/"auxiliary.jsonl").open("w", encoding="utf-8")
        self.finished = False
        for camera in CAMERAS:
            (self.root/"images"/camera).mkdir(parents=True)

    def record(self, *, physics_step, state, action, active_commands, cameras,
               camera_physics_steps, auxiliary):
        import numpy as np
        from PIL import Image
        index = len(self.frames)
        if physics_step != index*self.stride:
            raise ValueError("nonuniform pre-action recording clock")
        if set(cameras) != set(CAMERAS) or camera_physics_steps != {c: physics_step for c in CAMERAS}:
            raise ValueError("all three training cameras must match the observation physics step")
        if len(state) != 32 or len(action) != 14 or set(active_commands) != set(self.state_names):
            raise ValueError("state/action/active command schema mismatch")
        if not all(math.isfinite(float(v)) for v in [*state, *action, *active_commands.values()]):
            raise ValueError("nonfinite robot observation/action")
        if not all(0 <= action[i] <= 1 for i in (6, 13)):
            raise ValueError("gripper actions must be scalar amounts")
        images = {}
        for name, rgb in cameras.items():
            rgb = np.asarray(rgb)
            if rgb.shape != (240, 320, 3) or rgb.dtype != np.uint8:
                raise ValueError("training RGB must be 240x320 uint8")
            relative = f"images/{name}/{index:06d}.png"
            Image.fromarray(rgb).save(self.root/relative)
            images[name] = relative
        frame = dict(index=index, timestamp=index/self.fps, physics_step=physics_step,
                     state=list(map(float,state)), action=list(map(float,action)), images=images,
                     camera_physics_steps=camera_physics_steps)
        self.stream.write(json.dumps(frame)+"\n"); self.stream.flush()
        self.aux.write(json.dumps(dict(index=index, physics_step=physics_step,
                                      active_commands=active_commands, simulation=auxiliary))+"\n")
        self.aux.flush()
        self.frames.append(frame)

    def finish(self, *, gates, failure_reason=None):
        if self.finished:
            return
        self.finished = True
        self.stream.close(); self.aux.close()
        passed = bool(self.frames) and all(gates.get(k) is True for k in REQUIRED_GATES) and failure_reason is None
        result = dict(sample_kind="synthetic_physx_dual_arm_relay", synthetic=True,
            task_success=passed, gates=gates, failure_reason=failure_reason,
            frame_count=len(self.frames), fps=self.fps, physics_hz=self.physics_hz,
            state_names=self.state_names, action_names=ACTION_NAMES, cameras=CAMERAS,
            state_semantics="measured independent active joint positions before action",
            action_semantics="arm position targets and gripper amounts held over the following control interval",
            timestamp_semantics="pre-action observation; physics_step / physics_hz",
            image_shape=[240,320,3], provenance=self.provenance,
            videos={}, video_errors=[])
        # Failure videos are diagnostics, never success demonstrations.
        try:
            import cv2
            for camera in CAMERAS:
                if not self.frames:
                    continue
                path=self.root/f"{camera}.mp4"
                writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*"mp4v"),self.fps,(320,240))
                if not writer.isOpened():
                    raise RuntimeError(f"video writer unavailable: {camera}")
                try:
                    for frame in self.frames:
                        writer.write(cv2.imread(str(self.root/frame["images"][camera])))
                finally:
                    writer.release()
                result["videos"][camera]=path.name
        except Exception as exc:
            result["video_errors"].append(repr(exc))
        (self.root/"capture.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        return result
