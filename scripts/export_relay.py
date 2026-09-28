"""Export an actually successful PhysX relay using the existing official SDK adapter."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from relay_recording import CAMERAS, REQUIRED_GATES


def export_relay(capture_root, output_root):
    root=Path(capture_root); meta=json.loads((root/"capture.json").read_text(encoding="utf-8"))
    if meta.get("sample_kind") != "synthetic_physx_dual_arm_relay" or meta.get("task_success") is not True:
        raise ValueError("only a physically successful continuous relay may enter the demonstration dataset")
    if any(meta.get("gates",{}).get(k) is not True for k in REQUIRED_GATES):
        raise ValueError("missing relay success gate")
    import numpy as np
    from PIL import Image
    from sim2data.core import Timing
    from sim2data.export import (ExportSchema, FrameSample, LeRobotV3Writer,
        load_lerobot_dataset, summarize_readback, read_training_batch)
    frames=[json.loads(line) for line in (root/"frames.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(frames) != meta["frame_count"] or not frames:
        raise ValueError("capture frame count mismatch")
    fps=meta["fps"]; hz=meta["physics_hz"]
    schema=ExportSchema(name="synthetic_physx_relay_active32_action14",
        state_names=tuple(meta["state_names"]),action_names=tuple(meta["action_names"]),
        camera_shapes={f"observation.images.{c}":(240,320,3) for c in CAMERAS})
    writer=LeRobotV3Writer(root=output_root,schema=schema,fps=fps,
        timing=Timing(physics_hz=hz,control_hz=fps),repo_id="sim2data/synthetic_relay",
        camera_encoder=SimpleNamespace(vcodec="h264"))
    writer.begin_episode("left table pick, supported handoff, right bin placement",
        privileged={"synthetic":True,"physical_gates":meta["gates"],"provenance":meta["provenance"]})
    for i,frame in enumerate(frames):
        if frame["index"] != i or frame["physics_step"] != i*(hz//fps) or abs(frame["timestamp"]-i/fps)>1e-9:
            raise ValueError("capture timing discontinuity")
        images={}
        for camera in CAMERAS:
            path=(root/frame["images"][camera]).resolve()
            if not path.is_relative_to(root.resolve()): raise ValueError("image escapes capture root")
            images[f"observation.images.{camera}"]=np.asarray(Image.open(path).convert("RGB"))
        writer.add_frame(FrameSample(state=np.array(frame["state"],dtype=np.float32),
            action=np.array(frame["action"],dtype=np.float32),cameras=images,
            task="left table pick, supported handoff, right bin placement",frame_index=i,
            physics_step=frame["physics_step"],timestamp=frame["timestamp"],
            camera_physics_steps=tuple(frame["camera_physics_steps"][c] for c in CAMERAS)))
    writer.save_episode(parallel_encoding=False); writer.finalize()
    dataset=load_lerobot_dataset(root=output_root)
    if len(dataset)!=len(frames) or dataset.num_episodes!=1: raise ValueError("official loader episode/frame mismatch")
    for i,frame in enumerate(frames):
        item=dataset[i]
        if not np.allclose(item["observation.state"].numpy(),frame["state"],atol=1e-6): raise ValueError("state readback mismatch")
        if not np.allclose(item["action"].numpy(),frame["action"],atol=1e-6): raise ValueError("action readback mismatch")
        if abs(float(item["timestamp"])-frame["timestamp"])>1e-4 or int(item["episode_index"])!=0:
            raise ValueError("timestamp/episode boundary mismatch")
        for camera in CAMERAS:
            if tuple(item[f"observation.images.{camera}"].shape)!=(3,240,320): raise ValueError("video readback shape mismatch")
    summary=summarize_readback(dataset); batch=read_training_batch(dataset,batch_size=min(2,len(dataset)))
    result=dict(official_loader_passed=True,frames=len(dataset),episodes=dataset.num_episodes,
        video_keys=list(summary.video_keys),state_dim=32,action_dim=14,all_frames_checked=True,
        batch_keys=list(batch),synthetic=True)
    (Path(output_root)/"relay_readback.json").write_text(json.dumps(result,indent=2))
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--capture",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    args=p.parse_args();print(json.dumps(export_relay(args.capture,args.out),indent=2))

