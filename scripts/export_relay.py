"""Export an actually successful PhysX relay using the existing official SDK adapter."""
import argparse
import json
import importlib.metadata
from pathlib import Path

from relay_recording import CAMERAS, REQUIRED_GATES


def export_relay(capture_root, output_root):
    root=Path(capture_root); meta=json.loads((root/"capture.json").read_text(encoding="utf-8"))
    if meta.get("sample_kind") != "synthetic_physx_dual_arm_relay" or meta.get("task_success") is not True:
        raise ValueError("only a physically successful continuous relay may enter the demonstration dataset")
    if any(meta.get("gates",{}).get(k) is not True for k in REQUIRED_GATES):
        raise ValueError("missing relay success gate")
    result=json.loads((root.parent/"result.json").read_text(encoding="utf-8")) if (root.parent/"result.json").is_file() else {}
    if result.get("passed") is not True or result.get("render_diagnostic",{}).get("passed") is not True:
        raise ValueError("relay RGB visibility acceptance is missing")
    if result.get("overhead_edge_clipped_frames")!=0:
        raise ValueError("overhead edge clipping acceptance is missing or failed")
    for label in ("left_robot","right_robot","thor"):
        if result.get("visibility_summary",{}).get("overhead",{}).get(label,{}).get("visible_frames")!=meta["frame_count"]:
            raise ValueError(f"overhead {label} visibility was not checked for every frame")
    if len(meta["state_names"])!=32 or len(meta["action_names"])!=14 or tuple(meta["cameras"])!=CAMERAS or meta.get("video_errors"):
        raise ValueError("relay state/action/camera schema or source video mismatch")
    import numpy as np
    from PIL import Image
    from sim2data.core import Timing
    from sim2data.export import (ExportSchema, FrameSample, LeRobotV3Writer,
        load_lerobot_dataset, summarize_readback, read_training_batch)
    from lerobot.configs.video import RGBEncoderConfig
    frames=[json.loads(line) for line in (root/"frames.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(frames) != meta["frame_count"] or not frames:
        raise ValueError("capture frame count mismatch")
    fps=meta["fps"]; hz=meta["physics_hz"]
    auxiliary=[json.loads(line) for line in (root/"auxiliary.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(auxiliary)!=len(frames):raise ValueError("auxiliary frame count mismatch")
    render_origin=None;render_clock_error=0.;camera_skew=0.
    for frame,aux in zip(frames,auxiliary,strict=True):
        if aux["index"]!=frame["index"] or aux["physics_step"]!=frame["physics_step"]:
            raise ValueError("auxiliary/frame observation alignment mismatch")
        times=[aux["simulation"]["camera_visibility"][c]["rendering_time"] for c in CAMERAS]
        if any(t is None or not np.isfinite(t) for t in times):raise ValueError("camera render time unavailable")
        if render_origin is None:render_origin=times[0]
        camera_skew=max(camera_skew,max(times)-min(times))
        render_clock_error=max(render_clock_error,max(abs(t-render_origin-frame["timestamp"]) for t in times))
    if camera_skew>1e-4 or render_clock_error>1e-4:raise ValueError("render/physics observation clock mismatch")
    schema=ExportSchema(name="synthetic_physx_relay_active32_action14",
        state_names=tuple(meta["state_names"]),action_names=tuple(meta["action_names"]),
        camera_shapes={f"observation.images.{c}":(240,320,3) for c in CAMERAS})
    writer=LeRobotV3Writer(root=output_root,schema=schema,fps=fps,
        timing=Timing(physics_hz=hz,control_hz=fps),repo_id="sim2data/synthetic_relay",
        camera_encoder=RGBEncoderConfig(vcodec="h264",preset="ultrafast",g=1,crf=18))
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
    errors=dict(state_max_abs=0.,action_max_abs=0.,timestamp_max_abs_s=0.,
                video_mean_abs_rgb255_max={c:0. for c in CAMERAS})
    for i,frame in enumerate(frames):
        item=dataset[i]
        if tuple(item["observation.state"].shape)!=(32,) or tuple(item["action"].shape)!=(14,):
            raise ValueError("official loader state/action shape mismatch")
        if not np.allclose(item["observation.state"].numpy(),frame["state"],atol=1e-6): raise ValueError("state readback mismatch")
        if not np.allclose(item["action"].numpy(),frame["action"],atol=1e-6): raise ValueError("action readback mismatch")
        if abs(float(item["timestamp"])-frame["timestamp"])>1e-4 or int(item["episode_index"])!=0:
            raise ValueError("timestamp/episode boundary mismatch")
        if int(item["frame_index"])!=i:raise ValueError("official loader frame boundary mismatch")
        errors["state_max_abs"]=max(errors["state_max_abs"],float(np.max(np.abs(item["observation.state"].numpy()-frame["state"]))))
        errors["action_max_abs"]=max(errors["action_max_abs"],float(np.max(np.abs(item["action"].numpy()-frame["action"]))))
        errors["timestamp_max_abs_s"]=max(errors["timestamp_max_abs_s"],abs(float(item["timestamp"])-frame["timestamp"]))
        for camera in CAMERAS:
            if tuple(item[f"observation.images.{camera}"].shape)!=(3,240,320): raise ValueError("video readback shape mismatch")
            decoded=item[f"observation.images.{camera}"].numpy().transpose(1,2,0)*255
            source=np.asarray(Image.open(root/frame["images"][camera]).convert("RGB"),dtype=np.float32)
            if not np.isfinite(decoded).all():raise ValueError("nonfinite official video readback")
            errors["video_mean_abs_rgb255_max"][camera]=max(errors["video_mean_abs_rgb255_max"][camera],float(np.mean(np.abs(decoded-source))))
    summary=summarize_readback(dataset); batch=read_training_batch(dataset,batch_size=min(2,len(dataset)))
    result=dict(official_loader_passed=True,frames=len(dataset),episodes=dataset.num_episodes,
        video_keys=list(summary.video_keys),state_dim=32,action_dim=14,all_frames_checked=True,
        batch_keys=list(batch),batch_shapes={k:list(v.shape) for k,v in batch.items() if hasattr(v,"shape")},
        errors=errors,sdk={k:importlib.metadata.version(k) for k in ("lerobot","torch","av","numpy")},
        source_render_clock=dict(camera_skew_max_s=camera_skew,physics_clock_error_max_s=render_clock_error,
                                 initial_time_s=render_origin,checked_frames=len(frames)),
        rgb_visibility_checked=True,synthetic=True)
    (Path(output_root)/"relay_readback.json").write_text(json.dumps(result,indent=2))
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--capture",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    args=p.parse_args();print(json.dumps(export_relay(args.capture,args.out),indent=2))
