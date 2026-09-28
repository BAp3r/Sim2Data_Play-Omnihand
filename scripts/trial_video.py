"""Encode available trial RGB, preserving failure status separately from success."""
import json
from pathlib import Path


def encode_trial(root, report):
    import cv2
    root=Path(root); images=report.get("images",[]); videos={}
    for camera in sorted({f["camera"] for f in images}):
        frames=[f for f in images if f["camera"]==camera]
        if not frames: continue
        stride=frames[1]["step"]-frames[0]["step"] if len(frames)>1 else max(1,round(1/(30*report["dt"])))
        if any(b["step"]-a["step"]!=stride for a,b in zip(frames,frames[1:])):
            raise ValueError("cannot label discontinuous PNG sequence with uniform video fps")
        fps=1/(stride*report["dt"])
        first=cv2.imread(str(root/frames[0]["file"]))
        if first is None: raise ValueError("trial RGB missing")
        path=root/f"{camera}.{'success' if report.get('passed') else 'failed'}.mp4"
        writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*"mp4v"),fps,(first.shape[1],first.shape[0]))
        if not writer.isOpened(): raise RuntimeError("trial video encoder unavailable")
        try:
            for frame in frames:
                rgb=cv2.imread(str(root/frame["file"]))
                if rgb is None: raise ValueError("trial RGB missing")
                writer.write(rgb)
        finally: writer.release()
        videos[camera]=dict(path=path.name,fps=fps,frames=len(frames),task_success=bool(report.get("passed")),
                            scope=report.get("scope"),first_physics_step=frames[0]["step"])
    (root/"video_status.json").write_text(json.dumps(videos,indent=2))
    return videos


if __name__=="__main__":
    import sys
    root=Path(sys.argv[1]);print(json.dumps(encode_trial(root,json.loads((root/"result.json").read_text())),indent=2))
