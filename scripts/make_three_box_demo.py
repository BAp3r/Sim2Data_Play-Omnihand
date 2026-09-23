"""Render a clearly labelled three-box kinematic planning demo.

This is a visual commissioning artifact. It does not run PhysX, move an
articulation, or claim contact/grasp success. Each box has a different pose and
size and follows an approach/close/lift/place storyboard on the revised flat
ground layout.
"""
from __future__ import annotations

import argparse, json, math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


BOXES = [
    {"id": "box_1", "xy": (-0.28, 0.10), "size": (0.08, 0.06), "color": (222, 150, 54)},
    {"id": "box_2", "xy": (-0.12, 0.19), "size": (0.06, 0.09), "color": (70, 155, 220)},
    {"id": "box_3", "xy": (0.10, 0.08), "size": (0.10, 0.05), "color": (108, 190, 105)},
]


def lerp(a, b, u): return a + (b - a) * u


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--plan", type=Path, required=True)
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    frames = []
    plan = {"kind": "synthetic_three_box_kinematic_demo", "production_collection_allowed": False,
            "physics_validated": False, "contact_validated": False, "ground": "full_flat_plane",
            "bin_top_z_m": 0.0, "boxes": BOXES, "stages": []}
    W, H = 960, 640
    def px(x, y): return (int(480 + x * 900), int(470 - y * 900))
    def draw_frame(index, active, phase, box_states, hand):
        im = Image.new("RGB", (W, H), (238, 241, 244)); d = ImageDraw.Draw(im)
        d.rectangle((45, 45, 915, 545), fill=(180, 185, 192), outline=(70, 70, 75), width=3)
        d.text((60, 58), "SIM2DATA synthetic planned demo | NO PHYSX CONTACT", fill=(125, 15, 15))
        # tabletop/flat-ground reference and outer bin with rim at tabletop height
        d.line((45, 470, 915, 470), fill=(35, 35, 35), width=4)
        d.text((60, 500), "full flat ground / tabletop reference z=0", fill=(40, 40, 40))
        bx, by = px(.52, .05); bw, bh = 180, 120
        d.rectangle((bx-bw//2, by-bh//2, bx+bw//2, by+bh//2), outline=(30, 80, 150), width=8)
        d.text((bx-70, by-12), "outer bin", fill=(25, 65, 130)); d.text((bx-70, by+12), "rim z=0", fill=(25,65,130))
        for i, state in enumerate(box_states):
            x, y, z, sx, sy, color = state
            cx, cy = px(x, y); ww, hh = int(sx*900), int(sy*900)
            lift = int(z * 450)
            d.rectangle((cx-ww//2, cy-hh//2-lift, cx+ww//2, cy+hh//2-lift), fill=color, outline=(30,30,30), width=3)
            d.text((cx-ww//2, cy-hh//2-lift-18), BOXES[i]["id"], fill=(20,20,20))
        hx, hy = px(*hand); d.ellipse((hx-18,hy-18,hx+18,hy+18), fill=(215,215,220), outline=(30,30,30), width=3)
        d.line((hx,hy,hx,hy+80), fill=(50,50,50), width=5)
        d.text((60, 585), f"box {active+1}/3  stage: {phase}  frame {index}", fill=(25,25,25))
        return im
    index = 0
    for i, box in enumerate(BOXES):
        x, y = box["xy"]; size = box["size"]
        target = (.52, .05)
        phases = [("approach", (x, y, 0.0), (x, y)), ("close", (x, y, 0.0), (x, y)),
                  ("lift", (x, y, 0.055), (x, y)), ("transfer", (target[0], target[1], 0.055), target),
                  ("place", (target[0], target[1], 0.0), target)]
        start = (x-.16, y+.18, 0.12)
        for name, end, hand_target in phases:
            count = 18
            for k in range(count):
                u = k/(count-1); hand = tuple(lerp(a,b,u) for a,b in zip(start[:2], hand_target))
                states=[]
                for j, other in enumerate(BOXES):
                    ox,oy=other["xy"]; oz=0.0
                    if j < i: ox,oy=target; oz=0.0
                    if j == i: ox,oy,oz=end[0],end[1],lerp(start[2],end[2],u) if name in ("approach","lift","transfer") else end[2]
                    states.append((ox,oy,oz,other["size"][0],other["size"][1],other["color"]))
                frames.append(draw_frame(index,i,name,states,hand)); index+=1
            start=(target[0],target[1],0.0)
        plan["stages"].append({"box":box["id"],"position_m":box["xy"],"size_m":size,
                               "sequence":["approach","close","lift","transfer","place"]})
    for i, im in enumerate(frames): im.save(args.out / f"frame_{i:05d}.png")
    (args.out / "plan.json").write_text(json.dumps(plan, indent=2), encoding="utf-8")
    print(json.dumps({"frames":len(frames),"fps":args.fps,"plan":str(args.out/'plan.json')}))


if __name__ == "__main__": main()
