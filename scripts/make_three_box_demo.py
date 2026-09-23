"""Create a deterministic, domain-randomized three-box visual plan.

The plan is consumed by the Isaac RTX recorder.  It deliberately does not
claim PhysX contact or a successful grasp: the current reviewed assembly has no
robot articulation or drives.  Box dimensions are sampled in metres on the
left half of the table and the official card-box reference is scaled from its
audited 0.70 x 0.50 x 0.50 m source extent to the sampled dimensions.
"""
from __future__ import annotations

import argparse, json, math, random
from pathlib import Path


SOURCE_EXTENT_M = (0.70, 0.50, 0.50)
BASE_SOURCE_SCALE = 0.12
DEFAULT_TARGET_XY = (0.02, 0.13)


def lerp(a, b, u): return a + (b - a) * u


def sample_boxes(seed: int) -> list[dict]:
    """Sample small, separated boxes on the left side of the tabletop."""
    rng = random.Random(seed)
    boxes = []
    for index in range(3):
        for _ in range(100):
            size = (rng.uniform(0.045, 0.072), rng.uniform(0.034, 0.058),
                    rng.uniform(0.035, 0.058))
            candidate = (rng.uniform(-0.38, -0.16), rng.uniform(0.055, 0.245))
            clearance = max(size[0], size[1]) * 0.7 + 0.018
            if all(math.dist(candidate, other["xy"]) > clearance +
                   max(size[0], size[1]) * 0.7 for other in boxes):
                break
        else:
            raise RuntimeError("domain randomization could not separate boxes")
        boxes.append({"id": f"box_{index + 1}", "xy": candidate, "size": size,
                      "yaw_rad": rng.uniform(-0.35, 0.35)})
    return boxes


def source_scale_for(size: tuple[float, float, float]) -> tuple[float, float, float]:
    # The source USD already authors a 0.70 x 0.50 x 0.50 m world extent.
    # The per-instance scale therefore maps that extent directly to the
    # sampled target size; the old 0.12 wrapper is only the audit reference.
    return tuple(size[i] / SOURCE_EXTENT_M[i] for i in range(3))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--plan", type=Path, required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seed", type=int, default=20260923)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    boxes = sample_boxes(args.seed)
    plan = {"sample_kind": "synthetic_three_box_domain_randomized_kinematic_demo",
            "production_collection_allowed": False, "physics_validated": False,
            "contact_validated": False, "ground": "full_flat_plane", "bin_top_z_m": 0.0,
            "domain_randomization": {"seed": args.seed, "xy_region_m": [[-0.38, -0.16], [0.055, 0.245]],
                                      "size_range_m": [[0.045, 0.072], [0.034, 0.058], [0.035, 0.058]],
                                      "source_extent_m": SOURCE_EXTENT_M,
                                      "reference_wrapper_scale": BASE_SOURCE_SCALE,
                                      "instance_scale_mode": "source_extent_direct"},
            "planner": {"library": "Pink", "status": "blocked_on_windows_dependency",
                        "fallback": "deterministic_reviewed_fk_playback",
                        "pink_package": "pin-pink==3.1.0"},
            "boxes": [{**box, "source_scale_xyz": source_scale_for(box["size"])} for box in boxes],
            "stages": [], "frames": []}
    index = 0
    for i, box in enumerate(boxes):
        x, y = box["xy"]; size = box["size"]
        target = DEFAULT_TARGET_XY
        phases = [("approach", (x, y, 0.0), (x, y)), ("close", (x, y, 0.0), (x, y)),
                  ("lift", (x, y, 0.055), (x, y)), ("transfer", (target[0], target[1], 0.055), target),
                  ("place", (target[0], target[1], 0.0), target)]
        start = (x-.16, y+.18, 0.12)
        for name, end, hand_target in phases:
            count = 18
            for k in range(count):
                u = k/(count-1); hand = tuple(lerp(a,b,u) for a,b in zip(start[:2], hand_target))
                states=[]
                for j, other in enumerate(boxes):
                    ox,oy=other["xy"]; oz=0.0
                    if j < i: ox,oy=target; oz=0.0
                    if j == i: ox,oy,oz=end[0],end[1],lerp(start[2],end[2],u) if name in ("approach","lift","transfer") else end[2]
                    states.append((ox,oy,oz,other["size"][0],other["size"][1],other["size"][2],other["yaw_rad"]))
                index += 1
                plan["frames"].append({"boxes":[{"id":other["id"],"x":st[0],"y":st[1],"z":st[2],
                    "scale_x":st[3]/SOURCE_EXTENT_M[0], "scale_y":st[4]/SOURCE_EXTENT_M[1],
                    "scale_z":st[5]/SOURCE_EXTENT_M[2], "yaw_rad":st[6]} for other,st in zip(boxes,states)],
                    "active": i, "phase": name})
            start=(target[0],target[1],0.0)
        plan["stages"].append({"box":box["id"],"position_m":box["xy"],"size_m":size,
                               "sequence":["approach","close","lift","transfer","place"]})
    args.plan.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(plan, indent=2)
    args.plan.write_text(serialized, encoding="utf-8")
    if args.plan.resolve() != (args.out / "plan.json").resolve():
        (args.out / "plan.json").write_text(serialized, encoding="utf-8")
    print(json.dumps({"frames":len(plan["frames"]), "fps":args.fps, "seed":args.seed,
                      "boxes":plan["boxes"], "plan":str(args.plan)}))


if __name__ == "__main__": main()
