"""Re-screen a fixed grasp template at an explicit scene-truth box position."""
import argparse
import copy
import json
from pathlib import Path


def plan_at_box(*, urdf, profile, manifest, side, search, center, out):
    import numpy as np
    import plan_contact_trajectory as planner
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    original=json.loads(Path(search).read_text(encoding="utf-8"));data=copy.deepcopy(original)
    old=np.asarray(original["box_center_m"]);new=np.asarray(center,dtype=float)
    if new.shape!=(3,) or not np.isfinite(new).all(): raise ValueError("finite box center required")
    delta=new-old
    for c in data["candidates"]:
        matrix=np.asarray(c["palm_pose_world"]["matrix_4x4"]);matrix[:3,3]+=delta
        c["palm_pose_world"]["matrix_4x4"]=matrix.tolist()
        c["palm_pose_world"]["position_m"]=matrix[:3,3].tolist()
    data["box_center_m"]=new.tolist()
    data["template_translation_m"]=delta.tolist();data["execution_allowed"]=False
    shifted=out/"search_translated.json";shifted.write_text(json.dumps(data,indent=2))
    previous=planner.BOX_CENTER.copy()
    try:
        planner.BOX_CENTER=new
        result=planner.create_plan(Path(urdf),Path(profile),Path(manifest),side,shifted)
    finally: planner.BOX_CENTER=previous
    result["template_translation_m"]=delta.tolist()
    (out/"plan.json").write_text(json.dumps(result,indent=2,allow_nan=False))
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    for k in ("urdf","profile","manifest","search","out"):parser.add_argument("--"+k,type=Path,required=True)
    parser.add_argument("--side",choices=("left","right"),required=True)
    parser.add_argument("--center",type=float,nargs=3,required=True)
    args=vars(parser.parse_args());result=plan_at_box(**args)
    print(json.dumps({"passed":result["passed"],"geometry_gate":result["planner"]["geometry_gate"]["failure_reasons"]}))
    raise SystemExit(0 if result["passed"] else 3)

