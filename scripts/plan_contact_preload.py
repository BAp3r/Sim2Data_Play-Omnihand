"""Map synthetic opposed fingertip force requests to active joint torques.

No runtime or physical acceptance is implied by this source-URDF calculation.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from plan_contact_trajectory import RobotModel, pose, positions_for, sha256


def create_preload(urdf, profile, plan_path, force):
    if not math.isfinite(force) or not 0 < force <= 5:
        raise ValueError("requested opposed force must be in (0, 5] N")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if not plan.get("execution_allowed") or plan["urdf_sha256"] != sha256(urdf) or plan["profile_sha256"] != sha256(profile):
        raise ValueError("passed source-bound plan required")
    side = plan["side"]
    active = json.loads(profile.read_text(encoding="utf-8"))["gripper_commissioning"][side]["active_joints"]
    model = RobotModel(urdf, side)
    closed = np.array(plan["hand_close"])
    base = pose(plan["planner"]["profile_base_xyz_m"], plan["planner"]["profile_base_rpy_rad"])
    def frames(values):
        targets = [dict(a, open_rad=float(v), close_rad=float(v)) for a,v in zip(active,values)]
        return model.fk(positions_for(model.joints, targets, model.arm_names, plan["grasp"], 1))
    fk = frames(closed)
    torque = np.zeros(len(active))
    records = []
    for role, share in (("thumb",1.), ("index",-.5), ("middle",-.5)):
        prefix = "l" if side == "left" else "R"
        link = f"{side}_hand__{prefix}_{role}_dip_link"
        shape = next(s for s in model.meshes if s["link"] == link)
        transform = base @ fk[link] @ shape["origin"]
        world = shape["points"] @ transform[:3,:3].T + transform[:3,3]
        face = plan["finger_contact_residuals"][role]
        k, coordinate = face["face_axis"], face["face_coordinate_m"]
        ids = np.flatnonzero(np.all(np.abs(world-np.array(plan["box_center"])) <= np.array(plan["box_size"])/2+.003, axis=1))
        if not len(ids):
            raise ValueError("source fingertip has no box face patch")
        index = ids[np.argmin(abs(world[ids,k]-coordinate))]
        point = shape["points"][index]
        jacobian = np.zeros((3,len(active)))
        for j, item in enumerate(active):
            values = closed.copy()
            limits = plan["planner"]["active_hand_limits_rad"][item["name"]]
            h = 1e-5 if values[j]+1e-5 < limits["upper_rad"] else -1e-5
            values[j] += h
            if values[j] < limits["lower_rad"]:
                raise ValueError("cannot differentiate fixed active joint")
            matrix = base @ frames(values)[link] @ shape["origin"]
            jacobian[:,j] = (matrix[:3,:3] @ point + matrix[:3,3] - world[index])/h
        requested = np.array(plan["poses_world"]["closing_axis_unit"])*force*share
        contribution = jacobian.T @ requested
        torque += contribution
        records.append(dict(link=link, point_world_m=world[index].tolist(),
                            requested_force_N=requested.tolist(), active_effort_Nm=contribution.tolist()))
    if not np.isfinite(torque).all() or np.max(np.abs(torque)) > plan["hand_max_force"]:
        raise ValueError("preload exceeds synthetic hand effort limit")
    return dict(synthetic=True, physics_validated=False, plan_sha256=sha256(plan_path),
                urdf_sha256=sha256(urdf), profile_sha256=sha256(profile),
                method="source collision point finite-difference FK Jacobian transpose; mimic derivatives included",
                requested_force_is_not_measured=True, joint_names=[a["name"] for a in active],
                effort_Nm=torque.tolist(), contacts=records)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "profile", "plan", "out"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--force", type=float, default=3.)
    args = parser.parse_args()
    if args.out.exists():
        raise SystemExit("fresh output required")
    result = create_preload(args.urdf, args.profile, args.plan, args.force)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result))
