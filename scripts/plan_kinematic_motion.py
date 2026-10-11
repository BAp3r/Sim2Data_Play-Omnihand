"""Plan a synthetic arm motion using Pinocchio and the reviewed real URDF.

CPU kinematics only. No fabricated arm model, dynamics, contact or RGB fallback.
The result is input to the separate Isaac RTX kinematic playback capture.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    import pinocchio as pin

    profile = json.loads(args.profile.read_text())
    digest = hashlib.sha256(args.urdf.read_bytes()).hexdigest()
    if digest != profile["arm_model"]["urdf_sha256"]:
        raise ValueError("URDF identity does not match approved synthetic profile")
    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    if model.nq != 6 or model.nv != 6:
        raise ValueError("selected arm must expose the six reviewed revolute joints")
    names = [str(n) for n in model.names][1:]
    if names != [f"joint{i}" for i in range(1, 7)]:
        raise ValueError(f"unexpected model joint ordering: {names}")
    q0 = np.array([0., -.7, .7, 0., 0., 0.])
    fid = model.getFrameId("link6")
    pin.framesForwardKinematics(model, data, q0)
    start = data.oMf[fid].copy()
    target = start.copy()
    target.translation = target.translation + np.array([0., 0., .025])
    q = q0.copy()
    for iteration in range(300):
        pin.framesForwardKinematics(model, data, q)
        error = target.translation - data.oMf[fid].translation
        if np.linalg.norm(error) < 1e-7:
            break
        jacobian = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-5*np.eye(3), error)
        q = pin.integrate(model, q, np.clip(delta, -.05, .05))
        q = np.clip(q, model.lowerPositionLimit + 1e-5, model.upperPositionLimit - 1e-5)
    pin.framesForwardKinematics(model, data, q)
    residual = np.linalg.norm(data.oMf[fid].translation-target.translation)
    if residual > 1e-5:
        raise RuntimeError(f"Pinocchio IK failed: residual {residual}")
    samples = []
    fps, count = 30, 121
    for index in range(count):
        u = index / 60 if index <= 60 else (120-index)/60
        blend = 10*u**3 - 15*u**4 + 6*u**5
        left = pin.interpolate(model, q0, q, float(blend))
        right = q0.copy()
        joint_vector = np.concatenate([left, right])
        transforms = {}
        for side, arm_q in [("left", left), ("right", right)]:
            pin.forwardKinematics(model, data, arm_q)
            pin.updateFramePlacements(model, data)
            transforms[side] = {
                f"link{i}": data.oMf[model.getFrameId(f"link{i}")].homogeneous.tolist()
                for i in range(1, 7)
            }
        samples.append({"index": index, "timestamp": index/fps,
                        "q": joint_vector.tolist(), "T_base_links": transforms})
    positions = np.asarray([s["q"] for s in samples])
    vmax = float(np.abs(np.diff(positions, axis=0)*fps).max())
    if vmax > .25:
        raise ValueError("trajectory exceeds synthetic commissioning velocity cap 0.25 rad/s")
    result = {
        "sample_kind": "synthetic_kinematic_planned_motion", "production_collection_allowed": False,
        "task_success": None, "physics_validated": False, "collision_validated": False,
        "planner": {"library": "Pinocchio", "version": pin.__version__, "module": pin.__file__,
                    "method": "damped least-squares position IK + Pinocchio interpolate + quintic timing",
                    "orientation_constrained": False,
                    "ik_iterations": iteration+1, "ik_position_residual_m": float(residual)},
        "urdf_sha256": digest, "profile_sha256": hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        "fps": fps, "joint_names": [f"{s}.{n}" for s in ("left", "right") for n in names],
        "synthetic_velocity_cap_rad_s": .25, "actual_peak_finite_difference_velocity_rad_s": vmax,
        "source_effort_velocity_zero": True,
        "T_base_flange_start": start.homogeneous.tolist(), "T_base_flange_goal": target.homogeneous.tolist(),
        "frames": samples,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)


if __name__ == "__main__":
    main()
