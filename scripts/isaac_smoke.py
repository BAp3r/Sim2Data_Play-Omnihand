"""Bounded synthetic table/drop/RGB smoke; never a robot or production collector.

Run only after checking GPU resources and the selected asset dependency closure.
The referenced vendor USD is read-only. All overrides live in a new memory stage.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset", type=Path, required=True)
    parser.add_argument("--asset-role", choices=("synthetic_fixture", "selected_card_box"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--graphics-api", choices=("default", "vulkan", "d3d12"), default="default",
                        help="Per-process Kit graphics backend diagnostic; no driver changes")
    args = parser.parse_args()
    asset = args.asset.resolve(strict=True)
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=False)
    result = {"scope": "synthetic_runtime_smoke_not_robot_acceptance",
              "asset": str(asset), "asset_role": args.asset_role,
              "passed": False, "phase": "starting_kit", "production_collection_allowed": False,
              "requested_graphics_api": args.graphics_api}
    (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    app = None
    try:
        from isaacsim import SimulationApp
        # SimulationApp forwards sys.argv and detects --portable-root here.
        # Kit's default portable cache otherwise lives inside the shared install.
        sys.argv = [sys.argv[0], "--portable-root", str(args.out / "kit")]
        if args.graphics_api != "default":
            sys.argv.append("--" + args.graphics_api)
            # Isaac's .kit explicitly sets app.vulkan=true; the shorthand alone
            # does not override that setting in the installed Windows 5.1 app.
            sys.argv.append("--/app/vulkan=" + ("false" if args.graphics_api == "d3d12" else "true"))
        app = SimulationApp({"headless": True, "width": 320, "height": 240,
                             "renderer": "RaytracedLighting", "anti_aliasing": 0,
                             "multi_gpu": False, "active_gpu": 0, "max_gpu_count": 1,
                             "limit_cpu_threads": 4,
                             "extra_args": ["--/app/extensions/registryEnabled=false",
                                            "--/app/extensions/syncRegistryOnStartup=false",
                                            f"--/log/file={args.out / 'kit.log'}"]})
        result["phase"] = "building_scene"
        import carb
        import numpy as np
        import omni.usd
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.prims import SingleRigidPrim
        from isaaclab.sim import SimulationCfg, SimulationContext

        source_stage = Usd.Stage.Open(str(asset))
        if source_stage is None:
            raise RuntimeError("Selected asset cannot be opened")
        result["asset_meters_per_unit"] = UsdGeom.GetStageMetersPerUnit(source_stage)
        result["asset_up_axis"] = str(UsdGeom.GetStageUpAxis(source_stage))
        if result["asset_meters_per_unit"] != 1.0 or result["asset_up_axis"] != "Z":
            raise RuntimeError("Smoke requires reviewed metre/Z-up asset; no implicit unit conversion")
        # Keep SimulationApp's standalone lifecycle flag. Overriding it skips
        # Isaac core's synchronous physics-context initialization.
        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        carb.settings.get_settings().set_bool("/isaaclab/render/active_viewport", True)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        table = UsdGeom.Cube.Define(stage, "/World/Table")
        table.CreateSizeAttr(1.0)
        table.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.025))
        table.AddScaleOp().Set(Gf.Vec3f(1.2, 0.8, 0.05))
        table.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())
        prim = stage.DefinePrim("/World/CardBox", "Xform")
        # Do not override the referenced default prim's concrete type (a Cube
        # fixture would otherwise become an empty Xform with invalid bounds).
        stage.DefinePrim("/World/CardBox/Asset").GetReferences().AddReference(str(asset))
        rigid = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        collisions = [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)
                      and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False]
        if len(rigid) != 1 or not collisions:
            raise RuntimeError("Selected asset must author one rigid body and enabled collision")
        bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(prim).ComputeAlignedRange()
        size = np.asarray(bounds.GetSize())
        if not np.isfinite(size).all() or not (size > 0).all() or max(size) > 0.5:
            raise RuntimeError(f"Unreviewed asset dimensions: {size}")
        # Initial reset placement only: do not overwrite any poses while simulating.
        wrapper = UsdGeom.Xformable(prim)
        wrapper.ClearXformOpOrder()
        wrapper.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.25 - bounds.GetMin()[2]))
        UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900.0)
        sim = SimulationContext(SimulationCfg(dt=1 / 240, device="cpu", use_fabric=False))
        body = SingleRigidPrim(str(rigid[0].GetPath()), name="smoke_card_box")
        eye = Gf.Vec3d(0.7, -0.8, 0.7)
        matrix = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(0, 0, 0.08), Gf.Vec3d(0, 0, 1)).GetInverse()
        q = matrix.ExtractRotationQuat()
        camera = Camera("/World/SmokeCamera", resolution=(320, 240))
        camera.set_world_pose(np.array(eye), np.array([q.GetReal(), *q.GetImaginary()]), camera_axes="usd")
        camera.set_focal_length(18.0)
        camera.set_horizontal_aperture(24.0)
        camera.set_vertical_aperture(18.0)
        camera.set_clipping_range(0.02, 10.0)
        sim.reset()
        body.initialize()
        camera.initialize()
        initial = body.get_world_pose()[0].tolist()
        trace = []
        for step in range(480):
            sim.step(render=step % 8 == 0)
            if step % 8 == 0:
                trace.append({"step": step + 1, "position": body.get_world_pose()[0].tolist(),
                              "linear_velocity": body.get_linear_velocity().tolist()})
        for _ in range(12):
            sim.render()
        rgb = camera.get_rgba()[..., :3]
        final = body.get_world_pose()[0].tolist()
        velocity = body.get_linear_velocity().tolist()
        Image.fromarray(rgb.astype(np.uint8)).save(args.out / "rgb.png")
        result.update({"simulation_context": "isaaclab.sim.SimulationContext", "physics_device": "cpu",
                       "rendering": "RTX headless RGB", "steps": 480, "dt": 1 / 240,
                       "asset_dimensions_m": size.tolist(), "initial_position": initial,
                       "final_position": final, "final_linear_velocity": velocity,
                       "rgb_shape": list(rgb.shape), "rgb_std": float(rgb.std()), "trace": trace,
                       "camera_K": camera.get_intrinsics_matrix().tolist(),
                       "camera_T_world_usd": np.array(matrix).T.tolist()})
        # A fall followed by stable support, plus nonempty rendered pixels.
        result["passed"] = bool(initial[2] - final[2] > 0.1 and -0.03 < final[2] < 0.3
                                and np.linalg.norm(velocity) < 0.05
                                and rgb.shape == (240, 320, 3) and rgb.std() > 2)
        if not result["passed"]:
            raise RuntimeError("Drop/support/RGB smoke checks failed")
        result["phase"] = "completed"
    except Exception:
        result["error"] = traceback.format_exc()
        raise
    finally:
        (args.out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
