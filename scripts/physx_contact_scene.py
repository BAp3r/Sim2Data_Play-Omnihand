"""Session-only official Thor/cardbox scene composition for PhysX commissioning."""
from __future__ import annotations

import hashlib
from pathlib import Path


def verify_asset(path, expected):
    actual = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    if actual.lower() != expected.lower():
        raise ValueError(f"source identity mismatch: {Path(path).name}")
    return actual


def place_robot_base(stage, robot_parent_path, base_cfg):
    from pxr import Gf, UsdGeom, UsdPhysics
    from scipy.spatial.transform import Rotation
    import numpy as np

    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", base_cfg["base_rpy_rad"]).as_matrix()
    transform[:3, 3] = base_cfg["base_xyz_m"]
    parent = stage.GetPrimAtPath(robot_parent_path)
    if not parent:
        raise ValueError("robot namespace missing")
    node = UsdGeom.Xformable(parent)
    node.ClearXformOpOrder()
    node.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
    # A fixed root's world-side anchor is not transformed by its prim namespace.
    anchors = []
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(robot_parent_path + "/") and prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                p = joint.GetLocalPos0Attr().Get() or Gf.Vec3f(0)
                q = joint.GetLocalRot0Attr().Get() or Gf.Quatf(1)
                before = list(p)
                qbase = Gf.Matrix4d(transform.T.tolist()).ExtractRotationQuat()
                joint.GetLocalPos0Attr().Set(Gf.Vec3f(*(transform[:3,:3] @ np.array(p) + transform[:3,3])))
                joint.GetLocalRot0Attr().Set(Gf.Quatf(qbase) * q)
                anchors.append({"joint": str(prim.GetPath()), "before_world_anchor": before,
                                "after_world_anchor": list(joint.GetLocalPos0Attr().Get())})
    return {"T_world_base": transform.tolist(), "world_fixed_anchors": anchors,
            "initial_placement_only": True, "synthetic": True}


def build_scene(stage, *, table_path, cardbox_path, profile, plan,
                robot_parent_path, side, manifest):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
    from scipy.spatial.transform import Rotation
    import numpy as np

    if stage.GetEditTarget().GetLayer() != stage.GetSessionLayer():
        raise ValueError("scene overrides must use session layer")
    table_record = next(item for item in manifest["commissioning_model_packages"]
                        if item["id"] == "isaac_5_1_thorlabs_table")
    dependency_path = Path(table_path).parent / "Props" / "instaceable_meshes.usd"
    if not dependency_path.is_file():
        raise FileNotFoundError(f"Thor instance dependency missing: {dependency_path}")
    hashes = {"thor": verify_asset(table_path, table_record["sha256"]),
              "thor_dependency": verify_asset(dependency_path, table_record["dependency_sha256"]),
              "cardbox": verify_asset(cardbox_path, manifest["assets"]["card_box"]["sha256"])}
    base = place_robot_base(stage, robot_parent_path, profile["robots"][side])
    table = UsdGeom.Xform.Define(stage, "/World/RuntimeThor")
    transform = np.eye(4)
    transform[:3,:3] = Rotation.from_euler("xyz", profile["table"]["T_world_asset"]["rpy_rad"]).as_matrix()
    transform[:3,3] = profile["table"]["T_world_asset"]["xyz_m"]
    table.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
    # Keep the caller's mapped-drive spelling: resolving it to UNC breaks
    # relative USD/MDL dependencies in the audited Windows Kit environment.
    stage.DefinePrim("/World/RuntimeThor/Asset").GetReferences().AddReference(Path(table_path).absolute().as_posix())
    ground_path = "/World/FullFlatGround"
    ground = UsdGeom.Mesh.Define(stage, ground_path)
    z = float(profile["table"]["ground_z_m"])
    ground.CreatePointsAttr([(-2,-2,z), (2,-2,z), (2,2,z), (-2,2,z)])
    ground.CreateFaceVertexCountsAttr([4]); ground.CreateFaceVertexIndicesAttr([0,1,2,3])
    ground.CreateSubdivisionSchemeAttr("none")
    ground.CreateDisplayColorAttr([(0.12,0.14,0.16)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    material = UsdShade.Material.Define(stage, "/World/ContactMaterial")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(plan["friction"]))
    api.CreateDynamicFrictionAttr(float(plan["friction"]))
    api.CreateRestitutionAttr(0)
    box_path = "/World/CardBox"
    box = UsdGeom.Xform.Define(stage, box_path)
    placement = box.AddTranslateOp()
    centered = UsdGeom.Xform.Define(stage, box_path + "/Centered")
    offset = centered.AddTranslateOp()
    scaled = UsdGeom.Xform.Define(stage, box_path + "/Centered/Scaled")
    scaled.AddScaleOp().Set(Gf.Vec3f(.12))
    stage.DefinePrim(box_path + "/Centered/Scaled/Asset").GetReferences().AddReference(Path(cardbox_path).absolute().as_posix())
    stage.Load()
    bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(box.GetPrim()).ComputeAlignedRange()
    dimensions = np.array(bounds.GetSize())
    expected = np.array(manifest["assets"]["card_box"]["usd"]["world_bounds_m"]) * .12
    if not np.allclose(dimensions, expected, atol=1e-5):
        raise ValueError(f"scaled cardbox bounds mismatch: {dimensions}")
    offset.Set(-bounds.GetMidpoint())
    center = np.array(plan["box_center"], dtype=float)
    support_z = float(profile["table"]["collision_top_z_m"])
    expected_z = support_z + dimensions[2]/2
    if abs(center[2] - expected_z) > .002:
        raise ValueError("plan box height differs from Thor collision support")
    center[2] = expected_z + .001  # Explicit 1 mm gravity settling clearance.
    placement.Set(Gf.Vec3d(*center))
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr(float(plan["mass_kg"]))
    UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                                                       materialPurpose="physics")
    # Synthetic bin retains a bottom and four colliding walls, rim at visual top.
    bin_cfg = profile["bin"]
    sx, sy, sz = bin_cfg["inner_size_xyz_m"]
    thickness = bin_cfg["wall_thickness_m"]
    x, y, _ = bin_cfg["bottom_center_xyz_m"]
    bottom_z = bin_cfg["top_z_m"] - sz - thickness/2
    cubes = [("Bottom", [x,y,bottom_z], [sx+2*thickness,sy+2*thickness,thickness])]
    for name, dx, dy, dims in (("Left",-(sx+thickness)/2,0,[thickness,sy+2*thickness,sz]),
                                ("Right",(sx+thickness)/2,0,[thickness,sy+2*thickness,sz]),
                                ("Near",0,-(sy+thickness)/2,[sx,thickness,sz]),
                                ("Far",0,(sy+thickness)/2,[sx,thickness,sz])):
        cubes.append((name,[x+dx,y+dy,bin_cfg["top_z_m"]-sz/2],dims))
    for name, pos, dims in cubes:
        cube = UsdGeom.Cube.Define(stage, "/World/Bin/"+name)
        cube.CreateSizeAttr(1); cube.AddTranslateOp().Set(Gf.Vec3d(*pos)); cube.AddScaleOp().Set(Gf.Vec3f(*dims))
        cube.CreateDisplayColorAttr([(.25,.30,.35)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    colliders = [str(p.GetPath()) for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
                 if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False]
    thor_colliders = [p for p in colliders if p.startswith("/World/RuntimeThor/")]
    card_colliders = [p for p in colliders if p.startswith(box_path+"/")]
    if not thor_colliders or not card_colliders:
        raise ValueError(f"resolved Thor/cardbox collision geometry missing: thor={len(thor_colliders)} card={len(card_colliders)} all={len(colliders)}")
    return {"kind": "thor_cardbox", "box_prim_path": box_path,
            "support_filter_path": "/World/RuntimeThor/.*", "ground_prim_path": ground_path,
            "box_center_world_m": center.tolist(), "box_dimensions_m": dimensions.tolist(),
            "uniform_scale": .12, "mass_kg": float(plan["mass_kg"]), "source_hashes": hashes, "base_placement": base,
            "thor_colliders": thor_colliders, "box_colliders": card_colliders,
            "table_visual_top_z_m": profile["table"]["visual_top_z_m"],
            "table_collision_top_z_m": support_z, "settle_clearance_m": .001,
            "synthetic": True, "physics_validated": False, "source_modified": False}
