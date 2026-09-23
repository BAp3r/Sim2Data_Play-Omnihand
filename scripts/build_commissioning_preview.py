"""CPU USD visual assembly preview. No physics, rendered RGB, or dataset claim.

Consumes already validated, self-contained commissioning URDFs. Requires pxr,
numpy and trimesh in an existing isolated interpreter; does not start Kit.
All outputs contain private derived geometry and must remain outside public Git.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zlib

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from sim2data.backends.isaaclab.preview_pose import resolve_preview_positions


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    matrix = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    return matrix


def origin(element):
    node = element.find("origin")
    if node is None:
        return np.eye(4)
    return transform([float(x) for x in node.get("xyz", "0 0 0").split()],
                     [float(x) for x in node.get("rpy", "0 0 0").split()])


def place(xform, matrix):
    # Gf matrices use row vectors; project/URDF transforms use column vectors.
    xform.AddTransformOp().Set(Gf.Matrix4d(matrix.T.tolist()))


def material(stage, prim, colour, roughness=.55, metallic=0.0):
    # Explicit PreviewSurface makes colours portable to Blender and Kit;
    # displayColor alone is not interpreted as a surface by every importer.
    key = "c_" + "_".join(str(round(float(c) * 65535)) for c in [*colour[:3], roughness, metallic])
    path = "/World/Looks/" + key
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour[:3]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    return mat


def corner_shading_normals(geometry, settings):
    """Angle-weighted, crease-limited visual normals without welding the mesh.

    Coincident corners are grouped only in temporary arrays. Authored source
    vertices, face indices and collision geometry are never changed.
    """
    tolerance = float(settings["weld_position_tolerance_m"])
    crease = float(settings["crease_angle_deg"])
    if not np.isfinite(tolerance) or tolerance <= 0 or not 0 < crease < 90:
        raise ValueError("Invalid visual normal tolerances")
    vertices = np.asarray(geometry.vertices)
    faces = np.asarray(geometry.faces)
    _, vertex_groups = np.unique(np.round(vertices / tolerance).astype(np.int64), axis=0, return_inverse=True)
    corner_groups = vertex_groups[faces.reshape(-1)]
    order = np.argsort(corner_groups, kind="stable")
    _, starts, degrees = np.unique(corner_groups[order], return_index=True, return_counts=True)
    source = np.repeat(np.asarray(geometry.face_normals), 3, axis=0)
    weights = np.asarray(geometry.face_angles).reshape(-1)
    normals = source.copy()
    threshold = np.cos(np.deg2rad(crease))
    for degree in np.unique(degrees):
        # Unusual nonmanifold stars retain their original corner normals.
        # Bound temporary pairwise arrays; ordinary source surface stars are small.
        if degree > 128:
            continue
        selected_starts = starts[degrees == degree]
        for offset in range(0, len(selected_starts), 4096):
            indices = order[selected_starts[offset:offset + 4096, None] + np.arange(degree)]
            incident = source[indices]
            compatible = np.einsum("nik,njk->nij", incident, incident) >= threshold
            weighted = compatible * weights[indices][:, None, :]
            smoothed = np.einsum("nij,njk->nik", weighted, incident)
            length = np.linalg.norm(smoothed, axis=2, keepdims=True)
            smoothed = np.divide(smoothed, length, out=incident.copy(), where=length > 1e-12)
            normals[indices] = smoothed
    return normals.reshape(-1, 3, 3)


def write_colour_ramp(path, first, second, width=256):
    """Small RGB PNG containing linear values; no image-library dependency."""
    ramp = np.linspace(first, second, width)
    pixels = np.uint8(np.round(np.clip(ramp, 0, 1) * 255))
    rows = b"".join(b"\x00" + pixels.tobytes() for _ in range(4))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, 4, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def continuous_panel_material(stage, prim, primary, accent, weights):
    """Portable UV texture field, continuous within and across smooth faces."""
    identity = json.dumps([primary, accent], sort_keys=True).encode()
    key = hashlib.sha256(identity).hexdigest()[:16]
    relative = Path("textures") / f"reference_panel_{key}.png"
    texture_path = Path(stage.GetRootLayer().realPath).parent / relative
    if not texture_path.exists():
        write_colour_ramp(texture_path, primary["color_linear_rgb"], accent["color_linear_rgb"])
    mat = UsdShade.Material.Define(stage, "/World/Looks/panel_" + key)
    surface = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("Surface"))
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(primary["roughness"])
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(primary["metallic"])
    reader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("UV"))
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("Colour"))
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(relative.as_posix()))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    uv = np.column_stack((np.asarray(weights).reshape(-1) * (255 / 256) + .5 / 256,
                          np.full(np.asarray(weights).size, .5))).astype(np.float32)
    UsdGeom.PrimvarsAPI(prim).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                         UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray.FromNumpy(uv))


def mesh(stage, path, geometry, colour, style=None):
    vertices = np.asarray(geometry.vertices, dtype=np.float32)
    faces = np.asarray(geometry.faces, dtype=np.int32)
    if not len(vertices) or not len(faces) or not np.isfinite(vertices).all():
        raise ValueError(f"Invalid mesh: {path}")
    prim = UsdGeom.Mesh.Define(stage, path)
    prim.CreatePointsAttr(vertices.tolist())
    prim.CreateFaceVertexCountsAttr([3] * len(faces))
    prim.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
    prim.CreateSubdivisionSchemeAttr("none")
    prim.CreateDisplayColorAttr([Gf.Vec3f(*colour[:3])])
    material(stage, prim.GetPrim(), colour)
    if style:
        primary, accent = style["primary"], style["accent"]
        material(stage, prim.GetPrim(), primary["color_linear_rgb"], primary["roughness"], primary["metallic"])
        prim.CreateDisplayColorAttr([Gf.Vec3f(*primary["color_linear_rgb"])])
        normals = np.asarray(style["corner_normals"], dtype=np.float32).reshape(-1, 3)
        prim.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals))
        prim.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if accent is not None:
            continuous_panel_material(stage, prim.GetPrim(), primary, accent, style["weights"])
    prim.CreateExtentAttr([Gf.Vec3f(*vertices.min(axis=0).tolist()),
                           Gf.Vec3f(*vertices.max(axis=0).tolist())])
    return len(vertices), len(faces)


def reference_style(name, geometry, appearance):
    """Approximate reference colours without changing any mesh geometry."""
    if appearance is None:
        return None
    palette, rules = appearance["palette"], appearance["selectors"]
    normals = corner_shading_normals(geometry, appearance["surface_shading"])
    accent, weights = None, None
    if "_arm__" in name:
        base = palette["charcoal"]
        if name.split("_arm__", 1)[1] in rules["arm_panel_links"]:
            accent = palette["ivory"]
            lower, upper = rules["arm_panel_normal_blend_range"]
            if not 0 <= lower < upper <= 1:
                raise ValueError("Invalid continuous panel blend range")
            coordinate = np.abs(normals[:, :, rules["arm_panel_normal_axis"]])
            weights = np.clip((coordinate - lower) / (upper - lower), 0, 1)
            weights = weights * weights * (3 - 2 * weights)
    elif "_hand__" in name:
        base = palette["ivory"]
        if name.lower().endswith("palm"):
            base = palette[rules["palm_material"]]
        elif "thumb_roll" in name:
            base = palette["alloy"]
    elif name.endswith("_mount_assembly"):
        base = palette["alloy"]
    elif name.endswith("_camera_housing"):
        base = palette["camera_alloy"]
    else:
        return None
    return {"primary": base, "accent": accent, "weights": weights, "corner_normals": normals}


def cube(stage, path, size, centre, colour, collision=False):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1)
    item.AddTranslateOp().Set(Gf.Vec3d(*centre))
    item.AddScaleOp().Set(Gf.Vec3f(*size))
    item.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    material(stage, item.GetPrim(), colour)
    if collision:
        UsdPhysics.CollisionAPI.Apply(item.GetPrim())
    return item


def flat_ground(stage, path, size_xy, z, colour=(0.12, 0.14, 0.16)):
    """Create a full, zero-thickness ground plane; never a giant cube."""
    mesh_prim = UsdGeom.Mesh.Define(stage, path)
    hx, hy = float(size_xy[0]) / 2, float(size_xy[1]) / 2
    mesh_prim.CreatePointsAttr([(-hx, -hy, z), (hx, -hy, z), (hx, hy, z), (-hx, hy, z)])
    mesh_prim.CreateFaceVertexCountsAttr([4])
    mesh_prim.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh_prim.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    material(stage, mesh_prim.GetPrim(), colour)
    UsdPhysics.CollisionAPI.Apply(mesh_prim.GetPrim())
    return mesh_prim


def load_geometry(node):
    item = node.find("mesh")
    if item is not None:
        filename = Path(item.attrib["filename"])
        if not filename.is_absolute() or not filename.is_file():
            raise ValueError("Composed URDF must resolve each mesh to an existing absolute path")
        geom = trimesh.load(str(filename), force="mesh", process=False)
        geom.apply_scale([float(x) for x in item.get("scale", "1 1 1").split()])
        return geom
    item = node.find("box")
    if item is not None:
        return trimesh.creation.box([float(x) for x in item.attrib["size"].split()])
    item = node.find("cylinder")
    if item is not None:
        return trimesh.creation.cylinder(float(item.attrib["radius"]), float(item.attrib["length"]))
    item = node.find("sphere")
    if item is not None:
        return trimesh.creation.icosphere(radius=float(item.attrib["radius"]))
    raise ValueError("Unsupported URDF visual geometry")


def add_robot(stage, path, urdf, base, appearance=None):
    root = ET.parse(urdf).getroot()
    links = {n.attrib["name"]: n for n in root.findall("link")}
    joints = {n.attrib["name"]: n for n in root.findall("joint")}
    incoming = {n.find("child").attrib["link"]: n for n in root.findall("joint")}
    roots = set(links) - set(incoming)
    if len(roots) != 1:
        raise ValueError("Expected one validated URDF root")
    container = UsdGeom.Xform.Define(stage, path)
    place(container, transform(base["base_xyz_m"], base["base_rpy_rad"]))
    paths, visiting = {}, set()
    positions = resolve_preview_positions(root, base.get("preview_joint_positions", {}))

    def add_link(name):
        if name in paths:
            return paths[name]
        if name in visiting:
            raise ValueError("Cyclic URDF")
        visiting.add(name)
        joint = incoming.get(name)
        parent = path if joint is None else add_link(joint.find("parent").attrib["link"])
        link_path = parent + "/" + name
        frame = UsdGeom.Xform.Define(stage, link_path)
        if joint is not None:
            q = positions[joint.attrib["name"]]
            motion = np.eye(4)
            axis_node = joint.find("axis")
            axis = np.array([float(x) for x in ("1 0 0" if axis_node is None else axis_node.get("xyz", "1 0 0")).split()])
            if joint.attrib["type"] in ("revolute", "continuous"):
                motion = trimesh.transformations.rotation_matrix(q, axis)
            elif joint.attrib["type"] == "prismatic":
                motion[:3, 3] = q * axis / np.linalg.norm(axis)
            elif joint.attrib["type"] != "fixed":
                raise ValueError("Preview supports only fixed/revolute/continuous/prismatic joints")
            place(frame, origin(joint) @ motion)
        # Limit-checked visualization pose, not commanded or calibrated state.
        for index, visual in enumerate(links[name].findall("visual")):
            visual_path = link_path + f"/visual_{index}"
            visual_frame = UsdGeom.Xform.Define(stage, visual_path)
            place(visual_frame, origin(visual))
            colour_node = visual.find("material/color")
            colour = [0.6, 0.65, 0.72] if colour_node is None else [
                float(x) for x in colour_node.attrib["rgba"].split()[:3]]
            geometry = load_geometry(visual.find("geometry"))
            mesh(stage, visual_path + "/mesh", geometry, colour, reference_style(name, geometry, appearance))
        visiting.remove(name)
        paths[name] = link_path
        return link_path

    for name in links:
        add_link(name)
    return {"links": len(links), "joints": len(incoming), "link_paths": paths,
            "preview_joint_positions": positions,
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest()}


def build(args):
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    appearance = None if args.appearance is None else json.loads(args.appearance.read_text(encoding="utf-8"))
    if appearance is not None and appearance.get("purpose") != "reference_appearance_only":
        raise ValueError("Appearance must be an explicit reference-only material design")
    if profile["purpose"] != "synthetic_commissioning_only" or profile["production_collection_enabled"] is not False:
        raise ValueError("Only explicit synthetic commissioning profiles accepted")
    args.out.mkdir(parents=True, exist_ok=False)
    result = {"scope": "static_visual_assembly_only", "passed": False,
              "physics_validated": False, "rgb_validated": False,
              "dataset_export_allowed": False, "production_collection_allowed": False}
    result_path = args.out / "preview_report.json"
    result_path.write_text(json.dumps(result, indent=2))
    stage = Usd.Stage.CreateNew(str(args.out / "assembly_preview.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetMetadata("comment", "SYNTHETIC STATIC PREVIEW: no physics, calibration or collection acceptance")
    table = profile["table"]
    if table.get("asset_id"):
        if args.table_usd is None:
            raise ValueError("Selected table asset requires explicit private --table-usd binding")
        table_source = args.table_usd.resolve(strict=True)
        source_stage = Usd.Stage.Open(str(table_source))
        if source_stage is None or not source_stage.GetDefaultPrim():
            raise ValueError("Table USD must open and provide a default prim")
        source_units = UsdGeom.GetStageMetersPerUnit(source_stage)
        source_axis = UsdGeom.GetStageUpAxis(source_stage)
        if source_units != table["source_meters_per_unit"] or source_axis != table["source_up_axis"]:
            raise ValueError("Table metadata differs from the audited binding")
        if not np.isfinite(source_units) or source_units <= 0:
            raise ValueError("Invalid table units")
        placement = table["T_world_asset"]
        container = UsdGeom.Xform.Define(stage, "/World/Table")
        # USD references do not convert layer units automatically. This changes
        # numeric coordinates to metres, never the physical table dimensions.
        unit_conversion = np.diag([source_units, source_units, source_units, 1.0])
        place(container, transform(placement["xyz_m"], placement["rpy_rad"]) @ unit_conversion)
        stage.DefinePrim("/World/Table/Asset").GetReferences().AddReference(str(table_source))
        bounds = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
            container.GetPrim()).ComputeAlignedRange()
        actual_size = np.asarray(bounds.GetSize())
        if not np.isfinite(actual_size).all() or not (actual_size > 0).all():
            raise ValueError("Table reference has invalid bounds")
        result["table"] = {"asset_id": table["asset_id"], "source": str(table_source),
                           "physical_dimensions_preserved": True, "source_meters_per_unit": source_units,
                           "source_up_axis": source_axis, "bounds_min_m": list(bounds.GetMin()),
                           "bounds_max_m": list(bounds.GetMax()), "physics_validated": False}
    else:
        if args.table_usd is not None:
            raise ValueError("Table binding supplied but no table asset selected in profile")
        cube(stage, "/World/Table", table["size_xyz_m"], table["center_xyz_m"], (0.22, 0.25, 0.28))
    # The floor is a continuous plane below the official Thor table. It is
    # separate from the tabletop and carries no guessed slab thickness.
    ground_z = float(table.get("ground_z_m", -0.7947))
    flat_ground(stage, "/World/FullFlatGround", (4.0, 4.0), ground_z)
    box = profile["box"]
    cube(stage, "/World/SyntheticBox", box["size_xyz_m"], box["center_xyz_m"], (0.6, 0.35, 0.14))
    relay = profile["relay_region"]
    cube(stage, "/World/RelayMarker", relay["size_xyz_m"], relay["center_xyz_m"], (0.15, 0.65, 0.25))
    bin_ = profile["bin"]
    x, y, z = bin_["bottom_center_xyz_m"]
    sx, sy, sz = bin_["inner_size_xyz_m"]
    t = bin_["wall_thickness_m"]
    cube(stage, "/World/Bin/Bottom", (sx+2*t, sy+2*t, t), (x,y,z), (0.2,0.35,0.65), collision=True)
    wall_center_z = float(bin_["top_z_m"]) - sz / 2.0
    for label, size, pos in [
        ("Left", (t,sy+2*t,sz), (x-(sx+t)/2,y,wall_center_z)),
        ("Right", (t,sy+2*t,sz), (x+(sx+t)/2,y,wall_center_z)),
        ("Near", (sx,t,sz), (x,y-(sy+t)/2,wall_center_z)),
        ("Far", (sx,t,sz), (x,y+(sy+t)/2,wall_center_z))]:
        cube(stage, "/World/Bin/"+label, size, pos, (0.2,0.35,0.65), collision=True)
    robots = {}
    for side, urdf in [("left", args.left), ("right", args.right)]:
        robots[side] = add_robot(stage, "/World/" + side, urdf, profile["robots"][side], appearance)
        paths = robots[side]["link_paths"]
        optical = [p for name,p in paths.items() if name.endswith("color_optical")]
        if len(optical) != 1:
            raise ValueError(f"Expected one color_optical link on {side}")
        camera = UsdGeom.Camera.Define(stage, optical[0]+"/Camera")
        # optical +Z forward,+Y down -> USD camera -Z forward,+Y up.
        place(camera, transform(rpy=(np.pi,0,0)))
        camera.CreateFocalLengthAttr(24)
        camera.CreateHorizontalApertureAttr(25.6)
        camera.CreateVerticalApertureAttr(19.2)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01,10))
    cam = profile["overhead"]
    camera = UsdGeom.Camera.Define(stage, "/World/overhead")
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*cam["eye_xyz_m"]), Gf.Vec3d(*cam["target_xyz_m"]), Gf.Vec3d(0,0,1)).GetInverse()
    camera.AddTransformOp().Set(matrix)
    camera.CreateFocalLengthAttr(cam["focal_length_mm"])
    camera.CreateHorizontalApertureAttr(cam["horizontal_aperture_mm"])
    camera.CreateVerticalApertureAttr(cam["horizontal_aperture_mm"]*0.75)
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(900)
    stage.GetRootLayer().Save()
    # Private review derivative includes referenced table geometry. Texture
    # asset paths remain external and must resolve on the reviewing machine.
    flattened = stage.Flatten()
    flat_stage = Usd.Stage.Open(flattened)
    texture_dependencies = []
    for shader_prim in flat_stage.Traverse():
        if not shader_prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(shader_prim)
        if shader.GetIdAttr().Get() != "UsdUVTexture":
            continue
        file_input = shader.GetInput("file")
        asset = file_input.Get()
        name = Path(asset.path).name if asset is not None else ""
        texture = args.out / "textures" / name
        if name.startswith("reference_panel_") and texture.is_file():
            relative = "textures/" + name
            file_input.Set(Sdf.AssetPath(relative))
            texture_dependencies.append({"path": relative, "sha256": hashlib.sha256(texture.read_bytes()).hexdigest()})
    flattened.Export(str(args.out / "assembly_preview.usdc"))
    reopened = Usd.Stage.Open(str(args.out / "assembly_preview.usda"))
    cameras = [str(p.GetPath()) for p in reopened.Traverse() if p.IsA(UsdGeom.Camera)]
    if len(cameras) != 3:
        raise RuntimeError("Preview must contain three cameras")
    bin_bottom_top = z + t / 2.0
    result.update(passed=True, robots=robots, camera_paths=cameras,
                  scene_geometry={
                      "ground": {"prim": "/World/FullFlatGround", "prim_type": "Mesh", "z_m": ground_z,
                                 "collision_api": True, "full_flat_ground": True},
                      "thor_table": {"visual_top_z_m": 0.0, "collision_top_z_m": -0.0155,
                                     "visual_collision_delta_m": 0.0155, "source_geometry_preserved": True,
                                     "collision_validation": "static source audit only"},
                      "bin": {"bottom_prim": "/World/Bin/Bottom", "wall_prims": ["/World/Bin/Left", "/World/Bin/Right", "/World/Bin/Near", "/World/Bin/Far"],
                              "bottom_center_z_m": z, "bottom_top_z_m": bin_bottom_top,
                              "top_z_m": float(bin_["top_z_m"]), "wall_collision_api": True,
                              "height_relation": "synthetic_top_rim_equal_to_thor_visual_top"},
                      "ground_cube_prims": [], "synthetic_height_assumptions": True},
                  appearance_sha256=None if args.appearance is None else hashlib.sha256(args.appearance.read_bytes()).hexdigest(),
                  appearance_scope=None if appearance is None else appearance["source_status"],
                  texture_dependencies=texture_dependencies,
                  shading_scope=None if appearance is None else appearance["surface_shading"]["scope"],
                  stage_prim_count=sum(1 for _ in reopened.Traverse()),
                  profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
                  limitations=["Limit-checked synthetic pose with URDF mimic; visual FK only", "No articulation/drive/collision cooking authored",
                               "No physical state or rendered images; cannot export dataset"])
    result_path.write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k != "robots"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("profile", "left", "right", "out"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--table-usd", type=Path, help="Private audited table asset; source remains read-only")
    parser.add_argument("--appearance", type=Path, help="Reviewed reference-only material palette; no geometry changes")
    build(parser.parse_args())
