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
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade


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


def material(stage, prim, colour):
    # Explicit PreviewSurface makes colours portable to Blender and Kit;
    # displayColor alone is not interpreted as a surface by every importer.
    key = "c_" + "_".join(str(round(float(c) * 65535)) for c in colour[:3])
    path = "/World/Looks/" + key
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour[:3]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.55)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def mesh(stage, path, geometry, colour):
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
    prim.CreateExtentAttr([Gf.Vec3f(*vertices.min(axis=0).tolist()),
                           Gf.Vec3f(*vertices.max(axis=0).tolist())])
    return len(vertices), len(faces)


def cube(stage, path, size, centre, colour):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1)
    item.AddTranslateOp().Set(Gf.Vec3d(*centre))
    item.AddScaleOp().Set(Gf.Vec3f(*size))
    item.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    material(stage, item.GetPrim(), colour)


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


def add_robot(stage, path, urdf, base):
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
    positions = {}

    def position(name, chain=()):
        if name in chain:
            raise ValueError("Cyclic mimic relationship")
        if name not in positions:
            mimic = joints[name].find("mimic")
            positions[name] = 0.0 if mimic is None else (
                float(mimic.get("multiplier", "1")) * position(mimic.attrib["joint"], chain+(name,))
                + float(mimic.get("offset", "0")))
        return positions[name]

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
            q = position(joint.attrib["name"])
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
        # Independent joints at zero, URDF mimic applied. No commanded state.
        for index, visual in enumerate(links[name].findall("visual")):
            visual_path = link_path + f"/visual_{index}"
            visual_frame = UsdGeom.Xform.Define(stage, visual_path)
            place(visual_frame, origin(visual))
            colour_node = visual.find("material/color")
            colour = [0.6, 0.65, 0.72] if colour_node is None else [
                float(x) for x in colour_node.attrib["rgba"].split()[:3]]
            mesh(stage, visual_path + "/mesh", load_geometry(visual.find("geometry")), colour)
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
    cube(stage, "/World/Table", table["size_xyz_m"], table["center_xyz_m"], (0.22, 0.25, 0.28))
    box = profile["box"]
    cube(stage, "/World/SyntheticBox", box["size_xyz_m"], box["center_xyz_m"], (0.6, 0.35, 0.14))
    relay = profile["relay_region"]
    cube(stage, "/World/RelayMarker", relay["size_xyz_m"], relay["center_xyz_m"], (0.15, 0.65, 0.25))
    bin_ = profile["bin"]
    x, y, z = bin_["bottom_center_xyz_m"]
    sx, sy, sz = bin_["inner_size_xyz_m"]
    t = bin_["wall_thickness_m"]
    cube(stage, "/World/Bin/Bottom", (sx+2*t, sy+2*t, t), (x,y,z), (0.2,0.35,0.65))
    for label, size, pos in [
        ("Left", (t,sy+2*t,sz), (x-(sx+t)/2,y,z+(sz+t)/2)),
        ("Right", (t,sy+2*t,sz), (x+(sx+t)/2,y,z+(sz+t)/2)),
        ("Near", (sx,t,sz), (x,y-(sy+t)/2,z+(sz+t)/2)),
        ("Far", (sx,t,sz), (x,y+(sy+t)/2,z+(sz+t)/2))]:
        cube(stage, "/World/Bin/"+label, size, pos, (0.2,0.35,0.65))
    robots = {}
    for side, urdf in [("left", args.left), ("right", args.right)]:
        robots[side] = add_robot(stage, "/World/" + side, urdf, profile["robots"][side])
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
    stage.GetRootLayer().Export(str(args.out / "assembly_preview.usdc"))
    reopened = Usd.Stage.Open(str(args.out / "assembly_preview.usda"))
    cameras = [str(p.GetPath()) for p in reopened.Traverse() if p.IsA(UsdGeom.Camera)]
    if len(cameras) != 3:
        raise RuntimeError("Preview must contain three cameras")
    result.update(passed=True, robots=robots, camera_paths=cameras,
                  stage_prim_count=sum(1 for _ in reopened.Traverse()),
                  profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
                  limitations=["Independent q=0 with URDF mimic; visual FK only", "No articulation/drive/collision cooking authored",
                               "No physical state or rendered images; cannot export dataset"])
    result_path.write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k != "robots"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("profile", "left", "right", "out"):
        parser.add_argument("--"+name, type=Path, required=True)
    build(parser.parse_args())
