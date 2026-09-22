"""Open the private USD assembly in Blender and render static review images.

Run with Blender --background --factory-startup --python this.py -- ... .
These CPU Cycles images are assembly review artifacts, never simulator frames.
"""
import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--wrist-closeups", action="store_true", help="Add labeled static inspection views, not sensors")
parser.add_argument("--symmetry-views", action="store_true", help="Add paired front/top orthographic review views")
parser.add_argument("--material-mode", choices=("diagnostic", "reference"), default="diagnostic",
                    help="Reference mode preserves authored robot materials; table MDL still gets a review fallback")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.out.mkdir(parents=True, exist_ok=False)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.usd_import(filepath=str(args.usd.resolve()))
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = 32
scene.cycles.use_denoising = True
scene.cycles.max_bounces = 4
scene.render.threads_mode = "FIXED"
scene.render.threads = 4
scene.render.resolution_x = 640
scene.render.resolution_y = 480
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.view_settings.view_transform = "AgX"
scene.view_settings.exposure = 0.0
scene["sim2data_scope"] = "static_assembly_review_not_simulator_capture"
scene["production_collection_allowed"] = False
# MDL OmniPBR is not portable to Cycles. Apply explicitly labeled review
# surfaces, preserving all source geometry and keeping source USD read-only.
review_surfaces = {}
for role, color, metallic in (("table", (.24, .26, .29, 1), .4),
                               ("mount", (.62, .32, .08, 1), .35),
                               ("camera", (.08, .23, .38, 1), .25)):
    mat = bpy.data.materials.new("ReviewOnly_" + role)
    mat.use_nodes = True
    node = mat.node_tree.nodes.get("Principled BSDF")
    node.inputs["Base Color"].default_value = color
    node.inputs["Metallic"].default_value = metallic
    node.inputs["Roughness"].default_value = .4
    review_surfaces[role] = mat
review_overrides = []
for obj in scene.objects:
    if obj.type != "MESH" and obj.instance_type != "COLLECTION":
        continue
    ancestor, role = obj, None
    while ancestor is not None:
        name = ancestor.name
        if "_hand__" in name:
            break
        if name.endswith("_camera_housing"):
            role = "camera"
            break
        if name.endswith("_mount_assembly"):
            role = "mount"
            break
        if name == "Table":
            role = "table"
            break
        ancestor = ancestor.parent
    if role and not (args.material_mode == "reference" and role != "table"):
        targets = [obj] if obj.type == "MESH" else [
            item for item in obj.instance_collection.all_objects if item.type == "MESH"]
        for target in targets:
            target.data.materials.clear()
            target.data.materials.append(review_surfaces[role])
            for polygon in target.data.polygons:
                polygon.material_index = 0
            review_overrides.append({"object": target.name, "review_role": role})
cameras = [obj for obj in scene.objects if obj.type == "CAMERA"]
if len(cameras) != 3:
    raise RuntimeError(f"Expected three imported cameras, got {len(cameras)}")
if args.wrist_closeups:
    for side in ("left", "right"):
        wrist = bpy.data.objects.get(f"{side}_arm__link6")
        if wrist is None:
            raise RuntimeError(f"Missing {side} wrist frame for inspection view")
        target = wrist.matrix_world @ Vector((0, 0, .12))
        view = bpy.data.objects.new(f"review_{side}_wrist", bpy.data.cameras.new(f"review_{side}_wrist"))
        scene.collection.objects.link(view)
        view.location = target + Vector((.26, -.32, .18))
        view.rotation_euler = (target-view.location).to_track_quat("-Z", "Y").to_euler()
        view.data.type = "ORTHO"
        view.data.ortho_scale = .34
        cameras.append(view)
if args.symmetry_views:
    frames = [bpy.data.objects.get(f"{side}_arm__link6") for side in ("left", "right")]
    if any(frame is None for frame in frames):
        raise RuntimeError("Missing wrist frames for paired symmetry review")
    target = sum((frame.matrix_world @ Vector((0, 0, .12)) for frame in frames), Vector()) / 2
    for label, offset in (("front", (0, .7, -.12)), ("top", (0, 0, .8))):
        name = "review_pair_" + label
        view = bpy.data.objects.new(name, bpy.data.cameras.new(name))
        scene.collection.objects.link(view)
        view.location = target + Vector(offset)
        view.rotation_euler = (target-view.location).to_track_quat("-Z", "Y").to_euler()
        view.data.type = "ORTHO"
        view.data.ortho_scale = .8
        cameras.append(view)
# Use one known review lighting setup; imported USD dome intensity does not
# have an equivalent Blender exposure and otherwise washes out the geometry.
for obj in list(scene.objects):
    if obj.type == "LIGHT":
        bpy.data.objects.remove(obj, do_unlink=True)
world = bpy.data.worlds.new("ReviewWorld")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (.25, .25, .25, 1)
world.node_tree.nodes["Background"].inputs["Strength"].default_value = .6
light_data = bpy.data.lights.new("ReviewArea", type="AREA")
light_data.energy = 70
light_data.shape = "DISK"
light_data.size = 3
light = bpy.data.objects.new("ReviewArea", light_data)
scene.collection.objects.link(light)
light.location = (0, 0, 2.0)
report = {"blender_version": bpy.app.version_string,
          "scope": "CPU_Cycles_static_assembly_review_not_training_RGB",
          "source_usd": str(args.usd.resolve()), "physics_validated": False,
          "dataset_export_allowed": False, "production_collection_allowed": False,
          "review_material_overrides": review_overrides,
          "material_mode": args.material_mode,
          "mesh_objects": sum(o.type == "MESH" for o in scene.objects), "cameras": []}
for camera in cameras:
    camera.data.clip_start = .005
    camera.data.clip_end = 20
    scene.camera = camera
    filename = camera.name.replace("/", "_") + ".png"
    scene.render.filepath = str((args.out / filename).resolve())
    bpy.ops.render.render(write_still=True)
    report["cameras"].append({"name": camera.name, "image": filename,
                              "role": "inspection_only" if camera.name.startswith("review_") else "imported_static_camera",
                              "T_world_camera": [list(row) for row in camera.matrix_world]})
scene.camera = next((camera for camera in cameras if camera.name == "overhead"), cameras[0])
bpy.ops.wm.save_as_mainfile(filepath=str((args.out / "assembly_review.blend").resolve()))
(args.out / "blender_review.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
