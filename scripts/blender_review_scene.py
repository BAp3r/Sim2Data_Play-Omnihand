"""Open the private USD assembly in Blender and render static review images.

Run with Blender --background --factory-startup --python this.py -- ... .
These CPU Cycles images are assembly review artifacts, never simulator frames.
"""
import argparse
import json
from pathlib import Path
import sys

import bpy


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.out.mkdir(parents=True, exist_ok=False)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.usd_import(filepath=str(args.usd.resolve()))
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = 16
scene.cycles.use_denoising = False
scene.cycles.max_bounces = 4
scene.render.threads_mode = "FIXED"
scene.render.threads = 4
scene.render.resolution_x = 640
scene.render.resolution_y = 480
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.view_settings.view_transform = "AgX"
scene.view_settings.exposure = -2.0
scene["sim2data_scope"] = "static_assembly_review_not_simulator_capture"
scene["production_collection_allowed"] = False
cameras = [obj for obj in scene.objects if obj.type == "CAMERA"]
if len(cameras) != 3:
    raise RuntimeError(f"Expected three imported cameras, got {len(cameras)}")
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
          "mesh_objects": sum(o.type == "MESH" for o in scene.objects), "cameras": []}
for camera in cameras:
    camera.data.clip_start = .005
    camera.data.clip_end = 20
    scene.camera = camera
    filename = camera.name.replace("/", "_") + ".png"
    scene.render.filepath = str((args.out / filename).resolve())
    bpy.ops.render.render(write_still=True)
    report["cameras"].append({"name": camera.name, "image": filename,
                              "T_world_camera": [list(row) for row in camera.matrix_world]})
scene.camera = next((camera for camera in cameras if camera.name == "overhead"), cameras[0])
bpy.ops.wm.save_as_mainfile(filepath=str((args.out / "assembly_review.blend").resolve()))
(args.out / "blender_review.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
