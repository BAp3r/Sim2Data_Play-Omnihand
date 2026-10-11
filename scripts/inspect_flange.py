"""CPU-only STEP BRep inspection; all generated geometry is private evidence.

Requires cadquery-ocp-novtk, numpy and matplotlib in a separate environment.
No mechanical datum, material density, or assembly calibration is inferred.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.collections import Sequence_TCollection_AsciiString
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def xyz(p):
    return [p.X(), p.Y(), p.Z()]


def items(shape, kind):
    exp = TopExp_Explorer(shape, kind)
    while exp.More():
        yield exp.Current()
        exp.Next()


def bounds(shape):
    box = Bnd_Box()
    # Exact trimmed BRep geometry, not control-point extrema or tessellation.
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    values = xyz(box.CornerMin()) + xyz(box.CornerMax())
    return {"min_mm": values[:3], "max_mm": values[3:],
            "size_mm": [values[i + 3] - values[i] for i in range(3)]}


def inspect(source, output):
    output.mkdir(parents=True, exist_ok=False)
    raw = source.read_bytes()
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError("STEP read failed")
    units = [Sequence_TCollection_AsciiString() for _ in range(3)]
    reader.FileUnits(*units)
    # OCCT conversion target is explicitly millimetres, regardless of source unit.
    reader.SetSystemLengthUnit(1.0)
    if not reader.TransferRoots():
        raise RuntimeError("STEP contains no transferable roots")
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError("STEP transfer produced a null shape")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    report = {
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "source_bytes": len(raw),
        "source_units": {key: [seq.Value(i).ToCString() for i in range(1, seq.Length() + 1)]
                         for key, seq in zip(("length", "angle", "solid_angle"), units)},
        "analysis_units": "millimetres; cubic millimetres; no density assumption",
        "system_length_unit_mm": reader.SystemLengthUnit(),
        "reader_roots": reader.NbRootsForTransfer(),
        "brep_valid": BRepCheck_Analyzer(shape).IsValid(),
        "solid_count": len(list(items(shape, TopAbs_SOLID))),
        "bbox": bounds(shape),
        "volume_mm3": props.Mass(),
        "uniform_volume_centroid_mm": xyz(props.CentreOfMass()),
        "packages": {p: importlib.metadata.version(p) for p in
                     ("cadquery-ocp-novtk", "numpy", "matplotlib")},
        "faces": [],
        "production_transforms": None,
    }
    face_shapes = list(items(shape, TopAbs_FACE))
    for index, face_shape in enumerate(face_shapes, 1):
        face = TopoDS.Face(face_shape)
        surf = BRepAdaptor_Surface(face)
        fp = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, fp)
        record = {"index": index, "kind": str(surf.GetType()),
                  "area_mm2": fp.Mass(), "centroid_mm": xyz(fp.CentreOfMass()),
                  "bbox": bounds(face), "orientation": str(face.Orientation())}
        if surf.GetType() == GeomAbs_Plane:
            plane = surf.Plane()
            record.update(plane_origin_mm=xyz(plane.Location()),
                          plane_axis=xyz(plane.Axis().Direction()))
        elif surf.GetType() == GeomAbs_Cylinder:
            cyl = surf.Cylinder()
            record.update(radius_mm=cyl.Radius(), axis_origin_mm=xyz(cyl.Location()),
                          axis_direction=xyz(cyl.Axis().Direction()),
                          uv_bounds=[surf.FirstUParameter(), surf.LastUParameter(),
                                     surf.FirstVParameter(), surf.LastVParameter()])
        report["faces"].append(record)

    mesh = BRepMesh_IncrementalMesh(shape, 0.05, False, 0.15, False)
    if not mesh.IsDone():
        raise RuntimeError("Tessellation failed")
    vertices, triangles = [], []
    for face_shape in face_shapes:
        face = TopoDS.Face(face_shape)
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            raise RuntimeError("Face has no triangulation")
        offset = len(vertices)
        vertices.extend(xyz(tri.Node(i).Transformed(loc.Transformation()))
                        for i in range(1, tri.NbNodes() + 1))
        for i in range(1, tri.NbTriangles() + 1):
            t = list(tri.Triangle(i).Get())
            if face.Orientation() == TopAbs_REVERSED:
                t[1], t[2] = t[2], t[1]
            triangles.append([offset + n - 1 for n in t])
    vertices, triangles = np.array(vertices), np.array(triangles)
    report["mesh"] = {"vertices": len(vertices), "triangles": len(triangles),
                      "linear_deflection_mm": 0.05, "angular_deflection_rad": 0.15,
                      "obj_numeric_unit": "metre", "cad_axes_preserved": True}
    with (output / "flange_cad_m.obj").open("w", encoding="ascii") as f:
        f.write("# Private derived mesh. Numeric coordinates in metres, original CAD axes.\n")
        for v in vertices / 1000:
            f.write("v {:.12g} {:.12g} {:.12g}\n".format(*v))
        for t in triangles + 1:
            f.write("f {} {} {}\n".format(*t))
    (output / "geometry.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_views(vertices, triangles, report["bbox"], output)
    print(json.dumps({k: report[k] for k in
                      ("source_sha256", "source_units", "brep_valid", "solid_count", "bbox", "volume_mm3", "mesh")}, indent=2))


def plot_views(vertices, triangles, bbox, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(15, 10))
    views = [("CAD +Z view", 90, -90), ("CAD -Y view", 0, -90),
             ("CAD +X view", 0, 0), ("CAD +Y view", 0, 90),
             ("CAD isometric A", 25, -55), ("CAD isometric B", -25, 125)]
    centre = (np.array(bbox["min_mm"]) + bbox["max_mm"]) / 2
    radius = max(bbox["size_mm"]) * 0.55
    for i, (title, elevation, azimuth) in enumerate(views, 1):
        ax = fig.add_subplot(2, 3, i, projection="3d", proj_type="ortho")
        faces = vertices[triangles]
        normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        norms = np.linalg.norm(normals, axis=1)
        normals = normals / np.maximum(norms[:, None], 1e-15)
        shade = 0.5 + 0.4 * np.abs(normals @ np.array([0.3, -0.5, 0.8]))
        colours = np.c_[shade * 0.7, shade * 0.82, shade * 0.95, np.ones(len(shade))]
        ax.add_collection3d(Poly3DCollection(faces, facecolors=colours, edgecolors="none"))
        for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), centre):
            setter(c - radius, c + radius)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elevation, azim=azimuth)
        ax.set(xlabel="CAD X / mm", ylabel="CAD Y / mm", zlabel="CAD Z / mm", title=title)
    fig.suptitle("Private STEP BRep tessellation / CAD frame only / CPU orthographic views")
    fig.tight_layout()
    fig.savefig(output / "cad_views.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path, help="New private evidence directory (must not exist)")
    args = parser.parse_args()
    inspect(args.source, args.output)
