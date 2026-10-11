"""Read-only USD/contact-buffer diagnostics for a single-box PhysX trial.

These helpers do not enable or disable physics, set poses, or establish a grasp.
The USD audit reports the composed source hierarchy and resolved physics material.
"""
from __future__ import annotations

import math


def audit_colliders(stage, roots=None):
    """Return collision ownership/material evidence without changing the stage."""
    from pxr import Usd, UsdPhysics, UsdShade

    roots = tuple(str(path).rstrip("/") for path in (roots or ()))
    rows = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if roots and not any(path == root or path.startswith(root + "/") for root in roots):
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        owners = []
        ancestor = prim
        while ancestor and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                body = UsdPhysics.RigidBodyAPI(ancestor)
                owners.append({
                    "path": str(ancestor.GetPath()),
                    "rigid_body_enabled": body.GetRigidBodyEnabledAttr().Get(),
                    "kinematic_enabled": body.GetKinematicEnabledAttr().Get(),
                    "disable_gravity": ancestor.GetAttribute("physxRigidBody:disableGravity").Get(),
                    "mass_kg": UsdPhysics.MassAPI(ancestor).GetMassAttr().Get()
                    if ancestor.HasAPI(UsdPhysics.MassAPI) else None,
                })
            ancestor = ancestor.GetParent()
        material, relationship = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        material_path = str(material.GetPath()) if material else None
        physics_material = None
        if material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
            api = UsdPhysics.MaterialAPI(material.GetPrim())
            physics_material = {
                "static_friction": api.GetStaticFrictionAttr().Get(),
                "dynamic_friction": api.GetDynamicFrictionAttr().Get(),
                "restitution": api.GetRestitutionAttr().Get(),
            }
        rows.append({
            "collider": path,
            "collision_enabled": UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
            "mesh_approximation": UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None,
            "nearest_rigid_body": owners[0]["path"] if owners else None,
            "rigid_body_ancestors": owners,
            "nested_rigid_body": len(owners) > 1,
            "physics_material": material_path,
            "physics_material_binding": str(relationship.GetPath()) if relationship else None,
            "physics_material_properties": physics_material,
        })
    return {"colliders": rows, "read_only": True, "physics_runtime_validated": False}


def validate_contact_matrix(matrix, filters, *, sensor_count=None, filter_count=None):
    """Reject ambiguous multi-sensor or missing/nonfinite contact readback.

    The caller may index matrix[0] only after this check succeeds. Reshaping to
    (-1, 3) first loses which dimension represented sensors versus filters.
    """
    expected = (1, len(filters), 3)
    shape = tuple(int(value) for value in getattr(matrix, "shape", ()))
    observed = {"matrix_shape": list(shape), "expected_matrix_shape": list(expected),
                "sensor_count": sensor_count, "filter_count": filter_count}
    if not filters or len(set(filters)) != len(filters):
        raise ValueError("contact filter order must be nonempty and unique")
    if shape != expected:
        raise ValueError(f"single-box contact matrix shape mismatch: {observed}")
    if sensor_count is not None and sensor_count != 1:
        raise ValueError(f"single-box contact sensor count mismatch: {observed}")
    if filter_count is not None and filter_count != len(filters):
        raise ValueError(f"contact filter count mismatch: {observed}")
    if not all(math.isfinite(float(value)) for vector in matrix[0] for value in vector):
        raise ValueError("contact matrix contains nonfinite force readback")
    return observed
