# Thor table asset audit

This record binds the official Isaac Sim table selected for the synthetic assembly
recipe.  It is a static asset audit, not a PhysX, rendering, contact, or
production-collection acceptance report.  The user-facing name `Thor table` is
mapped here to the official `ThorlabsTable` package path.

## Selected source

| Field | Value |
| --- | --- |
| Package | NVIDIA Isaac Sim content package `5.1.0`, Isaac `5.1` namespace |
| Asset | `5.1.0/Assets/Isaac/5.1/Isaac/Props/Mounts/ThorlabsTable/table_instanceable.usd` |
| Default prim | `/table` |
| Source units | `metersPerUnit=1.0`, `upAxis=Z` |
| Visual/collision payload | `Props/instaceable_meshes.usd` (the filename spelling is part of the package) |
| License record | `5.1.0/PACKAGE-LICENSES/isaac-sim-assets-LICENSE.txt` |
| `table_instanceable.usd` SHA256 | `F5A00DA4C05DB6DD0441F8C76A886A898435E61890DD1F445FF7E8F79E1E2495` |
| `Props/instaceable_meshes.usd` SHA256 | `B7827DE44E5051D8D7803507B9362DBF8521181996449A88934B59CEB0D54DB6` |

The source files were opened read-only from the existing private official asset
store.  The package license file was present.  Redistribution and public asset
publication remain subject to the package terms; no source asset is copied into
this repository.

`table_instanceable.usd` defines instanceable `/table/visuals` and
`/table/collisions` children, each referencing a prim in the payload.  A normal
stage traversal therefore exposes the instance roots rather than the prototype
meshes.  The bounds below include the resolved instance prototypes.

## Geometry and proposed placement

The resolved visual instance `/table/visuals` has bounds in native metres:

```text
min = [-0.176249, -0.379300, -0.794700]
max = [ 0.723751,  0.379300,  0.000000]
size = [ 0.900000,  0.758600,  0.794700]
```

The synthetic recipe places the visual top on the project desktop plane
`world z=0` and centers the table in x:

```yaml
T_world_asset:
  translation_m: [-0.2737505, 0.0, 0.0]
  rotation_wxyz: [1.0, 0.0, 0.0, 0.0]
  scale: [1.0, 1.0, 1.0]
```

This is a recipe placement, not a measured laboratory table pose.  The project
coordinate and quaternion conventions remain those in `docs/DESIGN.md`.

## Collision boundary and support caveat

The resolved collision instance `/table/collisions` has bounds:

```text
min = [-0.176250, -0.375000, -0.740500]
max = [ 0.723750,  0.375000, -0.015500]
size = [ 0.900000,  0.750000,  0.725000]
```

`/table_collisions/Cube` carries `PhysicsCollisionAPI` and
`PhysicsMeshCollisionAPI`; collision is enabled and the authored mesh
approximation is `convexHull`.  The collision envelope's highest point is
`-0.0155 m` (15.5 mm below the visual top at `z=0`).  The visual top and the
collision support surface must therefore not be treated as the same datum.  A
box placed on the visual plane can initially be separated from this collider;
contact support is unresolved until a runtime check and an approved support
adjustment are supplied.  The source USD must not be edited and a collider
offset or shim must not be silently introduced to manufacture contact.

The table does not author a rigid-body API.  It is intended as a static scene
fixture candidate.  Mass and inertia are consequently not bound by this audit.

## Materials and dependencies

The payload visual prototype contains one mesh with approximately 740,994
points and material subsets that reference the built-in `OmniPBR.mdl` token.
The static asset path inspection found no external texture asset paths in the
payload.  This is dependency metadata only; Kit material resolution and final
RGB appearance still require the selected runtime.

## Evidence and limits

Private, ignored evidence for this audit is under
`.local/evidence/m3/thor_table/`:

- `thor_probe.json`: stage metadata, instance-resolved world bounds, and
  payload schema inspection;
- `thor_bounds_collision.txt`: prototype bounds and collision API inspection;
- `thor_traversal.txt`: instance/prototype traversal details;
- `thor_material_paths.txt`: payload material asset-path inspection.

The audit used the existing remote pxr interpreter and did not modify the NAS
or copy the official asset.  No Isaac Kit scene-open, PhysX cooking/contact
check, RGB render, mass/inertia estimate, or grasp test was run.  The selected
asset is therefore ready as a documented private binding input for B/runtime
work, while physical support and production collection remain blocked.
