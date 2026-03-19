# CadQuery Hybrid Helpers

`sdk_hybrid` keeps the articulated-object, URDF export, QC, and testing stack from the main SDK, but uses CadQuery for authored visual meshes.

Use this when you want:

- explicit links, joints, and tests to remain in Python/URDF space
- visible geometry to be authored with CadQuery
- compile-time generated collisions derived from the exported visual mesh

## Optional dependency

CadQuery is only required when you call the CadQuery helpers.

```python
from sdk_hybrid import AssetContext, mesh_from_cadquery
```

Generated scripts may also import CadQuery directly:

```python
import cadquery as cq
```

## Recommended pattern

Keep articulation metadata explicit in `sdk_hybrid`, and use CadQuery only for the visual-only mesh-producing part:

```python
import cadquery as cq

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    mesh_from_cadquery,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject("cabinet_door", assets=ASSETS)

    body = model.part("body")
    body.visual(Box((0.6, 0.3, 0.8)))
    body.inertial = Inertial.from_geometry(Box((0.6, 0.3, 0.8)), mass=12.0)

    door_shape = (
        cq.Workplane("XY")
        .box(0.58, 0.02, 0.78)
        .edges("|Z").fillet(0.01)
    )
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(door_shape, "door.obj", assets=ASSETS),
    )
    door.inertial = Inertial.from_geometry(Box((0.58, 0.02, 0.78)), mass=1.2)

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.29, 0.15, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.7, effort=5.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.015)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(max_pose_samples=64)
    return ctx.report()
```

Joint-limit rules are the same as the base SDK:

- `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC` require `MotionLimits` with `effort`, `velocity`, `lower`, and `upper`.
- `ArticulationType.CONTINUOUS` requires `MotionLimits(effort=..., velocity=...)` and must not set `lower` or `upper`.
- `ArticulationType.FIXED` and `ArticulationType.FLOATING` should not set `motion_limits`.

Example continuous articulation:

```python
model.articulation(
    "base_to_pan_head",
    ArticulationType.CONTINUOUS,
    parent="base",
    child="pan_head",
    origin=Origin(xyz=(0.0, 0.0, 0.4)),
    axis=(0.0, 0.0, 1.0),
    motion_limits=MotionLimits(effort=6.0, velocity=1.2),
)
```

## Units

CadQuery is unitless. For `sdk_hybrid`, keep the same unit system as the rest of the repository: meters and radians.

If you intentionally model in millimeters inside CadQuery, pass `unit_scale=0.001` to `mesh_from_cadquery(...)` so the written OBJ lands in meters.

## Available helpers

- `AssetContext.from_script(__file__)`
- `cadquery_local_aabb(model, *, tolerance=0.001, angular_tolerance=0.1, unit_scale=1.0)`
- `tessellate_cadquery(model, *, tolerance=0.001, angular_tolerance=0.1, unit_scale=1.0)`
- `save_cadquery_obj(model, filename, *, assets=None, tolerance=0.001, angular_tolerance=0.1, unit_scale=1.0)`
- `mesh_from_cadquery(model, filename, *, assets=None, tolerance=0.001, angular_tolerance=0.1, unit_scale=1.0) -> Mesh`
- `export_cadquery_mesh(model, filename, *, assets=None, tolerance=0.001, angular_tolerance=0.1, unit_scale=1.0) -> CadQueryMeshExport`

## Practical guidance

- Use CadQuery primarily for visual meshes.
- Use `mesh_from_cadquery(...)` to export a visual mesh, then attach it with `part.visual(...)`.
- Keep inertials explicit rather than inferring them from CadQuery mesh bounds.
- Keep `assets=AssetContext.from_script(__file__)` attached to the model so relative mesh refs under `meshes/` remain resolvable during QC and tests.
- Do not try to infer URDF joints from CadQuery assemblies. Declare joints explicitly in `ArticulatedObject`.
- Be careful with repeated `faces(...).workplane()` feature loops: the workplane origin can drift between iterations, so repeated cuts are often safer when built from a fixed global plane and subtracted explicitly.
