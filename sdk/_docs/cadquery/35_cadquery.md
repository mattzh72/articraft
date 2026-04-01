# CadQuery Hybrid Helpers

## Purpose

`sdk_hybrid` keeps the articulated-object, testing, and export stack from the
base SDK, but uses CadQuery to author visible mesh geometry.

Use this when:

- articulated structure should stay explicit in Python/URDF space
- visible geometry is easier to author in CadQuery
- the final authored visuals should be mesh-backed

## Import

```python
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    export_cadquery_mesh,
    mesh_from_cadquery,
    mesh_from_input,
    tessellate_cadquery,
)
```

## Recommended Surface

- `tessellate_cadquery(...)`
- `export_cadquery_mesh(...)`
- `mesh_from_cadquery(...)`

## Units

CadQuery is unitless. In `sdk_hybrid`, authored geometry should land in meters.

- If the CadQuery model is already authored in meters, keep `unit_scale=1.0`.
- If it is authored in millimeters, pass `unit_scale=0.001`.

## Helper Reference

### `tessellate_cadquery(...)`

```python
tessellate_cadquery(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]
```

- Returns raw vertices and triangle faces.
- Use this when you need tessellated geometry directly rather than an exported
  OBJ.

### `export_cadquery_mesh(...)`

```python
export_cadquery_mesh(
    model: object,
    name: str,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> CadQueryMeshExport
```

```python
CadQueryMeshExport(
    mesh: Mesh,
    local_aabb,
    center_xyz,
    size_xyz,
)
```

- Materializes the OBJ internally and returns the managed `sdk.Mesh` plus local
  bounds.

### `mesh_from_cadquery(...)`

```python
mesh_from_cadquery(
    model: object,
    name: str,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Mesh
```

- This is the normal helper for attaching a CadQuery-authored visual to a part.

## Recommended Pattern

Use CadQuery for visible mesh authoring, then attach the result as a normal
visual on an `ArticulatedObject` part.

door_shape = (
    cq.Workplane("XY")
    .box(0.58, 0.02, 0.78)
    .edges("|Z").fillet(0.01)
)

door = model.part("door")
door.visual(mesh_from_cadquery(door_shape, "door"))
```

## Advice

- Keep joints explicit in `ArticulatedObject`. Do not try to infer URDF joints
  from CadQuery assemblies.
- Keep inertials explicit rather than deriving them from the exported mesh.
- Use stable logical mesh names such as `"door"` or `"left_bracket"` instead of
  filesystem paths.
- Be careful with repeated `faces(...).workplane()` loops in CadQuery. For
  repeated cuts or features, fixed reference planes are often more stable.

## Example

```python
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject("cabinet_door")

    body = model.part("body")
    body.visual(Box((0.6, 0.3, 0.8)))

    door_shape = (
        cq.Workplane("XY")
        .box(0.58, 0.02, 0.78)
        .edges("|Z").fillet(0.01)
    )
    door = model.part("door")
    door.visual(mesh_from_cadquery(door_shape, "door"))

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
```

## See Also

- `../common/80_testing.md` for the shared testing API
- `../common/00_quickstart.md` for the overall script contract

## Clarifications for agent usage

- `mesh_from_cadquery(...)` and `export_cadquery_mesh(...)` accept CadQuery `Shape`, `Workplane`, and `Assembly` inputs. Workplanes are exported from their resolved solid values; assemblies export each component at its current CadQuery location.
- Constrained CadQuery assemblies must be solved before export. The SDK does not call `solve()` for you.
- Export preserves the CadQuery local frame of the authored shape/component. The SDK does not recenter meshes, infer hinge pivots, or move geometry to an articulation axis automatically.
- If an articulation pivot should be at a hinge, either model the CadQuery shape in that local frame or attach the mesh with `visual(origin=Origin(...))` so the mesh frame and joint frame line up explicitly.
