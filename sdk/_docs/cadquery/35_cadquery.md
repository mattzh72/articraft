# CadQuery Helpers

## Purpose

`sdk` keeps the articulated-object, testing, and export stack from the
base SDK, but uses CadQuery to author visible mesh geometry.

Use this when:

- articulated structure should stay explicit in Python/URDF space
- visible geometry is easier to author in CadQuery
- the final authored visuals should be mesh-backed

## Import

```python
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cadquery_local_aabb,
    export_cadquery_components,
    export_cadquery_mesh,
    mesh_components_from_cadquery,
    mesh_from_cadquery,
    mesh_from_input,
    save_cadquery_obj,
    tessellate_cadquery,
)
```

## Recommended Surface

- `cadquery_local_aabb(...)`
- `tessellate_cadquery(...)`
- `export_cadquery_mesh(...)`
- `export_cadquery_components(...)`
- `mesh_from_cadquery(...)`
- `mesh_components_from_cadquery(...)`

Compatibility helper:

- `save_cadquery_obj(...)`

## Units

CadQuery is unitless. In `sdk`, authored geometry must land in meters.

- If the CadQuery model is already authored in meters, keep `unit_scale=1.0`.
- If it is authored in millimeters, either convert the literals to meters or
  pass `unit_scale=0.001` exactly once at export time.
- `unit_scale` applies to the entire tessellated result, including solved
  assembly/component locations.

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

### `cadquery_local_aabb(...)`

```python
cadquery_local_aabb(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]
```

- Returns the local axis-aligned bounding box of the tessellated CadQuery
  result.
- This is useful when you need a size/placement probe before attaching the mesh
  into an `ArticulatedObject`.

### `export_cadquery_mesh(...)`

```python
export_cadquery_mesh(
    model: object,
    name: str,
    *,
    assets=None,
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
- `name`: logical mesh name such as `"door_panel"` or a legacy path-like name
  such as `"door_panel.obj"`.
- `assets`: optional explicit asset owner or root. Read
  `../common/40_assets.md` when you need stable on-disk paths outside the
  managed harness.
- `tolerance`: linear tessellation tolerance.
- `angular_tolerance`: angular tessellation tolerance in radians.
- `unit_scale`: scales both tessellated geometry and CadQuery assembly
  locations.

### `export_cadquery_components(...)`

```python
export_cadquery_components(
    model: object,
    name: str,
    *,
    assets=None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> list[CadQueryMeshExport]
```

- Splits a CadQuery multi-shape workplane or assembly into one exported mesh per
  component.
- Uses numbered managed names such as `"pair__component_001.obj"` under the
  chosen logical name.
- Returns one `CadQueryMeshExport` per resolved component in CadQuery order.

### `mesh_from_cadquery(...)`

```python
mesh_from_cadquery(
    model: object,
    name: str,
    *,
    assets=None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Mesh
```

- This is the normal helper for attaching a CadQuery-authored visual to a part.
- Parameters match `export_cadquery_mesh(...)`, but only the managed `Mesh` is
  returned.

### `mesh_components_from_cadquery(...)`

```python
mesh_components_from_cadquery(
    model: object,
    name: str,
    *,
    assets=None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> list[Mesh]
```

- Component-only convenience wrapper over `export_cadquery_components(...)`.
- Use this when one CadQuery source should become multiple separate mesh visuals
  or subfeatures.

### `save_cadquery_obj(...)`

```python
save_cadquery_obj(
    model: object,
    name: str,
    *,
    assets=None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Path
```

- Public compatibility helper that returns the materialized OBJ path directly.
- Prefer `export_cadquery_mesh(...)` or `mesh_from_cadquery(...)` for new code
  when you want the managed `Mesh` object.

## Recommended Pattern

Use CadQuery for visible mesh authoring, then attach the result as a normal
visual on an `ArticulatedObject` part.

SDK materials are assigned to the resulting `Visual`, not to CadQuery faces or
sub-bodies inside one exported mesh. CadQuery face colors, per-face materials,
UVs, and texture coordinates are not preserved by `mesh_from_cadquery(...)`.
If different regions need different materials, export them as separate
CadQuery meshes or components and attach each one as its own visual with its
own `material=...`.

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

## Clarifications for agent usage

- `mesh_from_cadquery(...)` and `export_cadquery_mesh(...)` accept CadQuery `Shape`, `Workplane`, and `Assembly` inputs. Workplanes are exported from their resolved solid values; assemblies export each component at its current CadQuery location.
- `cadquery_local_aabb(...)`, `export_cadquery_components(...)`, and `mesh_components_from_cadquery(...)` accept the same CadQuery input types.
- Constrained CadQuery assemblies must be solved before export. The SDK does not call `solve()` for you.
- Choose one unit story per authored CadQuery model: either author the shape and assembly locations directly in meters, or keep them in source units and pass the matching `unit_scale` when exporting to `sdk`.
- Export preserves the CadQuery local frame of the authored shape/component. The SDK does not recenter meshes, infer hinge pivots, or move geometry to an articulation axis automatically.
- If an articulation pivot should be at a hinge, either model the CadQuery shape in that local frame or attach the mesh with `visual(origin=Origin(...))` so the mesh frame and joint frame line up explicitly.
- Materials are applied per SDK visual. Split multi-material CadQuery geometry
  into separate exported meshes/components before attaching visuals.
- When you need one mesh per CadQuery component instead of a single combined
  mesh, use `export_cadquery_components(...)` or
  `mesh_components_from_cadquery(...)`.

## See Also

- `../common/80_testing.md` for the shared testing API
- `../common/00_quickstart.md` for the overall script contract
- `39d_cadquery_gears.md` for the vendored gear builders and Workplane gear
  plugin helpers
- `../common/40_assets.md` for explicit asset-root control
