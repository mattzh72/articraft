# Core Types

## Purpose

This page documents the core data types used across the base `sdk`:
transforms, geometry descriptors, materials, inertials, parts, and
articulations.

## Import

```python
from sdk import (
    Origin,
    Box,
    Cylinder,
    Sphere,
    Mesh,
    Material,
    Visual,
    Inertia,
    Inertial,
    MotionLimits,
    MotionProperties,
    Part,
    Articulation,
    ArticulationType,
    scale_geometry_to_size,
)
```

## Units

- Distances are meters.
- Rotations are roll, pitch, yaw in radians.
- Revolute and continuous joint positions are radians; prismatic joint
  positions are meters measured along the configured joint axis.
- URDF cylinders are aligned with local `+Z`.

## Recommended Surface

- `Origin`
- `Box`, `Cylinder`, `Sphere`, `Mesh`
- `Material`, `Visual`
- `Inertia`, `Inertial.from_geometry(...)`
- `MotionLimits`, `MotionProperties`
- `Part.visual(...)`, `Part.get_visual(...)`
- `Articulation`, `ArticulationType`
- `scale_geometry_to_size(...)`

## Transforms

### `Origin`

```python
Origin(
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
)
```

- `xyz`: translation in meters.
- `rpy`: rotation in radians.
- Both fields must contain exactly 3 numbers.

## Geometry Descriptors

Geometry descriptors describe shape only. Put transforms on the containing
`Visual` or `Collision` via `origin=Origin(...)`.

### `Box`

```python
Box(size: tuple[float, float, float])
```

- `size`: full extents `(sx, sy, sz)` in meters.

### `Cylinder`

```python
Cylinder(radius: float, length: float)
```

- `radius`: cylinder radius in meters.
- `length`: full cylinder length along local `+Z`.

### `Sphere`

```python
Sphere(radius: float)
```

- `radius`: sphere radius in meters.

### `Mesh`

```python
Mesh(
    filename: str | os.PathLike[str],
    scale: tuple[float, float, float] | None = None,
    source_geometry: Box | Cylinder | Sphere | None = None,
    source_transform: tuple[tuple[float, float, float, float], ...] | None = None,
)
```

- `filename`: absolute or asset-relative mesh path.
- `scale`: optional per-axis scale.
- `source_geometry`: optional primitive provenance for meshes that were
  generated from a primitive.
- `source_transform`: optional provenance transform. This requires
  `source_geometry`.

`filename` is required and must be non-empty.

## Materials and Visuals

### `Material`

```python
Material(
    name: str,
    rgba: tuple[float, float, float, float] | None = None,
    texture: str | None = None,
)
```

- `name`: required material name.
- `rgba`: 3 or 4 floats. A 3-tuple is expanded to `(r, g, b, 1.0)`.
- `texture`: optional texture path.

### `Visual`

```python
Visual(
    geometry: Box | Cylinder | Sphere | Mesh,
    origin: Origin = Origin(),
    material: Material | str | None = None,
    name: str | None = None,
)
```

- `geometry`: visible geometry descriptor.
- `origin`: local transform of this visual on its part.
- `material`: either a `Material` object or a registered material name.
- `name`: optional visual name. Use this when tests or probe snippets need to
  target a local feature.

## Inertials

### `Inertia`

```python
Inertia(
    ixx: float,
    ixy: float,
    ixz: float,
    iyy: float,
    iyz: float,
    izz: float,
)
```

Stores the inertia tensor components for one part.

### `Inertial`

```python
Inertial(
    mass: float,
    inertia: Inertia,
    origin: Origin = Origin(),
)
```

### `Inertial.from_geometry(...)`

```python
Inertial.from_geometry(
    geometry: Box | Cylinder | Sphere | Mesh,
    mass: float,
    *,
    origin: Origin | None = None,
) -> Inertial
```

- `mass`: required positive mass.
- `origin`: optional inertial origin.
- Supports `Box`, `Cylinder`, and `Sphere`.
- Raises `ValidationError` for mesh geometry.

## Motion

### `MotionLimits`

```python
MotionLimits(
    effort: float = 1.0,
    velocity: float = 1.0,
    lower: float | None = None,
    upper: float | None = None,
)
```

- `effort`: positive effort limit.
- `velocity`: positive velocity limit.
- `lower`, `upper`: optional joint-position bounds.

### `MotionProperties`

```python
MotionProperties(
    damping: float | None = None,
    friction: float | None = None,
)
```

Both fields are optional.

### `ArticulationType`

```python
ArticulationType.REVOLUTE
ArticulationType.CONTINUOUS
ArticulationType.PRISMATIC
ArticulationType.FIXED
ArticulationType.FLOATING
```

Use these enum values when creating articulations.

## Parts and Articulations

### `Part`

```python
Part(
    name: str,
    visuals: list[Visual] = [],
    collisions: list[Collision] = [],
    inertial: Inertial | None = None,
    meta: dict[str, object] = {},
)
```

Recommended part authoring entry points:

```python
part.visual(
    geometry,
    *,
    origin: Origin | None = None,
    material: Material | str | None = None,
    name: str | None = None,
) -> Visual

part.get_visual(name: str) -> Visual
```

- Use `part.visual(...)` to append one named or unnamed visual.
- Use `part.get_visual(...)` when tests or probe tooling need a specific local
  feature.

### `Articulation`

```python
Articulation(
    name: str,
    articulation_type: ArticulationType | str,
    parent: str,
    child: str,
    origin: Origin = Origin(),
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
    motion_limits: MotionLimits | None = None,
    motion_properties: MotionProperties | None = None,
    meta: dict[str, object] = {},
)
```

- `parent`, `child`: part names.
- `origin`: transform from the parent part frame into the articulation frame.
- `axis`: motion axis for revolute, continuous, and prismatic joints,
  expressed in the articulation frame.
- At `q=0`, the child part frame is coincident with the articulation frame.
- Positive `REVOLUTE` and `CONTINUOUS` motion follows the right-hand rule
  around `axis`. Positive `PRISMATIC` motion translates the child along `axis`.
- `origin.rpy` rotates the articulation frame relative to the parent, so it
  also rotates the meaning of `axis`.
- If increasing joint position moves a lid, door, or flap into the parent
  instead of outward, negate `axis`.
- `motion_limits`: required for `REVOLUTE`, `PRISMATIC`, and `CONTINUOUS` with
  the usual constraints described in `30_articulated_object.md`.

For authored models, prefer `model.articulation(...)` rather than constructing
`Articulation` objects manually.

## Supported Public Fields On Resolved Objects

Objects returned by `model.get_part(...)`, `model.get_articulation(...)`, and
`part.get_visual(...)` are public SDK objects. In tests, probe snippets, and
other authored code, reading the following fields and aliases is supported.

### `Part`

- `part.name`
- `part.visuals`
- `part.collisions`
- `part.inertial`
- `part.meta`
- `part.assets`

### `Visual`

- `visual.name`
- `visual.geometry`
- `visual.origin`
- `visual.material`

### `Articulation`

- `joint.name`
- `joint.articulation_type`
- `joint.joint_type`
- `joint.parent`
- `joint.child`
- `joint.origin`
- `joint.axis`
- `joint.motion_limits`
- `joint.limit`
- `joint.motion_properties`
- `joint.dynamics`
- `joint.meta`

### `MotionLimits`

- `limits.effort`
- `limits.velocity`
- `limits.lower`
- `limits.upper`

Example:

```python
hinge = object_model.get_articulation("lid_hinge")
limits = hinge.motion_limits
if limits is not None and limits.lower is not None and limits.upper is not None:
    lower = limits.lower
    upper = limits.upper
```

## Resizing Helper

### `scale_geometry_to_size(...)`

```python
scale_geometry_to_size(
    geometry,
    target_size,
    *,
    mode="stretch",
    asset_root=None,
    filename=None,
)
```

- `geometry`: primitive geometry or `Mesh`.
- `target_size`: target extents `(x, y, z)`. Each axis may be a positive float
  or `None`.
- `mode`: `"stretch"` or `"uniform"`.
- `asset_root`: mesh root used when resizing mesh geometry.
- `filename`: required when a resized primitive can no longer remain a primitive
  and must be emitted as a mesh.

### Advice

- Use `mode="stretch"` when exact target extents matter more than preserving
  aspect ratio.
- Use `mode="uniform"` when the shape should stay proportional.
- Primitive resizing stays primitive only when the result is still representable
  as that primitive. For example, stretching a sphere unevenly requires
  `filename=...` so the helper can emit a mesh-backed result.

## Examples

```python
badge = Material(name="badge", rgba=(0.8, 0.2, 0.1, 1.0))

panel = Part(name="panel")
panel.visual(
    Box((0.10, 0.16, 0.003)),
    origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    material=badge,
    name="panel_shell",
)
```

```python
scaled = scale_geometry_to_size(
    Sphere(radius=0.02),
    (0.04, 0.06, 0.01),
    filename="assets/meshes/badge.obj",
)
```

## See Also

- `30_articulated_object.md` for model authoring helpers
- `50_placement.md` for mounting and surface wrapping helpers
- `80_testing.md` for test-time use of parts and named visuals

## Clarifications for agent usage

- `Origin` is always a local rigid transform: `xyz` is translation in meters and `rpy` is roll, pitch, yaw in radians using the right-hand rule.
- A part visual or collision is placed in its part-local frame with `origin=Origin(...)`; an articulation `origin` defines the child joint frame relative to the parent part frame.
- `Mesh(...)` requires either `filename` or `name`. Use `filename` for a concrete mesh asset path and `name` for managed/materialized mesh assets that will be resolved later.
- `Part.collisions` is not part of the authored source-model path for agent use. Author visuals; derived collision geometry is handled by validation/materialization.
- `ArticulationType.FLOATING` is supported but advanced. Its authored/test pose value is `Origin(...)`, where `xyz` is relative translation in the joint frame and `rpy` is relative rotation in that same frame.
- For `REVOLUTE`, `CONTINUOUS`, and `PRISMATIC`, author `axis` as a unit vector in the articulation frame. Positive revolute motion follows the right-hand rule around `+axis`; positive prismatic motion translates along `+axis`.
