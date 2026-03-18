# Core Types

Import these from top-level `sdk`:

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
)
```

## Coordinates

- Distances are meters.
- Rotations are roll/pitch/yaw in radians.
- URDF cylinders are aligned with local `+Z`.

## Geometry

- `Box(size=(sx, sy, sz))`
- `Cylinder(radius=r, length=l)`
- `Sphere(radius=r)`
- `Mesh(filename="meshes/part.obj", scale=(sx, sy, sz) | None)`

Primitive geometry constructors describe shape only. Put transforms on the containing element.

## Material

Use the `Material` constructor directly when you want to create a material object yourself:

```python
steel = Material(name="steel", rgba=(0.55, 0.57, 0.60, 1.0))
glass = Material(name="glass", rgba=(0.72, 0.84, 0.92, 0.35))
```

Note:

- `Material(...)` uses `rgba=...`, not `color=...`.
- If you want the `color` alias, use `model.material(...)` on `ArticulatedObject` instead.

## Part

`Part` is the main authored geometry container.

Authoring is visual-first:

```python
part.visual(geometry, origin=Origin(...), material=..., name=...)
part.inertial = Inertial.from_geometry(geometry, mass=..., origin=Origin(...))
```

The `sdk` package no longer exposes collision-authoring helpers. Compile/export generates collisions automatically from visuals.

## Inertial

```python
Inertial.from_geometry(geometry, mass=..., origin=Origin(...) | None)
```

Supports `Box`, `Cylinder`, and `Sphere`.

## Articulation

```python
Articulation(
    name="hinge",
    articulation_type=ArticulationType.REVOLUTE,
    parent="base",
    child="lid",
    origin=Origin(...),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(...) | None,
    motion_properties=MotionProperties(...) | None,
)
```

Use:

- `ArticulationType.REVOLUTE`
- `ArticulationType.CONTINUOUS`
- `ArticulationType.PRISMATIC`
- `ArticulationType.FIXED`
- `ArticulationType.FLOATING`

Strict motion-limit rules:

- `REVOLUTE` and `PRISMATIC` articulations must set `motion_limits=MotionLimits(...)` with positive `effort` and `velocity`, plus both `lower` and `upper`.
- `CONTINUOUS` articulations must set `motion_limits=MotionLimits(effort=..., velocity=...)`.
- `CONTINUOUS` articulations must not set `lower` or `upper`.
- `FIXED` and `FLOATING` articulations do not use `motion_limits`.

Example continuous articulation:

```python
Articulation(
    name="mast_slew",
    articulation_type=ArticulationType.CONTINUOUS,
    parent="chassis",
    child="mast",
    origin=Origin(...),
    axis=(0.0, 0.0, 1.0),
    motion_limits=MotionLimits(effort=12.0, velocity=1.5),
)
```

For authored models, the canonical constructor is `model.articulation(...)`.
