# ArticulatedObject

## Purpose

`ArticulatedObject` is the root authored assembly. Use it to create parts,
articulations, materials, and model-level metadata.

## Import

```python
from sdk import ArticulatedObject
```

## Recommended Surface

- `ArticulatedObject(...)`
- `model.part(...)`
- `model.articulation(...)`
- `model.material(...)`
- `model.get_part(...)`
- `model.get_articulation(...)`
- `model.root_parts()`
- `model.validate(strict=True)`

## Construction

```python
ArticulatedObject(
    name: str,
    parts: list[Part] = [],
    articulations: list[Articulation] = [],
    materials: list[Material] = [],
    meta: dict[str, object] = {},
)
```

Important fields:

- `name`: model name.
- `parts`: authored parts.
- `articulations`: authored articulations.
- `materials`: registered materials.
- `meta`: optional model-level metadata.
- Mesh assets are runtime-managed. Author mesh-backed visuals through helpers
  such as `mesh_from_geometry(...)`, `mesh_from_input(...)`, and
  `mesh_from_cadquery(...)`.

## Authoring Helpers

### `model.part(...)`

```python
model.part(
    name: str,
    *,
    visuals: Iterable[Visual] | None = None,
    inertial: Inertial | None = None,
    meta: dict[str, object] | None = None,
) -> Part
```

- Creates a `Part`, appends it to `model.parts`, and returns it.
- The returned part is the normal place to call `part.visual(...)`.

### `model.articulation(...)`

```python
model.articulation(
    name: str,
    articulation_type: ArticulationType | str,
    parent: str | Part,
    child: str | Part,
    *,
    origin: Origin | None = None,
    axis: tuple[float, float, float] | None = None,
    motion_limits: MotionLimits | None = None,
    motion_properties: MotionProperties | None = None,
    meta: dict[str, object] | None = None,
) -> Articulation
```

- `parent`, `child`: either part objects or part names.
- `origin`: transform from the parent part frame into the articulation frame.
  Defaults to `Origin()`.
- `axis`: motion axis expressed in the articulation frame. Defaults to
  `(0.0, 0.0, 1.0)`.
- `motion_limits`: joint limits where required.
- `motion_properties`: optional damping and friction.

## Frame And Direction Conventions

Articulations use a URDF-style joint frame:

1. `origin` places the articulation frame relative to the parent part frame.
2. `axis` is written in that articulation frame, not in world space.
3. At `q=0`, the child part frame is coincident with the articulation frame.
4. Positive `REVOLUTE` and `CONTINUOUS` motion follows the right-hand rule
   around `axis`.
5. Positive `PRISMATIC` motion translates the child along `+axis`.

In practice, the usual mistake is choosing the correct hinge line but the wrong
axis sign. If increasing the joint value makes the child close into the parent,
negate `axis` instead of swapping `lower` and `upper`.

### Example: Lid That Should Open Upward

```python
# Closed lid geometry extends along local +X from the hinge line.
# Using -Y makes positive q lift the free edge toward +Z.
model.articulation(
    "body_to_lid",
    ArticulationType.REVOLUTE,
    parent=body,
    child=lid,
    origin=Origin(xyz=(-0.09, 0.0, 0.05)),
    axis=(0.0, -1.0, 0.0),
    motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
)
```

### Example: Mirrored Lid With The Same "Positive Means Open" Convention

```python
# If the closed panel extends along local -X from the hinge line instead,
# flip the axis sign so positive q still opens upward/outward.
model.articulation(
    "body_to_mirrored_lid",
    ArticulationType.REVOLUTE,
    parent=body,
    child=mirrored_lid,
    origin=Origin(xyz=(0.09, 0.0, 0.05)),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
)
```

### Example: Drawer That Should Extend Outward

```python
# Positive prismatic q moves the drawer out along +X.
model.articulation(
    "cabinet_to_drawer",
    ArticulationType.PRISMATIC,
    parent=cabinet,
    child=drawer,
    origin=Origin(xyz=(0.0, 0.0, 0.12)),
    axis=(1.0, 0.0, 0.0),
    motion_limits=MotionLimits(effort=40.0, velocity=0.25, lower=0.0, upper=0.28),
)
```

### Retained Insertion For Prismatic And Nested Stages

For sleeves, telescoping poles, nested rails, and similar slide-in-slide
assemblies, size the moving member for the fully extended pose, not just the
collapsed silhouette. If one part slides out of another, model the sliding
member with enough hidden length that it still remains engaged at the joint
upper limit.

Use this heuristic:

`sliding member length >= visible exposed length at max extension + minimum retained insertion`

In practice:

- put the prismatic `origin` at the sleeve entry, socket lip, or other nominal
  seating plane rather than at an arbitrary part center
- choose `motion_limits.upper` as the usable travel after preserving retained
  insertion, not the full member length
- let the child geometry extend past the joint frame in the hidden direction if
  that is what the real mechanism requires

```python
# The inner mast extends below the visible seat so it stays engaged at max travel.
outer_sleeve = model.part("outer_sleeve")
inner_mast = model.part("inner_mast")

outer_sleeve.visual(
    Cylinder(radius=0.022, length=0.240),
    origin=Origin(xyz=(0.0, 0.0, 0.120)),
)
inner_mast.visual(
    Cylinder(radius=0.016, length=0.620),
    origin=Origin(xyz=(0.0, 0.0, 0.110)),
)

model.articulation(
    "sleeve_to_mast",
    ArticulationType.PRISMATIC,
    parent=outer_sleeve,
    child=inner_mast,
    # Place the joint at the sleeve entry / seating plane.
    origin=Origin(xyz=(0.0, 0.0, 0.240)),
    axis=(0.0, 0.0, 1.0),
    # Upper travel stops before the inner mast fully exits the sleeve.
    motion_limits=MotionLimits(lower=0.0, upper=0.260, effort=80.0, velocity=0.20),
)
```

### `model.material(...)`

```python
model.material(
    name: str,
    *,
    rgba: Sequence[float] | None = None,
    color: Sequence[float] | None = None,
    texture: str | None = None,
) -> Material
```

- Registers the material on `model.materials`.
- Use either `rgba=...` or `color=...`, not both.
- A 3-tuple color is expanded to `(r, g, b, 1.0)`.

## Lookup Helpers

### `model.get_part(name) -> Part`

```python
model.get_part(name: str | Part) -> Part
```

Returns the named part. Raises `ValidationError` if it does not exist.

### `model.get_articulation(name) -> Articulation`

```python
model.get_articulation(name: str | Articulation) -> Articulation
```

Returns the named articulation. Raises `ValidationError` if it does not exist.

### `model.root_parts() -> list[Part]`

Returns all parts that are not the child of another articulation.

## Validation

### `model.validate(...)`

```python
model.validate(
    *,
    strict: bool = True,
    strict_mesh_paths: bool = False,
) -> None
```

Validation includes:

- at least one part exists
- part names are unique
- articulation names are unique
- material names are unique
- articulations reference existing parent and child parts
- each part has at most one parent articulation
- geometry and materials are valid
- in strict mode, the articulation graph is connected from at least one root

## Joint Rules

Use these rules when calling `model.articulation(...)`:

- `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC` require
  `MotionLimits(effort=..., velocity=..., lower=..., upper=...)`.
- `ArticulationType.CONTINUOUS` requires
  `MotionLimits(effort=..., velocity=...)` and must not set `lower` or `upper`.
- `ArticulationType.FIXED` and `ArticulationType.FLOATING` must not use
  `motion_limits`.
- In strict mode, articulated motion axes must be non-zero 3-vectors.

## Example

```python
model = ArticulatedObject(name="desk_lamp")

base = model.part("base")
arm = model.part("arm")

model.articulation(
    "base_to_arm",
    ArticulationType.REVOLUTE,
    parent=base,
    child=arm,
    origin=Origin(xyz=(0.0, 0.0, 0.08)),
    # Arm geometry extends along local +X from the shoulder, so -Y makes
    # positive q pitch the arm upward.
    axis=(0.0, -1.0, 0.0),
    motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.8, upper=0.8),
)
```

## See Also

- `20_core_types.md` for `Part`, `Articulation`, `MotionLimits`, and materials
- `80_testing.md` for geometry and articulation QC

## Clarifications for agent usage

- `origin` defines the joint frame in the parent part frame. The child part then moves relative to that joint frame.
- `axis` is expressed in the articulation frame after applying `origin.rpy`, not in world space. Author it as a unit vector.
- `FLOATING` articulations are advanced and do not use `axis` or scalar motion limits. In testing/probe tooling, move them with `Origin(xyz=..., rpy=...)`, not a scalar.
- If increasing a scalar joint value moves the child in the wrong direction, negate `axis`; do not swap semantic open/close meaning by flipping limits.
