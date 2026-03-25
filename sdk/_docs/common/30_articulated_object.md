# ArticulatedObject

Import:

```python
from sdk import ArticulatedObject
```

`ArticulatedObject` is the root authored assembly.

## Construction

```python
model = ArticulatedObject(name="desk_lamp")
```

Fields:

- `name: str`
- `parts: list[Part]`
- `articulations: list[Articulation]`
- `materials: list[Material]`
- `meta: dict[str, object]`

## Authoring API

Create parts with:

```python
base = model.part("base")
lid = model.part("lid")
```

Create motion with:

```python
model.articulation(
    "base_to_lid",
    ArticulationType.REVOLUTE,
    parent="base",
    child="lid",
    origin=Origin(...),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(...),
)
```

Joint-limit rules:

- `revolute` and `prismatic` articulations require `MotionLimits` with `effort`, `velocity`, `lower`, and `upper`.
- `continuous` articulations require `MotionLimits` with `effort` and `velocity`, and must not set `lower` or `upper`.
- `fixed` and `floating` articulations should not set `motion_limits`.

Example continuous articulation:

```python
model.articulation(
    "chassis_to_mast",
    ArticulationType.CONTINUOUS,
    parent="chassis",
    child="mast",
    origin=Origin(...),
    axis=(0.0, 0.0, 1.0),
    motion_limits=MotionLimits(effort=12.0, velocity=1.5),
)
```

Lookup helpers:

```python
model.get_part("base")
model.get_articulation("base_to_lid")
model.root_parts()
```

Create and register named materials with:

```python
steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
glass = model.material("glass", color=(0.72, 0.84, 0.92, 0.35))

base.visual(Box((0.20, 0.20, 0.05)), material=steel)
lid.visual(Box((0.18, 0.18, 0.02)), material="glass")
```

Notes:

- `model.material(...)` registers the material on `model.materials`.
- `model.material(...)` accepts either `rgba=...` or `color=...`.
- The lower-level `Material(...)` constructor uses `rgba=...` and does not accept `color=...`.

## Validation

```python
model.validate(strict=True)
```

This checks:

- at least one part exists
- part and articulation names are unique
- every articulation references existing parts
- axes are valid in strict mode
- `revolute` and `prismatic` articulations include `effort`, `velocity`, `lower`, and `upper`
- `continuous` articulations include `effort` and `velocity`, but no `lower` or `upper`
- the articulation graph is connected from at least one root part

## Geometry validation

Use `TestContext` inside `run_tests()` for geometry and articulation QC.

- Keep `ctx.check_model_valid()`, `ctx.check_mesh_files_exist()`, `ctx.check_part_geometry_connected()`, and `ctx.check_no_part_overlaps()` as the default scaffold gates.
- Use prompt-specific `expect_*` assertions as the primary proof that parts are attached and the mechanism behaves correctly.
- Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is genuinely uncertain or mechanically important.
- `ctx.warn_if_articulation_origin_near_geometry(...)` and `ctx.warn_if_overlaps(...)` are deprecated as blanket scaffold defaults in new generated code.
