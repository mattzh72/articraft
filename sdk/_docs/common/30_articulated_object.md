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
- limits and axes are valid in strict mode
- the articulation graph is connected from at least one root part

## Overlap validation

```python
model.validate_geometry(
    asset_root=HERE,
    geometry_source="collision",   # or "visual"
    max_pose_samples=256,
    overlap_tol=None,
    overlap_volume_tol=None,
    ignore_adjacent=True,
    ignore_fixed=True,
    allowed_pairs=None,
    seed=0,
)
```

Use `geometry_source="collision"` for physical QC. In `sdk`, collision geometry is generated automatically from visuals during compile/test setup.
