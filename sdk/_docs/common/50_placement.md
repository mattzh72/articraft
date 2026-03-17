# Placement Helpers

Import:

```python
from sdk import (
    align_centers_xy,
    part_local_aabb,
    place_in_front_of,
    place_on_face,
    place_on_face_uv,
    place_on_top,
    proud_for_flush_mount,
)
```

These helpers compute `Origin` placements for mounting one part relative to another.

## Common helpers

- `align_centers_xy(child_aabb, parent_aabb) -> Origin`
- `part_local_aabb(part, asset_root=...) -> AABB | None`
- `place_on_top(child_aabb, parent_aabb, clearance=0.0, align_xy=True) -> Origin`
- `place_in_front_of(child_aabb, parent_aabb, gap=0.0, align_yz=True) -> Origin`

## Face mounting

```python
origin = place_on_face(
    parent_part,
    "+x",
    face_pos=(0.03, 0.04),
    proud=0.005,
    asset_root=HERE,
)
```

Normalized face coordinates:

```python
origin = place_on_face_uv(
    parent_part,
    "+x",
    uv=(0.8, 0.5),
    proud=0.005,
    asset_root=HERE,
)
```

## Flush mounting

```python
proud = proud_for_flush_mount(child_part, axis="z", clearance=0.001, asset_root=HERE)
```

This computes the offset needed so a mounted child sits flush instead of being embedded in its parent.
