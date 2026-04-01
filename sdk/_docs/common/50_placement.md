# Placement Helpers

## Purpose

Use these helpers when one thing needs to be aligned, mounted, or wrapped
relative to another.

## Import

```python
from sdk import (
    align_centers,
    place_on_surface,
    place_on_face,
    place_on_face_uv,
    proud_for_flush_mount,
    surface_frame,
    wrap_mesh_onto_surface,
    wrap_profile_onto_surface,
)
```

## Recommended APIs

| Intent | Helper |
| --- | --- |
| Align centers along selected axes | `align_centers(...)` |
| Mount a rigid child onto a target surface | `place_on_surface(...)` |
| Mount on a box-like face | `place_on_face(...)`, `place_on_face_uv(...)` |
| Compute flush-mount offset | `proud_for_flush_mount(...)` |
| Query a surface point and tangent frame | `surface_frame(...)` |
| Wrap existing mesh geometry onto curvature | `wrap_mesh_onto_surface(...)` |
| Wrap a 2D profile with thickness onto curvature | `wrap_profile_onto_surface(...)` |

## Subject Conventions

These helpers use a few shared input conventions:

- `Part`: uses the union of its visuals by default
- `Visual`: uses that visual only
- `Box`, `Cylinder`, `Sphere`, `Mesh`: uses the geometry directly
- `MeshGeometry`: accepted by wrapping helpers

`place_on_face(...)` and `proud_for_flush_mount(...)` require a `Part`.

## Alignment

### `align_centers(...)`

```python
align_centers(
    child_aabb,
    parent_aabb,
    *,
    axes=("x", "y", "z"),
) -> Origin
```

- `child_aabb`, `parent_aabb`: `(min_xyz, max_xyz)` AABB pairs.
- `axes`: one axis, a compact string such as `"xy"`, or an iterable of axis
  names.
- Returns an `Origin` whose translation aligns the chosen center coordinates.

## Face-Based Mounting

### `place_on_face(...)`

```python
place_on_face(
    parent_link: Part,
    face: str,
    *,
    face_pos: tuple[float, float] = (0.0, 0.0),
    proud: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = True,
) -> Origin
```

- `parent_link`: box-like parent part.
- `face`: one of `"+x"`, `"-x"`, `"+y"`, `"-y"`, `"+z"`, `"-z"`.
- `face_pos`: local face-plane offset in meters.
- `proud`: offset outward from the chosen face normal.
- Returns a rigid `Origin`.

### `place_on_face_uv(...)`

```python
place_on_face_uv(
    parent_link: Part,
    face: str,
    *,
    uv: tuple[float, float] = (0.5, 0.5),
    uv_margin: float | tuple[float, float] = 0.0,
    proud: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = True,
) -> Origin
```

- `uv`: normalized face coordinates in `[0, 1]`.
- `uv_margin`: inward margin, either one float or per-axis pair.
- Returns a rigid `Origin`.

### `proud_for_flush_mount(...)`

```python
proud_for_flush_mount(
    child_link: Part,
    *,
    axis: str = "z",
    clearance: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = True,
) -> float
```

- Returns the outward offset that keeps a centered child flush instead of
  half-embedded when used with `place_on_face(...)`.
- `axis`: child local thickness axis, one of `"x"`, `"y"`, or `"z"`.
- `clearance`: extra positive stand-off in meters.

## Surface Queries

### `surface_frame(...)`

```python
surface_frame(
    target,
    *,
    point_hint=None,
    direction=None,
    asset_root=None,
    prefer_collisions: bool = False,
    up_hint=(0.0, 0.0, 1.0),
) -> SurfaceFrame
```

```python
SurfaceFrame(
    point: tuple[float, float, float],
    normal: tuple[float, float, float],
    tangent_u: tuple[float, float, float],
    tangent_v: tuple[float, float, float],
)
```

- Provide exactly one of `point_hint` or `direction`.
- `point_hint`: world-space point near the target region of interest.
- `direction`: outward direction from the target's rough center.
- `up_hint`: preferred world-space up direction used when building tangents.
- Returns the hit point, normal, and tangent basis.

## Rigid Surface Mounting

### `place_on_surface(...)`

```python
place_on_surface(
    child,
    target,
    *,
    point_hint=None,
    direction=None,
    child_axis: str = "+z",
    clearance: float = 0.0,
    spin: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = False,
    child_prefer_collisions: bool = False,
    up_hint=(0.0, 0.0, 1.0),
) -> Origin
```

- `child_axis`: child local axis that should point outward from the target
  surface, for example `"+z"` or `"-y"`.
- `clearance`: extra stand-off along the surface normal in meters.
- `spin`: extra rotation around the surface normal in radians.
- Returns a rigid `Origin`.

## Conformal Surface Wrapping

### `wrap_mesh_onto_surface(...)`

```python
wrap_mesh_onto_surface(
    mesh,
    target,
    *,
    point_hint=None,
    direction=None,
    child_axis: str = "+z",
    visible_relief: float = 0.0,
    mapping: str = "auto",
    surface_max_edge: float | None = None,
    max_edge: float | None = None,
    spin: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = False,
    up_hint=(0.0, 0.0, 1.0),
) -> MeshGeometry
```

- `mesh`: `MeshGeometry` or `Mesh`.
- `target`: wrap target.
- `visible_relief`: offset the visible face outward from the target in meters.
- `mapping`:
  - `"auto"`: prefer intrinsic sphere/cylinder wrapping, else nearest-surface
  - `"intrinsic"`: require intrinsic sphere/cylinder wrapping
  - `"nearest"`: always use nearest-surface projection
- `surface_max_edge` / `max_edge`: optional pre-wrap subdivision target. Use
  only one name.

Returns baked `MeshGeometry`.

### `wrap_profile_onto_surface(...)`

```python
wrap_profile_onto_surface(
    profile,
    target,
    *,
    thickness: float,
    hole_profiles=(),
    point_hint=None,
    direction=None,
    visible_relief: float = 0.0,
    mapping: str = "auto",
    surface_max_edge: float | None = None,
    max_edge: float | None = None,
    spin: float = 0.0,
    asset_root=None,
    prefer_collisions: bool = False,
    up_hint=(0.0, 0.0, 1.0),
) -> MeshGeometry
```

- `profile`: 2D outer loop in local XY.
- `thickness`: positive thickness in meters.
- `hole_profiles`: optional 2D through-cut loops.
- The visible face is treated as local `z=0`; thickness extends inward along
  local `-z`.

Returns baked `MeshGeometry`.

## Advice

### Choosing rigid placement vs wrapping

- Use `place_on_surface(...)` when the child should remain rigid, such as a
  button, foot, tab, or knob.
- Use `wrap_mesh_onto_surface(...)` or `wrap_profile_onto_surface(...)` when
  the geometry must follow curvature across its area, such as a label, plaque,
  or thin trim piece.

### Choosing `place_on_surface(...)` vs `place_on_face(...)`

- Use `place_on_face(...)` only when the parent is truly box-like and the face
  itself is the semantic reference.
- Use `place_on_surface(...)` when the target surface is the source of truth,
  including curved or mesh-backed targets.

### Understanding wrapping controls

- `child_axis` says which local axis is the visible outward-facing direction.
- `mapping="intrinsic"` is appropriate when you care about shape preservation on
  spheres or cylinder sidewalls.
- Lower `surface_max_edge` when a wrapped result is visibly cutting chords
  through a curved surface.
- `visible_relief` is for small stand-off, not large rigid spacing.

## Examples

### Face mount

```python
origin = place_on_face_uv(
    panel,
    "+z",
    uv=(0.75, 0.5),
    proud=proud_for_flush_mount(button, axis="z", clearance=0.0005),
    asset_root=HERE,
)
```

### Rigid surface mount

```python
origin = place_on_surface(
    child=button,
    target=panel,
    point_hint=(0.12, 0.0, 0.04),
    child_axis="+z",
    clearance=0.0005,
    spin=0.2,
    asset_root=HERE,
)
```

### Conformal wrap

```python
wrapped = wrap_profile_onto_surface(
    profile,
    Sphere(radius=0.09),
    thickness=0.0015,
    direction=(1.0, 0.2, 0.1),
    mapping="intrinsic",
    visible_relief=0.00005,
    surface_max_edge=0.006,
)
```

## See Also

- `20_core_types.md` for geometry descriptors and `Origin`
- `40_mesh_geometry.md` for `MeshGeometry`

## Clarifications for agent usage

- `up_hint` does more than break ties: it defines the zero-spin tangent basis by projecting the hint into the surface tangent plane. Changing `up_hint` changes what `spin=0` means.
- Angle inputs are radians and follow the right-hand rule.
- Intrinsic wrapping support is intentionally narrow: use it for spheres and cylinder sidewalls, not arbitrary cylindrical end caps or general meshes.
