# Procedural Meshes (`sdk.mesh`)

The SDK includes a lightweight procedural triangle-mesh builder. This is primarily used to generate `.obj` meshes that are then referenced from exported object assets via `sdk.Mesh(filename=...)`.

When a mesh is written under `assets/meshes/`, the SDK exports it as a relative reference like `assets/meshes/part.obj`. Attach `assets=AssetContext.from_script(__file__)` to the model or pass `asset_root=ASSETS.asset_root` to `TestContext(...)` so QC and tests can resolve it later.

Important naming distinction:

- `sdk.MeshGeometry` / `sdk.BoxGeometry` / ... are **triangle meshes** (procedural geometry).
- `sdk.Mesh` / `sdk.Box` / ... are **object geometry descriptors** used inside authored visuals and compiled collision geometry.

## Core types

Import:

```python
from sdk import MeshGeometry, BufferGeometry
```

- `MeshGeometry` is a dataclass:
  - `vertices: list[tuple[float, float, float]]`
  - `faces: list[tuple[int, int, int]]` (0-based triangle indices)
- `BufferGeometry` is an alias for `MeshGeometry`.

### `MeshGeometry` methods (mutating)

```python
geom.add_vertex(x, y, z) -> int
geom.add_face(a, b, c) -> None
geom.copy() -> MeshGeometry
geom.clone() -> MeshGeometry
geom.merge(other) -> MeshGeometry
geom.translate(dx, dy, dz) -> MeshGeometry
geom.scale(sx, sy=None, sz=None) -> MeshGeometry
geom.rotate(axis, angle_rad, origin=(0.0, 0.0, 0.0)) -> MeshGeometry
geom.rotate_x(angle_rad) -> MeshGeometry
geom.rotate_y(angle_rad) -> MeshGeometry
geom.rotate_z(angle_rad) -> MeshGeometry
geom.to_obj() -> str
geom.save_obj(path: str | Path) -> None
```

All transforms mutate the geometry in place and return `self` for chaining.

`rotate(...)` requires:

- `axis`: a non-zero 3D direction `(x, y, z)`
- `angle_rad`: the rotation angle in radians
- `origin`: optional pivot point, default `(0.0, 0.0, 0.0)`

`rotate_x(...)`, `rotate_y(...)`, and `rotate_z(...)` are convenience wrappers over `rotate(...)`.

`copy()` / `clone()` return a deep copy and are useful when you want to repeat a
mesh pattern (for example spokes, whisk loops, cages, or repeated trim pieces)
without re-generating the source geometry.

## Primitive mesh builders

Import:

```python
from sdk import (
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    ConeGeometry,
    DomeGeometry,
    SphereGeometry,
    TorusGeometry,
)
```

Signatures:

- `BoxGeometry(size: Sequence[float])`
- `CapsuleGeometry(radius: float, length: float, radial_segments: int = 24, height_segments: int = 8)`
- `CylinderGeometry(radius: float, height: float, radial_segments: int = 24, closed: bool = True)`
- `ConeGeometry(radius: float, height: float, radial_segments: int = 24, closed: bool = True)`
- `DomeGeometry(radius: float | (rx, ry, rz), radial_segments: int = 24, height_segments: int = 12, closed: bool = True)`
- `SphereGeometry(radius: float, width_segments: int = 24, height_segments: int = 16)`
- `TorusGeometry(radius: float, tube: float, radial_segments: int = 16, tubular_segments: int = 32)`

Notes:

- `CapsuleGeometry` is centered at the origin, aligned with Z, and uses `length` for the cylindrical mid-section between the spherical caps.
- `CylinderGeometry` and `ConeGeometry` are centered at the origin and extend along Z.
- `DomeGeometry` builds the upper half of a sphere/ellipsoid. Its base lies on `z=0`, and the dome extends toward `+Z`.
- `SphereGeometry` is centered at the origin.

## Profile/loft/extrude helpers

These helpers are useful for “CAD-like” parts such as casings, panels, and knobs.

Import:

```python
from sdk import (
    LatheGeometry,
    LoftGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    SweepGeometry,
)
```

### `LatheGeometry`

```python
LatheGeometry(profile, segments=32, closed=True)
```

- `profile` is an iterable of `(r, z)` points.
- Radii must be non-negative.
- With `closed=True` (default), the profile is treated as a closed loop in the
  `(r, z)` half-plane and revolved into a closed solid around the Z axis.
- Profiles that touch the axis are welded into shared apex/axis vertices so the
  resulting mesh is suitable for solid booleans.
- Pass `closed=False` for the legacy open surface-of-revolution behavior.

Open-front hollow shade example:

```python
from sdk import LatheGeometry

shade = LatheGeometry(
    [
        (0.0, -0.030),   # rear centerline
        (0.018, -0.030), # outer rear wall
        (0.023, -0.022),
        (0.027, -0.010),
        (0.031, 0.010),
        (0.036, 0.027),
        (0.039, 0.040),  # front rim outer edge
        (0.033, 0.0385), # front rim inner edge
        (0.027, 0.018),
        (0.022, -0.002),
        (0.018, -0.015),
        (0.014, -0.021),
        (0.014, -0.025), # inner rear wall
        (0.0, -0.025),   # back to centerline only at the rear
    ],
    segments=44,
)
```

- This pattern creates a thin-walled cone-like shade with an open front and a
  hollow interior.
- For shade-like forms, prefer one closed wall loop like this over
  `boolean_difference(outer, inner)` when both revolved profiles would otherwise
  run to the axis at the open end, because that produces a capped nose instead
  of an aperture.

### `LoftGeometry`

```python
LoftGeometry(profiles, cap=True, closed=True)
```

- `LoftGeometry` is now considered a low-level legacy primitive.
- For new shell/exterior loft authoring, prefer `section_loft(...)` and
  `repair_loft(...)`. See `46_section_lofts.md`.
- `profiles` is an iterable of profiles, each profile an iterable of `(x, y, z)`.
- All profiles must have the same point count.
- If `closed=True`, the profile is treated as a closed loop (wraps the last segment).
- Each profile should usually be a non-degenerate closed loop in the XY plane at constant `z`.
- Area validation is done in the XY projection, so profiles like `(x_i, y_const, z_i)` or `(x_const, y_i, z_i)` collapse and fail.
- If you want an XZ- or YZ-oriented section, author the loop in XY first, then rotate the resulting mesh.
- If `cap=True` and `closed=True`, the first and last profiles are capped. Caps require planar profiles (constant z within a small tolerance).
- Cap triangulation uses the `manifold3d` triangulation backend for better
  robustness on concave outlines.

### `ExtrudeGeometry`

```python
ExtrudeGeometry(profile, height, cap=True, center=True, closed=True)
ExtrudeGeometry.centered(profile, height, cap=True, closed=True)
ExtrudeGeometry.from_z0(profile, height, cap=True, closed=True)
```

- Extrudes a 2D `(x, y)` profile along Z.
- If `center=True`, extrusion spans `[-height/2, +height/2]`; otherwise it spans `[0, height]`.
- Capped closed extrusions use the same cap triangulation path as `LoftGeometry`.
- For thin parts that need to hug curved targets, prefer `wrap_profile_onto_surface(...)`
  from `50_placement.md` over manually extruding first and then reasoning about
  rigid placement.

### `ExtrudeWithHolesGeometry`

```python
ExtrudeWithHolesGeometry(
    outer_profile,
    hole_profiles,
    height,
    cap=True,
    center=True,
    closed=True,
)
```

- Extrudes an outer 2D profile with one or more through-holes.
- Hole loops must lie inside the outer profile.
- With `cap=True` and `closed=True`, the helper uses manifold-backed polygon
  triangulation for the top and bottom caps when available.
- Hole winding is normalized internally. You do not need to reverse hole loops.
- If `cap=False` or `closed=False`, the helper falls back to side walls only.
- This pairs naturally with `wrap_profile_onto_surface(..., hole_profiles=...)`
  when you need a wrapped badge, vent, or panel cutout on a curved target.

### `SweepGeometry`

```python
SweepGeometry(profile, path, cap=False, closed=True)
```

- Sweeps a 2D profile (in XY) along a polyline `path` of `(x, y, z)` points.
- This sweep is translation-only (no “banking”/orientation along the path).
- For tube-like parts that should follow the path tangent, prefer
  `tube_from_spline_points(...)`, `sweep_profile_along_spline(...)`,
  `wire_from_points(...)`, or `PipeGeometry`.

## Parametric profiles and organic casings

These helpers remain available, but for new general-purpose lofted shells prefer
`section_loft(...)` and `repair_loft(...)` from `46_section_lofts.md`.

Import:

```python
from sdk import (
    sample_catmull_rom_spline_2d,
    sample_cubic_bezier_spline_2d,
    rounded_rect_profile,
    superellipse_profile,
    superellipse_side_loft,
    split_superellipse_side_loft,
    resample_side_sections,
)
```

### `sample_catmull_rom_spline_2d`

```python
sample_catmull_rom_spline_2d(points, samples_per_segment=12, closed=False, alpha=0.5) -> list[(x, y)]
```

- Fits a smooth interpolating spline through the input points.
- This is the recommended default when you have clicked/estimated points and
  want a smooth outline without managing Bezier handles.
- `closed=True` returns a closed loop with the first point repeated at the end.

### `sample_cubic_bezier_spline_2d`

```python
sample_cubic_bezier_spline_2d(control_points, samples_per_segment=12) -> list[tuple[float, float]]
```

- Advanced helper for explicit Bezier handle control.
- `control_points` is a chained cubic layout: `[P0, P1, P2, P3, P4, P5, P6, ...]`
- Must satisfy:
  - at least 4 points
  - `(n - 1) % 3 == 0`

### `rounded_rect_profile`

```python
rounded_rect_profile(width, height, radius, corner_segments=6) -> list[(x, y)]
```

Returns a CCW outline centered at the origin.

### `superellipse_profile`

```python
superellipse_profile(width, height, exponent=2.6, segments=48) -> list[(x, y)]
```

Returns a CCW outline centered at the origin.

### `superellipse_side_loft`

```python
superellipse_side_loft(
    sections,                          # (y, z_min, z_max, width)
    exponents=2.8,                     # float or one-per-section
    segments=56,
    cap=True,
    closed=True,
    min_height=1e-4,
    min_width=1e-4,
) -> MeshGeometry
```

Builds an “organic casing” by lofting superellipse cross-sections along +Y, where each section is:

- `y`: position along the loft axis
- `z_min`, `z_max`: vertical extent
- `width`: span in X

This helper is specialized and remains useful for compatibility and simple
casing forms. It is no longer the recommended general loft API.

### `split_superellipse_side_loft`

```python
split_superellipse_side_loft(
    sections,
    split_y=...,
    exponents=2.8,
    segments=56,
    cap=True,
    closed=True,
    min_height=1e-4,
    min_width=1e-4,
) -> (rear_geom, front_geom, seam_section)
```

Splits one continuous side loft into two watertight meshes at `split_y`, returning:

- `rear_geom: MeshGeometry`
- `front_geom: MeshGeometry`
- `seam_section: (y, z_min, z_max, width)` used by both halves

### `resample_side_sections`

```python
resample_side_sections(
    sections,
    samples_per_span=2,
    smooth_passes=0,
    min_height=1e-4,
    min_width=1e-4,
) -> list[(y, z_min, z_max, width)]
```

Densifies and optionally smooths a section list (useful for noisy/sparse rails).

## Panels and openings

Import:

```python
from sdk import LouverPanelGeometry, cut_opening_on_face
```

### `LouverPanelGeometry`

```python
LouverPanelGeometry(
    panel_size,             # (width, height)
    thickness,
    frame=0.008,
    slat_pitch=0.024,
    slat_width=0.010,
    slat_angle_deg=32.0,
    corner_radius=0.004,
    center=True,
    fin_thickness=None,
)
```

Builds a rectangular panel (XY) extruded along Z with slot cutouts and angled fins.

### `cut_opening_on_face`

```python
cut_opening_on_face(
    shell_geometry,
    face="+x" | "-x" | "+y" | "-y" | "+z" | "-z",
    opening_profile=...,     # 2D (u, v) profile in face tangent coordinates
    depth=...,
    offset=(0.0, 0.0),
    taper=0.0,
) -> MeshGeometry
```

Adds an “opening throat” by lofting side walls inward from the selected outer face.

- This does **not** boolean-subtract material.
- It merges the new wall geometry into `shell_geometry`.

## Boolean operations

Import:

```python
from sdk import boolean_union, boolean_difference, boolean_intersection
```

Signatures:

```python
boolean_union(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry
boolean_difference(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry
boolean_intersection(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry
```

Notes:

- These require `manifold3d`.
- Both inputs must be manifold solids; the helper will raise if the mesh is not a valid solid for boolean ops.

## Writing meshes and referencing them in compiled exports

Import:

```python
from sdk import Mesh, mesh_from_geometry
```

### `mesh_from_geometry`

```python
mesh = mesh_from_geometry(geometry, filename=".../assets/meshes/part.obj")  # -> sdk.Mesh
```

`mesh_from_geometry(...)` accepts either a string path or a `pathlib.Path`:

```python
from pathlib import Path

mesh_dir = Path(__file__).resolve().parent / "assets" / "meshes"
mesh = mesh_from_geometry(geometry, mesh_dir / "part.obj")
```

Behavior:

- Saves the OBJ file to `filename`.
- Returns a `Mesh(filename=...)` reference.
- If the output path lives under `assets/meshes`, the returned `Mesh.filename` is made relative to that subtree, e.g.:
  - `/abs/.../assets/meshes/part.obj` -> `assets/meshes/part.obj`

This is the recommended way to ensure portable script-relative mesh filenames.

Recommended pattern:

```python
from sdk import AssetContext, ArticulatedObject, BoxGeometry, mesh_from_geometry

ASSETS = AssetContext.from_script(__file__)
model = ArticulatedObject("example", assets=ASSETS)
mesh = mesh_from_geometry(BoxGeometry((0.1, 0.2, 0.3)), str(ASSETS.mesh_path("part.obj")))
```
