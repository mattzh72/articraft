# Procedural Meshes (`sdk.mesh`)

## Purpose

Use these helpers when the visible shape should be authored as procedural mesh
geometry and then exported as an OBJ-backed `sdk.Mesh`.

## Import

```python
from sdk import (
    MeshGeometry,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    ConeGeometry,
    DomeGeometry,
    SphereGeometry,
    TorusGeometry,
    LatheGeometry,
    LoftGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    VentGrilleGeometry,
    SweepGeometry,
    rounded_rect_profile,
    superellipse_profile,
    sample_catmull_rom_spline_2d,
    sample_cubic_bezier_spline_2d,
    superellipse_side_loft,
    split_superellipse_side_loft,
    resample_side_sections,
    cut_opening_on_face,
    mesh_from_geometry,
)
```

## Recommended Surface

- `MeshGeometry`
- primitive builders: `BoxGeometry`, `CylinderGeometry`, `ConeGeometry`,
  `SphereGeometry`, `DomeGeometry`, `CapsuleGeometry`, `TorusGeometry`
- loft/extrude/sweep builders: `LatheGeometry`, `ExtrudeGeometry`,
  `ExtrudeWithHolesGeometry`, `VentGrilleGeometry`, `SweepGeometry`
- profile helpers: `rounded_rect_profile`, `superellipse_profile`
- shell helpers: `superellipse_side_loft`, `split_superellipse_side_loft`,
  `resample_side_sections`
- `cut_opening_on_face(...)`
- `mesh_from_geometry(...)`

## Core Mesh Type

### `MeshGeometry`

```python
MeshGeometry(
    vertices: list[tuple[float, float, float]] = [],
    faces: list[tuple[int, int, int]] = [],
)
```

Methods:

```python
geom.add_vertex(x, y, z) -> int
geom.add_face(a, b, c) -> None
geom.copy() -> MeshGeometry
geom.clone() -> MeshGeometry
geom.merge(other) -> MeshGeometry
geom.translate(dx, dy, dz) -> MeshGeometry
geom.scale(sx, sy=None, sz=None) -> MeshGeometry
geom.rotate(axis, angle, origin=(0.0, 0.0, 0.0)) -> MeshGeometry
geom.rotate_x(angle) -> MeshGeometry
geom.rotate_y(angle) -> MeshGeometry
geom.rotate_z(angle) -> MeshGeometry
geom.to_obj() -> str
geom.save_obj(path) -> None
```

- Vertices are 3D points in meters.
- Faces are 0-based triangle indices.
- All transforms mutate in place and return `self`.

## Primitive Builders

```python
BoxGeometry(size)
CylinderGeometry(radius, height, *, radial_segments=24, closed=True)
ConeGeometry(radius, height, *, radial_segments=24, closed=True)
SphereGeometry(radius, *, width_segments=24, height_segments=16)
DomeGeometry(radius, *, radial_segments=24, height_segments=12, closed=True)
CapsuleGeometry(radius, length, *, radial_segments=24, height_segments=8)
TorusGeometry(radius, tube, *, radial_segments=16, tubular_segments=32)
```

Notes:

- `BoxGeometry` is centered at the origin.
- `CylinderGeometry` and `ConeGeometry` are centered and extend along local `Z`.
- `CapsuleGeometry.length` is the cylindrical mid-section length between caps.
- `DomeGeometry` builds the upper half with its base on `z=0`.

## Loft / Extrude / Sweep Builders

### `LatheGeometry`

```python
LatheGeometry(profile, *, segments=32, closed=True)
LatheGeometry.from_shell_profiles(
    outer_profile,
    inner_profile,
    *,
    segments=32,
    start_cap="flat",
    end_cap="flat",
    lip_samples=6,
)
```

- `profile`: iterable of `(radius, z)` points.
- Radii must be non-negative.
- `from_shell_profiles(...)` is the recommended entry point for thin-walled
  revolved shells.

### `LoftGeometry`

```python
LoftGeometry(profiles, *, cap=True, closed=True)
```

- `profiles`: iterable of 3D point loops.
- All profiles must have the same point count.
- This is a low-level mesh loft helper. Prefer `section_loft(...)` for new
  authored shell work.

### `ExtrudeGeometry`

```python
ExtrudeGeometry(profile, height, *, cap=True, center=True, closed=True)
ExtrudeGeometry.centered(profile, height, *, cap=True, closed=True)
ExtrudeGeometry.from_z0(profile, height, *, cap=True, closed=True)
```

- `profile`: 2D closed profile in local XY.
- `height`: positive extrusion length along Z.

### `ExtrudeWithHolesGeometry`

```python
ExtrudeWithHolesGeometry(
    outer_profile,
    hole_profiles,
    height,
    *,
    cap=True,
    center=True,
    closed=True,
)
```

- `outer_profile`: outer 2D loop.
- `hole_profiles`: zero or more through-cut loops inside the outer profile.

### `VentGrilleGeometry`

```python
VentGrilleGeometry(
    panel_size,
    *,
    frame: float = 0.012,
    face_thickness: float = 0.004,
    duct_depth: float = 0.026,
    duct_wall: float = 0.003,
    slat_pitch: float = 0.018,
    slat_width: float = 0.009,
    slat_angle_deg: float = 35.0,
    slat_thickness: float | None = None,
    corner_radius: float = 0.0,
)
```

- Builds a vent grille shell with true open slots, fused slats, and a shallow
  rear sleeve.
- `panel_size`: `(width, height)` of the front flange.
- The front flange is centered on local `z=0`; the sleeve extends toward `-Z`.
- `slat_pitch` must be greater than `slat_width`.
- `slat_angle_deg` uses degrees.
- This helper is intended for complete vent/register-style shells rather than
  a standalone louver insert panel.

### `SweepGeometry`

```python
SweepGeometry(profile, path, *, cap=False, closed=True)
```

- `profile`: 2D profile.
- `path`: 3D path points.
- Use this only for simple translational sweeps. For tube- and rail-like parts,
  see `45_wires.md`.

## Profile and Shell Helpers

### `rounded_rect_profile(...)`

```python
rounded_rect_profile(
    width: float,
    height: float,
    radius: float,
    *,
    corner_segments: int = 6,
) -> list[tuple[float, float]]
```

### `superellipse_profile(...)`

```python
superellipse_profile(
    width: float,
    height: float,
    exponent: float = 2.6,
    *,
    segments: int = 48,
) -> list[tuple[float, float]]
```

### `sample_catmull_rom_spline_2d(...)`

```python
sample_catmull_rom_spline_2d(
    points,
    *,
    samples_per_segment: int = 12,
    closed: bool = False,
    alpha: float = 0.5,
) -> list[tuple[float, float]]
```

### `sample_cubic_bezier_spline_2d(...)`

```python
sample_cubic_bezier_spline_2d(
    control_points,
    *,
    samples_per_segment: int = 12,
) -> list[tuple[float, float]]
```

### `superellipse_side_loft(...)`

```python
superellipse_side_loft(
    sections,
    *,
    exponents=2.8,
    segments: int = 56,
    cap: bool = True,
    closed: bool = True,
    min_height: float = 0.0001,
    min_width: float = 0.0001,
) -> MeshGeometry
```

### `split_superellipse_side_loft(...)`

```python
split_superellipse_side_loft(
    sections,
    *,
    split_y: float,
    exponents=2.8,
    segments: int = 56,
    cap: bool = True,
    closed: bool = True,
    min_height: float = 0.0001,
    min_width: float = 0.0001,
) -> tuple[MeshGeometry, MeshGeometry, tuple[float, float, float, float]]
```

### `resample_side_sections(...)`

```python
resample_side_sections(
    sections,
    *,
    samples_per_span: int = 2,
    smooth_passes: int = 0,
    min_height: float = 0.0001,
    min_width: float = 0.0001,
) -> list[tuple[float, float, float, float]]
```

## Panel Openings

### `cut_opening_on_face(...)`

```python
cut_opening_on_face(
    shell_geometry: MeshGeometry,
    *,
    face: str,
    opening_profile,
    depth: float,
    offset=(0.0, 0.0),
    taper: float = 0.0,
) -> MeshGeometry
```

- Cuts an opening into the chosen box-like face of an existing mesh shell.
- `face`: one of `"+x"`, `"-x"`, `"+y"`, `"-y"`, `"+z"`, `"-z"`.

## Exporting to `sdk.Mesh`

### `mesh_from_geometry(...)`

```python
mesh_from_geometry(
    geometry: MeshGeometry,
    name: str,
) -> Mesh
```

- Materializes the mesh to an internal OBJ managed by the runtime.
- Returns an `sdk.Mesh` descriptor pointing at the managed asset.
- Use this when the final authored visual should be mesh-backed.

## Advice

### Mutating behavior

- `MeshGeometry` transforms mutate in place.
- Call `copy()` or `clone()` before reusing a base mesh in multiple variants.

### Choosing higher-level helpers

- Prefer `section_loft(...)` over raw `LoftGeometry(...)` for new shell/exterior
  loft authoring.
- Prefer the spline-first wire/tube guidance in `45_wires.md` over manual
  sweeps for rails, loops, and frames. In practice, start with
  `tube_from_spline_points(...)` or `sweep_profile_along_spline(...)` unless
  the geometry is intentionally hard-cornered.

### Exporting mesh-backed visuals

- Use procedural meshes to author visible shape.
- Convert the final mesh to `sdk.Mesh` with `mesh_from_geometry(...)`.
- Use stable logical names such as `"shell"` or `"rear_bracket"` rather than
  paths.

## Examples

```python
shell = ExtrudeGeometry(
    rounded_rect_profile(0.12, 0.08, 0.01),
    0.03,
    cap=True,
    center=True,
)
mesh = mesh_from_geometry(shell, "shell")
```

```python
lip = LatheGeometry.from_shell_profiles(
    [(0.42, -0.30), (0.55, -0.12), (0.62, 0.00)],
    [(0.30, -0.24), (0.40, -0.10), (0.48, 0.00)],
    segments=72,
    end_cap="round",
    lip_samples=10,
)
```

```python
wall_vent = VentGrilleGeometry(
    (0.18, 0.10),
    frame=0.012,
    face_thickness=0.004,
    duct_depth=0.026,
    duct_wall=0.003,
    slat_pitch=0.018,
    slat_width=0.009,
    slat_angle_deg=35.0,
    corner_radius=0.006,
)
mesh = mesh_from_geometry(wall_vent, "wall_vent")
```

## See Also

- `45_wires.md` for rails, loops, tubes, and frames
- `46_section_lofts.md` for the recommended loft API
- `50_placement.md` for wrapping and mounting mesh-backed geometry

## Clarifications for agent usage

- Angle arguments are radians and use the right-hand rule.
- `VentGrilleGeometry.slat_angle_deg` also uses degrees.
- `LoftGeometry` validates profiles through their XY projection. End caps only behave as expected when the first and last profiles are planar at constant `z`.
- `ExtrudeGeometry(..., center=True)` produces a solid centered on the profile plane, spanning `z in [-height/2, +height/2]`. Use the `from_z0(...)` form when the intended span is `z in [0, height]`.
- `rounded_rect_profile(...)` and `superellipse_profile(...)` return centered counter-clockwise XY loops.
- `cut_opening_on_face(...)` does not subtract material from a closed solid by itself. It adds the interior throat wall geometry and works best when the target face is already open or when you perform the outer cut separately.
- The side-loft helpers use explicit section tuples in the final part frame. For `superellipse_side_loft(...)`, `split_superellipse_side_loft(...)`, and `resample_side_sections(...)`, each section is `(y, z_min, z_max, width)` with the loft axis along `+Y` and each profile section lying in `XZ`.
