# Wires, Tubes, and Frames (`sdk.mesh`)

## Purpose

Use this stack for thin curved parts such as handles, loops, whisk cages, fan
guards, baskets, and tubular frames. Start with the spline helpers for almost
all continuously bent parts. Reach for `wire_from_points(...)` only when the
straight runs and corners are intentionally part of the design.

## Import

```python
from sdk import (
    WirePath,
    wire_from_points,
    tube_from_spline_points,
    sweep_profile_along_spline,
    rounded_rect_profile,
)
```

## Recommended APIs

| Shape intent | Helper |
| --- | --- |
| Default for smooth circular rails, handles, loops, and frames | `tube_from_spline_points(...)` |
| Default for smooth non-circular rails and trim | `sweep_profile_along_spline(...)` |
| Readable manual path authoring for smooth arcs and bezier-like runs | `WirePath` plus a spline/sweep helper |
| Intentional hard-corner wire geometry only | `wire_from_points(...)` |

## API Reference

### `tube_from_spline_points(...)`

```python
tube_from_spline_points(
    points,
    *,
    radius: float,
    samples_per_segment: int = 12,
    closed_spline: bool = False,
    spline: str = "catmull_rom",
    alpha: float = 0.5,
    radial_segments: int = 16,
    cap_ends: bool = True,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry
```

- Fits a spline through the input points, then builds a circular tube.
- `spline`: `"catmull_rom"` or `"bezier"`.
- `alpha`: Catmull-Rom parameter, used only for `"catmull_rom"`.
- `closed_spline`: closes the fitted path.
- `cap_ends`: adds end caps when the path is open.

### `sweep_profile_along_spline(...)`

```python
sweep_profile_along_spline(
    points,
    *,
    profile,
    samples_per_segment: int = 12,
    closed_spline: bool = False,
    spline: str = "catmull_rom",
    alpha: float = 0.5,
    cap_profile: bool = True,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry
```

- Fits a spline through the input points, then sweeps a closed 2D profile.
- Use this when the section is not circular.

### `wire_from_points(...)`

```python
wire_from_points(
    points,
    *,
    radius: float,
    radial_segments: int = 16,
    closed_path: bool = False,
    cap_ends: bool = False,
    corner_mode: str = "fillet",
    corner_radius: float = 0.0,
    corner_segments: int = 8,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry
```

- Builds one continuous tube from an explicit polyline path.
- Treat this as a hard-corner polyline helper, not the default smooth-tube
  helper.
- `corner_mode`: `"fillet"`, `"miter"`, or `"bevel"`.
- `corner_radius`: bend radius for filleted or beveled corners.
- `closed_path`: closes the centerline loop.

### `WirePath`

```python
WirePath(start)
WirePath.from_points(points) -> WirePath
```

Methods:

```python
wp.line_to(point) -> WirePath
wp.line_by(dx, dy, dz) -> WirePath
wp.bezier_to(control1, control2, end, *, samples=12) -> WirePath
wp.arc(*, center, normal, angle, segments=16) -> WirePath
wp.extend(points) -> WirePath
wp.to_points() -> list[tuple[float, float, float]]
```

Use `WirePath` when readability matters more than fitting a spline from an
existing point set. In most cases, finish with `wp.to_points()` and feed the
result into `tube_from_spline_points(...)` or `sweep_profile_along_spline(...)`
rather than `wire_from_points(...)`.

## Advice

### Default decision order

- If the part should read as one continuously bent tube or wire, start with
  `tube_from_spline_points(...)`.
- If the part should read as one continuously bent rail but the section is not
  circular, use `sweep_profile_along_spline(...)`.
- Use `WirePath` when the path is easier to author procedurally than to specify
  as a finished point list, then pass `wp.to_points()` into a spline/sweep
  helper.
- Use `wire_from_points(...)` only when the corners themselves are part of the
  design, such as elbows, basket corners, or welded rectangular guards.

### Avoiding macaroni artifacts

- A common failure mode is sparse points plus `wire_from_points(...)` plus
  `corner_mode="fillet"` and `corner_radius` used to fake a smooth organic
  bend.
- That pattern often produces macaroni-like artifacts because the input is
  still an explicit polyline with corner treatment, not a truly smooth
  centerline.
- If you are reaching for `corner_radius` to make a sparse polyline look
  organic, you are probably using the wrong helper.
- In that situation, move to `tube_from_spline_points(...)` or
  `sweep_profile_along_spline(...)` and increase `samples_per_segment` first.

### Tuning smoothness

- Increase `samples_per_segment` first when a spline-based result looks coarse.
- Increase `radial_segments` when the tube still looks visibly faceted.
- Use `corner_mode="miter"` only when the centerline is already smooth or when
  you need explicit hard elbows.
- Use `corner_mode="fillet"` only for intentionally piecewise-linear shapes
  where softened corners are still visibly part of the silhouette.

## Examples

### Smooth handle

```python
handle = tube_from_spline_points(
    [
        (-0.03, 0.00, 0.00),
        (-0.01, 0.02, 0.01),
        (0.01, 0.02, 0.01),
        (0.03, 0.00, 0.00),
    ],
    radius=0.002,
    samples_per_segment=18,
    radial_segments=20,
    cap_ends=True,
)
```

### Custom-profile rail

```python
trim = sweep_profile_along_spline(
    [
        (-0.04, 0.00, 0.00),
        (-0.01, 0.02, 0.01),
        (0.02, 0.01, 0.01),
        (0.05, 0.00, 0.00),
    ],
    profile=rounded_rect_profile(0.004, 0.002, radius=0.0007),
    samples_per_segment=18,
    cap_profile=True,
)
```

### Readable smooth path with `WirePath`

```python
wp = (
    WirePath((-0.03, 0.00, 0.00))
    .bezier_to(
        (-0.02, 0.02, 0.01),
        (0.02, 0.02, 0.01),
        (0.03, 0.00, 0.00),
        samples=18,
    )
)

handle = tube_from_spline_points(
    wp.to_points(),
    radius=0.002,
    samples_per_segment=6,
    radial_segments=20,
    cap_ends=True,
)
```

Use this pattern when arcs or bezier-like runs are easier to author
procedurally but the final part should still read as one continuous bend.

### Intentional hard-corner wire

```python
loop = wire_from_points(
    [
        (-0.04, -0.02, 0.00),
        (-0.04, 0.02, 0.00),
        (0.04, 0.02, 0.00),
        (0.04, -0.02, 0.00),
    ],
    radius=0.0015,
    closed_path=True,
    corner_mode="fillet",
    corner_radius=0.006,
)
```

This is a good fit because the part is intentionally boxy: the straight runs
and elbow locations are part of the silhouette.

## See Also

- `40_mesh_geometry.md` for general procedural mesh authoring

## Clarifications for agent usage

- Spline/path angle conventions use radians and the right-hand rule.
- `spline="bezier"` expects chained cubic Bezier control points rather than a generic interpolating spline-through-points contract. For an open chain, `(n - 1) % 3 == 0`; for a closed chain, the authored point list must already close back on itself.
- `up_hint` defines the transported frame used to orient swept profiles. Changing it changes the profile roll even when the path points stay fixed.
