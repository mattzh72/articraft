# Wires, Tubes, and Frames (`sdk.mesh`)

Use this stack for thin curved parts such as:

- bottle bails and loops
- wire handles and pulls
- whisk cages
- fan guards
- shopping-cart baskets
- stroller / hand-truck / tubular support frames

## Recommended defaults

For most tasks, choose one of these two paths:

1. One smooth path:
   `tube_from_spline_points(...)`
2. One smooth path with a custom profile:
   `sweep_profile_along_spline(...)`

These are the preferred APIs for new generated code. If a part has several rails,
model it as several spline sweeps. Do not approximate one smooth rail by
chaining many short tube segments.

## Choosing between spline tubes and polyline tubes

As a rule of thumb, choose the helper that matches the intended shape language
of the part.

- Use `tube_from_spline_points(...)` when the real part should read as one
  continuously bent tube or wire, especially if you only have a sparse set of
  estimated points.
- Use `wire_from_points(...)` when the path is intentionally piecewise-linear or
  when the corners themselves are part of the design, such as elbows,
  rectangular guards, welded basket corners, or other explicit bends.
- For straight two-point members, either approach can work, but a direct
  straight tube path is often simpler.

A useful check is: if the viewer should perceive the part as a smooth bent
member rather than a sequence of straight runs joined at corners, prefer the
spline-based helper.

## Import

```python
from sdk import (
    tube_from_spline_points,
    sweep_profile_along_spline,
    wire_from_points,
)
```

Legacy/advanced helpers such as `sample_cubic_bezier_spline_3d`,
`ArcPipeGeometry`, and `WirePolylineGeometry` remain available for compatibility,
but they are no longer the recommended default surface.

## 1) Interpolating spline through points

```python
sample_catmull_rom_spline_3d(
    points,
    samples_per_segment=12,
    closed=False,
    alpha=0.5,
) -> list[(x, y, z)]
```

- Fits a smooth spline through the given points.
- Use this when you have clicked/estimated points and want a clean fitted path.
- `closed=True` returns a closed loop with the first point repeated at the end.
- The default `alpha=0.5` is centripetal Catmull-Rom, which is a good general
  choice for avoiding exaggerated overshoot.

## 2) Single spline tube path

```python
tube_from_spline_points(
    points,
    radius,
    samples_per_segment=12,
    closed_spline=False,
    spline="catmull_rom",
    alpha=0.5,
    radial_segments=16,
    cap_ends=True,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length=1e-6,
) -> MeshGeometry
```

- Fits a spline through the points, then builds a circular tube along that path.
- This is the default helper for single handle/tube/wire paths.
- Internally it uses a mitered sweep because the sampled spline is already smooth.

## 3) Single spline profile sweep

```python
sweep_profile_along_spline(
    points,
    profile=...,
    samples_per_segment=12,
    closed_spline=False,
    spline="catmull_rom",
    alpha=0.5,
    cap_profile=True,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length=1e-6,
) -> MeshGeometry
```

- Fits a spline through the points, then sweeps a closed 2D profile along that path.
- Use this for non-circular handles, trim, rails, and custom section sweeps.
- For the common circular-tube case, prefer `tube_from_spline_points(...)`.

## 4) Single wire/tube path (advanced)

```python
wire_from_points(
    points,
    radius,
    radial_segments=16,
    closed_path=False,
    cap_ends=False,
    corner_mode="fillet",   # "fillet" | "miter" | "bevel"
    corner_radius=0.0,
    corner_segments=8,
    up_hint=(0.0, 0.0, 1.0),
    min_segment_length=1e-6,
) -> MeshGeometry
```

Use this for one continuous path such as a loop, bail, handle, hook, or a
single bent support tube.

Practical guidance:

- If your path was already smoothed by `sample_catmull_rom_spline_3d(...)` or
  dense Bezier sampling, prefer `corner_mode="miter"`.
- If your path is sparse and piecewise-linear, use `corner_mode="fillet"` with
  nonzero `corner_radius`.
- Increase `radial_segments` to make the tube rounder.

Be careful using `wire_from_points(...)` with a sparse 3-6 point path when the
intended result is a smooth bent tube. Even with fillets, the result can still
read as segmented rather than continuously curved. In those cases,
`tube_from_spline_points(...)` is usually the better fit.

Compile-time collision generation handles the physical geometry for wire visuals automatically.

## 5) Path authoring builder

```python
from sdk import WirePath

wp = WirePath((x0, y0, z0))
wp.line_to((x1, y1, z1))
wp.bezier_to(c1, c2, end, samples=12)
wp.arc(center=center, normal=normal, angle=..., segments=16)
points = wp.to_points()
```

Use `WirePath` when readability matters more than spline fitting from a loose
point set.

## 6) Advanced / low-level

```python
from sdk import PipeGeometry

PipeGeometry(profile_2d, path_3d, cap=False, closed=True, path_closed=False, up_hint=(0, 0, 1))
```

Use `PipeGeometry` only when you need:

- a non-circular profile
- direct control of the swept profile shape
- custom low-level path/frame behavior

`WirePolylineGeometry`, `ArcPipeGeometry`, and explicit cubic Bezier sampling are
still available, but prefer the higher-level APIs above for new code.

## Tuning

- Increase centerline sampling density first.
- Then increase `radial_segments` if the tube still looks faceted.
- Use `miter` for already-smooth centerlines.
- Use `fillet` for sparse bend points.
- Keep `min_segment_length` small enough that neighboring points are not
  accidentally collapsed.
- For baskets, guards, and frames, prefer a few longer spline sweeps over a
  dense network of tiny path segments.

## Recipes

### Smooth handle from fitted points

```python
from sdk import tube_from_spline_points

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

### Swept trim / non-circular rail

```python
from sdk import rounded_rect_profile, sweep_profile_along_spline

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

### Basket / guard / frame rails

```python
from sdk import tube_from_spline_points

side_rail = tube_from_spline_points(
    [
        (-0.10, -0.08, 0.00),
        (-0.10, -0.02, 0.06),
        (-0.10, 0.04, 0.12),
        (-0.10, 0.08, 0.16),
    ],
    radius=0.006,
    samples_per_segment=16,
)
```

Compile-time collision generation derives collisions from the resulting rail mesh automatically.
