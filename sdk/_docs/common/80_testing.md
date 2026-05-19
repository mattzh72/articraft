# Testing

Import from top-level `sdk`:

```python
from sdk import AllowedOverlap, TestContext, TestFailure, TestReport
```

`TestContext` is the SDK test harness for authored models. It records blocking
failures, non-blocking warnings, and explicit allowances, then returns a
`TestReport` from `run_tests()`.

## Public Types

### `TestFailure`

```python
TestFailure(name: str, details: str)
```

- `name`: recorded check name.
- `details`: failure detail string stored in the report.

### `AllowedOverlap`

```python
AllowedOverlap(
    link_a: str,
    link_b: str,
    reason: str,
    elem_a: str | None = None,
    elem_b: str | None = None,
)
```

- Structured overlap allowance recorded by `allow_overlap(...)`.

### `TestReport`

```python
TestReport(
    passed: bool,
    checks_run: int,
    checks: tuple[str, ...],
    failures: tuple[TestFailure, ...],
    warnings: tuple[str, ...] = (),
    allowances: tuple[str, ...] = (),
    allowed_isolated_parts: tuple[str, ...] = (),
    allowed_overlaps: tuple[AllowedOverlap, ...] = (),
)
```

- `passed`: `True` when no blocking failures were recorded.
- `checks_run`: number of recorded checks.
- `checks`: ordered check names.
- `failures`: blocking failures.
- `warnings`: warning messages and failed warning-tier checks.
- `allowances`: human-readable entries recorded by `allow_*`.
- `allowed_isolated_parts`: names of parts explicitly allowed by `allow_isolated_part(...)`.
- `allowed_overlaps`: structured overlap allowances recorded by `allow_overlap(...)`.

## Construction

```python
ctx = TestContext(model, seed=0)
```

- `model`: the `ArticulatedObject` under test.
- `seed`: deterministic sampling seed for pose-sampled checks.

Generated models should end `run_tests()` with:

```python
return ctx.report()
```

`compile_model` automatically runs the baseline sanity/QC pass:

- model validity
- exactly one root part
- mesh asset readiness
- floating disconnected-part-group detection
- disconnected geometry-island detection within a part
- current-pose real 3D overlap detection

Use `run_tests()` for prompt-specific exact assertions such as `expect_gap(...)`,
`expect_overlap(...)`, `expect_contact(...)`, and `expect_within(...)`.

Keep pose-specific checks lean. Add articulated-pose assertions only when a
prompt-critical mechanism remains ambiguous after exact rest-pose checks.

## High-Signal Testing Habits

- Prefer exact part or element relationship checks over broad heuristics when
  the mounting path, retained insertion, or clearance is what matters.
- If a part has multiple visual regions, use exact contact/support checks for
  the critical mounts instead of broad whole-part approximations.
- Delay brittle numeric thresholds and exact `elem_*` checks until the geometry
  representation is stable.
- Once `run_tests()` references a visual by exact `elem_*` name, treat that
  name as a contract. Preserve it or update every dependent check in the same
  edit.
- Do not add broad lower/upper pose sweeps by default. Use one or two decisive
  articulated poses when a prompt-critical mechanism still needs proof.
- When support, floating status, or overlap meaning is ambiguous, probe first,
  then encode the lasting invariant in `run_tests()`.

## Parameter Conventions

These rules apply across most of the API. Method signatures below are
authoritative when a helper differs.

- `link_*`, `part`, `positive_link`, `negative_link`, `inner_link`, and
  `outer_link` accept either a `Part` object or a part name string. The API uses
  both `link` and `part` terminology; both refer to authored parts.
- `elem_*`, `positive_elem`, `negative_elem`, `inner_elem`, and `outer_elem`
  accept either a named `Visual` object or a visual name string belonging to the
  referenced part.
- `axis` must be `"x"`, `"y"`, or `"z"` and always refers to the positive world
  axis direction.
- `axes` accepts `"x"`, `"y"`, `"z"`, combinations such as `"xy"`, or a sequence
  such as `("x", "y")`.
- Distances and tolerances are meters.
- `name` overrides the recorded check name in the final report.
- `max_pose_samples` controls how many sampled articulation poses are checked.
- `contact_tol=None`, `overlap_tol=None`, and `overlap_volume_tol=None` mean
  “use the SDK default”.

## Reporting Helpers

### `report() -> TestReport`

Returns the final report for the checks recorded so far.

### `check(name: str, ok: bool, details: str = "") -> bool`

Records a custom blocking check.

- `name`: report name.
- `ok`: pass/fail result.
- `details`: stored when `ok` is `False`.

### `fail(name: str, details: str) -> bool`

Convenience wrapper for `check(name, False, details)`.

### `warn(text: str) -> None`

Appends a non-blocking warning string to the report.

## Allowances

### `allow_overlap(link_a, link_b, *, reason, elem_a=None, elem_b=None) -> None`

Records an intentional real 3D interpenetration so overlap checks do not fail on
that case. Do not use this just because one part sits inside another part's
footprint or cavity.

- `reason`: required justification string.
- `elem_a`, `elem_b`: optional element-level scope. If omitted, the allowance
  applies to the full part pair.

### Acceptable Overlap Patterns

Use `allow_overlap(...)` for small intentional overlap that is local, mostly
hidden, and mechanically explanatory. Broad whole-part allowances are the wrong
default.

- Nested proxy fits: a slider, mast, or drawer member represented as moving
  inside a simplified solid sleeve. Pair the allowance with
  `expect_within(...)` and `expect_overlap(...)` on the retained-insertion axis.
- Captured pins or shafts: a hinge pin, knob stem, or axle intentionally nested
  inside a barrel, bushing, or hub proxy. Pair the allowance with
  `expect_contact(...)`, `expect_gap(..., max_penetration=...)`, or a decisive
  pose check showing the mechanism moves correctly.
- Seated trim or compliant compression: a bezel, cap, plug, seal, or bumper
  that needs a tiny local embed to read as seated or compressed. Pair the
  allowance with `expect_contact(...)` or `expect_gap(..., max_penetration=...)`
  on the seating interface.

### `allow_isolated_part(part, *, reason) -> None`

Records that a named part is allowed to remain isolated in the compiler-owned
floating/disconnected-part-group pass.

If the intentional floating assembly is a multi-part group, allow each authored
part in that group.

### `allow_coplanar_surfaces(link_a, link_b, *, reason, elem_a=None, elem_b=None) -> None`

Records an intentional coplanar-surface relationship for
`warn_if_coplanar_surfaces(...)`.

### Nested Sliders And Telescoping Fits

The compiler-owned overlap pass treats real 3D interpenetration as a failure
unless you explicitly allow it.

Use this decision rule for nested prismatic fits:

- if the outer member is modeled as a true hollow or clearanced sleeve, prove
  the fit with `expect_within(...)`, `expect_gap(...)`, and retained-insertion
  checks; do not use `allow_overlap(...)`
- if the outer member is a simplified solid proxy and the authored mechanism is
  still intended to represent one member sliding inside another, scoped
  `allow_overlap(...)` calls are acceptable, but only for the specific named
  elements that stand in for the sleeve/member fit

For telescoping poles, rails, and sleeves, the usual exact-check pattern is:

- `expect_within(...)` on the non-motion axes to prove centering
- `expect_overlap(..., axes="<slide axis>")` at rest and at max extension to
  prove retained insertion
- a `with ctx.pose(...)` check showing the child actually moves in the intended
  direction

`expect_overlap(...)` is a projected overlap check, not a collision waiver. It
proves retained length along an axis; it does not suppress the compiler-owned
overlap pass.

```python
ctx.allow_overlap(
    outer_stage,
    inner_stage,
    elem_a="outer_sleeve",
    elem_b="inner_member",
    reason="The inner member is intentionally represented as sliding inside the sleeve proxy.",
)

ctx.expect_within(
    inner_stage,
    outer_stage,
    axes="xy",
    inner_elem="inner_member",
    outer_elem="outer_sleeve",
    margin=0.002,
    name="inner member stays centered in the sleeve",
)
ctx.expect_overlap(
    inner_stage,
    outer_stage,
    axes="z",
    elem_a="inner_member",
    elem_b="outer_sleeve",
    min_overlap=0.080,
    name="collapsed stage remains inserted in the sleeve",
)

rest_pos = ctx.part_world_position(inner_stage)
with ctx.pose({slide_joint: slide_upper}):
    ctx.expect_within(
        inner_stage,
        outer_stage,
        axes="xy",
        inner_elem="inner_member",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="extended stage stays centered in the sleeve",
    )
    ctx.expect_overlap(
        inner_stage,
        outer_stage,
        axes="z",
        elem_a="inner_member",
        elem_b="outer_sleeve",
        min_overlap=0.030,
        name="extended stage retains insertion in the sleeve",
    )
    extended_pos = ctx.part_world_position(inner_stage)

ctx.check(
    "stage extends upward",
    rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.02,
    details=f"rest={rest_pos}, extended={extended_pos}",
)
```

The wrong default is a broad whole-part allowance such as
`ctx.allow_overlap("cabinet", "drawer", ...)` when only one named sleeve/member
interface is intentionally overlapping. Scope the allowance to the specific
elements whenever the representation gives you stable names to target.

## Pose and World-Space Queries

### `pose(joint_positions: dict[object, float | Origin] | None = None, **kwargs: float | Origin) -> Iterator[None]`

Temporary pose override context manager.

```python
with ctx.pose({hinge: 0.5}):
    ...

with ctx.pose(hinge=0.5):
    ...
```

- `joint_positions`: mapping of articulation object or joint name to position. Scalar joints use `float`; floating joints use `Origin(...)`.
- `**kwargs`: joint-name shorthand.
- Revolute and continuous positions are radians; prismatic positions are meters. Floating poses use `Origin.xyz` in meters and `Origin.rpy` in radians.
- Positive scalar values follow the configured joint convention: right-hand rule for revolute/continuous, translation along `+axis` for prismatic. Floating articulations do not use `axis`.
- Mimic followers are derived automatically from their source articulation. Pose the source joint, not the mimic follower.
- Restores the previous pose on exit.

When debugging a reversed hinge or slider, compare the closed pose and an
opened/extended pose with `part_world_position(...)`, `part_world_aabb(...)`,
or prompt-specific `expect_*` checks. A good default is to confirm that the
upper-limit pose moves outward/upward in the intended direction, not just that
it avoids overlap.

### `part_world_position(part) -> tuple[float, float, float] | None`

Returns the part origin position in world coordinates for the current pose.

### `link_world_position(link) -> tuple[float, float, float] | None`

Alias for `part_world_position(...)`.

### `part_world_aabb(part) -> tuple[Vec3, Vec3] | None`

Returns the world-space AABB of the full part in the current pose.

### `link_world_aabb(link) -> tuple[Vec3, Vec3] | None`

Alias for `part_world_aabb(...)`.

### `part_element_world_aabb(part, *, elem) -> tuple[Vec3, Vec3] | None`

Returns the world-space AABB of one named visual element on a part.

## Structural Checks

All methods in this section record a named check and return `True` on pass,
`False` on fail.

### `fail_if_articulation_origin_far_from_geometry(*, tol=0.015, reason=None, name=None) -> bool`

Fails when an articulation origin is farther than `tol` from nearby geometry.

- `tol`: non-negative absolute tolerance in meters.
- `reason`: optional note recorded as a warning when using a relaxed tolerance.

Available, but not part of the recommended default stack because the tolerance
is absolute rather than scale-aware.

### `warn_if_articulation_origin_far_from_geometry(*, tol=0.015, reason=None, name=None) -> bool`

Warning-tier version of the same check.

### `warn_if_coplanar_surfaces(*, max_pose_samples=32, plane_tol=0.001, min_overlap=0.02, min_overlap_ratio=0.35, ignore_adjacent=True, ignore_fixed=True, name=None) -> bool`

Warning-tier heuristic for suspicious coplanar or nearly coplanar surfaces.

- `plane_tol`: maximum plane separation.
- `min_overlap`: minimum in-plane overlap distance on both in-plane axes.
- `min_overlap_ratio`: minimum overlap area ratio, in `[0, 1]`.
- `ignore_adjacent`, `ignore_fixed`: same meaning as the sampled overlap checks.

Use only when this specific heuristic answers a real uncertainty.

## Exact Assertions

All methods in this section record a named check and return `True` on pass,
`False` on fail.

### `expect_origin_distance(link_a, link_b, *, axes="xy", min_dist=0.0, max_dist=None, name=None) -> bool`

Checks the distance between part origins along the requested axes.

- `min_dist`: inclusive lower bound.
- `max_dist`: optional inclusive upper bound.

### `expect_origin_gap(positive_link, negative_link, *, axis, min_gap=0.0, max_gap=None, name=None) -> bool`

Checks the signed origin-to-origin gap along one positive world axis.

- `positive_link`: object expected on the positive side of the axis.
- `negative_link`: object expected on the negative side of the axis.
- `min_gap`: inclusive lower bound.
- `max_gap`: optional inclusive upper bound.

### `expect_contact(link_a, link_b, *, contact_tol=1e-6, elem_a=None, elem_b=None, name=None) -> bool`

Checks exact minimum distance between two parts or two named elements.

- `contact_tol`: maximum allowed separation to still count as contact.
- `elem_a`, `elem_b`: optional named element scope.

### `expect_gap(positive_link, negative_link, *, axis, min_gap=None, max_gap=None, max_penetration=None, positive_elem=None, negative_elem=None, elem_a=None, elem_b=None, name=None) -> bool`

Checks signed exact-geometry gap along a positive world axis.

- The measured gap is `positive.min[axis] - negative.max[axis]`.
- `min_gap`: inclusive lower bound. If omitted, it is derived from
  `max_penetration`.
- `max_gap`: optional inclusive upper bound.
- `max_penetration`: convenience way to express the allowed overlap depth;
  equivalent to a lower bound of `-max_penetration`.
- `positive_elem`, `negative_elem`: optional named element scope.
- `elem_a`, `elem_b`: accepted as compatibility aliases for
  `positive_elem`/`negative_elem`.

This is the main exact clearance and seating helper.

### `expect_overlap(link_a, link_b, *, axes="xy", min_overlap=0.0, elem_a=None, elem_b=None, name=None) -> bool`

Checks exact projected overlap between two parts or named elements. This is a
footprint/projection check, not a collision/contact check.

- `min_overlap`: required overlap on every requested axis.
- `elem_a`, `elem_b`: optional named element scope.

### `expect_within(inner_link, outer_link, *, axes="xy", margin=0.0, inner_elem=None, outer_elem=None, elem_a=None, elem_b=None, name=None) -> bool`

Checks that one part or named element stays within another on the requested
axes.

- `margin`: allowed slack outside the outer bounds.
- `inner_elem`, `outer_elem`: optional named element scope.
- `elem_a`, `elem_b`: accepted as compatibility aliases for
  `inner_elem`/`outer_elem`.

For nested sliders, use `expect_within(...)` on the non-motion axes and pair it
with `expect_overlap(...)` or `expect_gap(...)` on the slide axis. Do not use
`expect_within(...)` by itself as proof that the moving member still remains
inserted at full extension.

## Floating pose notes

- `ctx.pose(...)` accepts mixed values: scalar `float` for scalar articulations and `Origin(...)` for `FLOATING`. Passing the wrong value kind is a validation error.
- For floating articulations, `Origin.xyz` is relative translation in the articulation frame and `Origin.rpy` is relative rotation in that same frame. Zero floating pose is `Origin()`.
- Mimic followers cannot be overridden directly in `ctx.pose(...)`; drive the source articulation instead.
- `sample_poses(...)` and pose-sampled QC use only `Origin()` for floating joints by default. To test additional floating poses, provide `joint.meta["qc_samples"] = [Origin(...), ...]`.
