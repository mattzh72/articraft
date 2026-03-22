# Testing

Import:

```python
from sdk import TestContext, TestFailure, TestReport
```

`TestContext` is the required test harness for generated models.

## Construction

```python
ctx = TestContext(
    object_model,
    asset_root=HERE,
    geometry_source="collision",
    seed=0,
)
```

`geometry_source="collision"` is the physical QC path. In `sdk`, collision geometry is generated from visuals during compile/test setup.

If the model was constructed with `ArticulatedObject(..., assets=AssetContext.from_script(__file__))`, `TestContext(object_model, ...)` can infer the mesh root automatically. If not, pass `asset_root=HERE` explicitly whenever the model uses mesh files.

## Default broad sensors and backstops

Every generated model should start from these broad sensors and backstops:

```python
ctx.check_model_valid()
ctx.check_mesh_files_exist()
ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
ctx.warn_if_part_geometry_disconnected(use="visual")
ctx.check_articulation_overlaps(max_pose_samples=128)
ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
```

Mesh-backed models:

- If any part uses `Mesh(filename="assets/meshes/...")` or `mesh_from_geometry(...)`, keep either model assets or `asset_root=...` wired into `TestContext`.
- Without that, mesh-aware checks can fail because the SDK cannot resolve relative mesh paths or write generated collision hulls.

What they catch:

- `warn_if_articulation_origin_near_geometry`: a deliberately dumb static heuristic that checks whether the articulation origin is near real geometry in both parent and child frames; default `tol=0.015`. The harness truncates this tolerance to 3 decimals and caps it at `0.15`.
- `warn_if_part_geometry_disconnected`: a deliberately dumb static heuristic that checks whether a single part appears to contain disconnected visual geometry islands.
- `warn_if_coplanar_surfaces`: a deliberately noisy visual heuristic that looks for nearly coplanar element AABB faces with strong in-plane overlap. Flush mounts, bezels, grilles, and panel seams can trigger it even when the model is acceptable.
- `check_articulation_overlaps`: a failure-tier overlap gate for non-fixed articulation-linked parent/child pairs. It is the recommended way to prove that `REVOLUTE`, `PRISMATIC`, and `CONTINUOUS` joints do not interpenetrate.
- `warn_if_overlaps`: a deliberately broad overlap sensor over generated collision geometry. It relies on generated collisions, AABB reasoning, and convex decomposition, so it can be noisy for thin wires, thin blades, concave shells, and other awkward geometry.

These broad checks are warning-tier sensors, not semantic proof. The agent still needs to reason about the object and add prompt-specific `expect_*` assertions as actual regression tests for silhouette, structure, proportions, and mechanism behavior.

Recommended default:

- Include `warn_if_overlaps(...)` on every generated model as a broad warning-tier sensor, even when you do not want overlap findings to fail the run.
- Include `check_articulation_overlaps(...)` on articulated models with non-fixed joints when joint clearance is a real requirement.
- Use `warn_if_coplanar_surfaces(...)` when flush-surface duplication is genuinely a useful sensor for the object, and prefer `ignore_adjacent=True, ignore_fixed=True` unless you have a specific reason not to.

Important:

- `warn_if_articulation_origin_near_geometry(...)` is not scale-aware; its tolerance is a fixed metric distance in meters.
- A tolerance that is sensible for a compact consumer object may be too strict, too loose, or simply irrelevant for a very large assembly.
- For large objects, treat it as a loose warning sensor or omit it, then use prompt-specific `expect_*` checks to prove the actual mounting and placement claims you care about.

The harness also runs automatic isolated-part checks at compile time:

- it checks multi-part objects for parts that are not contacting any other part in the checked pose
- `visual` isolated-part findings are warnings because they use broad visible-envelope contact
- `collision` isolated-part findings are blocking by default because even the conservative generated collision envelopes still do not touch
- a collision isolated-part failure should usually be treated as a real floating-geometry or bad-mount bug

## Overlap allowances

```python
ctx.allow_overlap("door", "body", reason="hinge barrel nests around the pin")
ctx.allow_coplanar_surfaces("door", "body", reason="flush mounted panel")
```

Use allowances narrowly. Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look attached instead of floating. For articulated mechanisms, use `ctx.allow_overlap(...)` only for specific justified cases such as bearing sleeves, hinge barrels, or enclosed hubs. Still call `ctx.warn_if_overlaps(...)` so the allowance is tracked.

## Articulation-overlap QC

```python
ctx.check_articulation_overlaps(
    max_pose_samples=128,
    overlap_tol=0.001,
    overlap_volume_tol=0.0,
)
```

Use this as the failure-tier QC gate for non-fixed articulation-linked pairs.

- It checks only parent/child pairs connected by `REVOLUTE`, `PRISMATIC`, or `CONTINUOUS` articulations.
- It does not check `FIXED` or `FLOATING` pairs in v1.
- It uses the `TestContext` geometry source, so `geometry_source="collision"` is the recommended mode.
- It respects `ctx.allow_overlap(...)`, which is the intended escape hatch for legitimate nested mechanisms.
- It complements rather than replaces broad `warn_if_overlaps(...)` sensing.

## Pose-aware queries

```python
ctx.part_world_position("lid")
ctx.part_world_aabb("lid", use="collision")
```

Use temporary mechanism poses with the context manager:

```python
with ctx.pose({"lid_hinge": 0.5}):
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)

with ctx.pose(lid_hinge=1.0):
    ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
```

`ctx.pose(...)` takes either:

- a mapping of joint name to position, for example `{"hinge": 0.5}`
- keyword arguments, for example `hinge=0.5`

Do not pass positional `(joint_name, value)` arguments.

## Intent checks

Use these to encode the intended layout and motion:

- `ctx.expect_origin_distance(...)`
- `ctx.expect_origin_gap(...)`
- `ctx.expect_aabb_within(...)`
- `ctx.expect_aabb_gap(...)`
- `ctx.expect_aabb_overlap(...)`
- `ctx.expect_aabb_contact(...)`
- `ctx.expect_joint_motion_axis(...)`

Treat these intent checks as primary. They should carry the burden of proving that mounted parts look attached and that moving parts stay believable across key poses, while `check_articulation_overlaps(...)` enforces non-fixed joint clearance and `warn_if_overlaps(...)` remains the global backstop.

Preferred signatures:

```python
ctx.expect_aabb_gap("upper", "lower", axis="z", max_gap=0.001, max_penetration=0.0)
ctx.expect_aabb_contact("bracket", "frame")
ctx.expect_joint_motion_axis("hinge", "lid", world_axis="z", direction="positive", min_delta=0.01)
```

For mounted or nested assemblies:

- default to near-zero `expect_aabb_gap(...)` checks unless the real object visibly has more clearance
- pair gap checks with footprint/overlap checks such as `expect_aabb_overlap(..., axes="xy", ...)` or `expect_aabb_within(..., axes="xy", ...)`
- use `expect_aabb_contact(...)` when the core invariant is direct touch/overlap rather than a bounded directional gap
- add pose-specific checks at important limits so the assembly still looks attached when articulated

Use `expect_joint_motion_axis(...)` only when changing the joint should move the link's AABB center along the tested world axis.

Good fits:

- hinged lids and doors that swing upward/downward
- drawers and sliders translating along one axis
- levers whose visible mass clearly shifts in the tested direction

Do not use it for centered continuous rotors, wheels, knobs, propellers, or fans spinning in place around their own centerline. For those, use `with ctx.pose(...):` plus pose-specific containment, overlap, or clearance checks at multiple angles instead.

For pairs that truly must not intersect across motion, add selective separation checks in the relevant poses rather than relying only on global overlap QC.

## Example

```python
ctx.check_model_valid()
ctx.check_mesh_files_exist()
ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
ctx.warn_if_part_geometry_disconnected(use="visual")
ctx.check_articulation_overlaps(max_pose_samples=128)
ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)
ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.001, max_penetration=0.0)
ctx.expect_aabb_contact("hinge_leaf", "frame")
ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.01)
with ctx.pose({"lid_hinge": 1.0}):
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.02)
```
