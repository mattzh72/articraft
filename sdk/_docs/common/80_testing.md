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

## Required checks

Every generated model should include all of these:

```python
ctx.check_model_valid()
ctx.check_mesh_files_exist()
ctx.check_articulation_origin_near_geometry(tol=0.01)
ctx.check_part_geometry_connected(use="visual")
ctx.check_no_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
```

Mesh-backed models:

- If any part uses `Mesh(filename="meshes/...")` or `mesh_from_geometry(...)`, keep either model assets or `asset_root=...` wired into `TestContext`.
- Without that, mesh-aware checks can fail because the SDK cannot resolve relative mesh paths or write generated collision hulls.

What they catch:

- `check_articulation_origin_near_geometry`: the articulation origin is near real geometry in both parent and child frames; default to `tol=0.01` and only loosen it when the geometry genuinely needs the extra slack.
- `check_part_geometry_connected`: a single part is not made of disconnected visual geometry islands.
- `check_no_overlaps`: generated collision geometry does not intersect across sampled articulation poses. Recommended usage is as a broad backstop with `ignore_adjacent=True, ignore_fixed=True`, then explicit intent checks for attachment and for pairs that must stay separate.

The harness also runs an automatic isolated-part warning pass at compile time:

- it checks multi-part objects for parts that are not contacting any other part in the checked pose
- it emits both `visual` and `collision` warnings when it finds isolated parts
- it is non-blocking, but should usually be treated as evidence of floating geometry or a bad mount

## Overlap allowances

```python
ctx.allow_overlap("door", "body", reason="hinge adjacency")
```

Use allowances narrowly. Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look attached instead of floating. Still call `ctx.check_no_overlaps(...)` so the allowance is tracked.

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

Treat these intent checks as primary. They should carry the burden of proving that mounted parts look attached and that moving parts stay believable across key poses.

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
ctx.check_articulation_origin_near_geometry(tol=0.01)
ctx.check_part_geometry_connected(use="visual")
ctx.check_no_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)
ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.001, max_penetration=0.0)
ctx.expect_aabb_contact("hinge_leaf", "frame")
ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.01)
with ctx.pose({"lid_hinge": 1.0}):
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.02)
```
