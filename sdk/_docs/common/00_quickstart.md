# SDK Quickstart

This repository's agent writes a Python script that defines an `ArticulatedObject`.
Use the top-level `sdk` package.

## Script contract

Always define:

- `def build_object_model() -> ArticulatedObject:`
- `def run_tests() -> TestReport:`
- `object_model = build_object_model()`

Do not emit URDF XML yourself. The harness compiles `object_model`, generates collisions, runs QC, and exports URDF.

## Golden path

1. Build parts with `model.part(...)`.
2. Author visuals with `part.visual(...)`.
3. Use `Inertial.from_geometry(...)` for primitive inertials when needed.
4. Add motion with `model.articulation(...)`.
5. In `run_tests()`, always include:
   - `ctx.check_model_valid()`
   - `ctx.check_mesh_files_exist()`
   - `ctx.warn_if_articulation_origin_near_geometry(tol=0.015)`
   - `ctx.warn_if_part_geometry_disconnected(use="visual")`
   - `ctx.check_articulation_overlaps(...)` when the model has non-fixed articulations
   - `ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)` when it is useful
   - `ctx.warn_if_overlaps(..., ignore_adjacent=True, ignore_fixed=True)` as a broad warning-tier backstop

Joint authoring rules:

- `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC` require `MotionLimits(effort=..., velocity=..., lower=..., upper=...)`.
- `ArticulationType.CONTINUOUS` requires `MotionLimits(effort=..., velocity=...)` and must not set `lower` or `upper`.
- `ArticulationType.FIXED` and `ArticulationType.FLOATING` do not use `motion_limits`.

Collisions are generated automatically at compile time:

- primitive visuals become matching primitive collisions
- mesh visuals are decomposed into convex hulls with CoACD
- physical overlap checks run on generated collisions, not authored ones
- overlap QC is conservative; use attachment checks as the primary realism checks
- compile also checks isolated parts in multi-part objects: `visual` isolation is a warning, while `collision` isolation is a blocking failure by default

## Recommended imports

```python
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
```

For mounting and conformal wrapping helpers, see `50_placement.md`.

## Minimal example

```python
from pathlib import Path
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="example", assets=ASSETS)

    base = model.part("base")
    base.visual(Box((0.20, 0.20, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)))
    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    lid = model.part("lid")
    lid.visual(Box((0.18, 0.18, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.01)))
    lid.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.02)),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected(use="visual")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
    ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.001, max_penetration=0.0)
    return ctx.report()


object_model = build_object_model()
```

Continuous-joint example:

```python
model.articulation(
    "base_to_turntable",
    ArticulationType.CONTINUOUS,
    parent="base",
    child="turntable",
    origin=Origin(xyz=(0.0, 0.0, 0.05)),
    axis=(0.0, 0.0, 1.0),
    motion_limits=MotionLimits(effort=8.0, velocity=2.0),
)
```

## Asset paths

Use script-local paths:

```python
from pathlib import Path
from sdk import AssetContext

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
```

When writing meshes, put them under `ASSETS.mesh_dir` and pass string paths:

```python
mesh = mesh_from_geometry(geom, str(MESH_DIR / "part.obj"))
```

Important:

- If a model writes or references meshes, prefer `ASSETS = AssetContext.from_script(__file__)`.
- Construct the model with `ArticulatedObject(..., assets=ASSETS)` so generated collision hulls can also be written under the same asset root.
- In tests, either use `TestContext(object_model)` when the model already has `assets=ASSETS`, or pass `asset_root=ASSETS.asset_root` explicitly.

## Rules of thumb

- Author the visible shape you actually want to see.
- Do not hand-author collisions in `sdk`; compile-time generation owns that now.
- Treat `warn_if_articulation_origin_near_geometry(tol=0.015)`, `warn_if_part_geometry_disconnected(use="visual")`, and `warn_if_coplanar_surfaces(use="visual")` as deliberately dumb static sensors.
- These broad checks use static AABB relationships, so they may or may not be useful for a given object.
- They can surface suspicious composition, but they do not prove realism, attachment quality, or correct motion.
- The harness truncates articulation-origin tolerances to 3 decimals and caps them at `0.15`.
- `warn_if_articulation_origin_near_geometry(...)` is not scale-aware; its tolerance is a fixed metric distance in meters.
- `warn_if_coplanar_surfaces(...)` is intentionally noisy: it looks for nearly coplanar visual-envelope faces and can fire on perfectly acceptable flush seams or mounted panels.
- Prefer relation-aware defaults such as `ignore_adjacent=True` and `ignore_fixed=True`, and use `allow_coplanar_surfaces(...)` narrowly for intentional flush mounts that still get reported.
- A tolerance that is sensible for a compact hinge may be meaningless for a vehicle-sized or aircraft-sized assembly.
- Only relax articulation-origin tolerance when the geometry genuinely needs it, and keep it tight.
- Use `ctx.check_articulation_overlaps(...)` as the failure-tier QC gate for `REVOLUTE`, `PRISMATIC`, and `CONTINUOUS` parent/child pairs when you need to prove non-fixed joint clearance.
- Use prompt-specific `expect_*` assertions as the real regression tests for visible structure, proportions, and mechanism behavior.
- Make attachment checks primary: use near-zero `expect_aabb_gap(...)`, footprint overlap, `expect_aabb_contact(...)` where appropriate, and pose-specific mounting checks to prove that parts look attached.
- Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
- Use `ctx.allow_overlap(...)` narrowly for legitimate nested mechanisms such as bearing sleeves, hinge barrels, or enclosed hubs, and still call `ctx.warn_if_overlaps(...)` so the allowance is tracked.
- Use `ctx.warn_if_overlaps(..., ignore_adjacent=True, ignore_fixed=True)` as a conservative warning-tier backstop for non-joint overlap issues, not as proof that attachment quality is good.
- Overlap QC is also heuristic: generated collisions, AABB reasoning, and convex decomposition can be noisy for thin wires, thin blades, concave shells, and other awkward shapes.
- The harness also runs automatic isolated-part checks at compile time.
- `visual` isolated-part findings stay warning-tier because they are based on broad visible-envelope contact.
- `collision` isolated-part findings fail by default because even the conservative generated collision envelopes still do not touch, which is usually a real floating-geometry or bad-mount bug.
- Add selective separation checks only for pairs that truly must remain clear across motion.
