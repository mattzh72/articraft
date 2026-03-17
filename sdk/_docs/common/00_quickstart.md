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
   - `ctx.check_articulation_origin_near_geometry(tol=0.01)`
   - `ctx.check_part_geometry_connected(use="visual")`
   - `ctx.check_no_overlaps(..., ignore_adjacent=True, ignore_fixed=True)`

Collisions are generated automatically at compile time:

- primitive visuals become matching primitive collisions
- mesh visuals are decomposed into convex hulls with CoACD
- physical overlap checks run on generated collisions, not authored ones
- overlap QC is conservative; use attachment checks as the primary realism checks
- compile also emits non-blocking isolated-part warnings when a part in a multi-part object is not contacting any other part in the checked pose

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
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
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

## Asset paths

Use script-local paths:

```python
from pathlib import Path
from sdk import AssetContext

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = HERE / "meshes"
```

When writing meshes, put them under `HERE / "meshes"` and pass string paths:

```python
mesh = mesh_from_geometry(geom, str(MESH_DIR / "part.obj"))
```

Important:

- If a model writes or references meshes, prefer `ASSETS = AssetContext.from_script(__file__)`.
- Construct the model with `ArticulatedObject(..., assets=ASSETS)` so generated collision hulls can also be written under the same asset root.
- In tests, either use `TestContext(object_model)` when the model already has `assets=ASSETS`, or pass `asset_root=HERE` explicitly.

## Rules of thumb

- Author the visible shape you actually want to see.
- Do not hand-author collisions in `sdk`; compile-time generation owns that now.
- Treat `check_articulation_origin_near_geometry(tol=0.01)` as necessary but not sufficient for attachment quality.
- Only relax articulation-origin tolerance when the geometry genuinely needs it, and keep it tight.
- Make attachment checks primary: use near-zero `expect_aabb_gap(...)`, footprint overlap, `expect_aabb_contact(...)` where appropriate, and pose-specific mounting checks to prove that parts look attached.
- Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
- Use `ctx.allow_overlap(...)` narrowly for legitimate nested mechanisms or conservative false-positives, and still call `ctx.check_no_overlaps(...)` so the allowance is tracked.
- Use `ctx.check_no_overlaps(..., ignore_adjacent=True, ignore_fixed=True)` as a conservative backstop, not as proof that attachment quality is good.
- The harness also runs an automatic isolated-part warning pass at compile time; if it reports a part that is not contacting any other part, treat that as a likely floating-geometry bug unless the separation is truly intentional.
- Add selective separation checks only for pairs that truly must remain clear across motion.
