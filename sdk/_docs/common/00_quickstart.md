# SDK Quickstart

This repository's agent writes a Python script that defines an `ArticulatedObject`.
Use the top-level `sdk` package.

## Script contract

Always define:

- `def build_object_model() -> ArticulatedObject:`
- `def run_tests() -> TestReport:`
- `object_model = build_object_model()`

Do not emit URDF XML yourself. The harness compiles `object_model`, derives exact collisions from visuals, runs QC, and exports URDF.

## Golden path

1. Build parts with `model.part(...)`.
2. Author visuals with `part.visual(...)`.
3. Use `Inertial.from_geometry(...)` for primitive inertials when needed.
4. Add motion with `model.articulation(...)`.
5. The scaffolded `run_tests()` already includes the default hard gates:
   - `ctx.check_model_valid()`
   - `ctx.check_mesh_files_exist()`
   - `ctx.check_part_geometry_connected()`
   Keep that block, then extend `run_tests()` with prompt-specific exact `expect_*` checks.
   If the object has a mounted subassembly, make the mount explicit with `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)`.
   If you add a warning-tier heuristic and it fires, investigate it with `probe_model` before editing geometry or relaxing thresholds.
6. In `run_tests()`, resolve the exact `Part`, `Articulation`, and named `Visual` objects you need from `object_model` once at the top, then pass those objects into `ctx.*`. Use names only at the lookup boundary, not as the main test-call style. For named visual features, prefer `part.get_visual("feature_name")`.

Joint authoring rules:

- `ArticulationType.REVOLUTE` and `ArticulationType.PRISMATIC` require `MotionLimits(effort=..., velocity=..., lower=..., upper=...)`.
- `ArticulationType.CONTINUOUS` requires `MotionLimits(effort=..., velocity=...)` and must not set `lower` or `upper`.
- `ArticulationType.FIXED` and `ArticulationType.FLOATING` do not use `motion_limits`.

Collisions are derived automatically at compile time:

- every visual element gets a matching collision entry
- collision geometry mirrors the visual geometry exactly
- overlap and contact QC run on collisions derived mechanically from visuals, not authored ones
- overlap QC is conservative; use exact attachment checks as the primary realism checks

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
    base.visual(Box((0.20, 0.20, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    lid = model.part("lid")
    lid.visual(Box((0.18, 0.18, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.01)), name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.02)),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_part_geometry_connected()
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance(lid, base, axes="xy", max_dist=0.02)
    ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.0)
    with ctx.pose({lid_hinge: 1.0}):
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.02)
    return ctx.report()


object_model = build_object_model()
```

`ctx.report()` returns the required `TestReport`. Treat `failures` as blocking, `warnings` as non-blocking QC/design evidence, and `allowances` as the audit trail for justified exceptions.

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
- Construct the model with `ArticulatedObject(..., assets=ASSETS)` so compile/test/export can resolve the same mesh paths under the same asset root.
- In tests, either use `TestContext(object_model)` when the model already has `assets=ASSETS`, or pass `asset_root=ASSETS.asset_root` explicitly.

## Rules of thumb

- Author the visible shape you actually want to see.
- Do not hand-author collisions in `sdk`; compile-time exact-collision derivation owns that now.
- Treat `check_part_geometry_connected()` as a blocking sensor for within-part floating geometry.
- Broad warning heuristics can surface suspicious composition, but they do not prove realism, attachment quality, or correct motion.
- Use prompt-specific `expect_*` assertions as the real regression tests for visible structure, proportions, and mechanism behavior.
- Make attachment checks primary: use near-zero `expect_gap(...)`, exact footprint overlap, `expect_contact(...)` where appropriate, and pose-specific mounting checks to prove that parts look attached.
- When the whole part is too broad for a small mount or hinge seat, resolve named local features from the part with `part.get_visual(...)` and pass those `Visual` objects into the exact `expect_*` helpers.
- In new code, prefer the articulation-spelled helpers and the exact `expect_*` family: `check_part_geometry_connected(...)`, `expect_gap(...)`, `expect_overlap(...)`, `expect_contact(...)`, and `expect_within(...)`.
- Avoid legacy or deprecated helpers in new generated code: `check_joint_origin_near_geometry(...)`, `warn_if_joint_origin_near_geometry(...)`, `warn_if_part_geometry_connected(...)`, `expect_aabb_*`, and `expect_joint_motion_axis(...)`.
- Deprecated as blanket scaffold defaults in new generated code: `warn_if_articulation_origin_near_geometry(...)` and `warn_if_overlaps(...)`. Add them only with specific justification.
- Use `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is genuinely uncertain or mechanically important.
- If support or floating is ambiguous, use `probe_model` first, then encode the exact invariant in `run_tests()`.
- Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
- Use `ctx.allow_overlap(...)` narrowly for legitimate nested mechanisms such as bearing sleeves, hinge barrels, or enclosed hubs.
- Add selective separation checks only for pairs that truly must remain clear across motion.
