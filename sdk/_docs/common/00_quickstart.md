# SDK Quickstart

## Purpose

Use this page to start a new Articraft SDK script. It defines the required
script contract, the authoring workspace rules, and one minimal end-to-end
example.
Detailed APIs live in the mounted `docs/sdk/references/...` files listed below.

## Virtual Workspace

You are editing a virtual authoring workspace.

- `model.py` is the only writable file.
- `docs/sdk/references/quickstart.md` is this always-loaded entrypoint.
- Everything under `docs/` is read-only SDK guidance.
- Import from `sdk` in `model.py`.
- Use `read_file(path=...)` to load exact reference text only when needed.

## Import Contract

Authoring helpers are exposed as top-level public imports from `sdk`.
The mounted docs are grouped by topic for reading convenience; those filenames
do not imply matching Python submodules.

```python
# Correct
from sdk import ArticulatedObject, MotionLimits, place_on_face

# Wrong
from sdk.placement import place_on_face
from sdk.core_types import MotionLimits
```

## Mounted Reference Layout

Always available in `docs/sdk/references/`:

- `quickstart.md`: script contract, workspace rules, minimal example, workflow, and the
  full reference inventory.
- `errors.md`: common compile and authoring failures, plus how to interpret them.
- `core-types.md`: geometry, material, articulation, and test-related core
  types, plus optional inertial helpers.
- `articulated-object.md`: object, part, and articulation authoring helpers and lookup
  patterns.
- `assets.md`: explicit asset-root helpers for standalone scripts and tests.
- `placement.md`: placement helpers for mounting, offsets, wrapping, and alignment.
- `probe-tooling.md`: `probe_model` helper catalog and inspection workflow.
- `testing.md`: `TestContext`, `expect_*` assertions, and test authoring patterns.
- `cadquery/overview.md`: when and why to use CadQuery-style geometry in Articraft.
- `cadquery/primer.md`: CadQuery mental model and core shape-building workflow.
- `cadquery/workplane.md`: workplane-based modeling patterns and common operations.
- `cadquery/sketch.md`: sketch-driven 2D profiles and profile construction tools.
- `cadquery/assembly.md`: CadQuery assembly helpers and composition patterns.
- `cadquery/gears.md`: vendored gear builders and the preserved `Workplane.gear()`
  plugin workflow.
- `cadquery/free-functions.md`: free-function geometry helpers and utility builders.
- `cadquery/api-ref.md`: compact CadQuery API reference and signatures.

Additional geometry references:

- `geometry/mesh-geometry.md`: mesh generation flow, managed meshes, and mesh-based
  geometry helpers.
- `geometry/wires.md`: wire and path construction helpers.
- `geometry/section-lofts.md`: section lofts, repairs, and section-driven geometry.

Read the exact document you need. Do not guess helper names or signatures from memory.

## Script Contract

Every generated script should define:

- `build_object_model() -> ArticulatedObject`
- `run_tests() -> TestReport`
- `object_model = build_object_model()`

`compile_model` compiles `object_model`, derives exact collisions from visuals,
runs tests, and exports the result. Do not emit URDF XML directly.

`compile_model` also owns the baseline sanity/QC pass. It automatically checks
model validity, exactly one root part, mesh assets, floating disconnected part
groups, disconnected geometry islands inside a part, and current-pose real 3D
overlaps.

## Recommended Imports

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
```

## Managed Mesh Pattern

Use logical mesh names. Articraft manages the materialized OBJ asset paths.

- Generate procedural meshes with `mesh_from_geometry(..., "part_name")`.
- Import existing OBJ inputs with `mesh_from_input("mesh_name")`.
- Use `TestContext(object_model)`; do not wire asset roots manually.
- If you are writing a standalone local script and need a stable asset root,
  read `docs/sdk/references/assets.md`.

## Minimal Example

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="example_box_lid")

    base = model.part("base")
    base.visual(
        Box((0.20, 0.20, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_shell",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.18, 0.18, 0.02)),
        # The lid part frame sits on the hinge line; the panel extends along +X.
        origin=Origin(xyz=(0.09, 0.0, 0.01)),
        name="lid_shell",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        # Positive q should open the lid upward, not into the base.
        # Because the closed lid extends along local +X from the hinge,
        # choose -Y so positive rotation lifts the free edge toward +Z.
        origin=Origin(xyz=(-0.09, 0.0, 0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.0)
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.05)

    return ctx.report()


object_model = build_object_model()
```

This example uses a hinge-line part frame for the lid. At `q=0`, the child
frame coincides with the articulation frame at the left edge of the lid. Since
the closed lid panel extends along local `+X` from that hinge, `axis=(0, -1, 0)`
makes positive angles open upward.

## Recommended Workflow

1. Build parts with `model.part(...)`.
2. Add visuals with `part.visual(...)`.
3. Add motion with `model.articulation(...)`.
4. Add prompt-specific `expect_*` assertions in `run_tests()`.
5. Use `allow_overlap(...)` and `allow_isolated_part(...)` only when the
   intended mechanism genuinely requires those exceptions.

`part.inertial` is optional. Add it only when a downstream simulation or
export consumer needs explicit mass properties.

## Authoring Habits

- Model visible openings, cavities, and hollow bodies explicitly. Do not cap a
  visible opening with a solid placeholder.
- If the object is layered or nested, model those layers with clear visual
  separation instead of collapsing them into one solid mass.
- Preserve the visible construction logic of major faces and covers. If a
  visible surface should read as one continuous piece, keep it connected and
  cut openings into it rather than replacing it with floating fragments.
- Hidden supports and internal structure can stay simple as long as the visible
  form reads correctly.
- Within one part, avoid disconnected visual islands. If a feature should read
  as mounted, give it a real rib, bracket, pin, collar, stem, or wall
  connection, or split it into a separate part.
- For mounted child parts, prefer placement helpers over hand-tuned
  `Origin(...)` offsets.
- Use `place_on_surface(...)` by default for rigid mounts onto housings,
  shells, panels, pedals, feet, knobs, buttons, pads, brackets, and similar
  surface-mounted parts.
- Use `place_on_face(...)` or `place_on_face_uv(...)` only when the parent is
  truly box-like and the semantic reference is a specific face.
- Use `proud_for_flush_mount(...)` when a centered child should sit flush
  instead of half-embedded.
- Use restrained real-world materials and colors rather than placeholder
  defaults.

## Reference Routing

- If you need type or helper signatures, read `docs/sdk/references/core-types.md`.
- If you need part/object construction patterns, read
  `docs/sdk/references/articulated-object.md`.
- If you need explicit asset-root control, read `docs/sdk/references/assets.md`.
- If you need placement logic, read `docs/sdk/references/placement.md`.
- If you need compile/debug interpretation, read `docs/sdk/references/errors.md`.
- If you need probe helper details, read `docs/sdk/references/probe-tooling.md`.
- If you need testing details, read `docs/sdk/references/testing.md`.
- If you need lower-level CadQuery geometry, read the relevant
  `docs/sdk/references/cadquery/*.md` document.
- If you need mesh, wire, or loft helpers, read the relevant
  `docs/sdk/references/geometry/*.md` document.
