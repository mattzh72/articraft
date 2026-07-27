# SDK Quickstart

This experiment uses `sdk_no_testing`. It has the normal geometry and articulation
surface, but it does not expose the testing SDK.

Write the whole artifact in `model.py`.

```python
from sdk_no_testing import ArticulatedObject


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="descriptive_name")
    return model


object_model = build_object_model()
```

Do not define `run_tests()` and do not import `TestContext`, `TestReport`,
`TestFailure`, or `AllowedOverlap`. The harness provides no compile tool and no
compile feedback in this condition. Use `probe_model` only for deliberate,
read-only inspection.

Use only these mounted reference paths:

- `docs/sdk/references/quickstart.md`
- `docs/sdk/references/errors.md`
- `docs/sdk/references/core-types.md`
- `docs/sdk/references/articulated-object.md`
- `docs/sdk/references/assets.md`
- `docs/sdk/references/placement.md`
- `docs/sdk/references/probe-tooling.md`
- `docs/sdk/references/geometry/mesh-geometry.md`
- `docs/sdk/references/geometry/panels-and-grilles.md`
- `docs/sdk/references/geometry/brackets-and-mounts.md`
- `docs/sdk/references/geometry/fans-and-rotors.md`
- `docs/sdk/references/geometry/knobs-and-controls.md`
- `docs/sdk/references/geometry/wires.md`
- `docs/sdk/references/geometry/section-lofts.md`
- `docs/sdk/references/geometry/bezels-and-frames.md`
- `docs/sdk/references/geometry/wheels-and-tires.md`
- `docs/sdk/references/geometry/hinges.md`
- `docs/sdk/references/cadquery/overview.md`
- `docs/sdk/references/cadquery/primer.md`
- `docs/sdk/references/cadquery/workplane.md`
- `docs/sdk/references/cadquery/sketch.md`
- `docs/sdk/references/cadquery/assembly.md`
- `docs/sdk/references/cadquery/gears.md`
- `docs/sdk/references/cadquery/free-functions.md`
- `docs/sdk/references/cadquery/api-ref.md`

Do not guess other filenames. Read one of the paths above when you need exact
helper names or signatures.
