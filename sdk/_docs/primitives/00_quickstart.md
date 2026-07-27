# Primitives Only SDK Quickstart

This experiment uses `sdk_primitives`. Build one rooted URDF tree using only
`Box`, `Cylinder`, and `Sphere` geometry.

Do not import `sdk`, CadQuery, mesh helpers, asset helpers, panel helpers, wheel
helpers, or any other geometry constructor. Combine several primitive visuals on
a part when its shape needs more detail.

```python
from sdk_primitives import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="descriptive_name")
    base = model.part("base")
    base.visual(Box((0.4, 0.3, 0.1)), material=model.material("base_color", rgba=(0.2, 0.3, 0.4, 1.0)))
    return model


def run_tests() -> TestReport:
    return TestContext(object_model).report()


object_model = build_object_model()
```

Every non-root part must have exactly one parent articulation. Use fixed joints
for rigid subassemblies and movable joints only for mechanisms requested by the
prompt.

Use only these mounted reference paths:

- `docs/sdk/references/quickstart.md`
- `docs/sdk/references/core-types.md`
- `docs/sdk/references/probe-tooling.md`
- `docs/sdk/references/testing.md`

Do not guess other filenames.
