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

The detailed geometry and articulation reference pages are available below
`docs/sdk/references/`.
