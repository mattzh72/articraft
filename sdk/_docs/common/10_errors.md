# Errors

All public errors in the SDK derive from `sdk.SDKError`.

## `SDKError`

```python
from sdk import SDKError
```

Base exception for the articulated object SDK.

## `ValidationError`

```python
from sdk import ValidationError
```

Raised when an articulated object definition or a QC check is invalid, inconsistent, or cannot be evaluated.

## Import Errors

If you hit `ModuleNotFoundError` for something like `sdk.placement`,
`sdk.testing`, or `sdk.core_types`, the usual cause is treating a docs topic
name as a Python submodule.

Import public authoring helpers from top-level `sdk` instead.

```python
# Correct
from sdk import TestContext, ValidationError, place_on_face

# Wrong
from sdk.testing import TestContext
from sdk.placement import place_on_face
```
