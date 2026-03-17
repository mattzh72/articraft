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
