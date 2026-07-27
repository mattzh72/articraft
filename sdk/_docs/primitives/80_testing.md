# Testing

Import `TestContext` and `TestReport` from `sdk_primitives`.

```python
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    return ctx.report()
```

The compile tool automatically runs the normal baseline checks. Add only
prompt-specific checks when they are useful. Primitive geometry is checked in
the same way as geometry from the full SDK.
