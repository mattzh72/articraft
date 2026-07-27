from __future__ import annotations

import sdk as _sdk

_BLOCKED = frozenset({"AllowedOverlap", "TestContext", "TestFailure", "TestReport"})
__all__ = [name for name in _sdk.__all__ if name not in _BLOCKED]

for _name in __all__:
    globals()[_name] = getattr(_sdk, _name)
del _name


def __getattr__(name: str):
    if name in _BLOCKED:
        raise AttributeError(f"module {__name__!r} does not expose testing SDK symbol {name!r}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
