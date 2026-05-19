from __future__ import annotations

from . import v0 as _v0

__all__ = list(_v0.__all__)
_LAZY_EXPORTS = frozenset(name for name in __all__ if name not in vars(_v0))

for _name in __all__:
    if _name not in _LAZY_EXPORTS:
        globals()[_name] = getattr(_v0, _name)
del _name


def __getattr__(name: str):
    if name == "LouverPanelGeometry":
        raise AttributeError(
            f"module {__name__!r} no longer exposes {name!r}; "
            "use 'VentGrilleGeometry' for public vent/grille mesh helpers"
        )
    try:
        value = getattr(_v0, name)
    except AttributeError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
