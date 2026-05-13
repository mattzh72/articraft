from __future__ import annotations

from typing import Any


class ViewerStoreComponent:
    def __init__(self, owner: Any) -> None:
        object.__setattr__(self, "_owner", owner)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._owner, name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name == "_owner":
            object.__setattr__(self, name, value)
            return
        setattr(self._owner, name, value)
