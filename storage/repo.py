from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from storage.layout import StorageLayout


@dataclass(slots=True)
class StorageRepo:
    root: Path
    layout: StorageLayout = field(init=False)

    def __post_init__(self) -> None:
        self.root = self.root.resolve()
        self.layout = StorageLayout(self.root)

    def ensure_layout(self) -> None:
        self.layout.ensure_base_dirs()

    def read_json(self, path: Path, *, default: Any = None) -> Any:
        if not path.exists():
            return default
        return json.loads(path.read_text(encoding="utf-8"))

    def write_json(self, path: Path, data: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    def write_text(self, path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
