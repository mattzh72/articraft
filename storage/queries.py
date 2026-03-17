from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from storage.repo import StorageRepo


@dataclass(slots=True)
class StorageQueries:
    repo: StorageRepo

    def list_record_ids(self) -> list[str]:
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return []
        return sorted(path.name for path in records_root.iterdir() if path.is_dir())

    def list_category_slugs(self) -> list[str]:
        categories_root = self.repo.layout.categories_root
        if not categories_root.exists():
            return []
        return sorted(path.name for path in categories_root.iterdir() if path.is_dir())

    def record_dir(self, record_id: str) -> Path:
        return self.repo.layout.record_dir(record_id)
