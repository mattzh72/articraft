from __future__ import annotations

from dataclasses import dataclass

from storage.models import CategoryRecord
from storage.repo import StorageRepo


@dataclass(slots=True)
class CategoryStore:
    repo: StorageRepo

    def save(self, category: CategoryRecord) -> None:
        path = self.repo.layout.category_metadata_path(category.slug)
        self.repo.write_json(path, category.to_dict())

    def load(self, category_slug: str) -> dict | None:
        return self.repo.read_json(self.repo.layout.category_metadata_path(category_slug))
