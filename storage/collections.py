from __future__ import annotations

from dataclasses import dataclass

from storage.models import DatasetCollection, WorkbenchCollection
from storage.repo import StorageRepo


@dataclass(slots=True)
class CollectionStore:
    repo: StorageRepo

    def save_dataset(self, collection: DatasetCollection) -> None:
        self.repo.write_json(self.repo.layout.dataset_collection_path(), collection.to_dict())

    def save_workbench(self, collection: WorkbenchCollection) -> None:
        self.repo.write_json(self.repo.layout.workbench_collection_path(), collection.to_dict())

    def load_dataset(self) -> dict | None:
        return self.repo.read_json(self.repo.layout.dataset_collection_path())

    def load_workbench(self) -> dict | None:
        return self.repo.read_json(self.repo.layout.workbench_collection_path())
