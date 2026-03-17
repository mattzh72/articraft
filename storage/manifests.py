from __future__ import annotations

from dataclasses import dataclass

from storage.collections import CollectionStore
from storage.repo import StorageRepo


@dataclass(slots=True)
class ManifestStore:
    repo: StorageRepo
    collections: CollectionStore

    def dataset_manifest(self) -> dict:
        collection = self.collections.load_dataset() or {"entries": []}
        generated = [
            {
                "name": entry["dataset_id"],
                "record_id": entry["record_id"],
            }
            for entry in collection.get("entries", [])
        ]
        return {"generated": generated}
