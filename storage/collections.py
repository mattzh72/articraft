from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone

from storage.models import WorkbenchCollection, WorkbenchEntry
from storage.repo import StorageRepo


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


@dataclass(slots=True)
class CollectionStore:
    repo: StorageRepo

    def save_workbench(self, collection: WorkbenchCollection) -> None:
        self.repo.write_json(self.repo.layout.local_workbench_path(), collection.to_dict())

    def load_workbench(self) -> dict | None:
        return self.repo.read_json(self.repo.layout.local_workbench_path())

    def ensure_workbench(self) -> dict:
        existing = self.load_workbench()
        if existing is not None:
            return existing
        collection = WorkbenchCollection(
            schema_version=1,
            collection="workbench",
            updated_at=_utc_now(),
        )
        self.save_workbench(collection)
        return collection.to_dict()

    def append_workbench_entry(
        self,
        *,
        record_id: str,
        added_at: str,
        label: str | None = None,
        tags: list[str] | None = None,
        archived: bool = False,
    ) -> dict:
        current = self.ensure_workbench()
        entries = list(current.get("entries", []))
        entries.append(
            WorkbenchEntry(
                record_id=record_id,
                added_at=added_at,
                label=label,
                tags=list(tags or []),
                archived=archived,
            ).to_dict()
        )
        updated = WorkbenchCollection(
            schema_version=int(current.get("schema_version", 1)),
            collection="workbench",
            updated_at=_utc_now(),
            entries=[
                WorkbenchEntry(
                    record_id=str(entry["record_id"]),
                    added_at=str(entry["added_at"]),
                    label=entry.get("label"),
                    tags=list(entry.get("tags", [])),
                    archived=bool(entry.get("archived", False)),
                )
                for entry in entries
            ],
        )
        self.save_workbench(updated)
        return updated.to_dict()

    def remove_workbench_entries(self, record_id: str) -> bool:
        current = self.load_workbench()
        if current is None:
            return False

        entries = list(current.get("entries", []))
        filtered_entries = [
            entry for entry in entries if str(entry.get("record_id", "")) != record_id
        ]
        if len(filtered_entries) == len(entries):
            return False

        updated = WorkbenchCollection(
            schema_version=int(current.get("schema_version", 1)),
            collection="workbench",
            updated_at=_utc_now(),
            entries=[
                WorkbenchEntry(
                    record_id=str(entry["record_id"]),
                    added_at=str(entry["added_at"]),
                    label=entry.get("label"),
                    tags=list(entry.get("tags", [])),
                    archived=bool(entry.get("archived", False)),
                )
                for entry in filtered_entries
            ],
        )
        self.save_workbench(updated)
        return True
