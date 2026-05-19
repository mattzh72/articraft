from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone

from storage.models import WorkbenchCollection, WorkbenchEntry
from storage.repo import StorageRepo


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _workbench_entry_from_payload(
    payload: dict,
    *,
    fallback_record_id: str | None = None,
) -> WorkbenchEntry | None:
    record_id = str(payload.get("record_id") or fallback_record_id or "").strip()
    added_at = str(payload.get("added_at") or "").strip()
    if not record_id or not added_at:
        return None
    raw_tags = payload.get("tags", [])
    tags = (
        [str(tag) for tag in raw_tags if isinstance(tag, str)] if isinstance(raw_tags, list) else []
    )
    label = payload.get("label")
    return WorkbenchEntry(
        record_id=record_id,
        added_at=added_at,
        label=label if isinstance(label, str) else None,
        tags=tags,
        archived=bool(payload.get("archived", False)),
    )


@dataclass(slots=True)
class CollectionStore:
    repo: StorageRepo

    def save_workbench(self, collection: WorkbenchCollection) -> None:
        payload = collection.to_dict()
        entries = payload.get("entries", []) if isinstance(payload, dict) else []
        for item in entries:
            if not isinstance(item, dict):
                continue
            record_id = str(item.get("record_id") or "").strip()
            if not record_id:
                continue
            self.repo.write_json(self.repo.layout.record_workbench_entry_path(record_id), item)

    def load_workbench(self) -> dict | None:
        entries_by_id: dict[str, WorkbenchEntry] = {}

        records_root = self.repo.layout.records_root
        if records_root.exists():
            for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
                payload = self.repo.read_json(
                    self.repo.layout.record_workbench_entry_path(record_dir.name)
                )
                if not isinstance(payload, dict):
                    continue
                entry = _workbench_entry_from_payload(payload, fallback_record_id=record_dir.name)
                if entry is not None:
                    entries_by_id[entry.record_id] = entry
        if entries_by_id:
            return WorkbenchCollection(
                schema_version=2,
                collection="workbench",
                updated_at=_utc_now(),
                entries=list(entries_by_id.values()),
            ).to_dict()
        return None

    def ensure_workbench(self) -> dict:
        existing = self.load_workbench()
        if existing is not None:
            return existing
        collection = WorkbenchCollection(
            schema_version=1,
            collection="workbench",
            updated_at=_utc_now(),
        )
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
        entry = WorkbenchEntry(
            record_id=record_id,
            added_at=added_at,
            label=label,
            tags=list(tags or []),
            archived=archived,
        )
        self.repo.write_json(
            self.repo.layout.record_workbench_entry_path(record_id), entry.to_dict()
        )
        return self.ensure_workbench()

    def upsert_workbench_entry(
        self,
        *,
        record_id: str,
        added_at: str,
        label: str | None = None,
        tags: list[str] | None = None,
        archived: bool = False,
    ) -> dict:
        return self.append_workbench_entry(
            record_id=record_id,
            added_at=added_at,
            label=label,
            tags=tags,
            archived=archived,
        )

    def remove_workbench_entries(self, record_id: str) -> bool:
        path = self.repo.layout.record_workbench_entry_path(record_id)
        if path.exists():
            path.unlink()
            return True
        return False
