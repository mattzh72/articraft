from __future__ import annotations

from dataclasses import dataclass

from storage.models import DatasetEntry
from storage.repo import StorageRepo


@dataclass(slots=True)
class DatasetStore:
    repo: StorageRepo

    def load_entry(self, record_id: str) -> dict | None:
        return self.repo.read_json(self.repo.layout.record_dataset_entry_path(record_id))

    def find_record_id_by_dataset_id(self, dataset_id: str) -> str | None:
        for entry in self.list_entries():
            if str(entry.get("dataset_id") or "") == dataset_id:
                record_id = str(entry.get("record_id") or "")
                return record_id or None
        return None

    def list_entries(self) -> list[dict]:
        entries: list[dict] = []
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return entries
        for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
            entry = self.load_entry(record_dir.name)
            if entry is not None:
                entries.append(entry)
        return sorted(
            entries, key=lambda entry: (str(entry["dataset_id"]), str(entry["record_id"]))
        )

    def promote_record(
        self,
        *,
        record_id: str,
        dataset_id: str,
        promoted_at: str,
        category_slug: str | None = None,
    ) -> dict:
        record_path = self.repo.layout.record_metadata_path(record_id)
        record = self.repo.read_json(record_path)
        if record is None:
            raise ValueError(f"Record not found: {record_id}")
        existing = self.load_entry(record_id)
        if existing is not None:
            raise ValueError(f"Record already promoted: {record_id}")
        existing_record_id = self.find_record_id_by_dataset_id(dataset_id)
        if existing_record_id is not None:
            raise ValueError(
                f"Dataset ID already exists: {dataset_id} (record {existing_record_id})"
            )

        record_category = record.get("category_slug")
        entry_category = category_slug or record_category
        if not entry_category:
            raise ValueError(f"Category slug required for record: {record_id}")
        if record_category and entry_category != record_category:
            raise ValueError(
                f"Category mismatch for {record_id}: record has {record_category}, entry requested {entry_category}"
            )

        entry = DatasetEntry(
            schema_version=1,
            record_id=record_id,
            dataset_id=dataset_id,
            category_slug=entry_category,
            promoted_at=promoted_at,
        )
        path = self.repo.layout.record_dataset_entry_path(record_id)
        self.repo.write_json(path, entry.to_dict())
        return entry.to_dict()

    def validate(self) -> list[str]:
        errors: list[str] = []
        seen_dataset_ids: dict[str, str] = {}

        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return errors

        for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
            entry_path = self.repo.layout.record_dataset_entry_path(record_dir.name)
            entry = self.repo.read_json(entry_path)
            if entry is None:
                continue

            record = self.repo.read_json(self.repo.layout.record_metadata_path(record_dir.name))
            if record is None:
                errors.append(f"{entry_path}: missing record.json")
                continue

            schema_version = entry.get("schema_version")
            if schema_version != 1:
                errors.append(f"{entry_path}: unsupported schema_version={schema_version!r}")

            entry_record_id = str(entry.get("record_id", ""))
            dataset_id = str(entry.get("dataset_id", ""))
            category_slug = str(entry.get("category_slug", ""))
            promoted_at = str(entry.get("promoted_at", ""))

            if not entry_record_id:
                errors.append(f"{entry_path}: missing record_id")
            elif entry_record_id != record_dir.name:
                errors.append(
                    f"{entry_path}: record_id={entry_record_id} does not match directory={record_dir.name}"
                )

            if not dataset_id:
                errors.append(f"{entry_path}: missing dataset_id")
            else:
                prior_record_id = seen_dataset_ids.get(dataset_id)
                if prior_record_id is not None and prior_record_id != record_dir.name:
                    errors.append(
                        f"{entry_path}: duplicate dataset_id={dataset_id} already used by record={prior_record_id}"
                    )
                else:
                    seen_dataset_ids[dataset_id] = record_dir.name

            if not category_slug:
                errors.append(f"{entry_path}: missing category_slug")
            record_category = record.get("category_slug")
            if record_category and category_slug and category_slug != record_category:
                errors.append(
                    f"{entry_path}: category_slug={category_slug} does not match record category={record_category}"
                )

            if not promoted_at:
                errors.append(f"{entry_path}: missing promoted_at")

        return errors

    def dataset_manifest(self) -> dict:
        entries = self.list_entries()
        generated = [
            {
                "name": entry["dataset_id"],
                "record_id": entry["record_id"],
            }
            for entry in entries
        ]
        return {"generated": generated}

    def write_dataset_manifest(self) -> dict:
        manifest = self.dataset_manifest()
        self.repo.write_json(self.repo.layout.dataset_manifest_path(), manifest)
        return manifest
