from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from storage.identifiers import validate_category_slug, validate_dataset_id, validate_record_id
from storage.models import DatasetEntry
from storage.records import remove_workbench_record_gitignore_marker
from storage.repo import StorageRepo


@dataclass(slots=True)
class DatasetStore:
    repo: StorageRepo
    _dataset_id_index_cache: dict[str, str] | None = field(default=None, init=False, repr=False)
    _dataset_id_index_token: tuple[int | None, int | None] | None = field(
        default=None,
        init=False,
        repr=False,
    )

    def _path_mtime_ns(self, path: Path) -> int | None:
        try:
            return path.stat().st_mtime_ns
        except OSError:
            return None

    def _dataset_id_index_fs_token(self) -> tuple[int | None, int | None]:
        return (
            self._path_mtime_ns(self.repo.layout.records_root),
            self._path_mtime_ns(self.repo.layout.dataset_manifest_path()),
        )

    def _build_dataset_id_index_from_records(self) -> dict[str, str]:
        dataset_id_to_record_id: dict[str, str] = {}
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return dataset_id_to_record_id

        for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
            entry = self.load_entry(record_dir.name)
            if not isinstance(entry, dict):
                continue
            dataset_id = str(entry.get("dataset_id") or "")
            record_id = str(entry.get("record_id") or record_dir.name)
            if dataset_id and record_id:
                dataset_id_to_record_id[dataset_id] = record_id
        return dataset_id_to_record_id

    def _dataset_id_index(self) -> dict[str, str]:
        token = self._dataset_id_index_fs_token()
        if self._dataset_id_index_cache is not None and self._dataset_id_index_token == token:
            return self._dataset_id_index_cache

        self._dataset_id_index_cache = self._build_dataset_id_index_from_records()
        self._dataset_id_index_token = token
        return self._dataset_id_index_cache

    def dataset_ids(self) -> set[str]:
        return set(self._dataset_id_index())

    def load_entry(self, record_id: str) -> dict | None:
        validated_record_id = validate_record_id(record_id)
        return self.repo.read_json(self.repo.layout.record_dataset_entry_path(validated_record_id))

    def find_record_id_by_dataset_id(self, dataset_id: str) -> str | None:
        record_id = self._dataset_id_index().get(dataset_id)
        return record_id or None

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
        record_id = validate_record_id(record_id)
        dataset_id = validate_dataset_id(dataset_id)
        category_slug = validate_category_slug(category_slug) if category_slug is not None else None
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

        raw_record_category = record.get("category_slug")
        record_category = (
            validate_category_slug(raw_record_category)
            if isinstance(raw_record_category, str) and raw_record_category.strip()
            else None
        )
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
        remove_workbench_record_gitignore_marker(self.repo.layout.record_dir(record_id))
        if self._dataset_id_index_cache is not None:
            self._dataset_id_index_cache[dataset_id] = record_id
            self._dataset_id_index_token = self._dataset_id_index_fs_token()
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
