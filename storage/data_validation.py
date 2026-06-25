from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.identifiers import RECORD_ID_RE as _RECORD_ID_RE
from storage.identifiers import SLUG_RE as _SLUG_RE
from storage.library_manifest import validate_manifest
from storage.repo import StorageRepo
from storage.revisions import active_model_path, active_provenance_path


@dataclass(slots=True, frozen=True)
class DataFormatValidationResult:
    errors: list[str]
    category_count: int = 0
    record_count: int = 0
    manifest_count: int = 0

    @property
    def ok(self) -> bool:
        return not self.errors


def validate_data_format(
    repo: StorageRepo,
    *,
    require_records: bool = False,
    record_ids: list[str] | None = None,
) -> DataFormatValidationResult:
    validator = _LocalLibraryValidator(
        repo,
        require_records=require_records,
        record_ids=record_ids,
    )
    return validator.validate()


class _LocalLibraryValidator:
    def __init__(
        self,
        repo: StorageRepo,
        *,
        require_records: bool,
        record_ids: list[str] | None,
    ) -> None:
        self.repo = repo
        self.require_records = require_records
        self.requested_record_ids = set(record_ids or [])
        self.errors: list[str] = []
        self.category_slugs: set[str] = set()
        self.category_count = 0
        self.record_count = 0
        self.manifest_count = 0

    def validate(self) -> DataFormatValidationResult:
        data_root = self.repo.layout.data_root
        if not data_root.exists():
            self._add_error(data_root, "missing data directory")
            return self._result()
        self._validate_json_files(data_root)
        self._validate_categories()
        self._validate_manifest()
        self._validate_records()
        return self._result()

    def _result(self) -> DataFormatValidationResult:
        return DataFormatValidationResult(
            errors=self.errors,
            category_count=self.category_count,
            record_count=self.record_count,
            manifest_count=self.manifest_count,
        )

    def _display_path(self, path: Path) -> str:
        try:
            return path.relative_to(self.repo.root).as_posix()
        except ValueError:
            return path.as_posix()

    def _add_error(self, path: Path, message: str) -> None:
        self.errors.append(f"{self._display_path(path)}: {message}")

    def _load_json(self, path: Path) -> Any | None:
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except OSError as exc:
            self._add_error(path, f"failed to read JSON: {exc}")
        except json.JSONDecodeError as exc:
            self._add_error(
                path,
                f"invalid JSON: {exc.msg} at line {exc.lineno} column {exc.colno}",
            )
        return None

    def _validate_json_files(self, data_root: Path) -> None:
        for path in sorted(data_root.rglob("*.json")):
            relative = path.relative_to(data_root).parts
            if relative and relative[0] in {"cache", "local"}:
                continue
            self._load_json(path)

    def _validate_categories(self) -> None:
        root = self.repo.layout.categories_root
        if not root.exists():
            return
        for category_dir in sorted(path for path in root.iterdir() if path.is_dir()):
            payload = self._load_json(category_dir / "category.json")
            if not isinstance(payload, dict):
                self._add_error(
                    category_dir / "category.json", "category metadata must be an object"
                )
                continue
            slug = payload.get("slug")
            if slug != category_dir.name:
                self._add_error(
                    category_dir / "category.json",
                    f"slug={slug!r} must match directory name {category_dir.name!r}",
                )
            if not isinstance(slug, str) or not _SLUG_RE.fullmatch(slug):
                self._add_error(category_dir / "category.json", "slug must be lowercase snake_case")
                continue
            self.category_slugs.add(slug)
            self.category_count += 1

    def _validate_manifest(self) -> None:
        errors = validate_manifest(self.repo, require_records=self.require_records)
        self.errors.extend(errors)
        if self.repo.layout.records_manifest_path.exists():
            try:
                self.manifest_count = sum(
                    1
                    for line in self.repo.layout.records_manifest_path.read_text(
                        encoding="utf-8"
                    ).splitlines()
                    if line.strip()
                )
            except OSError:
                pass

    def _validate_records(self) -> None:
        root = self.repo.layout.records_root
        if not root.exists():
            if self.require_records:
                self._add_error(root, "missing records directory")
            return
        existing_ids = {path.name for path in root.iterdir() if path.is_dir()}
        for record_id in sorted(self.requested_record_ids - existing_ids):
            self._add_error(self.repo.layout.record_dir(record_id), "record not found")
        for record_dir in sorted(path for path in root.iterdir() if path.is_dir()):
            if self.requested_record_ids and record_dir.name not in self.requested_record_ids:
                continue
            self._validate_record_dir(record_dir)

    def _validate_record_dir(self, record_dir: Path) -> None:
        record_id = record_dir.name
        if not _RECORD_ID_RE.fullmatch(record_id):
            self._add_error(record_dir, "record directory name must be a valid record_id")
            return
        record_path = record_dir / "record.json"
        record = self._load_json(record_path)
        if not isinstance(record, dict):
            self._add_error(record_path, "record metadata must be an object")
            return
        self.record_count += 1
        if record.get("record_id") != record_id:
            self._add_error(record_path, "record_id must match directory name")
        category_slug = record.get("category_slug")
        if category_slug is not None:
            if not isinstance(category_slug, str) or not _SLUG_RE.fullmatch(category_slug):
                self._add_error(record_path, f"invalid category_slug={category_slug!r}")
            elif self.category_slugs and category_slug not in self.category_slugs:
                self._add_error(
                    record_path, f"category_slug references missing category={category_slug}"
                )
        model_path = active_model_path(self.repo, record_id, record=record)
        provenance_path = active_provenance_path(self.repo, record_id, record=record)
        if not model_path.exists():
            self._add_error(model_path, "missing active model.py")
        if not provenance_path.exists():
            self._add_error(provenance_path, "missing active provenance.json")
