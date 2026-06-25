from __future__ import annotations

import logging
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from storage.identifiers import validate_record_id
from storage.library_manifest import remove_record as remove_manifest_record
from storage.library_manifest import upsert_record
from storage.models import Provenance, Record
from storage.repo import StorageRepo
from storage.revisions import (
    INITIAL_REVISION_ID,
    active_inputs_dir,
    active_provenance_path,
    validate_revision_id,
)

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class RecordStore:
    repo: StorageRepo

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

    def ensure_record_dirs(self, record_id: str) -> Path:
        record_id = validate_record_id(record_id)
        record_dir = self.repo.layout.record_dir(record_id)
        record_dir.mkdir(parents=True, exist_ok=True)
        self.repo.layout.record_revision_inputs_dir(record_id, INITIAL_REVISION_ID).mkdir(
            parents=True, exist_ok=True
        )
        return record_dir

    def write_record(self, record: Record) -> Path:
        record_id = validate_record_id(record.record_id)
        self.ensure_record_dirs(record_id)
        path = self.repo.layout.record_metadata_path(record_id)
        payload = record.to_dict()
        payload["record_id"] = record_id
        self.repo.write_json(path, payload)
        upsert_record(self.repo, record_id)
        return path

    def write_provenance(
        self,
        record_id: str,
        provenance: Provenance,
        *,
        revision_id: str | None = None,
    ) -> Path:
        record_id = validate_record_id(record_id)
        if revision_id is None:
            path = active_provenance_path(self.repo, record_id)
        else:
            path = self.repo.layout.record_revision_provenance_path(
                record_id, validate_revision_id(revision_id)
            )
        self.repo.write_json(path, provenance.to_dict())
        return path

    def copy_input_image(
        self,
        record_id: str,
        source: Path,
        destination_name: str | None = None,
        *,
        missing_ok: bool = False,
        revision_id: str | None = None,
    ) -> Path | None:
        record_id = validate_record_id(record_id)
        inputs_dir = (
            self.repo.layout.record_revision_inputs_dir(
                record_id, validate_revision_id(revision_id)
            )
            if revision_id is not None
            else active_inputs_dir(self.repo, record_id)
        )
        inputs_dir.mkdir(parents=True, exist_ok=True)
        destination = inputs_dir / (destination_name or source.name)
        if source.resolve() == destination.resolve():
            return destination
        try:
            shutil.copy2(source, destination)
        except FileNotFoundError:
            if not missing_ok:
                raise
            logger.warning("Skipping missing input image for record %s: %s", record_id, source)
            return None
        return destination

    def load_record(self, record_id: str) -> dict | None:
        record_id = validate_record_id(record_id)
        return self.repo.read_json(self.repo.layout.record_metadata_path(record_id))

    def _update_rating_field(
        self,
        record_id: str,
        *,
        rating_field: str,
        rated_by_field: str,
        rating: int | None,
    ) -> dict | None:
        record_id = validate_record_id(record_id)
        record = self.load_record(record_id)
        if not isinstance(record, dict):
            return None
        record[rating_field] = rating
        record[rated_by_field] = None
        record["updated_at"] = self._utc_now()
        self.repo.write_json(self.repo.layout.record_metadata_path(record_id), record)
        upsert_record(self.repo, record_id)
        return record

    def update_rating(self, record_id: str, rating: int) -> dict | None:
        return self._update_rating_field(
            record_id,
            rating_field="rating",
            rated_by_field="rated_by",
            rating=rating,
        )

    def update_secondary_rating(self, record_id: str, rating: int | None) -> dict | None:
        return self._update_rating_field(
            record_id,
            rating_field="secondary_rating",
            rated_by_field="secondary_rated_by",
            rating=rating,
        )

    def delete_record(self, record_id: str) -> bool:
        record_id = validate_record_id(record_id)
        record_dir = self.repo.layout.record_dir(record_id)
        if not record_dir.exists():
            return False
        shutil.rmtree(record_dir)
        materialization_dir = self.repo.layout.record_materialization_dir(record_id)
        if materialization_dir.exists():
            shutil.rmtree(materialization_dir)
        remove_manifest_record(self.repo, record_id)
        return True
