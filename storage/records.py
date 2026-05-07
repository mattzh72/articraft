from __future__ import annotations

import logging
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from storage.models import Provenance, Record
from storage.repo import StorageRepo

logger = logging.getLogger(__name__)

WORKBENCH_RECORD_GITIGNORE_TEXT = "# Articraft local workbench record. Do not commit.\n*\n"


def write_workbench_record_gitignore_marker(record_dir: Path) -> None:
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / ".gitignore").write_text(
        WORKBENCH_RECORD_GITIGNORE_TEXT,
        encoding="utf-8",
    )


def remove_workbench_record_gitignore_marker(record_dir: Path) -> None:
    marker = record_dir / ".gitignore"
    if not marker.exists():
        return
    try:
        marker_text = marker.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return
    if marker_text == WORKBENCH_RECORD_GITIGNORE_TEXT:
        marker.unlink()


@dataclass(slots=True)
class RecordStore:
    repo: StorageRepo

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

    def ensure_record_dirs(self, record_id: str) -> Path:
        record_dir = self.repo.layout.record_dir(record_id)
        record_dir.mkdir(parents=True, exist_ok=True)
        self.repo.layout.record_inputs_dir(record_id).mkdir(parents=True, exist_ok=True)
        return record_dir

    def write_record(self, record: Record) -> Path:
        self.ensure_record_dirs(record.record_id)
        path = self.repo.layout.record_metadata_path(record.record_id)
        self.repo.write_json(path, record.to_dict())
        record_dir = self.repo.layout.record_dir(record.record_id)
        if "workbench" in record.collections and "dataset" not in record.collections:
            write_workbench_record_gitignore_marker(record_dir)
        else:
            remove_workbench_record_gitignore_marker(record_dir)
        return path

    def write_provenance(self, record_id: str, provenance: Provenance) -> Path:
        path = self.repo.layout.record_dir(record_id) / "provenance.json"
        self.repo.write_json(path, provenance.to_dict())
        return path

    def copy_input_image(
        self,
        record_id: str,
        source: Path,
        destination_name: str | None = None,
        *,
        missing_ok: bool = False,
    ) -> Path | None:
        inputs_dir = self.repo.layout.record_inputs_dir(record_id)
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
        return self.repo.read_json(self.repo.layout.record_metadata_path(record_id))

    def _update_rating_field(
        self,
        record_id: str,
        *,
        rating_field: str,
        rated_by_field: str,
        rating: int | None,
    ) -> dict | None:
        record = self.load_record(record_id)
        if not isinstance(record, dict):
            return None
        record[rating_field] = rating
        # Commit-time attribution becomes stale after a local edit, so clear it until git sync restores it.
        record[rated_by_field] = None
        record["updated_at"] = self._utc_now()
        self.repo.write_json(self.repo.layout.record_metadata_path(record_id), record)
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
        record_dir = self.repo.layout.record_dir(record_id)
        if not record_dir.exists():
            return False
        shutil.rmtree(record_dir)
        materialization_dir = self.repo.layout.record_materialization_dir(record_id)
        if materialization_dir.exists():
            shutil.rmtree(materialization_dir)
        return True
