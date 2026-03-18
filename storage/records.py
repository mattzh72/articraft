from __future__ import annotations

import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from storage.models import CompileReport, Provenance, Record
from storage.repo import StorageRepo


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
        return path

    def write_compile_report(self, record_id: str, report: CompileReport) -> Path:
        path = self.repo.layout.record_dir(record_id) / "compile_report.json"
        self.repo.write_json(path, report.to_dict())
        return path

    def write_provenance(self, record_id: str, provenance: Provenance) -> Path:
        path = self.repo.layout.record_dir(record_id) / "provenance.json"
        self.repo.write_json(path, provenance.to_dict())
        return path

    def copy_input_image(
        self, record_id: str, source: Path, destination_name: str | None = None
    ) -> Path:
        inputs_dir = self.repo.layout.record_inputs_dir(record_id)
        inputs_dir.mkdir(parents=True, exist_ok=True)
        destination = inputs_dir / (destination_name or source.name)
        shutil.copy2(source, destination)
        return destination

    def load_record(self, record_id: str) -> dict | None:
        return self.repo.read_json(self.repo.layout.record_metadata_path(record_id))

    def update_rating(self, record_id: str, rating: int) -> dict | None:
        record = self.load_record(record_id)
        if not isinstance(record, dict):
            return None
        record["rating"] = rating
        record["updated_at"] = self._utc_now()
        self.repo.write_json(self.repo.layout.record_metadata_path(record_id), record)
        return record

    def delete_record(self, record_id: str) -> bool:
        record_dir = self.repo.layout.record_dir(record_id)
        if not record_dir.exists():
            return False
        shutil.rmtree(record_dir)
        return True
