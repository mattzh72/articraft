from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.models import AssetStatus, MaterializationStatus
from storage.repo import StorageRepo


def build_materialization_fingerprint(
    *,
    model_py_sha256: str | None,
    model_urdf_sha256: str | None,
    sdk_fingerprint: str | None,
    materializer_version: str = "v1",
) -> str:
    payload = "|".join(
        [
            model_py_sha256 or "",
            model_urdf_sha256 or "",
            sdk_fingerprint or "",
            materializer_version,
        ]
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


@dataclass(slots=True)
class MaterializationStore:
    repo: StorageRepo

    def asset_status(self, record_id: str) -> AssetStatus:
        assets_dir = self.repo.layout.record_assets_dir(record_id)
        return AssetStatus(
            record_id=record_id,
            assets_dir=assets_dir,
            meshes_present=self.repo.layout.record_asset_meshes_dir(record_id).exists(),
            glb_present=self.repo.layout.record_asset_glb_dir(record_id).exists(),
            viewer_present=self.repo.layout.record_asset_viewer_dir(record_id).exists(),
        )


def _record_artifact_path(
    record_dir: Path, record: dict[str, Any] | None, key: str, default: str
) -> Path:
    artifacts = record.get("artifacts") if isinstance(record, dict) else None
    if isinstance(artifacts, dict):
        value = artifacts.get(key)
        if isinstance(value, str) and value.strip():
            return record_dir / value
    return record_dir / default


def record_artifact_paths(
    repo: StorageRepo,
    record_id: str,
    *,
    record: dict[str, Any] | None = None,
) -> dict[str, Path]:
    record_dir = repo.layout.record_dir(record_id)
    return {
        "model_py": _record_artifact_path(record_dir, record, "model_py", "model.py"),
        "model_urdf": _record_artifact_path(record_dir, record, "model_urdf", "model.urdf"),
        "compile_report_json": _record_artifact_path(
            record_dir,
            record,
            "compile_report_json",
            "compile_report.json",
        ),
        "provenance_json": _record_artifact_path(
            record_dir,
            record,
            "provenance_json",
            "provenance.json",
        ),
    }


def ensure_record_artifacts_exist(
    repo: StorageRepo,
    record_id: str,
    *,
    record: dict[str, Any] | None = None,
    required: tuple[str, ...],
) -> None:
    paths = record_artifact_paths(repo, record_id, record=record)
    missing = [name for name in required if not paths[name].exists()]
    if not missing:
        return

    missing_labels = ", ".join(missing)
    raise FileNotFoundError(
        f"Record {record_id} is missing canonical artifact(s): {missing_labels}"
    )


def _has_nonempty_dir(path: Path) -> bool:
    return path.exists() and path.is_dir() and any(path.iterdir())


def record_has_materialized_assets(repo: StorageRepo, record_id: str) -> bool:
    return any(
        _has_nonempty_dir(path)
        for path in (
            repo.layout.record_asset_meshes_dir(record_id),
            repo.layout.record_asset_glb_dir(record_id),
            repo.layout.record_asset_viewer_dir(record_id),
        )
    )


def urdf_references_external_meshes(urdf_path: Path) -> bool:
    if not urdf_path.exists() or not urdf_path.is_file():
        return False
    try:
        text = urdf_path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    lowered = text.lower()
    return "<mesh" in lowered and "filename=" in lowered


def infer_materialization_status(
    repo: StorageRepo,
    record_id: str,
    *,
    record: dict[str, Any] | None = None,
) -> MaterializationStatus:
    if record_has_materialized_assets(repo, record_id):
        return "available"

    record_dir = repo.layout.record_dir(record_id)
    urdf_path = _record_artifact_path(record_dir, record, "model_urdf", "model.urdf")
    if urdf_path.exists() and not urdf_references_external_meshes(urdf_path):
        return "available"
    return "missing"
