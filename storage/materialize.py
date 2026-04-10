from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from storage.models import AssetStatus, CompileReport, MaterializationStatus
from storage.repo import StorageRepo


def build_materialization_fingerprint(
    *,
    model_py_sha256: str | None,
    model_urdf_sha256: str | None = None,
    sdk_fingerprint: str | None = None,
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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def build_compile_fingerprint_inputs(
    *,
    model_path: Path,
    sdk_fingerprint: str | None = None,
) -> dict[str, str | None]:
    return {
        "model_py_sha256": sha256_file(model_path),
        "sdk_fingerprint": sdk_fingerprint,
    }


def build_compile_fingerprint_from_inputs(
    inputs: Mapping[str, Any],
    *,
    materializer_version: str = "v1",
) -> str:
    model_py_sha256 = inputs.get("model_py_sha256")
    sdk_fingerprint = inputs.get("sdk_fingerprint")
    return build_materialization_fingerprint(
        model_py_sha256=str(model_py_sha256) if isinstance(model_py_sha256, str) else None,
        sdk_fingerprint=str(sdk_fingerprint) if isinstance(sdk_fingerprint, str) else None,
        materializer_version=materializer_version,
    )


@dataclass(slots=True)
class MaterializationStore:
    repo: StorageRepo

    def ensure_record_dirs(self, record_id: str) -> Path:
        materialization_dir = self.repo.layout.record_materialization_dir(record_id)
        materialization_dir.mkdir(parents=True, exist_ok=True)
        return materialization_dir

    def write_compile_report(self, record_id: str, report: CompileReport) -> Path:
        path = self.repo.layout.record_materialization_compile_report_path(record_id)
        self.repo.write_json(path, report.to_dict())
        return path

    def asset_status(self, record_id: str) -> AssetStatus:
        assets_dir = self.repo.layout.record_materialization_assets_dir(record_id)
        return AssetStatus(
            record_id=record_id,
            assets_dir=assets_dir,
            meshes_present=self.repo.layout.record_materialization_asset_meshes_dir(
                record_id
            ).exists(),
            glb_present=self.repo.layout.record_materialization_asset_glb_dir(record_id).exists(),
            viewer_present=self.repo.layout.record_materialization_asset_viewer_dir(
                record_id
            ).exists(),
        )


def canonical_record_paths(repo: StorageRepo, record_id: str) -> dict[str, Path]:
    record_dir = repo.layout.record_dir(record_id)
    return {
        "record_json": repo.layout.record_metadata_path(record_id),
        "prompt_txt": record_dir / "prompt.txt",
        "model_py": record_dir / "model.py",
        "provenance_json": record_dir / "provenance.json",
        "cost_json": record_dir / "cost.json",
        "inputs_dir": repo.layout.record_inputs_dir(record_id),
        "traces_dir": repo.layout.record_traces_dir(record_id),
        "dataset_entry_json": repo.layout.record_dataset_entry_path(record_id),
    }


def materialization_paths(repo: StorageRepo, record_id: str) -> dict[str, Path]:
    return {
        "root": repo.layout.record_materialization_dir(record_id),
        "model_urdf": repo.layout.record_materialization_urdf_path(record_id),
        "compile_report_json": repo.layout.record_materialization_compile_report_path(record_id),
        "assets_dir": repo.layout.record_materialization_assets_dir(record_id),
        "meshes_dir": repo.layout.record_materialization_asset_meshes_dir(record_id),
        "glb_dir": repo.layout.record_materialization_asset_glb_dir(record_id),
        "viewer_dir": repo.layout.record_materialization_asset_viewer_dir(record_id),
    }


def ensure_record_artifacts_exist(
    repo: StorageRepo,
    record_id: str,
    *,
    required: tuple[str, ...],
) -> None:
    paths = canonical_record_paths(repo, record_id)
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
            repo.layout.record_materialization_asset_meshes_dir(record_id),
            repo.layout.record_materialization_asset_glb_dir(record_id),
            repo.layout.record_materialization_asset_viewer_dir(record_id),
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
) -> MaterializationStatus:
    if record_has_materialized_assets(repo, record_id):
        return "available"

    urdf_path = repo.layout.record_materialization_urdf_path(record_id)
    if urdf_path.exists() and not urdf_references_external_meshes(urdf_path):
        return "available"
    return "missing"
