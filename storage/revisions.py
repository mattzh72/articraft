from __future__ import annotations

import hashlib
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.identifiers import validate_record_id
from storage.repo import StorageRepo

INITIAL_REVISION_ID = "rev_000001"
REVISION_ID_RE = re.compile(r"^rev_[0-9]{6}$")


def validate_revision_id(value: str | None, *, label: str = "revision_id") -> str:
    revision_id = str(value or "").strip()
    if not revision_id:
        raise ValueError(f"{label} is required.")
    if not REVISION_ID_RE.fullmatch(revision_id):
        raise ValueError(f"{label} must match rev_000001 style six-digit IDs.")
    return revision_id


def revision_sort_key(revision_id: str) -> int:
    validate_revision_id(revision_id)
    return int(revision_id.removeprefix("rev_"))


def next_revision_id(repo: StorageRepo, record_id: str) -> str:
    record_id = validate_record_id(record_id)
    root = repo.layout.record_revisions_dir(record_id)
    if not root.exists():
        return INITIAL_REVISION_ID
    highest = 0
    for path in root.iterdir():
        if not path.is_dir() or not REVISION_ID_RE.fullmatch(path.name):
            continue
        highest = max(highest, revision_sort_key(path.name))
    return f"rev_{highest + 1:06d}"


def active_revision_id(
    repo: StorageRepo, record_id: str, record: dict[str, Any] | None = None
) -> str:
    record_id = validate_record_id(record_id)
    payload = record
    if payload is None:
        payload = repo.read_json(repo.layout.record_metadata_path(record_id))
    if isinstance(payload, dict):
        revision_id = str(payload.get("active_revision_id") or "").strip()
        if revision_id:
            return validate_revision_id(revision_id)
    return INITIAL_REVISION_ID


def record_revision_dir(
    repo: StorageRepo,
    record_id: str,
    revision_id: str | None = None,
    *,
    record: dict[str, Any] | None = None,
) -> Path:
    resolved = (
        active_revision_id(repo, record_id, record=record) if revision_id is None else revision_id
    )
    return repo.layout.record_revision_dir(record_id, validate_revision_id(resolved))


def revision_relative_path(revision_id: str, filename: str) -> str:
    return f"revisions/{validate_revision_id(revision_id)}/{filename}"


def active_artifact_path(
    repo: StorageRepo,
    record_id: str,
    artifact_key: str,
    *,
    record: dict[str, Any] | None = None,
) -> Path | None:
    payload = record or repo.read_json(repo.layout.record_metadata_path(record_id))
    if not isinstance(payload, dict):
        return None
    record_dir = repo.layout.record_dir(record_id)
    artifacts = payload.get("artifacts") if isinstance(payload.get("artifacts"), dict) else {}
    value = artifacts.get(artifact_key) if isinstance(artifacts, dict) else None
    if isinstance(value, str) and value.strip():
        path = Path(value)
        if not path.is_absolute() and ".." not in path.parts:
            return record_dir / path
    revision_id = active_revision_id(repo, record_id, record=payload)
    fallback_names = {
        "prompt_txt": "prompt.txt",
        "model_py": "model.py",
        "provenance_json": "provenance.json",
        "cost_json": "cost.json",
        "inputs_dir": "inputs",
        "traces_dir": "traces",
    }
    fallback_name = fallback_names.get(artifact_key)
    if fallback_name is None:
        return None
    if payload.get("schema_version") != 3:
        return record_dir / fallback_name
    return repo.layout.record_revision_dir(record_id, revision_id) / fallback_name


def active_prompt_path(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "prompt_txt", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "prompt.txt"
    )


def active_model_path(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "model_py", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "model.py"
    )


def active_provenance_path(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "provenance_json", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "provenance.json"
    )


def active_cost_path(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "cost_json", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "cost.json"
    )


def active_inputs_dir(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "inputs_dir", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "inputs"
    )


def active_traces_dir(
    repo: StorageRepo, record_id: str, *, record: dict[str, Any] | None = None
) -> Path:
    return active_artifact_path(repo, record_id, "traces_dir", record=record) or (
        record_revision_dir(repo, record_id, record=record) / "traces"
    )


def sha256_file(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def revision_artifacts_payload(*, revision_id: str, has_cost_file: bool) -> dict[str, str | None]:
    return {
        "prompt_txt": revision_relative_path(revision_id, "prompt.txt"),
        "prompt_series_json": None,
        "model_py": revision_relative_path(revision_id, "model.py"),
        "provenance_json": revision_relative_path(revision_id, "provenance.json"),
        "cost_json": revision_relative_path(revision_id, "cost.json") if has_cost_file else None,
        "inputs_dir": revision_relative_path(revision_id, "inputs"),
        "traces_dir": revision_relative_path(revision_id, "traces"),
    }


def build_revision_payload(
    *,
    record_id: str,
    revision_id: str,
    created_at: str,
    prompt_text: str,
    prompt_kind: str,
    source: dict[str, Any],
    generation: dict[str, Any],
    artifacts: dict[str, Any],
    hashes: dict[str, Any],
    run_summary: dict[str, Any] | None = None,
    parent: dict[str, str] | None = None,
    seed: dict[str, str] | None = None,
    inherited_inputs: list[dict[str, str]] | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "record_id": validate_record_id(record_id),
        "revision_id": validate_revision_id(revision_id),
        "created_at": created_at,
        "prompt_kind": prompt_kind,
        "prompt_sha256": hashlib.sha256(prompt_text.encode("utf-8")).hexdigest(),
        "source": dict(source),
        "generation": dict(generation),
        "artifacts": dict(artifacts),
        "hashes": dict(hashes),
        "run_summary": dict(run_summary or {}),
        "parent": dict(parent) if parent else None,
        "seed": dict(seed) if seed else None,
        "inherited_inputs": list(inherited_inputs or []),
    }


@dataclass(slots=True, frozen=True)
class HistoryRevision:
    record_id: str
    revision_id: str
    active: bool
    prompt: str | None
    revision: dict[str, Any]
    record: dict[str, Any]


def list_record_revisions(repo: StorageRepo, record_id: str) -> list[HistoryRevision]:
    record = repo.read_json(repo.layout.record_metadata_path(record_id))
    if not isinstance(record, dict):
        return []
    current = active_revision_id(repo, record_id, record=record)
    rows: list[HistoryRevision] = []
    root = repo.layout.record_revisions_dir(record_id)
    if not root.exists():
        return rows
    for revision_dir in sorted(root.iterdir(), key=lambda path: path.name):
        if not revision_dir.is_dir() or not REVISION_ID_RE.fullmatch(revision_dir.name):
            continue
        revision = repo.read_json(revision_dir / "revision.json", default={}) or {}
        prompt_path = revision_dir / "prompt.txt"
        prompt = prompt_path.read_text(encoding="utf-8") if prompt_path.exists() else None
        rows.append(
            HistoryRevision(
                record_id=record_id,
                revision_id=revision_dir.name,
                active=revision_dir.name == current,
                prompt=prompt,
                revision=revision if isinstance(revision, dict) else {},
                record=record,
            )
        )
    return rows


def descendants_for_record(repo: StorageRepo, record_id: str) -> list[tuple[str, dict[str, Any]]]:
    descendants: list[tuple[str, dict[str, Any]]] = []
    root = repo.layout.records_root
    if not root.exists():
        return descendants
    for candidate in sorted(path for path in root.iterdir() if path.is_dir()):
        record = repo.read_json(candidate / "record.json")
        if not isinstance(record, dict):
            continue
        if _lineage_contains_parent(repo, record, record_id):
            descendants.append((candidate.name, record))
    return descendants


def _lineage_contains_parent(
    repo: StorageRepo,
    record: dict[str, Any],
    target_record_id: str,
) -> bool:
    lineage = record.get("lineage")
    parent_record_id = (
        str(lineage.get("parent_record_id") or "").strip() if isinstance(lineage, dict) else ""
    )
    seen: set[str] = set()
    while parent_record_id and parent_record_id not in seen:
        if parent_record_id == target_record_id:
            return True
        seen.add(parent_record_id)
        parent = repo.read_json(repo.layout.record_metadata_path(parent_record_id))
        parent_lineage = (
            parent.get("lineage")
            if isinstance(parent, dict) and isinstance(parent.get("lineage"), dict)
            else {}
        )
        parent_record_id = str(parent_lineage.get("parent_record_id") or "").strip()
    return False


def copytree_if_exists(source: Path, destination: Path) -> None:
    if not source.exists():
        return
    if destination.exists():
        shutil.rmtree(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination)
