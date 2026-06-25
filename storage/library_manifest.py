from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any

from storage.identifiers import validate_record_id
from storage.repo import StorageRepo
from storage.revisions import active_cost_path, active_provenance_path, active_traces_dir
from storage.value_normalization import (
    coerce_int as _coerce_int,
)
from storage.value_normalization import (
    coerce_rating as _coerce_rating,
)
from storage.value_normalization import (
    coerce_string_list as _coerce_string_list,
)
from storage.value_normalization import (
    cost_totals as _cost_totals,
)
from storage.value_normalization import (
    string_or_none as _string_or_none,
)

LIBRARY_MANIFEST_SCHEMA_VERSION = 1


class LibraryManifestError(ValueError):
    pass


def load_manifest(repo: StorageRepo) -> list[dict[str, Any]]:
    path = repo.layout.records_manifest_path
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    seen: set[str] = set()
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            payload = line.strip()
            if not payload:
                continue
            try:
                row = json.loads(payload)
            except json.JSONDecodeError as exc:
                raise LibraryManifestError(
                    f"{path}: line {line_number}: invalid JSON: {exc.msg} at column {exc.colno}"
                ) from exc
            if not isinstance(row, dict):
                raise LibraryManifestError(f"{path}: line {line_number}: row must be an object")
            if row.get("schema_version") != LIBRARY_MANIFEST_SCHEMA_VERSION:
                raise LibraryManifestError(
                    f"{path}: line {line_number}: unsupported schema_version="
                    f"{row.get('schema_version')!r}"
                )
            record_id = str(row.get("record_id") or "")
            validate_record_id(record_id)
            if record_id in seen:
                raise LibraryManifestError(
                    f"{path}: line {line_number}: duplicate record_id={record_id}"
                )
            seen.add(record_id)
            rows.append(row)
    return rows


def manifest_by_id(repo: StorageRepo) -> dict[str, dict[str, Any]]:
    return {str(row["record_id"]): row for row in load_manifest(repo)}


def record_exists(repo: StorageRepo, record_id: str) -> bool:
    return repo.layout.record_metadata_path(validate_record_id(record_id)).is_file()


def record_id_available(repo: StorageRepo, record_id: str) -> bool:
    record_id = validate_record_id(record_id)
    if repo.layout.record_dir(record_id).exists():
        return False
    return record_id not in manifest_by_id(repo)


def upsert_record(repo: StorageRepo, record_id: str) -> dict[str, Any]:
    row = build_manifest_row(repo, record_id, migrate_metadata=True)
    rows = manifest_by_id(repo)
    rows[record_id] = row
    _write_rows(repo, rows.values())
    return row


def remove_record(repo: StorageRepo, record_id: str) -> None:
    record_id = validate_record_id(record_id)
    rows = manifest_by_id(repo)
    rows.pop(record_id, None)
    _write_rows(repo, rows.values())


def rebuild_manifest(repo: StorageRepo, *, migrate_metadata: bool = True) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    records_root = repo.layout.records_root
    if records_root.exists():
        for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
            if (record_dir / "record.json").is_file():
                rows.append(
                    build_manifest_row(
                        repo,
                        record_dir.name,
                        migrate_metadata=migrate_metadata,
                    )
                )
    _write_rows(repo, rows)
    return rows


def validate_manifest(repo: StorageRepo, *, require_records: bool = False) -> list[str]:
    errors: list[str] = []
    try:
        rows = load_manifest(repo)
    except (LibraryManifestError, ValueError) as exc:
        return [str(exc)]
    seen: set[str] = set()
    for row in rows:
        record_id = str(row.get("record_id") or "")
        if record_id in seen:
            errors.append(f"{repo.layout.records_manifest_path}: duplicate record_id={record_id}")
        seen.add(record_id)
        if require_records and not record_exists(repo, record_id):
            errors.append(f"{repo.layout.record_dir(record_id)}: record payload is missing")
    return errors


def build_manifest_row(
    repo: StorageRepo,
    record_id: str,
    *,
    migrate_metadata: bool = False,
) -> dict[str, Any]:
    record_id = validate_record_id(record_id)
    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path, default=None)
    if not isinstance(record, dict):
        raise LibraryManifestError(f"Record not found: {record_id}")

    category_slug = _string_or_none(record.get("category_slug"))
    category_title = _string_or_none(record.get("category_title")) or _category_title(
        repo, category_slug
    )
    label = _string_or_none(record.get("label"))
    tags = _coerce_string_list(record.get("tags"))

    if migrate_metadata:
        changed = False
        for key, value in (
            ("category_slug", category_slug),
            ("category_title", category_title),
            ("label", label),
            ("tags", tags),
        ):
            if value in (None, "", []):
                continue
            if record.get(key) != value:
                record[key] = value
                changed = True
        if "collections" in record:
            record.pop("collections", None)
            changed = True
        if changed:
            repo.write_json(record_path, record)

    display = record.get("display") if isinstance(record.get("display"), dict) else {}
    source = record.get("source") if isinstance(record.get("source"), dict) else {}
    creator = record.get("creator") if isinstance(record.get("creator"), dict) else {}
    lineage = record.get("lineage") if isinstance(record.get("lineage"), dict) else {}
    provenance_path = active_provenance_path(repo, record_id, record=record)
    provenance = repo.read_json(provenance_path, default=None) if provenance_path.exists() else None
    run_summary = provenance.get("run_summary") if isinstance(provenance, dict) else {}
    generation = provenance.get("generation") if isinstance(provenance, dict) else {}
    artifacts = record.get("artifacts") if isinstance(record.get("artifacts"), dict) else {}
    cost_path = (
        repo.layout.record_dir(record_id) / str(artifacts["cost_json"])
        if isinstance(artifacts.get("cost_json"), str) and artifacts["cost_json"]
        else active_cost_path(repo, record_id, record=record)
    )
    cost = repo.read_json(cost_path, default=None) if cost_path.exists() else None
    total_cost_usd, input_tokens, output_tokens = _cost_totals(cost)
    revision_root = repo.layout.record_revisions_dir(record_id)
    revision_count = (
        len([path for path in revision_root.iterdir() if path.is_dir()])
        if revision_root.is_dir()
        else 0
    )
    creator_mode = _string_or_none(creator.get("mode")) if isinstance(creator, dict) else None

    return {
        "schema_version": LIBRARY_MANIFEST_SCHEMA_VERSION,
        "record_id": record_id,
        "title": str(display.get("title") or label or record_id),
        "prompt_preview": str(display.get("prompt_preview") or ""),
        "category_slug": category_slug,
        "category_title": category_title,
        "label": label,
        "tags": tags,
        "rating": _coerce_rating(record.get("rating")),
        "secondary_rating": _coerce_rating(record.get("secondary_rating")),
        "author": _string_or_none(record.get("author")),
        "rated_by": _string_or_none(record.get("rated_by")),
        "secondary_rated_by": _string_or_none(record.get("secondary_rated_by")),
        "created_at": _string_or_none(record.get("created_at")),
        "updated_at": _string_or_none(record.get("updated_at")),
        "sdk_package": _string_or_none(record.get("sdk_package")),
        "provider": _string_or_none(record.get("provider")),
        "model_id": _string_or_none(record.get("model_id")),
        "creator_mode": creator_mode,
        "external_agent": _string_or_none(creator.get("agent"))
        if creator_mode == "external_agent"
        else None,
        "agent_harness": _agent_harness(creator),
        "has_traces": _has_traces(repo, record_id, record, creator),
        "thinking_level": _string_or_none(generation.get("thinking_level"))
        if isinstance(generation, dict)
        else None,
        "turn_count": _coerce_int(run_summary.get("turn_count"))
        if isinstance(run_summary, dict)
        else None,
        "input_tokens": input_tokens,
        "output_tokens": output_tokens,
        "total_cost_usd": total_cost_usd,
        "run_id": _string_or_none(source.get("run_id")) if isinstance(source, dict) else None,
        "run_status": _string_or_none(run_summary.get("final_status"))
        if isinstance(run_summary, dict)
        else None,
        "active_revision_id": _string_or_none(record.get("active_revision_id")),
        "origin_record_id": _string_or_none(lineage.get("origin_record_id")),
        "parent_record_id": _string_or_none(lineage.get("parent_record_id")),
        "revision_count": revision_count,
        "has_history": revision_count > 1
        or _string_or_none(lineage.get("parent_record_id")) is not None,
        "has_provenance": provenance_path.exists(),
        "has_cost": cost_path.exists(),
        "has_compile_report": repo.layout.record_materialization_compile_report_path(
            record_id
        ).exists(),
    }


def _write_rows(repo: StorageRepo, rows: Any) -> None:
    path = repo.layout.records_manifest_path
    path.parent.mkdir(parents=True, exist_ok=True)
    ordered = sorted(rows, key=lambda row: str(row.get("record_id") or ""))
    text = "".join(json.dumps(row, separators=(",", ":"), sort_keys=True) + "\n" for row in ordered)
    with tempfile.NamedTemporaryFile(
        "w",
        dir=path.parent,
        encoding="utf-8",
        prefix=f"{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as temporary:
        temporary_path = Path(temporary.name)
        temporary.write(text)
    try:
        os.replace(temporary_path, path)
    except OSError:
        temporary_path.unlink(missing_ok=True)
        raise


def _category_title(repo: StorageRepo, category_slug: str | None) -> str | None:
    if not category_slug:
        return None
    category = repo.read_json(repo.layout.category_metadata_path(category_slug), default={})
    if isinstance(category, dict):
        title = _string_or_none(category.get("title"))
        if title:
            return title
    return category_slug.replace("_", " ").title()


def _agent_harness(creator: dict[str, Any]) -> str:
    if creator.get("mode") == "external_agent":
        agent = _string_or_none(creator.get("agent"))
        if agent:
            return agent
    return "articraft"


def _has_traces(
    repo: StorageRepo,
    record_id: str,
    record: dict[str, Any],
    creator: dict[str, Any],
) -> bool:
    if creator.get("trace_available") is False:
        return False
    traces_dir = active_traces_dir(repo, record_id, record=record)
    return traces_dir.is_dir() and any(traces_dir.iterdir())
