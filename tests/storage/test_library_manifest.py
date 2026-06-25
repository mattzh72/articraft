from __future__ import annotations

import json
from pathlib import Path

import pytest

from storage.library_manifest import (
    load_manifest,
    rebuild_manifest,
    record_id_available,
    remove_record,
    upsert_record,
    validate_manifest,
)
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload


def _write_record(
    repo: StorageRepo,
    record_id: str,
    *,
    title: str = "Desk Lamp",
    category_slug: str | None = None,
    label: str | None = None,
    tags: list[str] | None = None,
    rating: int | None = None,
) -> Path:
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "prompt.txt").write_text("make a desk lamp", encoding="utf-8")
    (revision_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    repo.write_json(
        revision_dir / "provenance.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "generation": {
                "provider": "openai",
                "model_id": "gpt-test",
                "thinking_level": "high",
            },
            "run_summary": {"turn_count": 2, "final_status": "success"},
        },
    )
    repo.write_json(
        repo.layout.record_metadata_path(record_id),
        {
            "schema_version": 3,
            "record_id": record_id,
            "created_at": "2026-01-01T00:00:00Z",
            "updated_at": "2026-01-02T00:00:00Z",
            "rating": rating,
            "kind": "generated_model",
            "prompt_kind": "single_prompt",
            "category_slug": category_slug,
            "source": {"run_id": "run_123", "prompt_index": None},
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-test",
            "label": label,
            "tags": tags or [],
            "display": {"title": title, "prompt_preview": "make a desk lamp"},
            "artifacts": revision_artifacts_payload(
                revision_id=INITIAL_REVISION_ID,
                has_cost_file=False,
            ),
            "hashes": {},
            "active_revision_id": INITIAL_REVISION_ID,
        },
    )
    return repo.layout.record_dir(record_id)


def test_rebuild_manifest_writes_one_row_per_complete_record(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(
        repo,
        "rec_lamp",
        category_slug="desk_lamp",
        label="curated",
        tags=["hinged"],
        rating=5,
    )

    rows = rebuild_manifest(repo)

    assert [row["record_id"] for row in rows] == ["rec_lamp"]
    assert rows[0]["title"] == "Desk Lamp"
    assert rows[0]["category_slug"] == "desk_lamp"
    assert rows[0]["label"] == "curated"
    assert rows[0]["tags"] == ["hinged"]
    assert rows[0]["rating"] == 5
    assert load_manifest(repo) == rows


def test_upsert_and_remove_manifest_row(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(repo, "rec_lamp", title="Lamp")

    row = upsert_record(repo, "rec_lamp")
    assert row["record_id"] == "rec_lamp"
    assert load_manifest(repo)[0]["title"] == "Lamp"

    remove_record(repo, "rec_lamp")
    assert load_manifest(repo) == []


def test_record_id_availability_checks_manifest_and_records(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    assert record_id_available(repo, "rec_lamp") is True
    _write_record(repo, "rec_lamp")
    assert record_id_available(repo, "rec_lamp") is False

    record_dir = repo.layout.record_dir("rec_lamp")
    for path in sorted(record_dir.rglob("*"), reverse=True):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
    record_dir.rmdir()
    repo.layout.records_manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "record_id": "rec_lamp",
            }
        )
        + "\n",
        encoding="utf-8",
    )

    assert record_id_available(repo, "rec_lamp") is False


def test_validate_manifest_rejects_missing_records(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    repo.layout.records_manifest_path.write_text(
        json.dumps({"schema_version": 1, "record_id": "rec_missing"}) + "\n",
        encoding="utf-8",
    )

    errors = validate_manifest(repo, require_records=True)

    assert errors
    assert "record payload is missing" in errors[0]


def test_rebuild_manifest_removes_legacy_collection_marker(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(repo, "rec_lamp", category_slug="lamps", label="legacy", tags=["old"])
    record_path = repo.layout.record_metadata_path("rec_lamp")
    record = repo.read_json(record_path)
    record["collections"] = ["legacy"]
    repo.write_json(record_path, record)

    [row] = rebuild_manifest(repo)
    migrated = repo.read_json(record_path)

    assert row["category_slug"] == "lamps"
    assert row["label"] == "legacy"
    assert row["tags"] == ["old"]
    assert migrated["category_slug"] == "lamps"
    assert migrated["label"] == "legacy"
    assert "collections" not in migrated


def test_load_manifest_rejects_duplicate_record_ids(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    row = {"schema_version": 1, "record_id": "rec_dup"}
    repo.layout.records_manifest_path.write_text(
        json.dumps(row) + "\n" + json.dumps(row) + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate record_id"):
        load_manifest(repo)
