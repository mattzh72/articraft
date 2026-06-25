from __future__ import annotations

from pathlib import Path

from storage.data_validation import validate_data_format
from storage.library_manifest import rebuild_manifest
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload


def _write_record(repo: StorageRepo, record_id: str, *, category_slug: str | None = None) -> None:
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "prompt.txt").write_text("make a lamp", encoding="utf-8")
    (revision_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    repo.write_json(revision_dir / "provenance.json", {"schema_version": 2, "record_id": record_id})
    repo.write_json(
        repo.layout.record_metadata_path(record_id),
        {
            "schema_version": 3,
            "record_id": record_id,
            "created_at": "2026-01-01T00:00:00Z",
            "updated_at": "2026-01-01T00:00:00Z",
            "rating": None,
            "kind": "generated_model",
            "prompt_kind": "single_prompt",
            "category_slug": category_slug,
            "source": {"run_id": None, "prompt_index": None},
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-test",
            "display": {"title": "Lamp", "prompt_preview": "make a lamp"},
            "artifacts": revision_artifacts_payload(
                revision_id=INITIAL_REVISION_ID,
                has_cost_file=False,
            ),
            "hashes": {},
            "active_revision_id": INITIAL_REVISION_ID,
        },
    )


def test_validate_data_format_accepts_complete_local_library(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    repo.write_json(
        repo.layout.category_metadata_path("desk_lamp"),
        {"schema_version": 1, "slug": "desk_lamp", "title": "Desk Lamp"},
    )
    _write_record(repo, "rec_lamp", category_slug="desk_lamp")
    rebuild_manifest(repo)

    result = validate_data_format(repo, require_records=True)

    assert result.ok
    assert result.category_count == 1
    assert result.record_count == 1
    assert result.manifest_count == 1


def test_validate_data_format_reports_missing_active_files(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(repo, "rec_lamp")
    repo.layout.record_revision_model_path("rec_lamp", INITIAL_REVISION_ID).unlink()
    rebuild_manifest(repo)

    result = validate_data_format(repo, require_records=True)

    assert not result.ok
    assert any("missing active model.py" in error for error in result.errors)


def test_validate_data_format_reports_requested_missing_record(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    result = validate_data_format(repo, record_ids=["rec_missing"])

    assert not result.ok
    assert any("record not found" in error for error in result.errors)
