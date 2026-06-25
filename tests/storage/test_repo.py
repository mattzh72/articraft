from __future__ import annotations

from pathlib import Path

from storage.library_manifest import load_manifest
from storage.models import DisplayMetadata, Record, RecordArtifacts, SourceRef
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload


def _record(record_id: str) -> Record:
    artifacts = revision_artifacts_payload(
        revision_id=INITIAL_REVISION_ID,
        has_cost_file=False,
    )
    return Record(
        schema_version=3,
        record_id=record_id,
        created_at="2026-01-01T00:00:00Z",
        updated_at="2026-01-01T00:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug=None,
        source=SourceRef(run_id=None),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-test",
        display=DisplayMetadata(title="Lamp", prompt_preview="make a lamp"),
        artifacts=RecordArtifacts(**artifacts),
        active_revision_id=INITIAL_REVISION_ID,
    )


def _write_active_files(repo: StorageRepo, record_id: str) -> None:
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "prompt.txt").write_text("make a lamp", encoding="utf-8")
    (revision_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    repo.write_json(revision_dir / "provenance.json", {"schema_version": 2, "record_id": record_id})


def test_storage_repo_json_round_trip(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    path = repo.layout.data_root / "local.json"
    repo.write_json(path, {"ok": True})

    assert repo.read_json(path) == {"ok": True}


def test_record_store_write_upserts_manifest(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_active_files(repo, "rec_lamp")

    RecordStore(repo).write_record(_record("rec_lamp"))

    rows = load_manifest(repo)
    assert len(rows) == 1
    assert rows[0]["record_id"] == "rec_lamp"
    assert rows[0]["title"] == "Lamp"


def test_record_store_delete_removes_record_and_manifest_row(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_active_files(repo, "rec_lamp")
    store = RecordStore(repo)
    store.write_record(_record("rec_lamp"))

    assert store.delete_record("rec_lamp") is True

    assert not repo.layout.record_dir("rec_lamp").exists()
    assert load_manifest(repo) == []
