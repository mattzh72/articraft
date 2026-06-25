from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient

from storage.library_manifest import rebuild_manifest
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload
from viewer.api.app import create_app


def _write_record(
    repo: StorageRepo,
    record_id: str,
    *,
    title: str,
    category_slug: str | None = None,
    rating: int | None = None,
) -> None:
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True, exist_ok=True)
    (revision_dir / "prompt.txt").write_text(f"make {title}", encoding="utf-8")
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
            "run_summary": {"turn_count": 1, "final_status": "success"},
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
            "source": {"run_id": None, "prompt_index": None},
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-test",
            "display": {"title": title, "prompt_preview": f"make {title}"},
            "artifacts": revision_artifacts_payload(
                revision_id=INITIAL_REVISION_ID,
                has_cost_file=False,
            ),
            "hashes": {},
            "active_revision_id": INITIAL_REVISION_ID,
        },
    )


def _client(tmp_path: Path) -> tuple[TestClient, StorageRepo]:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    return TestClient(create_app(repo_root=tmp_path)), repo


def test_bootstrap_and_browse_records_from_manifest(tmp_path: Path) -> None:
    client, repo = _client(tmp_path)
    _write_record(repo, "rec_lamp", title="Desk Lamp", category_slug="desk_lamp", rating=5)
    rebuild_manifest(repo)

    bootstrap = client.get("/api/bootstrap")
    assert bootstrap.status_code == 200
    assert bootstrap.json()["library_records"][0]["record_id"] == "rec_lamp"

    browse = client.get("/api/records/browse", params={"q": "lamp"})
    assert browse.status_code == 200
    payload = browse.json()
    assert payload["total"] == 1
    assert payload["records"][0]["title"] == "Desk Lamp"
    assert payload["facets"]["categories"] == ["desk_lamp"]


def test_record_summary_search_and_stats(tmp_path: Path) -> None:
    client, repo = _client(tmp_path)
    _write_record(repo, "rec_lamp", title="Desk Lamp", rating=4)
    rebuild_manifest(repo)

    summary = client.get("/api/records/rec_lamp/summary")
    assert summary.status_code == 200
    assert summary.json()["effective_rating"] == 4

    search = client.get("/api/records/search", params={"q": "desk"})
    assert search.status_code == 200
    assert [row["record_id"] for row in search.json()] == ["rec_lamp"]

    stats = client.get("/api/stats")
    assert stats.status_code == 200
    assert stats.json()["total_records"] == 1
    assert stats.json()["rating_distribution"] == {"4": 1}


def test_rating_update_upserts_manifest(tmp_path: Path) -> None:
    client, repo = _client(tmp_path)
    _write_record(repo, "rec_lamp", title="Desk Lamp")
    rebuild_manifest(repo)

    response = client.put("/api/records/rec_lamp/rating", json={"rating": 5})

    assert response.status_code == 200
    assert response.json()["rating"] == 5
    [row] = rebuild_manifest(repo, migrate_metadata=False)
    assert row["rating"] == 5


def test_delete_record_removes_record_and_manifest_row(tmp_path: Path) -> None:
    client, repo = _client(tmp_path)
    _write_record(repo, "rec_lamp", title="Desk Lamp")
    rebuild_manifest(repo)

    response = client.delete("/api/records/rec_lamp")

    assert response.status_code == 200
    assert not repo.layout.record_dir("rec_lamp").exists()
    assert repo.layout.records_manifest_path.read_text(encoding="utf-8") == ""
