from __future__ import annotations

import json
from pathlib import Path

from cli.common import provider_for_record_image, refresh_dataset_manifest_if_member
from storage.repo import StorageRepo


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def test_provider_for_record_image_prefers_override_then_record_provider(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_json(
        repo.layout.record_metadata_path("rec_gemini"),
        {
            "schema_version": 3,
            "record_id": "rec_gemini",
            "provider": "gemini",
            "active_revision_id": "rev_000001",
        },
    )

    assert provider_for_record_image(repo, "rec_gemini", provider_override="openai") == "openai"
    assert provider_for_record_image(repo, "rec_gemini") == "gemini"


def test_refresh_dataset_manifest_if_member_writes_manifest(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_json(
        repo.layout.record_dataset_entry_path("rec_dataset"),
        {
            "schema_version": 1,
            "record_id": "rec_dataset",
            "dataset_id": "ds_dataset",
            "category_slug": "hinge",
            "promoted_at": "2026-03-18T00:01:00Z",
        },
    )

    assert refresh_dataset_manifest_if_member(repo, "rec_dataset") is True
    assert refresh_dataset_manifest_if_member(repo, "rec_other") is False

    manifest = json.loads(repo.layout.dataset_manifest_path().read_text(encoding="utf-8"))
    assert manifest == {"generated": [{"name": "ds_dataset", "record_id": "rec_dataset"}]}
