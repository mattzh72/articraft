from __future__ import annotations

from pathlib import Path

from cli.common import provider_for_record_image, resolve_data_dir, storage_repo_from_args
from storage.repo import StorageRepo
from storage.revisions import INITIAL_REVISION_ID, revision_artifacts_payload


class Args:
    def __init__(self, repo_root: Path, data_dir: Path | None = None) -> None:
        self.repo_root = repo_root
        self.data_dir = data_dir


def test_resolve_data_dir_prefers_explicit_path(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setenv("ARTICRAFT_DATA_DIR", str(tmp_path / "env-data"))

    assert resolve_data_dir(tmp_path, tmp_path / "explicit") == (tmp_path / "explicit").resolve()


def test_resolve_data_dir_uses_env_then_repo_default(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setenv("ARTICRAFT_DATA_DIR", str(tmp_path / "env-data"))
    assert resolve_data_dir(tmp_path) == (tmp_path / "env-data").resolve()

    monkeypatch.delenv("ARTICRAFT_DATA_DIR")
    assert resolve_data_dir(tmp_path) == (tmp_path / "data").resolve()


def test_storage_repo_from_args_uses_resolved_data_dir(tmp_path: Path) -> None:
    repo = storage_repo_from_args(Args(tmp_path, tmp_path / "external-data"))

    assert repo.layout.data_root == (tmp_path / "external-data").resolve()


def test_provider_for_record_image_prefers_record_metadata(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_id = "rec_lamp"
    revision_dir = repo.layout.record_revision_dir(record_id, INITIAL_REVISION_ID)
    revision_dir.mkdir(parents=True)
    repo.write_json(
        repo.layout.record_metadata_path(record_id),
        {
            "schema_version": 3,
            "record_id": record_id,
            "provider": "gemini",
            "artifacts": revision_artifacts_payload(
                revision_id=INITIAL_REVISION_ID,
                has_cost_file=False,
            ),
            "active_revision_id": INITIAL_REVISION_ID,
        },
    )

    assert provider_for_record_image(repo, record_id) == "gemini"
