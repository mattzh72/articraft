from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from scripts import git_hooks
from storage.models import DisplayMetadata, Record, RecordArtifacts, SourceRef
from storage.records import RecordStore
from storage.repo import StorageRepo


def _git(repo_root: Path, *args: str, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
        env={**os.environ, **(env or {})},
    )
    return result.stdout.strip()


def _init_git_repo(repo_root: Path) -> None:
    _git(repo_root, "init")
    _git(repo_root, "config", "user.name", "Test User")
    _git(repo_root, "config", "user.email", "test@example.com")


def test_run_post_commit_record_author_sync_amends_latest_commit(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_id = "rec_hook_001"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-30T10:00:00Z",
            updated_at="2026-03-30T10:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_hook_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Hook", prompt_preview="hook"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
        )
    )
    (repo.layout.record_dir(record_id) / "model.py").write_text("# hook model\n", encoding="utf-8")

    _init_git_repo(tmp_path)
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add hook record",
        env={
            "GIT_AUTHOR_NAME": "Ruining Li",
            "GIT_AUTHOR_EMAIL": "ruining@example.com",
            "GIT_COMMITTER_NAME": "Ruining Li",
            "GIT_COMMITTER_EMAIL": "ruining@example.com",
        },
    )
    original_head = _git(tmp_path, "rev-parse", "HEAD")

    result = git_hooks.run_post_commit_record_author_sync(tmp_path)

    updated_record = repo.read_json(repo.layout.record_metadata_path(record_id))
    current_head = _git(tmp_path, "rev-parse", "HEAD")
    assert result.touched_record_ids == [record_id]
    assert result.touched_record_metadata_ids == [record_id]
    assert result.updated_author_record_ids == [record_id]
    assert result.updated_rated_by_record_ids == [record_id]
    assert result.updated_secondary_rated_by_record_ids == []
    assert result.updated_record_ids == [record_id]
    assert updated_record["author"] == "RuiningLi"
    assert updated_record["rated_by"] == "RuiningLi"
    assert updated_record["secondary_rated_by"] is None
    assert current_head != original_head
    assert _git(tmp_path, "rev-list", "--count", "HEAD") == "1"
    assert _git(tmp_path, "log", "-1", "--format=%an") == "Ruining Li"


def test_run_post_commit_record_author_sync_leaves_initialized_author_unchanged(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_id = "rec_hook_002"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-30T10:00:00Z",
            updated_at="2026-03-30T10:00:00Z",
            rating=None,
            author="mattzh72",
            rated_by="mattzh72",
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_hook_002"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Hook", prompt_preview="hook"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
        )
    )
    model_path = repo.layout.record_dir(record_id) / "model.py"
    model_path.write_text("# hook model\n", encoding="utf-8")

    _init_git_repo(tmp_path)
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add initialized hook record",
        env={
            "GIT_AUTHOR_NAME": "Matthew Zhou",
            "GIT_AUTHOR_EMAIL": "matt@example.com",
            "GIT_COMMITTER_NAME": "Matthew Zhou",
            "GIT_COMMITTER_EMAIL": "matt@example.com",
        },
    )

    model_path.write_text("# hook model v2\n", encoding="utf-8")
    _git(tmp_path, "add", str(model_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Modify model",
        env={
            "GIT_AUTHOR_NAME": "shawlyu",
            "GIT_AUTHOR_EMAIL": "shawlyu@example.com",
            "GIT_COMMITTER_NAME": "shawlyu",
            "GIT_COMMITTER_EMAIL": "shawlyu@example.com",
        },
    )
    original_head = _git(tmp_path, "rev-parse", "HEAD")

    result = git_hooks.run_post_commit_record_author_sync(tmp_path)

    persisted = repo.read_json(repo.layout.record_metadata_path(record_id))
    assert result.touched_record_ids == [record_id]
    assert result.touched_record_metadata_ids == []
    assert result.updated_author_record_ids == []
    assert result.updated_rated_by_record_ids == []
    assert result.updated_secondary_rated_by_record_ids == []
    assert result.updated_record_ids == []
    assert persisted["author"] == "mattzh72"
    assert persisted["rated_by"] == "mattzh72"
    assert persisted["secondary_rated_by"] is None
    assert _git(tmp_path, "rev-parse", "HEAD") == original_head


def test_run_post_commit_record_author_sync_updates_rated_by_when_rating_changes(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_id = "rec_hook_003"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-30T10:00:00Z",
            updated_at="2026-03-30T10:00:00Z",
            rating=None,
            author="mattzh72",
            rated_by="mattzh72",
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_hook_003"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Hook", prompt_preview="hook"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
        )
    )

    _init_git_repo(tmp_path)
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add record with existing rated_by",
        env={
            "GIT_AUTHOR_NAME": "Matthew Zhou",
            "GIT_AUTHOR_EMAIL": "matt@example.com",
            "GIT_COMMITTER_NAME": "Matthew Zhou",
            "GIT_COMMITTER_EMAIL": "matt@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["rating"] = 4
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Update rating",
        env={
            "GIT_AUTHOR_NAME": "shawlyu",
            "GIT_AUTHOR_EMAIL": "shawlyu@example.com",
            "GIT_COMMITTER_NAME": "shawlyu",
            "GIT_COMMITTER_EMAIL": "shawlyu@example.com",
        },
    )
    original_head = _git(tmp_path, "rev-parse", "HEAD")

    result = git_hooks.run_post_commit_record_author_sync(tmp_path)

    persisted = repo.read_json(record_path)
    assert result.touched_record_ids == [record_id]
    assert result.touched_record_metadata_ids == [record_id]
    assert result.updated_author_record_ids == []
    assert result.updated_rated_by_record_ids == [record_id]
    assert result.updated_secondary_rated_by_record_ids == []
    assert persisted["author"] == "mattzh72"
    assert persisted["rated_by"] == "shawlyu"
    assert _git(tmp_path, "rev-parse", "HEAD") != original_head


def test_run_post_commit_record_author_sync_updates_secondary_rated_by_when_secondary_rating_changes(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_id = "rec_hook_004"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-30T10:00:00Z",
            updated_at="2026-03-30T10:00:00Z",
            rating=None,
            author="mattzh72",
            rated_by="mattzh72",
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_hook_004"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Hook", prompt_preview="hook"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
        )
    )

    _init_git_repo(tmp_path)
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add record with no secondary rating",
        env={
            "GIT_AUTHOR_NAME": "Matthew Zhou",
            "GIT_AUTHOR_EMAIL": "matt@example.com",
            "GIT_COMMITTER_NAME": "Matthew Zhou",
            "GIT_COMMITTER_EMAIL": "matt@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["secondary_rating"] = 2
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Update secondary rating",
        env={
            "GIT_AUTHOR_NAME": "shawlyu",
            "GIT_AUTHOR_EMAIL": "shawlyu@example.com",
            "GIT_COMMITTER_NAME": "shawlyu",
            "GIT_COMMITTER_EMAIL": "shawlyu@example.com",
        },
    )
    original_head = _git(tmp_path, "rev-parse", "HEAD")

    result = git_hooks.run_post_commit_record_author_sync(tmp_path)

    persisted = repo.read_json(record_path)
    assert result.touched_record_ids == [record_id]
    assert result.touched_record_metadata_ids == [record_id]
    assert result.updated_author_record_ids == []
    assert result.updated_rated_by_record_ids == []
    assert result.updated_secondary_rated_by_record_ids == [record_id]
    assert persisted["secondary_rated_by"] == "shawlyu"
    assert _git(tmp_path, "rev-parse", "HEAD") != original_head


def test_install_post_commit_hook_refuses_to_overwrite_unmanaged_hook(tmp_path: Path) -> None:
    _init_git_repo(tmp_path)
    hook_path = tmp_path / ".git" / "hooks" / "post-commit"
    hook_path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Refusing to overwrite unmanaged post-commit hook"):
        git_hooks.install_post_commit_hook(tmp_path)


def test_get_post_commit_hook_status_returns_missing_when_hook_absent(tmp_path: Path) -> None:
    _init_git_repo(tmp_path)

    status = git_hooks.get_post_commit_hook_status(tmp_path)

    assert status.status == "missing"
    assert status.hook_path == tmp_path / ".git" / "hooks" / "post-commit"
    assert status.installed is False


def test_get_post_commit_hook_status_returns_installed_for_managed_hook(tmp_path: Path) -> None:
    _init_git_repo(tmp_path)
    hook_path = git_hooks.install_post_commit_hook(tmp_path)

    status = git_hooks.get_post_commit_hook_status(tmp_path)

    assert status.status == "installed"
    assert status.hook_path == hook_path
    assert status.installed is True


def test_get_post_commit_hook_status_returns_unmanaged_for_custom_hook(tmp_path: Path) -> None:
    _init_git_repo(tmp_path)
    hook_path = tmp_path / ".git" / "hooks" / "post-commit"
    hook_path.parent.mkdir(parents=True, exist_ok=True)
    hook_path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    status = git_hooks.get_post_commit_hook_status(tmp_path)

    assert status.status == "unmanaged"
    assert status.hook_path == hook_path
    assert status.installed is False
