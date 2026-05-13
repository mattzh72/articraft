from __future__ import annotations

import os
import subprocess
from pathlib import Path

from storage.models import DisplayMetadata, Record, RecordArtifacts, SourceRef
from storage.record_authors import (
    sync_record_authors,
    sync_record_rated_by,
    sync_record_secondary_rated_by,
)
from storage.records import RecordStore
from storage.repo import StorageRepo


def _git(repo_root: Path, *args: str, env: dict[str, str] | None = None) -> None:
    subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
        env={**os.environ, **(env or {})},
    )


def test_sync_record_authors_backfills_from_model_py_add_commit(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_author_001"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_author_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Author test", prompt_preview="author test"),
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
    (repo.layout.record_dir(record_id) / "model.py").write_text("# test model\n", encoding="utf-8")

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add author test record",
        env={
            "GIT_AUTHOR_NAME": "Model Author",
            "GIT_AUTHOR_EMAIL": "author@example.com",
            "GIT_COMMITTER_NAME": "Model Author",
            "GIT_COMMITTER_EMAIL": "author@example.com",
        },
    )

    summary = sync_record_authors(repo)
    record = repo.read_json(repo.layout.record_metadata_path(record_id))

    assert record["author"] == "Model Author"
    assert summary.updated_record_ids == [record_id]
    assert summary.already_set_record_ids == []
    assert summary.missing_git_author_record_ids == []


def test_sync_record_authors_preserves_existing_author_without_mutation(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_author_002"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_author_002"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Author keep", prompt_preview="author keep"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
            author="Manual Author",
        )
    )
    (repo.layout.record_dir(record_id) / "model.py").write_text("# test model\n", encoding="utf-8")

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add author keep record",
        env={
            "GIT_AUTHOR_NAME": "Original Author",
            "GIT_AUTHOR_EMAIL": "original@example.com",
            "GIT_COMMITTER_NAME": "Original Author",
            "GIT_COMMITTER_EMAIL": "original@example.com",
        },
    )

    summary = sync_record_authors(repo)
    record = repo.read_json(repo.layout.record_metadata_path(record_id))

    assert record["author"] == "Manual Author"
    assert summary.updated_record_ids == []
    assert summary.already_set_record_ids == [record_id]


def test_sync_record_rated_by_backfills_from_latest_rating_line_blame(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_rating_001"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_rating_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Rating test", prompt_preview="rating test"),
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

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add unrated record",
        env={
            "GIT_AUTHOR_NAME": "Initial Author",
            "GIT_AUTHOR_EMAIL": "initial@example.com",
            "GIT_COMMITTER_NAME": "Initial Author",
            "GIT_COMMITTER_EMAIL": "initial@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["rating"] = 5
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Set rating",
        env={
            "GIT_AUTHOR_NAME": "Rating Author",
            "GIT_AUTHOR_EMAIL": "rating@example.com",
            "GIT_COMMITTER_NAME": "Rating Author",
            "GIT_COMMITTER_EMAIL": "rating@example.com",
        },
    )

    summary = sync_record_rated_by(repo)
    record = repo.read_json(record_path)

    assert record["rated_by"] == "Rating Author"
    assert summary.updated_record_ids == [record_id]
    assert summary.unchanged_record_ids == []
    assert summary.missing_git_author_record_ids == []


def test_sync_record_rated_by_updates_existing_value_when_rating_changes(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_rating_002"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_rating_002"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Rating keep", prompt_preview="rating keep"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
            rated_by="Existing Reviewer",
        )
    )

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add initially rated-by record",
        env={
            "GIT_AUTHOR_NAME": "Initial Author",
            "GIT_AUTHOR_EMAIL": "initial@example.com",
            "GIT_COMMITTER_NAME": "Initial Author",
            "GIT_COMMITTER_EMAIL": "initial@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["rating"] = 3
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Change rating",
        env={
            "GIT_AUTHOR_NAME": "Updated Reviewer",
            "GIT_AUTHOR_EMAIL": "updated@example.com",
            "GIT_COMMITTER_NAME": "Updated Reviewer",
            "GIT_COMMITTER_EMAIL": "updated@example.com",
        },
    )

    summary = sync_record_rated_by(repo)
    record = repo.read_json(record_path)

    assert record["rated_by"] == "Updated Reviewer"
    assert summary.updated_record_ids == [record_id]
    assert summary.unchanged_record_ids == []


def test_sync_record_rated_by_ignores_secondary_rating_line_updates(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_rating_003"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_rating_003"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Secondary rating isolation", prompt_preview="secondary"),
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

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add record",
        env={
            "GIT_AUTHOR_NAME": "Primary Reviewer",
            "GIT_AUTHOR_EMAIL": "primary@example.com",
            "GIT_COMMITTER_NAME": "Primary Reviewer",
            "GIT_COMMITTER_EMAIL": "primary@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["secondary_rating"] = 5
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add secondary rating",
        env={
            "GIT_AUTHOR_NAME": "Secondary Reviewer",
            "GIT_AUTHOR_EMAIL": "secondary@example.com",
            "GIT_COMMITTER_NAME": "Secondary Reviewer",
            "GIT_COMMITTER_EMAIL": "secondary@example.com",
        },
    )

    primary_summary = sync_record_rated_by(repo)
    secondary_summary = sync_record_secondary_rated_by(repo)
    persisted = repo.read_json(record_path)

    assert persisted["rated_by"] == "Primary Reviewer"
    assert persisted["secondary_rated_by"] == "Secondary Reviewer"
    assert primary_summary.updated_record_ids == [record_id]
    assert secondary_summary.updated_record_ids == [record_id]


def test_sync_record_secondary_rated_by_clears_when_secondary_rating_is_null(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_rating_004"
    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id=record_id,
            created_at="2026-03-20T00:00:00Z",
            updated_at="2026-03-20T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_rating_004"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Secondary rating clear", prompt_preview="secondary"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["dataset"],
            secondary_rating=4,
            secondary_rated_by="Existing Secondary Reviewer",
        )
    )

    _git(tmp_path, "init")
    _git(tmp_path, "config", "user.name", "Test User")
    _git(tmp_path, "config", "user.email", "test@example.com")
    _git(tmp_path, "add", ".")
    _git(
        tmp_path,
        "commit",
        "-m",
        "Add rated secondary record",
        env={
            "GIT_AUTHOR_NAME": "Initial Author",
            "GIT_AUTHOR_EMAIL": "initial@example.com",
            "GIT_COMMITTER_NAME": "Initial Author",
            "GIT_COMMITTER_EMAIL": "initial@example.com",
        },
    )

    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    record["secondary_rating"] = None
    repo.write_json(record_path, record)
    _git(tmp_path, "add", str(record_path.relative_to(tmp_path)))
    _git(
        tmp_path,
        "commit",
        "-m",
        "Clear secondary rating",
        env={
            "GIT_AUTHOR_NAME": "Clearing Reviewer",
            "GIT_AUTHOR_EMAIL": "clearing@example.com",
            "GIT_COMMITTER_NAME": "Clearing Reviewer",
            "GIT_COMMITTER_EMAIL": "clearing@example.com",
        },
    )

    summary = sync_record_secondary_rated_by(repo)
    persisted = repo.read_json(record_path)

    assert persisted["secondary_rated_by"] is None
    assert summary.updated_record_ids == [record_id]
