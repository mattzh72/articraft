from __future__ import annotations

import io
import json
import subprocess
import sys
import types
from contextlib import redirect_stdout
from pathlib import Path

import pytest

from agent import runner
from cli.dataset import main as dataset_main
from scripts import git_hooks
from storage import dataset_workflow
from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.models import (
    CategoryRecord,
    DisplayMetadata,
    Record,
    RecordArtifacts,
    RunRecord,
    SourceRef,
)
from storage.record_authors import RecordAuthorSyncSummary, RecordRatedBySyncSummary
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from tests.helpers import FakeAgent


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FakeAgent)


def _init_git_repo(repo_root: Path) -> None:
    subprocess.run(["git", "init"], cwd=repo_root, check=True, capture_output=True, text=True)


def _create_workbench_record(
    repo: StorageRepo,
    *,
    record_id: str,
    run_id: str,
    title: str = "CLI",
) -> None:
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id=record_id,
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=None,
            source=SourceRef(run_id=run_id),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title=title, prompt_preview="cli prompt"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["workbench"],
        )
    )
    CollectionStore(repo).append_workbench_entry(
        record_id=record_id,
        added_at="2026-03-18T00:00:30Z",
        label=f"{title} entry",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id=run_id,
            run_mode="workbench_single",
            collection="workbench",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug=None,
            prompt_count=1,
        )
    )


def _patch_dataset_tokens(monkeypatch: pytest.MonkeyPatch, *tokens: str) -> None:
    iterator = iter(tokens)
    monkeypatch.setattr(dataset_workflow, "new_dataset_token", lambda: next(iterator))


def test_dataset_cli_commands_cover_promote_delete_and_prune(tmp_path: Path) -> None:
    repo_root = tmp_path
    repo = StorageRepo(repo_root)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="hinges",
            title="Hinges",
            prompt_batch_ids=["seed_batch"],
        )
    )

    record = Record(
        schema_version=1,
        record_id="rec_cli",
        created_at="2026-03-18T00:00:00Z",
        updated_at="2026-03-18T00:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_cli"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(title="CLI", prompt_preview="cli prompt"),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    RecordStore(repo).write_record(record)
    CollectionStore(repo).append_workbench_entry(
        record_id="rec_cli",
        added_at="2026-03-18T00:00:30Z",
        label="CLI entry",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_cli",
            run_mode="dataset_single",
            collection="dataset",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug="hinges",
            prompt_count=1,
        )
    )
    repo.layout.search_index_path().write_text("search-cache-placeholder", encoding="utf-8")
    legacy_search_sqlite_path = repo.layout.cache_root / "search.sqlite"
    legacy_search_sqlite_path.write_bytes(b"legacy sqlite placeholder")
    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "promote",
                    "--record-id",
                    "rec_cli",
                    "--dataset-id",
                    "cli_dataset_001",
                    "--promoted-at",
                    "2026-03-18T00:01:00Z",
                ]
            )
            == 0
        )
        category = json.loads(
            repo.layout.category_metadata_path("hinges").read_text(encoding="utf-8")
        )
        assert category["prompt_batch_ids"] == ["seed_batch"]
        assert dataset_main(["--repo-root", str(repo_root), "validate"]) == 0
        assert dataset_main(["--repo-root", str(repo_root), "build-manifest"]) == 0
        assert dataset_main(["--repo-root", str(repo_root), "status"]) == 0
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-record",
                    "--record-path",
                    str(repo.layout.record_dir("rec_cli")),
                ]
            )
            == 0
        )
        assert repo.layout.record_metadata_path("rec_cli").exists()
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-record",
                    "--record-path",
                    str(repo.layout.record_dir("rec_cli")),
                    "--execute",
                    "--confirm-record-id",
                    "wrong-record-id",
                ]
            )
            == 1
        )
        assert repo.layout.record_metadata_path("rec_cli").exists()
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-record",
                    "--record-path",
                    str(repo.layout.record_dir("rec_cli")),
                    "--execute",
                    "--confirm-record-id",
                    "rec_cli",
                ]
            )
            == 0
        )
        assert not repo.layout.record_dir("rec_cli").exists()
        category = json.loads(
            repo.layout.category_metadata_path("hinges").read_text(encoding="utf-8")
        )
        assert category["prompt_batch_ids"] == ["seed_batch"]
        assert repo.layout.run_metadata_path("run_cli").exists()
        assert repo.layout.search_index_path().exists()
        assert (CollectionStore(repo).load_workbench() or {}).get("entries", []) == []
        assert DatasetStore(repo).list_entries() == []
        empty_record_stage_dir = repo.layout.run_staging_dir("run_cli") / "rec_empty"
        empty_record_stage_dir.mkdir(parents=True, exist_ok=True)
        empty_nested_failure_dir = repo.layout.run_failures_dir("run_cli") / "rec_empty" / "logs"
        empty_nested_failure_dir.mkdir(parents=True, exist_ok=True)
        nonempty_stage_dir = repo.layout.run_staging_dir("run_cli") / "rec_keep"
        nonempty_stage_dir.mkdir(parents=True, exist_ok=True)
        (nonempty_stage_dir / "artifact.txt").write_text("keep", encoding="utf-8")
        failed_stage_dir = repo.layout.run_staging_dir("run_cli") / "rec_failed"
        failed_stage_dir.mkdir(parents=True, exist_ok=True)
        (failed_stage_dir / "artifact.txt").write_text("failed", encoding="utf-8")
        RunStore(repo).append_result(
            "run_cli",
            {
                "record_id": "rec_failed",
                "status": "failed",
                "staging_dir": "data/cache/runs/run_cli/staging/rec_failed",
            },
        )
        search_index_before_prune = repo.layout.search_index_path().read_bytes()
        assert legacy_search_sqlite_path.exists()
        assert dataset_main(["--repo-root", str(repo_root), "prune-cache"]) == 0
        assert empty_record_stage_dir.exists()
        assert empty_nested_failure_dir.exists()
        assert nonempty_stage_dir.exists()
        assert failed_stage_dir.exists()
        assert legacy_search_sqlite_path.exists()
        assert dataset_main(["--repo-root", str(repo_root), "prune-cache", "--execute"]) == 0
        assert not empty_record_stage_dir.exists()
        assert not empty_nested_failure_dir.exists()
        assert nonempty_stage_dir.exists()
        assert not failed_stage_dir.exists()
        assert not legacy_search_sqlite_path.exists()
        assert repo.layout.search_index_path().read_bytes() == search_index_before_prune

    category_output = io.StringIO()
    with redirect_stdout(category_output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-category",
                    "--category-slug",
                    "hinges",
                ]
            )
            == 0
        )
        assert repo.layout.category_metadata_path("hinges").exists()
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-category",
                    "--category-slug",
                    "hinges",
                    "--execute",
                    "--confirm-slug",
                    "wrong-slug",
                ]
            )
            == 1
        )
        assert repo.layout.category_metadata_path("hinges").exists()
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(repo_root),
                    "delete-category",
                    "--category-slug",
                    "hinges",
                    "--execute",
                    "--confirm-slug",
                    "hinges",
                ]
            )
            == 0
        )

    manifest = repo.layout.dataset_manifest_path().read_text(encoding="utf-8")
    assert '"generated": []' in manifest
    assert "dataset_entries=1" in output.getvalue()
    assert "Preview only. Re-run with --execute --confirm-record-id rec_cli" in output.getvalue()
    assert "Refusing to delete record" in output.getvalue()
    assert (
        "Deleted record_id=rec_cli category_slug=hinges run_id_retained=run_cli"
        in output.getvalue()
    )
    assert "failed_staging_dirs_to_remove=1" in output.getvalue()
    assert "legacy_files_to_remove=1" in output.getvalue()
    assert "sample_legacy_file=search.sqlite" in output.getvalue()
    assert "empty_dirs_to_remove=" in output.getvalue()
    assert "Removed cache paths:" in output.getvalue()
    assert "Preview only. Re-run with --execute to remove these cache paths." in output.getvalue()
    assert "record_dir=" in output.getvalue()
    assert "in_workbench=yes" in output.getvalue()
    assert "Preview only. Re-run with --execute --confirm-slug hinges" in category_output.getvalue()
    assert "Refusing to delete category" in category_output.getvalue()
    assert (
        "Deleted category_slug=hinges records_deleted=0 run_cache_entries_retained=1"
        in category_output.getvalue()
    )
    assert not repo.layout.category_dir("hinges").exists()


def test_promote_record_creates_category_and_moves_record_out_of_workbench(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0001")
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _create_workbench_record(
        repo,
        record_id="rec_router",
        run_id="run_router",
        title="Router",
    )

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "promote-record",
                    "rec_router",
                    "Internet Router",
                    "--promoted-at",
                    "2026-03-19T15:00:00Z",
                ]
            )
            == 0
        )

    record = json.loads(repo.layout.record_metadata_path("rec_router").read_text(encoding="utf-8"))
    assert record["category_slug"] == "internet_router"
    assert record["collections"] == ["dataset"]
    assert record["updated_at"] == "2026-03-19T15:00:00Z"

    dataset_entry = json.loads(
        repo.layout.record_dataset_entry_path("rec_router").read_text(encoding="utf-8")
    )
    assert dataset_entry == {
        "schema_version": 1,
        "dataset_id": "ds_internet_router_0001",
        "record_id": "rec_router",
        "category_slug": "internet_router",
        "promoted_at": "2026-03-19T15:00:00Z",
    }

    category = json.loads(
        repo.layout.category_metadata_path("internet_router").read_text(encoding="utf-8")
    )
    assert category == {
        "schema_version": 1,
        "slug": "internet_router",
        "title": "Internet Router",
        "description": "",
    }

    run_metadata = json.loads(
        repo.layout.run_metadata_path("run_router").read_text(encoding="utf-8")
    )
    assert run_metadata["category_slug"] == "internet_router"
    assert run_metadata["updated_at"] == "2026-03-19T15:00:00Z"

    manifest = json.loads(repo.layout.dataset_manifest_path().read_text(encoding="utf-8"))
    assert manifest == {
        "generated": [{"name": "ds_internet_router_0001", "record_id": "rec_router"}]
    }
    assert repo.layout.search_index_path().exists()
    assert (CollectionStore(repo).load_workbench() or {}).get("entries", []) == []
    assert (
        "Promoted record_id=rec_router category_slug=internet_router dataset_id=ds_internet_router_0001"
        in output.getvalue()
    )


def test_promote_record_reuses_existing_category_and_allocates_next_dataset_id(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0002")
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="internet_router",
            title="Internet Router",
            description="existing",
            prompt_batch_ids=["seed_batch"],
            target_sdk_version="base",
            current_count=1,
            last_item_index=1,
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            run_count=1,
        )
    )
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id="rec_existing_router",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="internet_router",
            source=SourceRef(run_id="run_existing_router"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Existing router", prompt_preview="existing"),
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
    DatasetStore(repo).promote_record(
        record_id="rec_existing_router",
        dataset_id="ds_internet_router_0001",
        promoted_at="2026-03-18T00:01:00Z",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_existing_router",
            run_mode="dataset_single",
            collection="dataset",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug="internet_router",
            prompt_count=1,
        )
    )

    _create_workbench_record(
        repo,
        record_id="rec_new_router",
        run_id="run_new_router",
        title="New Router",
    )

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "promote-record",
                    str(repo.layout.record_dir("rec_new_router")),
                    "Internet Router",
                    "--promoted-at",
                    "2026-03-19T15:05:00Z",
                ]
            )
            == 0
        )

    dataset_entry = json.loads(
        repo.layout.record_dataset_entry_path("rec_new_router").read_text(encoding="utf-8")
    )
    assert dataset_entry["dataset_id"] == "ds_internet_router_0002"
    assert dataset_entry["category_slug"] == "internet_router"

    category = json.loads(
        repo.layout.category_metadata_path("internet_router").read_text(encoding="utf-8")
    )
    assert category == {
        "schema_version": 1,
        "slug": "internet_router",
        "title": "Internet Router",
        "description": "existing",
        "prompt_batch_ids": ["seed_batch"],
        "target_sdk_version": "base",
    }

    workbench_entries = (CollectionStore(repo).load_workbench() or {}).get("entries", [])
    assert workbench_entries == []
    assert "dataset_id=ds_internet_router_0002" in output.getvalue()


def test_run_single_generates_dataset_record_and_creates_category(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0001")
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    assert (
        dataset_main(
            [
                "--repo-root",
                str(tmp_path),
                "run-single",
                "Create a compact home router with a vented body and two hinged antennas.",
                "--category-slug",
                "internet_router",
                "--provider",
                "openai",
                "--model-id",
                "gpt-5.4",
                "--thinking-level",
                "high",
                "--max-cost-usd",
                "1.5",
                "--sdk-package",
                "sdk",
                "--record-id",
                "rec_router_single",
            ]
        )
        == 0
    )

    record = json.loads(
        repo.layout.record_metadata_path("rec_router_single").read_text(encoding="utf-8")
    )
    assert record["collections"] == ["dataset"]
    assert record["category_slug"] == "internet_router"
    provenance = json.loads(
        (repo.layout.record_dir("rec_router_single") / "provenance.json").read_text(
            encoding="utf-8"
        )
    )
    assert provenance["prompting"]["scaffold_mode"] == "lite"
    assert provenance["generation"]["max_cost_usd"] == 1.5

    dataset_entry = json.loads(
        repo.layout.record_dataset_entry_path("rec_router_single").read_text(encoding="utf-8")
    )
    assert dataset_entry["dataset_id"] == "ds_internet_router_0001"
    assert dataset_entry["category_slug"] == "internet_router"

    category = json.loads(
        repo.layout.category_metadata_path("internet_router").read_text(encoding="utf-8")
    )
    assert category == {
        "schema_version": 1,
        "slug": "internet_router",
        "title": "Internet Router",
        "description": "",
    }

    manifest = json.loads(repo.layout.dataset_manifest_path().read_text(encoding="utf-8"))
    assert manifest == {
        "generated": [{"name": "ds_internet_router_0001", "record_id": "rec_router_single"}]
    }
    run_dir = next(path for path in repo.layout.runs_root.iterdir() if path.is_dir())
    run_metadata = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["settings_summary"]["max_cost_usd"] == 1.5
    assert repo.layout.search_index_path().exists()
    assert (CollectionStore(repo).load_workbench() or {}).get("entries", []) == []

    captured = capsys.readouterr().out
    assert "Generated record_id=rec_router_single" in captured
    assert "dataset_id=ds_internet_router_0001" in captured


def test_run_single_reuses_existing_category_and_allocates_next_dataset_id(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0002")
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="internet_router",
            title="Internet Router",
            description="existing",
            prompt_batch_ids=["seed_batch"],
            target_sdk_version="base",
            current_count=1,
            last_item_index=1,
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            run_count=1,
        )
    )
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id="rec_existing_router",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="internet_router",
            source=SourceRef(run_id="run_existing_router"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Existing router", prompt_preview="existing"),
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
    DatasetStore(repo).promote_record(
        record_id="rec_existing_router",
        dataset_id="ds_internet_router_0001",
        category_slug="internet_router",
        promoted_at="2026-03-18T00:01:00Z",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_existing_router",
            run_mode="dataset_single",
            collection="dataset",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug="internet_router",
            prompt_count=1,
        )
    )

    assert (
        dataset_main(
            [
                "--repo-root",
                str(tmp_path),
                "run-single",
                "Create a rackmount internet router with swing-up antennas.",
                "--category-slug",
                "internet_router",
                "--provider",
                "openai",
                "--model-id",
                "gpt-5.4",
                "--thinking-level",
                "high",
                "--sdk-package",
                "sdk",
                "--record-id",
                "rec_router_single_2",
            ]
        )
        == 0
    )

    dataset_entry = json.loads(
        repo.layout.record_dataset_entry_path("rec_router_single_2").read_text(encoding="utf-8")
    )
    assert dataset_entry["dataset_id"] == "ds_internet_router_0002"
    provenance = json.loads(
        (repo.layout.record_dir("rec_router_single_2") / "provenance.json").read_text(
            encoding="utf-8"
        )
    )
    assert provenance["prompting"]["scaffold_mode"] == "lite"

    category = json.loads(
        repo.layout.category_metadata_path("internet_router").read_text(encoding="utf-8")
    )
    assert category == {
        "schema_version": 1,
        "slug": "internet_router",
        "title": "Internet Router",
        "description": "existing",
        "prompt_batch_ids": ["seed_batch"],
        "target_sdk_version": "base",
    }

    captured = capsys.readouterr().out
    assert "dataset_id=ds_internet_router_0002" in captured


def test_run_single_warns_when_post_commit_hook_missing(
    fake_agent: None,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0001")
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)
    (tmp_path / ".git" / "hooks" / "post-commit").unlink()

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "run-single",
                    "Create a compact home router with a vented body and two hinged antennas.",
                    "--category-slug",
                    "internet_router",
                    "--provider",
                    "openai",
                    "--model-id",
                    "gpt-5.4",
                    "--thinking-level",
                    "high",
                    "--sdk-package",
                    "sdk",
                    "--record-id",
                    "rec_router_single",
                ]
            )
            == 0
        )

    captured = output.getvalue()
    assert "Warning: managed post-commit hook is missing" in captured
    assert "just setup" in captured


def test_status_does_not_warn_when_post_commit_hook_missing(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)
    (tmp_path / ".git" / "hooks" / "post-commit").unlink()

    assert dataset_main(["--repo-root", str(tmp_path), "status"]) == 0

    captured = capsys.readouterr().out
    assert "Warning: managed post-commit hook" not in captured


def test_run_batch_warns_when_post_commit_hook_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)
    (tmp_path / ".git" / "hooks" / "post-commit").unlink()

    class _NoopAwake:
        def __enter__(self) -> None:
            return None

        def __exit__(self, exc_type, exc, tb) -> bool:
            return False

    def _fake_build_batch_config(**kwargs: object) -> object:
        assert kwargs["repo_root"] == tmp_path
        return type("Config", (), {"keep_awake": False})()

    async def _fake_run_dataset_batch(config: object) -> dict[str, object]:
        return {
            "run_id": "run_batch_001",
            "status": "success",
            "success_count": 1,
            "failed_count": 0,
        }

    fake_module = types.ModuleType("agent.batch_runner")
    fake_module.build_batch_config = _fake_build_batch_config
    fake_module.keep_system_awake = lambda enabled: _NoopAwake()
    fake_module.run_dataset_batch = _fake_run_dataset_batch
    monkeypatch.setitem(sys.modules, "agent.batch_runner", fake_module)

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "run-batch",
                    "data/batch_specs/example.csv",
                ]
            )
            == 0
        )

    captured = output.getvalue()
    assert "Warning: managed post-commit hook is missing" in captured
    assert "Batch run_id=run_batch_001 status=success successes=1 failures=0" in captured


def test_run_single_does_not_warn_when_post_commit_hook_installed(
    fake_agent: None,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0001")
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "run-single",
                    "Create a compact home router with a vented body and two hinged antennas.",
                    "--category-slug",
                    "internet_router",
                    "--provider",
                    "openai",
                    "--model-id",
                    "gpt-5.4",
                    "--thinking-level",
                    "high",
                    "--sdk-package",
                    "sdk",
                    "--record-id",
                    "rec_router_single",
                ]
            )
            == 0
        )

    assert "Warning: managed post-commit hook" not in output.getvalue()


def test_sync_authors_cli_reports_summary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    monkeypatch.setattr(
        "cli.dataset.sync_record_authors",
        lambda repo: RecordAuthorSyncSummary(
            scanned=3,
            updated_record_ids=["rec_001", "rec_002"],
            already_set_record_ids=["rec_003"],
            missing_git_author_record_ids=["rec_004"],
        ),
    )

    output = io.StringIO()
    with redirect_stdout(output):
        assert dataset_main(["--repo-root", str(tmp_path), "sync-authors"]) == 0

    captured = output.getvalue()
    assert "Synced record authors scanned=3 updated=2 already_set=1" in captured
    assert "sample_updated_record_ids=rec_001, rec_002" in captured
    assert "sample_missing_git_author_record_ids=rec_004" in captured


def test_sync_rated_by_cli_reports_summary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    monkeypatch.setattr(
        "cli.dataset.sync_record_rated_by",
        lambda repo: RecordRatedBySyncSummary(
            scanned=3,
            updated_record_ids=["rec_010", "rec_011"],
            unchanged_record_ids=["rec_012"],
            missing_git_author_record_ids=["rec_013"],
        ),
    )

    output = io.StringIO()
    with redirect_stdout(output):
        assert dataset_main(["--repo-root", str(tmp_path), "sync-rated-by"]) == 0

    captured = output.getvalue()
    assert "Synced record rated_by scanned=3 updated=2 unchanged=1" in captured
    assert "sample_updated_record_ids=rec_010, rec_011" in captured
    assert "sample_missing_git_author_record_ids=rec_013" in captured


def test_supercategory_cli_commands_cover_list_mutation_and_delete(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="internet_router",
            title="Internet Router",
        )
    )
    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="monitor_mount",
            title="Monitor Mount",
        )
    )

    output = io.StringIO()
    with redirect_stdout(output):
        assert dataset_main(["--repo-root", str(tmp_path), "list-supercategories"]) == 0
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "upsert-supercategory",
                    "--supercategory-slug",
                    "electronics_and_optics",
                    "--title",
                    "Electronics & Optics",
                    "--description",
                    "Devices and optical equipment",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "upsert-supercategory",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "set-supercategory",
                    "--category-slug",
                    "internet_router",
                    "--supercategory-slug",
                    "electronics_and_optics",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "set-supercategory",
                    "--category-slug",
                    "internet_router",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "clear-supercategory",
                    "--category-slug",
                    "internet_router",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "set-supercategory",
                    "--category-slug",
                    "monitor_mount",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "delete-supercategory",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                ]
            )
            == 0
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "delete-supercategory",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                    "--execute",
                    "--confirm-slug",
                    "wrong-slug",
                ]
            )
            == 1
        )
        assert (
            dataset_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "delete-supercategory",
                    "--supercategory-slug",
                    "mounts_and_positioning",
                    "--execute",
                    "--confirm-slug",
                    "mounts_and_positioning",
                ]
            )
            == 0
        )

    manifest = json.loads(repo.layout.supercategories_path.read_text(encoding="utf-8"))
    assert manifest == {
        "schema_version": 1,
        "supercategories": [
            {
                "slug": "electronics_and_optics",
                "title": "Electronics & Optics",
                "description": "Devices and optical equipment",
                "category_slugs": [],
            }
        ],
    }
    assert "supercategory_count=0" in output.getvalue()
    assert (
        "Created supercategory_slug=electronics_and_optics title=Electronics & Optics category_count=0"
        in output.getvalue()
    )
    assert (
        "Created supercategory_slug=mounts_and_positioning title=Mounts And Positioning category_count=0"
        in output.getvalue()
    )
    assert (
        "Assigned category_slug=internet_router supercategory_slug=electronics_and_optics "
        "previous_supercategory_slug=(uncategorized)" in output.getvalue()
    )
    assert (
        "Assigned category_slug=internet_router supercategory_slug=mounts_and_positioning "
        "previous_supercategory_slug=electronics_and_optics" in output.getvalue()
    )
    assert (
        "Cleared category_slug=internet_router previous_supercategory_slug=mounts_and_positioning"
        in output.getvalue()
    )
    assert (
        "Preview only. Re-run with --execute --confirm-slug mounts_and_positioning "
        "to permanently delete this supercategory." in output.getvalue()
    )
    assert "Refusing to delete supercategory" in output.getvalue()
    assert (
        "Deleted supercategory_slug=mounts_and_positioning categories_uncategorized=1"
        in output.getvalue()
    )
