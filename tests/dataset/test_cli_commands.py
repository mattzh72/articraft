from __future__ import annotations

import io
import json
from contextlib import redirect_stdout
from pathlib import Path

from cli.dataset import main as dataset_main
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
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore


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
                model_urdf="model.urdf",
                compile_report_json="compile_report.json",
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
            model_urdf="model.urdf",
            compile_report_json="compile_report.json",
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
        assert category["current_count"] == 1
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
        assert category["current_count"] == 0
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
        assert dataset_main(["--repo-root", str(repo_root), "prune-cache"]) == 0
        assert empty_record_stage_dir.exists()
        assert empty_nested_failure_dir.exists()
        assert nonempty_stage_dir.exists()
        assert failed_stage_dir.exists()
        assert dataset_main(["--repo-root", str(repo_root), "prune-cache", "--execute"]) == 0
        assert not empty_record_stage_dir.exists()
        assert not empty_nested_failure_dir.exists()
        assert nonempty_stage_dir.exists()
        assert not failed_stage_dir.exists()
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
    assert "empty_dirs_to_remove=" in output.getvalue()
    assert "Removed cache paths: 6" in output.getvalue()
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
) -> None:
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
    assert category["title"] == "Internet Router"
    assert category["slug"] == "internet_router"
    assert category["target_sdk_version"] == "base"
    assert category["current_count"] == 1
    assert category["last_item_index"] == 1
    assert category["run_count"] == 1

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
) -> None:
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
                model_urdf="model.urdf",
                compile_report_json="compile_report.json",
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
    assert category["title"] == "Internet Router"
    assert category["description"] == "existing"
    assert category["prompt_batch_ids"] == ["seed_batch"]
    assert category["current_count"] == 2
    assert category["last_item_index"] == 2
    assert category["run_count"] == 2

    workbench_entries = (CollectionStore(repo).load_workbench() or {}).get("entries", [])
    assert workbench_entries == []
    assert "dataset_id=ds_internet_router_0002" in output.getvalue()
