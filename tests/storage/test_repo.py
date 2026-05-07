from __future__ import annotations

import json
from pathlib import Path

from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.materialize import MaterializationStore, build_materialization_fingerprint
from storage.models import (
    CategoryRecord,
    CompileReport,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    PromptingSettings,
    Provenance,
    Record,
    RecordArtifacts,
    RunRecord,
    RunSummary,
    SdkSettings,
    SourceRef,
)
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore


def test_storage_repo_round_trips_records_collections_and_runs(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    collections = CollectionStore(repo)
    collections.append_workbench_entry(
        record_id="rec_123",
        added_at="2026-03-18T00:00:00Z",
        label="trial",
        tags=["hinge"],
    )
    workbench = collections.load_workbench()
    assert workbench is not None
    assert workbench["entries"][0]["record_id"] == "rec_123"
    assert repo.layout.local_workbench_path().exists()

    categories = CategoryStore(repo)
    categories.save(
        CategoryRecord(
            schema_version=1,
            slug="hinges",
            title="Hinges",
        )
    )
    assert repo.layout.category_metadata_path("hinges").exists()

    record_store = RecordStore(repo)
    record = Record(
        schema_version=2,
        record_id="rec_123",
        created_at="2026-03-18T00:00:00Z",
        updated_at="2026-03-18T00:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_123", prompt_batch_id="batch_001", prompt_index=3),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(title="Hinge", prompt_preview="model a hinge"),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
    )
    record_store.write_record(record)
    assert repo.layout.record_metadata_path("rec_123").exists()
    assert repo.read_json(repo.layout.record_metadata_path("rec_123"))["rating"] is None
    assert repo.layout.record_inputs_dir("rec_123").exists()
    assert not repo.layout.record_assets_dir("rec_123").exists()

    datasets = DatasetStore(repo)
    promoted = datasets.promote_record(
        record_id="rec_123",
        dataset_id="hinge_dataset_001",
        promoted_at="2026-03-18T00:01:00Z",
    )
    assert promoted["record_id"] == "rec_123"
    assert repo.layout.record_dataset_entry_path("rec_123").exists()
    assert datasets.validate() == []
    manifest = datasets.write_dataset_manifest()
    assert manifest["generated"] == [{"name": "hinge_dataset_001", "record_id": "rec_123"}]
    assert repo.layout.dataset_manifest_path().exists()

    report = CompileReport(
        schema_version=1,
        record_id="rec_123",
        status="success",
        urdf_path="model.urdf",
        warnings=[],
    )
    materialize = MaterializationStore(repo)
    materialize.write_compile_report("rec_123", report)
    assert repo.layout.record_materialization_compile_report_path("rec_123").exists()

    provenance = Provenance(
        schema_version=2,
        record_id="rec_123",
        generation=GenerationSettings(provider="openai", model_id="gpt-5.4", thinking_level="high"),
        prompting=PromptingSettings(
            system_prompt_file="designer_system_prompt_openai.txt",
            system_prompt_sha256="abc",
        ),
        sdk=SdkSettings(sdk_package="sdk", sdk_version="workspace", sdk_fingerprint="sdk-hash"),
        environment=EnvironmentSettings(python_version="3.11.11", platform="darwin-arm64"),
        run_summary=RunSummary(final_status="success"),
    )
    record_store.write_provenance("rec_123", provenance)
    assert (repo.layout.record_dir("rec_123") / "provenance.json").exists()
    assert categories.load("hinges") is not None

    status = materialize.asset_status("rec_123")
    assert not status.meshes_present
    assert not status.glb_present
    assert not status.viewer_present
    assert status.assets_dir == repo.layout.record_materialization_assets_dir("rec_123")

    fingerprint = build_materialization_fingerprint(
        model_py_sha256="py-hash",
        sdk_fingerprint="sdk-hash",
    )
    assert len(fingerprint) == 64

    runs = RunStore(repo)
    runs.write_run(
        RunRecord(
            schema_version=1,
            run_id="run_123",
            run_mode="workbench_single",
            collection="workbench",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            prompt_count=1,
        )
    )
    assert repo.layout.run_metadata_path("run_123").exists()
    runs.append_result("run_123", {"record_id": "rec_123", "status": "success"})
    results = repo.layout.run_results_path("run_123").read_text(encoding="utf-8")
    assert '"record_id": "rec_123"' in results
    assert categories.delete("hinges")
    assert not repo.layout.category_dir("hinges").exists()


def test_workbench_records_write_local_gitignore_marker(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    RecordStore(repo).write_record(
        Record(
            schema_version=2,
            record_id="rec_workbench",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_123"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title="Workbench", prompt_preview="workbench"),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json="cost.json",
            ),
            collections=["workbench"],
        )
    )

    marker = repo.layout.record_dir("rec_workbench") / ".gitignore"
    assert marker.exists()
    assert "*" in marker.read_text(encoding="utf-8")

    DatasetStore(repo).promote_record(
        record_id="rec_workbench",
        dataset_id="ds_hinges_workbench",
        category_slug="hinges",
        promoted_at="2026-03-18T00:01:00Z",
    )
    assert not marker.exists()


def test_run_store_upsert_result_compacts_latest_rows(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    runs = RunStore(repo)
    runs.write_run(
        RunRecord(
            schema_version=1,
            run_id="run_123",
            run_mode="dataset_batch",
            collection="dataset",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:01Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="running",
            prompt_count=2,
        )
    )

    runs.upsert_result("run_123", {"row_id": "row_a", "status": "running"}, key="row_id")
    runs.upsert_result("run_123", {"row_id": "row_b", "status": "running"}, key="row_id")
    runs.upsert_result("run_123", {"row_id": "row_a", "status": "success"}, key="row_id")

    raw_rows = [
        json.loads(line)
        for line in repo.layout.run_results_path("run_123").read_text(encoding="utf-8").splitlines()
    ]
    assert [row["status"] for row in raw_rows] == ["running", "running", "success"]

    latest_rows = runs.read_latest_results("run_123", key="row_id")
    assert latest_rows == [
        {"row_id": "row_a", "status": "success"},
        {"row_id": "row_b", "status": "running"},
    ]

    runs.compact_results("run_123", key="row_id")
    compacted_rows = [
        json.loads(line)
        for line in repo.layout.run_results_path("run_123").read_text(encoding="utf-8").splitlines()
    ]
    assert compacted_rows == latest_rows


def test_dataset_store_updates_cached_dataset_id_index_after_promote(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    datasets = DatasetStore(repo)
    record_store = RecordStore(repo)

    first_record = Record(
        schema_version=2,
        record_id="rec_existing",
        created_at="2026-03-18T00:00:00Z",
        updated_at="2026-03-18T00:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_123", prompt_index=1),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(title="Existing", prompt_preview="existing"),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
    )
    second_record = Record(
        schema_version=2,
        record_id="rec_new",
        created_at="2026-03-18T00:00:00Z",
        updated_at="2026-03-18T00:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_123", prompt_index=2),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(title="New", prompt_preview="new"),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
    )
    record_store.write_record(first_record)
    record_store.write_record(second_record)
    datasets.promote_record(
        record_id="rec_existing",
        dataset_id="ds_hinges_existing",
        category_slug="hinges",
        promoted_at="2026-03-18T00:00:00Z",
    )

    assert datasets.find_record_id_by_dataset_id("ds_hinges_existing") == "rec_existing"
    datasets.promote_record(
        record_id="rec_new",
        dataset_id="ds_hinges_new",
        category_slug="hinges",
        promoted_at="2026-03-18T00:00:01Z",
    )
    assert datasets.find_record_id_by_dataset_id("ds_hinges_new") == "rec_new"
