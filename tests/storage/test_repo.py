from __future__ import annotations

import sys
from pathlib import Path
from tempfile import TemporaryDirectory

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.materialize import MaterializationStore, build_materialization_fingerprint
from storage.models import (
    CompileReport,
    DisplayMetadata,
    EnvironmentSettings,
    GenerationSettings,
    MaterializationInputs,
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


def main() -> None:
    with TemporaryDirectory() as tmpdir:
        repo = StorageRepo(Path(tmpdir))
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

        record_store = RecordStore(repo)
        record = Record(
            schema_version=1,
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
                model_urdf="model.urdf",
                compile_report_json="compile_report.json",
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
        )
        record_store.write_compile_report("rec_123", report)
        assert (repo.layout.record_dir("rec_123") / "compile_report.json").exists()

        provenance = Provenance(
            schema_version=1,
            record_id="rec_123",
            generation=GenerationSettings(
                provider="openai", model_id="gpt-5.4", thinking_level="high"
            ),
            prompting=PromptingSettings(
                system_prompt_file="designer_system_prompt_openai.txt",
                system_prompt_sha256="abc",
                sdk_docs_mode="core",
            ),
            sdk=SdkSettings(sdk_package="sdk", sdk_version="workspace", sdk_fingerprint="sdk-hash"),
            environment=EnvironmentSettings(python_version="3.11.11", platform="darwin-arm64"),
            run_summary=RunSummary(final_status="success"),
            materialization=MaterializationInputs(
                model_py_sha256="py-hash",
                model_urdf_sha256="urdf-hash",
                sdk_fingerprint="sdk-hash",
            ),
        )
        record_store.write_provenance("rec_123", provenance)
        assert (repo.layout.record_dir("rec_123") / "provenance.json").exists()

        materialize = MaterializationStore(repo)
        status = materialize.asset_status("rec_123")
        assert not status.meshes_present
        assert not status.glb_present
        assert not status.viewer_present
        assert status.assets_dir == repo.layout.record_assets_dir("rec_123")

        fingerprint = build_materialization_fingerprint(
            model_py_sha256="py-hash",
            model_urdf_sha256="urdf-hash",
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


if __name__ == "__main__":
    main()
