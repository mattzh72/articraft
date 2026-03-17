from __future__ import annotations

import sys
from pathlib import Path
from tempfile import TemporaryDirectory


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from storage.collections import CollectionStore
from storage.materialize import MaterializationStore, build_materialization_fingerprint
from storage.models import (
    CompileReport,
    DatasetCollection,
    DisplayMetadata,
    PromptingSettings,
    Provenance,
    Record,
    RecordArtifacts,
    RunSummary,
    SdkSettings,
    SourceRef,
    EnvironmentSettings,
    GenerationSettings,
    MaterializationInputs,
)
from storage.records import RecordStore
from storage.repo import StorageRepo


def main() -> None:
    with TemporaryDirectory() as tmpdir:
        repo = StorageRepo(Path(tmpdir))
        repo.ensure_layout()

        collections = CollectionStore(repo)
        collections.save_dataset(
            DatasetCollection(schema_version=1, collection="dataset", updated_at="2026-03-18T00:00:00Z")
        )
        assert repo.layout.dataset_collection_path().exists()

        record_store = RecordStore(repo)
        record = Record(
            schema_version=1,
            record_id="rec_123",
            created_at="2026-03-18T00:00:00Z",
            updated_at="2026-03-18T00:00:00Z",
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
        assert repo.layout.record_inputs_dir("rec_123").exists()
        assert not repo.layout.record_assets_dir("rec_123").exists()

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
            generation=GenerationSettings(provider="openai", model_id="gpt-5.4", thinking_level="high"),
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


if __name__ == "__main__":
    main()
