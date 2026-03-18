from __future__ import annotations

import io
import sys
from contextlib import redirect_stdout
from pathlib import Path
from tempfile import TemporaryDirectory

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from cli.dataset import main as dataset_main
from storage.models import DisplayMetadata, Record, RecordArtifacts, SourceRef
from storage.records import RecordStore
from storage.repo import StorageRepo


def main() -> None:
    with TemporaryDirectory() as tmpdir:
        repo_root = Path(tmpdir)
        repo = StorageRepo(repo_root)
        repo.ensure_layout()

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
            assert dataset_main(["--repo-root", str(repo_root), "validate"]) == 0
            assert dataset_main(["--repo-root", str(repo_root), "build-manifest"]) == 0
            assert dataset_main(["--repo-root", str(repo_root), "status"]) == 0

        manifest = repo.layout.dataset_manifest_path().read_text(encoding="utf-8")
        assert '"name": "cli_dataset_001"' in manifest
        assert "dataset_entries=1" in output.getvalue()


if __name__ == "__main__":
    main()
