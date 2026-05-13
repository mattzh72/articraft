from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from agent.feedback import build_compile_signal_bundle
from cli import compile_record
from storage.repo import StorageRepo


def test_compile_record_prints_structured_compile_signals(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_dir = repo.layout.record_dir("rec_structured")
    record_dir.mkdir(parents=True)
    (record_dir / "model.py").write_text("# model\n", encoding="utf-8")

    class FakeViewerStore:
        def __init__(self, repo_root: Path) -> None:
            self.repo = StorageRepo(repo_root)

        def materialize_record_assets(self, record_id: str, **kwargs):
            del kwargs
            report_path = self.repo.layout.record_materialization_compile_report_path(record_id)
            self.repo.write_json(
                report_path,
                {
                    "schema_version": 1,
                    "record_id": record_id,
                    "status": "success",
                    "urdf_path": "model.urdf",
                    "warnings": [],
                    "signal_bundle": build_compile_signal_bundle(
                        status="success",
                        warnings=[],
                    ).to_dict(),
                },
            )
            return SimpleNamespace(warnings=[])

    monkeypatch.setattr(compile_record, "ViewerStore", FakeViewerStore)

    exit_code = compile_record.main(["--repo-root", str(tmp_path), str(record_dir)])

    assert exit_code == 0
    output = capsys.readouterr().out
    assert "<compile_signals>" in output
    assert "Compile passed cleanly." in output
