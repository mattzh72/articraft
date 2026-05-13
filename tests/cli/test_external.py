from __future__ import annotations

import json
from pathlib import Path

import pytest

from cli import external as external_cli
from cli import main as articraft_cli
from storage.categories import CategoryStore
from storage.models import CategoryRecord, DisplayMetadata, Record, RecordArtifacts, SourceRef
from storage.records import RecordStore
from storage.repo import StorageRepo


def _fake_compile(repo_root: Path):
    def _compile(argv: list[str]) -> int:
        record_dir = next(
            Path(item)
            for item in argv
            if Path(item).name.startswith("rec_") and not item.startswith("--")
        )
        record_id = record_dir.name
        repo = StorageRepo(repo_root)
        compile_report = repo.layout.record_materialization_compile_report_path(record_id)
        compile_report.parent.mkdir(parents=True, exist_ok=True)
        compile_report.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "record_id": record_id,
                    "status": "success",
                    "urdf_path": "model.urdf",
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        return 0

    return _compile


def test_external_init_creates_identified_workbench_record(tmp_path: Path) -> None:
    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "init",
            "--agent",
            "codex",
            "--model-id",
            "gpt-5.5-2026-04-23",
            "--thinking-level",
            "high",
            "washing machine",
        ]
    )

    assert exit_code == 0
    record_dir = next((tmp_path / "data" / "records").iterdir())
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))

    assert record["collections"] == ["workbench"]
    assert record["provider"] == "openai"
    assert record["model_id"] == "gpt-5.5-2026-04-23"
    assert record["creator"] == {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": False,
    }
    assert provenance["generation"]["provider"] == "openai"
    assert provenance["generation"]["model_id"] == "gpt-5.5-2026-04-23"
    assert provenance["generation"]["thinking_level"] == "high"
    assert provenance["generation"]["openai_transport"] is None
    assert provenance["generation"]["openai_reasoning_summary"] is None
    assert provenance["run_summary"]["final_status"] == "draft"


def test_external_init_defaults_claude_code_to_anthropic(tmp_path: Path) -> None:
    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "init",
            "--agent",
            "claude-code",
            "washing machine",
        ]
    )

    assert exit_code == 0
    record_dir = next((tmp_path / "data" / "records").iterdir())
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert record["provider"] == "anthropic"
    assert record["model_id"] is None
    assert provenance["generation"]["provider"] == "anthropic"
    assert provenance["generation"]["model_id"] is None
    assert provenance["generation"]["openai_transport"] is None
    assert provenance["generation"]["openai_reasoning_summary"] is None


def test_external_init_rejects_unknown_agent(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc_info:
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "init",
                "--agent",
                "other",
                "washing machine",
            ]
        )

    assert exc_info.value.code == 2


def test_external_compile_command_is_removed(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc_info:
        external_cli.main(["--repo-root", str(tmp_path), "compile", "data/records/rec_one"])

    assert exc_info.value.code == 2


def test_external_check_runs_full_strict_compile(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert (
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "init",
                "--agent",
                "codex",
                "washing machine",
            ]
        )
        == 0
    )
    record_dir = next((tmp_path / "data" / "records").iterdir())
    calls: list[list[str]] = []

    compile_impl = _fake_compile(tmp_path)

    def _capture_compile(argv: list[str]) -> int:
        calls.append(argv)
        return compile_impl(argv)

    monkeypatch.setattr(external_cli.compile_record_cli, "main", _capture_compile)

    exit_code = external_cli.main(["--repo-root", str(tmp_path), "check", str(record_dir)])

    assert exit_code == 0
    assert calls == [
        [
            "--repo-root",
            str(tmp_path),
            "--target",
            "full",
            str(record_dir),
            "--validate",
            "--strict-geom-qc",
        ]
    ]


def test_external_finalize_workbench_keeps_record_local(
    tmp_path: Path,
    monkeypatch,
) -> None:
    assert (
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "init",
                "--agent",
                "claude-code",
                "washing machine",
            ]
        )
        == 0
    )
    record_dir = next((tmp_path / "data" / "records").iterdir())
    monkeypatch.setattr(external_cli.compile_record_cli, "main", _fake_compile(tmp_path))

    exit_code = external_cli.main(["--repo-root", str(tmp_path), "finalize", str(record_dir)])

    assert exit_code == 0
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert record["collections"] == ["workbench"]
    assert record["kind"] == "external_model"
    assert (record_dir / ".gitignore").exists()
    assert provenance["run_summary"]["final_status"] == "external_ready"


def test_external_finalize_with_category_promotes_to_dataset(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="washing_machine",
            title="Washing Machine",
        )
    )
    assert (
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "init",
                "--agent",
                "codex",
                "washing machine",
            ]
        )
        == 0
    )
    record_dir = next((tmp_path / "data" / "records").iterdir())
    monkeypatch.setattr(external_cli.compile_record_cli, "main", _fake_compile(tmp_path))

    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "finalize",
            str(record_dir),
            "--category-slug",
            "washing_machine",
        ]
    )

    assert exit_code == 0
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    dataset_entry = json.loads((record_dir / "dataset_entry.json").read_text(encoding="utf-8"))
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert record["collections"] == ["dataset"]
    assert record["category_slug"] == "washing_machine"
    assert record["creator"]["agent"] == "codex"
    assert dataset_entry["category_slug"] == "washing_machine"
    assert not (record_dir / ".gitignore").exists()
    assert provenance["run_summary"]["final_status"] == "external_finalized"


def test_external_categories_prints_counts(tmp_path: Path, capsys) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="washing_machine", title="Washing Machine")
    )

    exit_code = external_cli.main(["--repo-root", str(tmp_path), "categories"])

    assert exit_code == 0
    output = capsys.readouterr().out
    assert "category_count=1" in output
    assert "washing_machine\ttitle=Washing Machine\trecords=0" in output


def test_external_examples_prints_high_rated_matches(tmp_path: Path, capsys) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)
    for record_id, rating, prompt in (
        ("rec_washer_good", 5, "front loading washing machine with hinged door"),
        ("rec_washer_bad", 3, "rough washing machine"),
        ("rec_hinge_good", 5, "cabinet hinge"),
    ):
        record_store.write_record(
            Record(
                schema_version=2,
                record_id=record_id,
                created_at="2026-03-18T00:00:00Z",
                updated_at="2026-03-18T00:00:00Z",
                rating=rating,
                kind="generated_model",
                prompt_kind="single_prompt",
                category_slug="washing_machine" if "washer" in record_id else "hinge",
                source=SourceRef(run_id=None),
                sdk_package="sdk",
                provider="openai",
                model_id="gpt-5.4",
                display=DisplayMetadata(title=prompt.title(), prompt_preview=prompt),
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
        record_dir = repo.layout.record_dir(record_id)
        (record_dir / "prompt.txt").write_text(prompt, encoding="utf-8")
        (record_dir / "model.py").write_text("# model\n", encoding="utf-8")
        (record_dir / "provenance.json").write_text("{}\n", encoding="utf-8")

    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "examples",
            "--query",
            "washing machine",
            "--rating-min",
            "5",
        ]
    )

    assert exit_code == 0
    output = capsys.readouterr().out
    assert "example_count=1 total_matches=1" in output
    assert "record_id=rec_washer_good" in output
    assert "record_id=rec_washer_bad" not in output
    assert "model=" in output


def test_top_level_external_command_delegates(tmp_path: Path, monkeypatch) -> None:
    calls: list[list[str]] = []

    def _fake_external(argv: list[str]) -> int:
        calls.append(argv)
        return 0

    monkeypatch.setattr(articraft_cli.external_cli, "main", _fake_external)

    exit_code = articraft_cli.main(
        [
            "external",
            "--repo-root",
            str(tmp_path),
            "init",
            "--agent",
            "codex",
            "washing machine",
        ]
    )

    assert exit_code == 0
    assert calls == [
        [
            "--repo-root",
            str(tmp_path),
            "init",
            "--agent",
            "codex",
            "washing machine",
        ]
    ]


def test_agent_docs_reference_external_contract() -> None:
    root = Path(__file__).resolve().parents[2]

    assert "EXTERNAL_AGENT_DATA.md" in (root / "AGENTS.md").read_text(encoding="utf-8")
    assert "EXTERNAL_AGENT_DATA.md" in (root / "CLAUDE.md").read_text(encoding="utf-8")
