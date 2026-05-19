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
from storage.revisions import active_model_path, active_provenance_path


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


def _record_payload(record_dir: Path) -> dict:
    return json.loads((record_dir / "record.json").read_text(encoding="utf-8"))


def _provenance_payload(repo_root: Path, record_dir: Path) -> dict:
    repo = StorageRepo(repo_root)
    record = _record_payload(record_dir)
    return json.loads(
        active_provenance_path(repo, record_dir.name, record=record).read_text(encoding="utf-8")
    )


def _dataset_entry_payload(record_dir: Path) -> dict:
    return json.loads((record_dir / "collections" / "dataset.json").read_text(encoding="utf-8"))


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
    record = _record_payload(record_dir)
    provenance = _provenance_payload(tmp_path, record_dir)

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
    record = _record_payload(record_dir)
    provenance = _provenance_payload(tmp_path, record_dir)
    assert record["provider"] == "anthropic"
    assert record["model_id"] is None
    assert provenance["generation"]["provider"] == "anthropic"
    assert provenance["generation"]["model_id"] is None
    assert provenance["generation"]["openai_transport"] is None
    assert provenance["generation"]["openai_reasoning_summary"] is None


def test_external_fork_workbench_creates_child_revision_without_parent_mutation(
    tmp_path: Path,
    capsys,
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
    parent_dir = next((tmp_path / "data" / "records").iterdir())
    repo = StorageRepo(tmp_path)
    parent_record_before = (parent_dir / "record.json").read_text(encoding="utf-8")
    parent_model = active_model_path(repo, parent_dir.name, record=_record_payload(parent_dir))
    parent_model.write_text("# parent unique model\n", encoding="utf-8")
    capsys.readouterr()

    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "fork",
            "--agent",
            "codex",
            str(parent_dir),
            "make the handle longer",
            "--record-id",
            "rec_external_child",
        ]
    )

    assert exit_code == 0
    output = capsys.readouterr().out
    assert "external fork created" in output
    assert "record_id=rec_external_child" in output
    assert "model=" in output
    assert "revisions/rev_000001/model.py" in output
    assert (parent_dir / "record.json").read_text(encoding="utf-8") == parent_record_before
    child_dir = tmp_path / "data" / "records" / "rec_external_child"
    child_record = _record_payload(child_dir)
    child_model = active_model_path(repo, "rec_external_child", record=child_record)
    assert child_model.read_text(encoding="utf-8") == "# parent unique model\n"
    assert child_record["lineage"]["parent_record_id"] == parent_dir.name
    assert child_record["lineage"]["parent_revision_id"] == "rev_000001"
    assert child_record["creator"] == {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": False,
    }
    assert (child_dir / "collections" / "workbench.json").exists()
    assert not (child_dir / "cost.json").exists()
    assert not (child_dir / "traces").exists()


def test_external_fork_dataset_creates_dataset_child_without_copying_parent_run_artifacts(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="washing_machine", title="Washing Machine")
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
    parent_dir = next((tmp_path / "data" / "records").iterdir())
    monkeypatch.setattr(external_cli.compile_record_cli, "main", _fake_compile(tmp_path))
    assert (
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "finalize",
                str(parent_dir),
                "--category-slug",
                "washing_machine",
            ]
        )
        == 0
    )
    parent_record = _record_payload(parent_dir)
    parent_revision_dir = parent_dir / "revisions" / parent_record["active_revision_id"]
    (parent_revision_dir / "cost.json").write_text('{"parent_only": true}\n', encoding="utf-8")
    (parent_revision_dir / "traces").mkdir(exist_ok=True)
    (parent_revision_dir / "traces" / "trajectory.jsonl").write_text("{}\n", encoding="utf-8")
    parent_provenance = _provenance_payload(tmp_path, parent_dir)
    parent_provenance["parent_only"] = True
    active_provenance_path(repo, parent_dir.name, record=parent_record).write_text(
        json.dumps(parent_provenance, indent=2) + "\n",
        encoding="utf-8",
    )

    exit_code = external_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "fork",
            "--agent",
            "claude-code",
            str(parent_dir),
            "make the washer door wider",
            "--record-id",
            "rec_dataset_child",
        ]
    )

    assert exit_code == 0
    child_dir = tmp_path / "data" / "records" / "rec_dataset_child"
    child_record = _record_payload(child_dir)
    child_entry = _dataset_entry_payload(child_dir)
    parent_entry = _dataset_entry_payload(parent_dir)
    child_revision_dir = child_dir / "revisions" / "rev_000001"
    child_provenance = _provenance_payload(tmp_path, child_dir)
    assert child_record["collections"] == ["dataset"]
    assert child_record["category_slug"] == "washing_machine"
    assert child_record["creator"]["agent"] == "claude-code"
    assert child_entry["dataset_id"].startswith(f"{parent_entry['dataset_id']}_edit_")
    assert child_entry["dataset_id"] != parent_entry["dataset_id"]
    assert not (child_revision_dir / "cost.json").exists()
    assert not (child_revision_dir / "traces" / "trajectory.jsonl").exists()
    assert "parent_only" not in child_provenance


def test_external_revise_command_is_removed(tmp_path: Path) -> None:
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

    with pytest.raises(SystemExit) as exc_info:
        external_cli.main(
            [
                "--repo-root",
                str(tmp_path),
                "revise",
                "--agent",
                "claude-code",
                str(record_dir),
                "make the drum larger",
            ]
        )

    assert exc_info.value.code == 2
    record = _record_payload(record_dir)
    assert record["active_revision_id"] == "rev_000001"
    assert not (record_dir / "revisions" / "rev_000002").exists()


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
    record = _record_payload(record_dir)
    provenance = _provenance_payload(tmp_path, record_dir)
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
    record = _record_payload(record_dir)
    dataset_entry = _dataset_entry_payload(record_dir)
    provenance = _provenance_payload(tmp_path, record_dir)
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

    external_docs = (root / "EXTERNAL_AGENT_DATA.md").read_text(encoding="utf-8")
    assert "EXTERNAL_AGENT_DATA.md" in (root / "AGENTS.md").read_text(encoding="utf-8")
    assert "EXTERNAL_AGENT_DATA.md" in (root / "CLAUDE.md").read_text(encoding="utf-8")
    assert "articraft external fork" in external_docs
    assert "articraft external revise" not in external_docs
    assert "the CLI-printed active model= path" in external_docs
    assert "data/records/<record_id>/model.py" not in external_docs
