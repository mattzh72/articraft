from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent import runner
from cli.workbench import main as workbench_main
from tests.helpers import FakeAgent


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FakeAgent)


def test_workbench_rerun_record_command(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a cabinet hinge",
            prompt_text="make a cabinet hinge",
            display_prompt="make a cabinet hinge",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="hinge rerun",
            tags=["hinge"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())
    original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    original_run_id = original_record["source"]["run_id"]

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
            ]
        )
        == 0
    )

    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert updated_record["record_id"] == record_dir.name
    assert updated_record["source"]["run_id"] != original_run_id
    assert (repo_root / "data" / "cache" / "search.sqlite").exists()

    captured = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in captured
    assert "search_index=" in captured


def test_workbench_rerun_record_command_accepts_legacy_sdk_docs_mode(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a coffee machine",
            prompt_text="make a coffee machine",
            display_prompt="make a coffee machine",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="coffee rerun",
            tags=["coffee"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())
    provenance_path = record_dir / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    provenance["prompting"]["sdk_docs_mode"] = "legacy_import"
    provenance_path.write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
            ]
        )
        == 0
    )

    captured = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in captured
    assert "search_index=" in captured


def test_workbench_rerun_record_command_removes_legacy_record_root_assets(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a remote turret",
            prompt_text="make a remote turret",
            display_prompt="make a remote turret",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="turret rerun",
            tags=["turret"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())
    legacy_meshes_dir = record_dir / "meshes"
    legacy_glb_dir = record_dir / "glb"
    legacy_viewer_dir = record_dir / "viewer"
    legacy_meshes_dir.mkdir(parents=True, exist_ok=True)
    legacy_glb_dir.mkdir(parents=True, exist_ok=True)
    legacy_viewer_dir.mkdir(parents=True, exist_ok=True)
    (legacy_meshes_dir / "stale.obj").write_text("# stale\n", encoding="utf-8")
    (legacy_glb_dir / "stale.glb").write_bytes(b"stale")
    (legacy_viewer_dir / "stale.json").write_text("{}", encoding="utf-8")

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
            ]
        )
        == 0
    )

    assert not legacy_meshes_dir.exists()
    assert not legacy_glb_dir.exists()
    assert not legacy_viewer_dir.exists()
    assert (record_dir / "assets" / "meshes" / "part.obj").exists()


def test_workbench_init_record_command(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "init-record",
                "build a folding reading lamp",
                "--provider",
                "openai",
                "--model-id",
                "gpt-5.4",
                "--thinking-level",
                "high",
                "--label",
                "reading lamp draft",
                "--tag",
                "draft",
                "--tag",
                "lamp",
            ]
        )
        == 0
    )

    records = sorted((repo_root / "data" / "records").iterdir())
    assert len(records) == 1
    record_dir = records[0]

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert record["record_id"] == record_dir.name
    assert record["kind"] == "draft_model"
    assert record["provider"] == "openai"
    assert record["model_id"] == "gpt-5.4"
    assert record["collections"] == ["workbench"]
    assert record["display"]["title"] == "reading lamp draft"

    assert (record_dir / "prompt.txt").read_text(encoding="utf-8") == "build a folding reading lamp"
    model_text = (record_dir / "model.py").read_text(encoding="utf-8")
    assert "Draft scaffold created by `articraft-workbench init-record`." in model_text

    compile_report = json.loads((record_dir / "compile_report.json").read_text(encoding="utf-8"))
    assert compile_report["status"] == "draft"

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["provider"] == "openai"
    assert provenance["generation"]["model_id"] == "gpt-5.4"
    assert provenance["run_summary"]["final_status"] == "draft"

    workbench = json.loads(
        (repo_root / "data" / "local" / "workbench.json").read_text(encoding="utf-8")
    )
    assert workbench["entries"][0]["record_id"] == record_dir.name
    assert workbench["entries"][0]["label"] == "reading lamp draft"
    assert workbench["entries"][0]["tags"] == ["draft", "lamp"]

    assert (repo_root / "data" / "cache" / "search.sqlite").exists()

    captured = capsys.readouterr().out
    assert f"initialized record_id={record_dir.name}" in captured
    assert "search_index=" in captured


def test_workbench_init_record_command_persists_input_image(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    image_path = repo_root / "reference.png"
    image_path.write_bytes(b"\x89PNG\r\n\x1a\n")

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "init-record",
                "build a folding reading lamp",
                "--provider",
                "openai",
                "--image",
                str(image_path),
            ]
        )
        == 0
    )

    records = sorted((repo_root / "data" / "records").iterdir())
    assert len(records) == 1
    record_dir = records[0]

    assert (record_dir / "inputs" / image_path.name).read_bytes() == image_path.read_bytes()

    captured = capsys.readouterr().out
    assert f"initialized record_id={record_dir.name}" in captured
