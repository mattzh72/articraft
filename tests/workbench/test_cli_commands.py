from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent import runner
from agent.defaults import DEFAULT_MAX_TURNS
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
    assert (repo_root / "data" / "cache" / "search_index.json").exists()

    captured = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in captured
    assert "search_index=" in captured


def test_workbench_rerun_record_command_accepts_model_and_thinking_overrides(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a countertop mixer",
            prompt_text="make a countertop mixer",
            display_prompt="make a countertop mixer",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            model_id="gpt-5.4",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="mixer rerun",
            tags=["mixer"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
                "--model-id",
                "gemini-3-flash-preview",
                "--thinking-level",
                "low",
            ]
        )
        == 0
    )

    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    updated_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert updated_record["provider"] == "gemini"
    assert updated_record["model_id"] == "gemini-3-flash-preview"
    assert updated_provenance["generation"]["provider"] == "gemini"
    assert updated_provenance["generation"]["model_id"] == "gemini-3-flash-preview"
    assert updated_provenance["generation"]["thinking_level"] == "low"

    captured = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in captured
    assert "search_index=" in captured


def test_workbench_rerun_record_command_prefers_source_run_thinking_parameters(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a midi keyboard",
            prompt_text="make a midi keyboard",
            display_prompt="make a midi keyboard",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            model_id="gpt-5.4",
            thinking_level="low",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="keyboard rerun",
            tags=["keyboard"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())
    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    source_run_id = record["source"]["run_id"]
    run_metadata_path = repo_root / "data" / "cache" / "runs" / source_run_id / "run.json"
    run_metadata = json.loads(run_metadata_path.read_text(encoding="utf-8"))
    run_metadata["settings_summary"]["thinking_level"] = "low"
    run_metadata_path.write_text(json.dumps(run_metadata, indent=2) + "\n", encoding="utf-8")

    provenance_path = record_dir / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    provenance["generation"]["thinking_level"] = "high"
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

    updated_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert updated_provenance["generation"]["thinking_level"] == "low"


def test_workbench_rerun_record_command_accepts_sdk_override(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a microphone boom arm",
            prompt_text="make a microphone boom arm",
            display_prompt="make a microphone boom arm",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            model_id="gpt-5.4",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk_hybrid",
            sdk_docs_mode="full",
            label="boom arm rerun",
            tags=["boom"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
                "--sdk-package",
                "sdk",
            ]
        )
        == 0
    )

    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    updated_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert updated_record["sdk_package"] == "sdk"
    assert updated_provenance["sdk"]["sdk_package"] == "sdk"
    assert (
        updated_provenance["prompting"]["system_prompt_file"] == "designer_system_prompt_openai.txt"
    )

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


def test_workbench_rerun_record_command_replaces_cached_materialization_outputs(
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
    canonical_meshes_dir = record_dir / "assets" / "meshes"
    canonical_glb_dir = record_dir / "assets" / "glb"
    canonical_viewer_dir = record_dir / "assets" / "viewer"
    canonical_meshes_dir.mkdir(parents=True, exist_ok=True)
    canonical_glb_dir.mkdir(parents=True, exist_ok=True)
    canonical_viewer_dir.mkdir(parents=True, exist_ok=True)
    (canonical_meshes_dir / "stale.obj").write_text("# stale\n", encoding="utf-8")
    (canonical_glb_dir / "stale.glb").write_bytes(b"stale")
    (canonical_viewer_dir / "stale.json").write_text("{}", encoding="utf-8")

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

    assert not (canonical_meshes_dir / "stale.obj").exists()
    assert not (canonical_glb_dir / "stale.glb").exists()
    assert not (canonical_viewer_dir / "stale.json").exists()
    materialization_dir = repo_root / "data" / "cache" / "record_materialization" / record_dir.name
    assert (materialization_dir / "assets" / "meshes" / "part.obj").exists()


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
    assert "def build_object_model() -> ArticulatedObject:" in model_text
    assert "def run_tests() -> TestReport:" in model_text
    assert "ctx.check_model_valid()" in model_text
    assert "ctx.check_mesh_files_exist()" in model_text
    assert "ctx.check_part_geometry_connected()" in model_text
    assert "ctx.warn_if_articulation_origin_near_geometry" not in model_text
    assert "ctx.warn_if_articulation_overlaps(max_pose_samples=128)" not in model_text
    assert (
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)"
        not in model_text
    )
    assert "Default exact visual connectivity gate" in model_text
    assert "If you add a warning-tier heuristic and it fires" in model_text
    assert "Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is" in model_text
    assert 'hinge_leaf = lid.get_visual("hinge_leaf")' in model_text
    assert 'ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)' in model_text
    assert "ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)" in model_text
    assert "expect_aabb_" not in model_text

    materialization_dir = repo_root / "data" / "cache" / "record_materialization" / record_dir.name
    assert not (materialization_dir / "compile_report.json").exists()

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["provider"] == "openai"
    assert provenance["generation"]["model_id"] == "gpt-5.4"
    assert provenance["generation"]["max_turns"] == DEFAULT_MAX_TURNS
    assert provenance["run_summary"]["final_status"] == "draft"

    workbench = json.loads(
        (repo_root / "data" / "local" / "workbench.json").read_text(encoding="utf-8")
    )
    assert workbench["entries"][0]["record_id"] == record_dir.name
    assert workbench["entries"][0]["label"] == "reading lamp draft"
    assert workbench["entries"][0]["tags"] == ["draft", "lamp"]

    assert (repo_root / "data" / "cache" / "search_index.json").exists()

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
