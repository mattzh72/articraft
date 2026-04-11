from __future__ import annotations

import asyncio
import io
import json
import subprocess
from contextlib import redirect_stdout
from pathlib import Path

import pytest

from agent import runner
from agent.defaults import DEFAULT_MAX_TURNS, GEMINI_3_FLASH_DEFAULT_MAX_TURNS
from cli.workbench import main as workbench_main
from scripts import git_hooks
from tests.helpers import FakeAgent


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FakeAgent)


def _init_git_repo(repo_root: Path) -> None:
    subprocess.run(["git", "init"], cwd=repo_root, check=True, capture_output=True, text=True)


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
            sdk_package="sdk",
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
    assert not (materialization_dir / "assets" / "meshes").exists()


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
                "--max-cost-usd",
                "2.5",
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
    assert "import cadquery as cq" in model_text
    assert "mesh_from_cadquery" in model_text
    assert "def build_object_model() -> ArticulatedObject:" in model_text
    assert "def run_tests() -> TestReport:" in model_text
    assert "ctx.check_model_valid()" not in model_text
    assert "ctx.check_mesh_assets_ready()" not in model_text
    assert "ctx.fail_if_isolated_parts()" not in model_text
    assert "ctx.warn_if_part_contains_disconnected_geometry_islands()" not in model_text
    assert "ctx.fail_if_parts_overlap_in_current_pose()" not in model_text
    assert "ctx.warn_if_articulation_origin_far_from_geometry" not in model_text
    assert "ctx.warn_if_articulation_overlaps(max_pose_samples=128)" not in model_text
    assert (
        "ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)"
        not in model_text
    )
    assert "`compile_model` automatically runs baseline sanity/QC:" in model_text
    assert "- exactly one root part" in model_text
    assert "Use `run_tests()` only for prompt-specific exact checks" in model_text
    assert "Keep pose-specific checks lean." in model_text
    assert 'hinge_leaf = lid.get_visual("hinge_leaf")' in model_text
    assert 'ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)' in model_text
    assert "ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)" in model_text
    assert "with ctx.pose({lid_hinge: hinge_limits.lower}):" not in model_text
    assert 'ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")' not in model_text
    assert "expect_aabb_" not in model_text

    materialization_dir = repo_root / "data" / "cache" / "record_materialization" / record_dir.name
    assert not (materialization_dir / "compile_report.json").exists()

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["provider"] == "openai"
    assert provenance["generation"]["model_id"] == "gpt-5.4"
    assert provenance["generation"]["max_turns"] == DEFAULT_MAX_TURNS
    assert provenance["generation"]["max_cost_usd"] == 2.5
    assert provenance["prompting"]["scaffold_mode"] == "lite"
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


def test_workbench_init_record_uses_model_specific_default_max_turns(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "init-record",
                "build a compact camera slider",
                "--provider",
                "gemini",
                "--model-id",
                "gemini-3-flash-preview",
            ]
        )
        == 0
    )

    records = sorted((repo_root / "data" / "records").iterdir())
    assert len(records) == 1
    record_dir = records[0]

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["model_id"] == "gemini-3-flash-preview"
    assert provenance["generation"]["max_turns"] == GEMINI_3_FLASH_DEFAULT_MAX_TURNS


def test_workbench_rerun_record_command_accepts_design_audit_override(
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
    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
                "--no-design-audit",
            ]
        )
        == 0
    )

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["prompting"]["post_success_design_audit"] is False
    output = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in output
    assert "search_index=" in output


def test_workbench_rerun_record_command_reuses_stored_max_cost_usd_and_accepts_override(
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
            max_cost_usd=2.5,
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
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["max_cost_usd"] == 2.5
    capsys.readouterr()

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
                "--max-cost-usd",
                "1.0",
            ]
        )
        == 0
    )
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["generation"]["max_cost_usd"] == 1.0


def test_workbench_init_record_command_accepts_design_audit_override(
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
                "--no-design-audit",
            ]
        )
        == 0
    )

    records = sorted((repo_root / "data" / "records").iterdir())
    assert len(records) == 1
    record_dir = records[0]

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["prompting"]["post_success_design_audit"] is False
    assert f"initialized record_id={record_dir.name}" in capsys.readouterr().out


def test_workbench_rerun_record_command_accepts_scaffold_mode_override(
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
    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
                "--scaffold-mode",
                "strict",
            ]
        )
        == 0
    )

    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert provenance["prompting"]["scaffold_mode"] == "strict"
    output = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in output
    assert "search_index=" in output


def test_workbench_init_record_command_accepts_scaffold_mode_override(
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
                "--scaffold-mode",
                "strict",
            ]
        )
        == 0
    )

    record_dir = next((repo_root / "data" / "records").iterdir())
    model_text = (record_dir / "model.py").read_text(encoding="utf-8")
    provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))

    assert "with ctx.pose({lid_hinge: hinge_limits.lower}):" in model_text
    assert provenance["prompting"]["scaffold_mode"] == "strict"
    assert f"initialized record_id={record_dir.name}" in capsys.readouterr().out


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


def test_workbench_init_record_warns_when_post_commit_hook_missing(tmp_path: Path) -> None:
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)
    (tmp_path / ".git" / "hooks" / "post-commit").unlink()

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            workbench_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "init-record",
                    "build a folding reading lamp",
                    "--provider",
                    "openai",
                ]
            )
            == 0
        )

    captured = output.getvalue()
    assert "Warning: managed post-commit hook is missing" in captured
    assert "just setup" in captured
    assert "just hooks-install" in captured
    assert "install-post-commit" in captured


def test_workbench_rerun_record_warns_when_post_commit_hook_missing(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    _init_git_repo(repo_root)
    git_hooks.install_post_commit_hook(repo_root)
    (repo_root / ".git" / "hooks" / "post-commit").unlink()
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

    record_dir = next((repo_root / "data" / "records").iterdir())
    output = io.StringIO()
    with redirect_stdout(output):
        assert workbench_main(["--repo-root", str(repo_root), "rerun-record", str(record_dir)]) == 0

    assert "Warning: managed post-commit hook is missing" in output.getvalue()


def test_workbench_init_record_does_not_warn_when_post_commit_hook_installed(
    tmp_path: Path,
) -> None:
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)

    output = io.StringIO()
    with redirect_stdout(output):
        assert (
            workbench_main(
                [
                    "--repo-root",
                    str(tmp_path),
                    "init-record",
                    "build a folding reading lamp",
                    "--provider",
                    "openai",
                ]
            )
            == 0
        )

    assert "Warning: managed post-commit hook" not in output.getvalue()


def test_workbench_status_does_not_warn_when_post_commit_hook_missing(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _init_git_repo(tmp_path)
    git_hooks.install_post_commit_hook(tmp_path)
    (tmp_path / ".git" / "hooks" / "post-commit").unlink()

    assert workbench_main(["--repo-root", str(tmp_path), "status"]) == 0

    assert "Warning: managed post-commit hook" not in capsys.readouterr().out
