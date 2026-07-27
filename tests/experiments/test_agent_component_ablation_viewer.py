from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from experiments.agent_component_ablations.package_viewer import package_run
from storage.library_manifest import load_manifest
from storage.repo import StorageRepo
from viewer.api.app import create_app

REPO_ROOT = Path(__file__).resolve().parents[2]


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_cell(
    cells_dir: Path,
    *,
    prompt_id: str,
    condition_id: str,
    exported: bool,
) -> None:
    cell_dir = cells_dir / f"{prompt_id}__{condition_id}"
    cell_dir.mkdir(parents=True)
    (cell_dir / "main.py").write_text("object_model = None\n", encoding="utf-8")
    (cell_dir / "system_prompt.txt").write_text("Build the object.\n", encoding="utf-8")
    _write_json(
        cell_dir / "condition.json",
        {"id": condition_id, "sdk_package": "sdk"},
    )
    _write_json(
        cell_dir / "status.json",
        {
            "status": "complete",
            "prompt_id": prompt_id,
            "condition_id": condition_id,
            "started_at": "2026-07-27T10:00:00+00:00",
            "finished_at": "2026-07-27T10:01:00+00:00",
            "generation_success": exported,
            "export_success": exported,
            "standardized_qc_success": exported,
        },
    )
    _write_json(
        cell_dir / "generation_result.json",
        {
            "turn_count": 2,
            "tool_call_count": 1,
            "compile_attempt_count": 1,
        },
    )
    stage = {"status": "success"} if exported else {"status": "failure", "error": "failed"}
    _write_json(
        cell_dir / "evaluation.json",
        {
            "export_only": stage,
            "native_full_compile": stage,
            "standardized_baseline_qc": stage,
        },
    )
    if exported:
        (cell_dir / "model.urdf").write_text('<robot name="test"/>\n', encoding="utf-8")
        mesh_dir = cell_dir / "assets" / "meshes"
        mesh_dir.mkdir(parents=True)
        (mesh_dir / "part.obj").write_text("o part\n", encoding="utf-8")
        cache_dir = cell_dir / "assets" / ".cadquery_cache"
        cache_dir.mkdir(parents=True)
        (cache_dir / "private.obj").write_text("o private\n", encoding="utf-8")


def _fake_run(run_dir: Path) -> None:
    prompts = [
        {
            "id": "test_object",
            "category": "test_category",
            "prompt": "Create a useful test object.",
        }
    ]
    conditions = [
        {
            "id": "full_baseline",
            "label": "Full SDK",
            "sdk_package": "sdk",
        },
        {
            "id": "single_pass",
            "label": "Single pass",
            "sdk_package": "sdk",
        },
    ]
    schedule = [
        {
            "cell_id": "test_object__full_baseline",
            "prompt_id": "test_object",
            "condition_id": "full_baseline",
        },
        {
            "cell_id": "test_object__single_pass",
            "prompt_id": "test_object",
            "condition_id": "single_pass",
        },
    ]
    _write_json(
        run_dir / "manifest.json",
        {
            "run_id": "test-run",
            "experiment": "agent_component_ablations",
            "created_at": "2026-07-27T10:00:00+00:00",
            "model": "gpt-5.6-sol",
            "thinking_level": "high",
            "prompt_metadata": {"prompts": prompts},
            "conditions": conditions,
            "schedule": schedule,
        },
    )
    _write_cell(
        run_dir / "cells",
        prompt_id="test_object",
        condition_id="full_baseline",
        exported=True,
    )
    _write_cell(
        run_dir / "cells",
        prompt_id="test_object",
        condition_id="single_pass",
        exported=False,
    )


def test_package_run_builds_browsable_viewer_data(tmp_path: Path) -> None:
    source_dir = tmp_path / "results" / "test-run"
    output_dir = tmp_path / "viewer" / "test-run"
    _fake_run(source_dir)

    summary = package_run(source_dir, output_dir, repo_root=REPO_ROOT)

    assert summary.record_count == 2
    assert summary.materialized_count == 1
    repo = StorageRepo(REPO_ROOT, data_root=output_dir)
    rows = load_manifest(repo)
    assert {row["category_slug"] for row in rows} == {"full_baseline", "single_pass"}

    exported_id = "rec_ablation_test_object__full_baseline"
    assert repo.layout.record_materialization_urdf_path(exported_id).is_file()
    assert (repo.layout.record_materialization_asset_meshes_dir(exported_id) / "part.obj").is_file()
    assert not (
        repo.layout.record_materialization_assets_dir(exported_id) / ".cadquery_cache"
    ).exists()

    client = TestClient(create_app(repo_root=REPO_ROOT, data_root=output_dir))
    browse = client.get("/api/records/browse")
    assert browse.status_code == 200
    assert browse.json()["total"] == 2
    assert browse.json()["facets"]["categories"] == ["full_baseline", "single_pass"]
    assert {record["materialization_status"] for record in browse.json()["records"]} == {
        "available",
        "missing",
    }
    urdf = client.get(f"/api/records/{exported_id}/files/model.urdf")
    assert urdf.status_code == 200
    assert "<robot" in urdf.text


def test_package_run_requires_force_to_replace_existing_output(tmp_path: Path) -> None:
    source_dir = tmp_path / "results" / "test-run"
    output_dir = tmp_path / "viewer" / "test-run"
    _fake_run(source_dir)
    package_run(source_dir, output_dir, repo_root=REPO_ROOT)

    try:
        package_run(source_dir, output_dir, repo_root=REPO_ROOT)
    except FileExistsError as exc:
        assert "--force" in str(exc)
    else:
        raise AssertionError("Expected an existing output directory to require --force")

    summary = package_run(source_dir, output_dir, force=True, repo_root=REPO_ROOT)
    assert summary.record_count == 2
