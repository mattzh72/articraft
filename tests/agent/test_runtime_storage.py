from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent import runner
from agent.models import AgentResult, TerminateReason
from agent.prompts import DESIGNER_PROMPT_NAME
from tests.helpers import FakeAgent


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FakeAgent)


class DeletingImageAgent(FakeAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        if isinstance(user_content, list):
            for item in user_content:
                if not isinstance(item, dict):
                    continue
                image_path = item.get("image_path")
                if isinstance(image_path, str):
                    Path(image_path).unlink()
                    break
        return await super().run(user_content)


class MeshVisualAgent(FakeAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text(
            "from __future__ import annotations\n\nobject_model = None\n",
            encoding="utf-8",
        )
        meshes_dir = self.file_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text(
            "\n".join(
                [
                    "o part",
                    "v 0 0 0",
                    "v 1 0 0",
                    "v 0 1 0",
                    "f 1 2 3",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        cost_path = self.file_path.parent / "cost.json"
        cost_path.write_text(
            json.dumps({"total": {"costs_usd": {"total": 0.123456}}}, indent=2),
            encoding="utf-8",
        )
        if self.trace_dir is not None:
            self.trace_dir.mkdir(parents=True, exist_ok=True)
            (self.trace_dir / "trajectory.jsonl").write_text(
                '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
                encoding="utf-8",
            )
        return AgentResult(
            success=True,
            reason=TerminateReason.CODE_VALID,
            message="done",
            conversation=[{"role": "user", "content": user_content}],
            final_code=self.file_path.read_text(encoding="utf-8"),
            urdf_xml=(
                "<robot name='mesh_visual'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            compile_warnings=[],
            turn_count=3,
            tool_call_count=5,
            compile_attempt_count=2,
            usage={"prompt_tokens": 10, "candidates_tokens": 5, "total_tokens": 15},
        )


def test_workbench_run_and_rerun_persist_runtime_artifacts(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a gearbox",
            prompt_text="make a gearbox",
            display_prompt="make a gearbox",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="gearbox try",
            tags=["gear", "test"],
        )
    )
    assert exit_code == 0

    records_root = repo_root / "data" / "records"
    record_dirs = [path for path in records_root.iterdir() if path.is_dir()]
    assert len(record_dirs) == 1
    record_dir = record_dirs[0]
    materialization_dir = repo_root / "data" / "cache" / "record_materialization" / record_dir.name

    assert (record_dir / "prompt.txt").read_text(encoding="utf-8") == "make a gearbox"
    assert (record_dir / "model.py").exists()
    assert (record_dir / "provenance.json").exists()
    assert (record_dir / "cost.json").exists()
    assert (record_dir / "traces").exists()
    assert (record_dir / "traces" / "trajectory.jsonl.zst").exists()
    assert not (record_dir / "traces" / "trajectory.jsonl").exists()
    assert not (record_dir / "traces" / "conversation.jsonl").exists()
    assert (materialization_dir / "model.urdf").read_text(
        encoding="utf-8"
    ) == "<robot name='test'/>"
    assert (materialization_dir / "compile_report.json").exists()
    assert (materialization_dir / "assets" / "meshes" / "part.obj").exists()

    workbench_path = repo_root / "data" / "local" / "workbench.json"
    workbench = json.loads(workbench_path.read_text(encoding="utf-8"))
    assert len(workbench["entries"]) == 1
    assert workbench["entries"][0]["record_id"] == record_dir.name
    assert workbench["entries"][0]["label"] == "gearbox try"

    runs_root = repo_root / "data" / "cache" / "runs"
    run_dirs = [path for path in runs_root.iterdir() if path.is_dir()]
    assert len(run_dirs) == 1
    run_metadata = json.loads((run_dirs[0] / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["status"] == "success"
    results_lines = (run_dirs[0] / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(results_lines) == 1
    assert json.loads(results_lines[0])["record_id"] == record_dir.name
    assert not (run_dirs[0] / "staging" / record_dir.name).exists()
    assert not (repo_root / "outputs").exists()

    original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    original_run_id = original_record["source"]["run_id"]
    original_created_at = original_record["created_at"]
    original_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    system_prompt_sha = original_provenance["prompting"]["system_prompt_sha256"]
    assert (repo_root / "data" / "system_prompts" / f"{system_prompt_sha}.txt").exists()

    workbench["entries"][0]["archived"] = True
    workbench_path.write_text(json.dumps(workbench, indent=2) + "\n", encoding="utf-8")

    (record_dir / "model.py").write_text("# stale\n", encoding="utf-8")
    (record_dir / "cost.json").write_text('{"stale": true}\n', encoding="utf-8")
    (record_dir / "traces" / "stale.txt").write_text("stale\n", encoding="utf-8")
    stale_glb_dir = record_dir / "assets" / "glb"
    stale_glb_dir.mkdir(parents=True, exist_ok=True)
    (stale_glb_dir / "stale.glb").write_text("stale\n", encoding="utf-8")
    stale_viewer_dir = record_dir / "assets" / "viewer"
    stale_viewer_dir.mkdir(parents=True, exist_ok=True)
    (stale_viewer_dir / "index.html").write_text("stale\n", encoding="utf-8")

    rerun_exit_code = asyncio.run(
        runner.rerun_record_in_place(
            repo_root=repo_root,
            record_id=record_dir.name,
        )
    )
    assert rerun_exit_code == 0

    record_dirs = [path for path in records_root.iterdir() if path.is_dir()]
    assert [path.name for path in record_dirs] == [record_dir.name]
    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert updated_record["record_id"] == record_dir.name
    assert updated_record["created_at"] == original_created_at
    assert updated_record["source"]["run_id"] != original_run_id
    assert (
        (record_dir / "model.py")
        .read_text(encoding="utf-8")
        .startswith("from __future__ import annotations")
    )
    assert (
        json.loads((record_dir / "cost.json").read_text(encoding="utf-8"))["total"]["costs_usd"][
            "total"
        ]
        == 0.123456
    )
    assert (record_dir / "traces" / "trajectory.jsonl.zst").exists()
    assert not (record_dir / "traces" / "trajectory.jsonl").exists()
    assert not (record_dir / "traces" / "conversation.jsonl").exists()
    assert not (record_dir / "traces" / "stale.txt").exists()
    assert not (record_dir / "assets" / "glb").exists()
    assert not (record_dir / "assets" / "viewer").exists()
    assert (materialization_dir / "assets" / "meshes" / "part.obj").exists()

    workbench = json.loads(workbench_path.read_text(encoding="utf-8"))
    assert len(workbench["entries"]) == 1
    assert workbench["entries"][0]["record_id"] == record_dir.name
    assert workbench["entries"][0]["label"] == "gearbox try"
    assert workbench["entries"][0]["tags"] == ["gear", "test"]
    assert workbench["entries"][0]["archived"] is True

    run_dirs = sorted(path for path in runs_root.iterdir() if path.is_dir())
    assert len(run_dirs) == 2
    latest_run_metadata = json.loads((run_dirs[-1] / "run.json").read_text(encoding="utf-8"))
    assert latest_run_metadata["status"] == "success"
    latest_results = (run_dirs[-1] / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(latest_results) == 1
    assert json.loads(latest_results[0])["record_id"] == record_dir.name
    assert not (run_dirs[-1] / "staging" / record_dir.name).exists()


def test_successful_run_rewrites_visual_meshes_to_glb(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", MeshVisualAgent)

    exit_code = asyncio.run(
        runner.run_from_input(
            "make a mesh part",
            prompt_text="make a mesh part",
            display_prompt="make a mesh part",
            repo_root=tmp_path,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
            label=None,
            tags=[],
        )
    )

    assert exit_code == 0

    records_root = tmp_path / "data" / "records"
    record_dirs = [path for path in records_root.iterdir() if path.is_dir()]
    assert len(record_dirs) == 1
    record_dir = record_dirs[0]
    materialization_dir = tmp_path / "data" / "cache" / "record_materialization" / record_dir.name

    assert "assets/meshes/part.glb" in (materialization_dir / "model.urdf").read_text(
        encoding="utf-8"
    )
    assert (materialization_dir / "assets" / "meshes" / "part.glb").exists()


def test_dataset_run_and_rerun_preserve_dataset_metadata(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a tower crane",
            prompt_text="make a tower crane",
            display_prompt="make a tower crane",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
            collection="dataset",
            category_slug="crane_tower",
            dataset_id="ds_crane_tower_0001",
        )
    )
    assert exit_code == 0

    records_root = repo_root / "data" / "records"
    record_dirs = [path for path in records_root.iterdir() if path.is_dir()]
    assert len(record_dirs) == 1
    record_dir = record_dirs[0]

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert record["collections"] == ["dataset"]
    assert record["category_slug"] == "crane_tower"

    dataset_entry = json.loads((record_dir / "dataset_entry.json").read_text(encoding="utf-8"))
    assert dataset_entry["dataset_id"] == "ds_crane_tower_0001"
    assert dataset_entry["category_slug"] == "crane_tower"

    category = json.loads(
        (repo_root / "data" / "categories" / "crane_tower" / "category.json").read_text(
            encoding="utf-8"
        )
    )
    assert category["title"] == "Crane Tower"
    assert category["current_count"] == 1
    assert category["last_item_index"] == 1
    assert category["run_count"] == 1

    manifest = json.loads(
        (repo_root / "data" / "cache" / "manifests" / "dataset.json").read_text(encoding="utf-8")
    )
    assert manifest["generated"] == [{"name": "ds_crane_tower_0001", "record_id": record_dir.name}]
    assert not (repo_root / "data" / "local" / "workbench.json").exists()

    runs_root = repo_root / "data" / "cache" / "runs"
    run_dirs = [path for path in runs_root.iterdir() if path.is_dir()]
    assert len(run_dirs) == 1
    run_metadata = json.loads((run_dirs[0] / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["status"] == "success"
    assert run_metadata["collection"] == "dataset"
    assert run_metadata["run_mode"] == "dataset_single"
    assert not (run_dirs[0] / "staging" / record_dir.name).exists()

    original_run_id = record["source"]["run_id"]
    original_promoted_at = dataset_entry["promoted_at"]
    (record_dir / "model.py").write_text("# stale dataset\n", encoding="utf-8")

    rerun_exit_code = asyncio.run(
        runner.rerun_record_in_place(
            repo_root=repo_root,
            record_id=record_dir.name,
        )
    )
    assert rerun_exit_code == 0

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert record["record_id"] == record_dir.name
    assert record["source"]["run_id"] != original_run_id
    assert record["collections"] == ["dataset"]
    assert record["category_slug"] == "crane_tower"
    assert (
        (record_dir / "model.py")
        .read_text(encoding="utf-8")
        .startswith("from __future__ import annotations")
    )

    dataset_entry = json.loads((record_dir / "dataset_entry.json").read_text(encoding="utf-8"))
    assert dataset_entry["dataset_id"] == "ds_crane_tower_0001"
    assert dataset_entry["category_slug"] == "crane_tower"
    assert dataset_entry["promoted_at"] == original_promoted_at

    category = json.loads(
        (repo_root / "data" / "categories" / "crane_tower" / "category.json").read_text(
            encoding="utf-8"
        )
    )
    assert category["current_count"] == 1
    assert category["last_item_index"] == 1
    assert category["run_count"] == 2

    manifest = json.loads(
        (repo_root / "data" / "cache" / "manifests" / "dataset.json").read_text(encoding="utf-8")
    )
    assert manifest["generated"] == [{"name": "ds_crane_tower_0001", "record_id": record_dir.name}]

    run_dirs = sorted(path for path in runs_root.iterdir() if path.is_dir())
    assert len(run_dirs) == 2
    latest_run_metadata = json.loads((run_dirs[-1] / "run.json").read_text(encoding="utf-8"))
    assert latest_run_metadata["status"] == "success"
    assert latest_run_metadata["collection"] == "dataset"
    assert latest_run_metadata["run_mode"] == "dataset_single"
    assert not (run_dirs[-1] / "staging" / record_dir.name).exists()


def test_workbench_run_succeeds_when_persisted_input_image_disappears(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", DeletingImageAgent)

    repo_root = tmp_path
    image_path = repo_root / "reference.png"
    image_path.write_bytes(b"\x89PNG\r\n\x1a\n")

    exit_code = asyncio.run(
        runner.run_from_input(
            [
                {"type": "input_text", "text": "make a lamp"},
                {"type": "input_image", "image_path": str(image_path), "detail": "high"},
            ],
            prompt_text="make a lamp",
            display_prompt="make a lamp",
            repo_root=repo_root,
            image_path=image_path,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="lamp try",
            tags=["lamp"],
        )
    )
    assert exit_code == 0

    record_dir = next((repo_root / "data" / "records").iterdir())
    materialization_dir = repo_root / "data" / "cache" / "record_materialization" / record_dir.name
    assert (record_dir / "record.json").exists()
    assert (record_dir / "provenance.json").exists()
    assert (materialization_dir / "compile_report.json").exists()
    assert list((record_dir / "inputs").iterdir()) == []

    run_dir = next((repo_root / "data" / "cache" / "runs").iterdir())
    run_metadata = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["status"] == "success"
    results = (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(results) == 1
    assert json.loads(results[0])["status"] == "success"
    assert not (run_dir / "staging" / record_dir.name).exists()
