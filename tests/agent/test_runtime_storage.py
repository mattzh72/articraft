from __future__ import annotations

import asyncio
import json
import logging
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


class MeshVisualWithUnusedAssetsAgent(MeshVisualAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        result = await super().run(user_content)
        meshes_dir = self.file_path.parent / "assets" / "meshes"
        (meshes_dir / "orphan.obj").write_text("# orphan\n", encoding="utf-8")
        glb_dir = self.file_path.parent / "assets" / "glb"
        glb_dir.mkdir(parents=True, exist_ok=True)
        (glb_dir / "orphan.glb").write_bytes(b"orphan")
        return result


class ContextResetAgent(FakeAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        result = await super().run(user_content)
        result.context_reset_count = 2
        return result


class OverBudgetAgent(FakeAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text("# over budget\n", encoding="utf-8")
        (self.file_path.parent / "cost.json").write_text(
            json.dumps({"total": {"costs_usd": {"total": 0.75}}}, indent=2),
            encoding="utf-8",
        )
        return AgentResult(
            success=False,
            reason=TerminateReason.COST_LIMIT,
            message="Cost limit exceeded after turn 2: cumulative $0.750000 exceeded limit $0.500000",
            conversation=[{"role": "user", "content": user_content}],
            final_code=self.file_path.read_text(encoding="utf-8"),
            urdf_xml=None,
            compile_warnings=[],
            turn_count=2,
            tool_call_count=1,
            compile_attempt_count=0,
            usage={"prompt_tokens": 10, "candidates_tokens": 5, "total_tokens": 15},
        )


class AllInTotalsAgent(FakeAgent):
    async def run(self, user_content: object):  # type: ignore[override]
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text(
            "from __future__ import annotations\n\nobject_model = None\n",
            encoding="utf-8",
        )
        (self.file_path.parent / "cost.json").write_text(
            json.dumps(
                {
                    "total": {
                        "tokens": {
                            "prompt_tokens": 100,
                            "candidates_tokens": 20,
                            "total_tokens": 120,
                            "cached_tokens": 0,
                        },
                        "costs_usd": {"total": 0.75},
                    },
                    "all_in_total": {
                        "tokens": {
                            "prompt_tokens": 140,
                            "candidates_tokens": 25,
                            "total_tokens": 165,
                            "cached_tokens": 10,
                            "uncached_prompt_tokens": 130,
                        },
                        "costs_usd": {"total": 0.91},
                    },
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return AgentResult(
            success=True,
            reason=TerminateReason.CODE_VALID,
            message="done",
            conversation=[{"role": "user", "content": user_content}],
            final_code=self.file_path.read_text(encoding="utf-8"),
            urdf_xml="<robot name='test'/>",
            compile_warnings=[],
            turn_count=1,
            tool_call_count=0,
            compile_attempt_count=0,
            usage={"prompt_tokens": 100, "candidates_tokens": 20, "total_tokens": 120},
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
    compile_report = json.loads(
        (materialization_dir / "compile_report.json").read_text(encoding="utf-8")
    )
    assert compile_report["metrics"]["compile_level"] == "full"
    assert compile_report["metrics"]["fingerprint_inputs"]["model_py_sha256"]
    assert compile_report["metrics"]["materialization_fingerprint"]
    assert not (materialization_dir / "assets" / "meshes").exists()

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
    assert run_metadata["settings_summary"]["scaffold_mode"] == "lite"
    results_lines = (run_dirs[0] / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(results_lines) == 1
    assert json.loads(results_lines[0])["record_id"] == record_dir.name
    assert not (run_dirs[0] / "staging" / record_dir.name).exists()
    assert not (repo_root / "outputs").exists()

    original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    original_run_id = original_record["source"]["run_id"]
    original_created_at = original_record["created_at"]
    original_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert original_provenance["prompting"]["scaffold_mode"] == "lite"
    system_prompt_sha = original_provenance["prompting"]["system_prompt_sha256"]
    assert (repo_root / "data" / "system_prompts" / f"{system_prompt_sha}.txt").exists()
    original_provenance["prompting"].pop("scaffold_mode", None)
    (record_dir / "provenance.json").write_text(
        json.dumps(original_provenance, indent=2) + "\n",
        encoding="utf-8",
    )

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
    updated_provenance = json.loads((record_dir / "provenance.json").read_text(encoding="utf-8"))
    assert updated_record["record_id"] == record_dir.name
    assert updated_record["created_at"] == original_created_at
    assert updated_record["source"]["run_id"] != original_run_id
    assert updated_provenance["prompting"]["scaffold_mode"] == "strict"
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
    assert not (materialization_dir / "assets" / "meshes").exists()

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


def test_run_from_input_stamps_author_and_rerun_preserves_existing_author(
    fake_agent: None,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "_resolve_runtime_record_author", lambda repo_root: "mattzh72")

    exit_code = asyncio.run(
        runner.run_from_input(
            "make a hinge",
            prompt_text="make a hinge",
            display_prompt="make a hinge",
            repo_root=tmp_path,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=10,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
        )
    )

    assert exit_code == 0
    record_dir = next((tmp_path / "data" / "records").iterdir())
    original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert original_record["author"] == "mattzh72"

    monkeypatch.setattr(runner, "_resolve_runtime_record_author", lambda repo_root: "RuiningLi")
    rerun_exit_code = asyncio.run(
        runner.rerun_record_in_place(
            repo_root=tmp_path,
            record_id=record_dir.name,
        )
    )

    assert rerun_exit_code == 0
    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert updated_record["author"] == "mattzh72"


def test_create_workbench_draft_record_stamps_author_immediately(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "_resolve_runtime_record_author", lambda repo_root: "mattzh72")

    record_dir = runner.create_workbench_draft_record(
        repo_root=tmp_path,
        prompt_text="make a draft hinge",
        provider="openai",
        thinking_level="high",
        max_turns=10,
        system_prompt_path=DESIGNER_PROMPT_NAME,
        sdk_package="sdk",
    )

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert record["author"] == "mattzh72"


def test_over_budget_run_persists_failed_run_metadata_without_record(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", OverBudgetAgent)

    exit_code = asyncio.run(
        runner.run_from_input(
            "make an expensive model",
            prompt_text="make an expensive model",
            display_prompt="make an expensive model",
            repo_root=tmp_path,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            max_cost_usd=0.5,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            sdk_docs_mode="full",
        )
    )

    assert exit_code == 2
    records_root = tmp_path / "data" / "records"
    assert not records_root.exists() or list(records_root.iterdir()) == []

    run_dir = next(
        path for path in (tmp_path / "data" / "cache" / "runs").iterdir() if path.is_dir()
    )
    run_metadata = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["status"] == "failed"
    assert run_metadata["settings_summary"]["max_cost_usd"] == 0.5

    result_rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert len(result_rows) == 1
    assert "Cost limit exceeded" in result_rows[0]["message"]

    staging_dirs = [path for path in (run_dir / "staging").iterdir() if path.is_dir()]
    assert len(staging_dirs) == 1
    assert (
        json.loads((staging_dirs[0] / "cost.json").read_text(encoding="utf-8"))["total"][
            "costs_usd"
        ]["total"]
        == 0.75
    )


def test_successful_run_logs_all_in_cost_totals(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", AllInTotalsAgent)

    with caplog.at_level(logging.INFO):
        exit_code = asyncio.run(
            runner.run_from_input(
                "make a model with compaction",
                prompt_text="make a model with compaction",
                display_prompt="make a model with compaction",
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
    assert (
        "Total tokens: {'prompt_tokens': 140, 'cached_tokens': 10, 'uncached_prompt_tokens': 130, 'candidates_tokens': 25, 'total_tokens': 165}"
        in caplog.text
    )
    assert "Total cost: $0.910000" in caplog.text


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


def test_successful_run_copies_only_referenced_mesh_assets(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", MeshVisualWithUnusedAssetsAgent)

    exit_code = asyncio.run(
        runner.run_from_input(
            "make a mesh part with extras",
            prompt_text="make a mesh part with extras",
            display_prompt="make a mesh part with extras",
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
    assert not (materialization_dir / "assets" / "meshes" / "orphan.obj").exists()
    assert not (materialization_dir / "assets" / "glb").exists()


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
    assert category == {
        "schema_version": 1,
        "slug": "crane_tower",
        "title": "Crane Tower",
        "description": "",
    }

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
    assert category == {
        "schema_version": 1,
        "slug": "crane_tower",
        "title": "Crane Tower",
        "description": "",
    }

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
    compile_report = json.loads(
        (materialization_dir / "compile_report.json").read_text(encoding="utf-8")
    )
    assert compile_report["metrics"]["compile_level"] == "full"
    assert compile_report["metrics"]["fingerprint_inputs"]["model_py_sha256"]
    assert compile_report["metrics"]["materialization_fingerprint"]
    assert list((record_dir / "inputs").iterdir()) == []

    run_dir = next((repo_root / "data" / "cache" / "runs").iterdir())
    run_metadata = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert run_metadata["status"] == "success"
    results = (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(results) == 1
    assert json.loads(results[0])["status"] == "success"
    assert not (run_dir / "staging" / record_dir.name).exists()
