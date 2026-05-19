from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from agent import runner
from agent.edit import edit_record as edit_record_impl
from agent.prompts import DESIGNER_PROMPT_NAME
from agent.run_context import RunExecutionOutcome
from storage.datasets import DatasetStore
from storage.repo import StorageRepo
from storage.revisions import active_model_path, active_provenance_path
from tests.helpers import FakeAgent
from viewer.api.app import create_app


class SeedInspectingAgent(FakeAgent):
    seed_snapshots: list[str] = []

    async def run(self, user_content: object):  # type: ignore[override]
        if self.file_path.exists():
            self.seed_snapshots.append(self.file_path.read_text(encoding="utf-8"))
        else:
            self.seed_snapshots.append("")
        return await super().run(user_content)


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    SeedInspectingAgent.seed_snapshots = []
    monkeypatch.setattr(runner, "ArticraftAgent", SeedInspectingAgent)


def _record_ids(repo_root: Path) -> list[str]:
    records_root = repo_root / "data" / "records"
    return sorted(path.name for path in records_root.iterdir() if path.is_dir())


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_copy_edit_dataset_child_uses_lineage_without_parent_artifact_copies(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    image_path = tmp_path / "reference.png"
    image_path.write_bytes(b"fake image")

    exit_code = asyncio.run(
        runner.run_from_input(
            "make a hinge",
            prompt_text="make a hinge",
            display_prompt="make a hinge",
            repo_root=repo_root,
            image_path=image_path,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            collection="dataset",
            category_slug="hinge",
            dataset_id="ds_hinge_seed",
        )
    )
    assert exit_code == 0
    parent_record_id = _record_ids(repo_root)[0]

    repo = StorageRepo(repo_root)
    parent_record_path = repo.layout.record_metadata_path(parent_record_id)
    parent_record_before = parent_record_path.read_text(encoding="utf-8")
    parent_model_path = active_model_path(repo, parent_record_id)
    parent_model_path.write_text("# parent unique model\n", encoding="utf-8")
    parent_cost_path = repo.layout.record_revision_cost_path(parent_record_id, "rev_000001")
    parent_cost_path.write_text('{"parent_only": true}\n', encoding="utf-8")

    outcome = asyncio.run(
        runner.edit_record(
            repo_root=repo_root,
            parent_record_id=parent_record_id,
            edit_prompt="make the hinge longer",
            record_id="rec_child_copy",
        )
    )

    assert outcome.exit_code == 0
    assert outcome.record_id == "rec_child_copy"
    assert "# parent unique model" in SeedInspectingAgent.seed_snapshots[-1]
    assert parent_record_path.read_text(encoding="utf-8") == parent_record_before
    assert parent_model_path.read_text(encoding="utf-8") == "# parent unique model\n"
    assert parent_cost_path.read_text(encoding="utf-8") == '{"parent_only": true}\n'

    child_record = _read_json(repo.layout.record_metadata_path("rec_child_copy"))
    assert child_record["schema_version"] == 3
    assert child_record["active_revision_id"] == "rev_000001"
    assert child_record["category_slug"] == "hinge"
    assert child_record["lineage"] == {
        "origin_record_id": parent_record_id,
        "parent_record_id": parent_record_id,
        "parent_revision_id": "rev_000001",
        "edit_mode": "copy",
    }

    child_dataset_entry = DatasetStore(repo).load_entry("rec_child_copy")
    assert isinstance(child_dataset_entry, dict)
    assert child_dataset_entry["category_slug"] == "hinge"
    assert child_dataset_entry["dataset_id"].startswith("ds_hinge_seed_edit_")
    assert child_dataset_entry["dataset_id"] != "ds_hinge_seed"

    child_revision_dir = repo.layout.record_revision_dir("rec_child_copy", "rev_000001")
    child_revision = _read_json(child_revision_dir / "revision.json")
    assert child_revision["parent"] == {
        "record_id": parent_record_id,
        "revision_id": "rev_000001",
    }
    assert child_revision["seed"] == {
        "record_id": parent_record_id,
        "revision_id": "rev_000001",
        "artifact": "model.py",
    }
    assert child_revision["inherited_inputs"]
    assert child_revision["inherited_inputs"][0]["record_id"] == parent_record_id
    assert list((child_revision_dir / "inputs").glob("*")) == []
    assert "parent_only" not in (child_revision_dir / "cost.json").read_text(encoding="utf-8")
    assert not (repo.layout.record_dir("rec_child_copy") / "prompt.txt").exists()
    assert not (repo.layout.record_dir("rec_child_copy") / "model.py").exists()

    grandchild = asyncio.run(
        runner.edit_record(
            repo_root=repo_root,
            parent_record_id="rec_child_copy",
            edit_prompt="make the child hinge heavier",
            record_id="rec_grandchild_copy",
        )
    )
    assert grandchild.exit_code == 0
    grandchild_revision = _read_json(
        repo.layout.record_revision_dir("rec_grandchild_copy", "rev_000001") / "revision.json"
    )
    assert grandchild_revision["inherited_inputs"]
    assert grandchild_revision["inherited_inputs"][0]["record_id"] == parent_record_id
    assert grandchild_revision["inherited_inputs"][0]["revision_id"] == "rev_000001"


def test_internal_edit_of_external_parent_uses_designer_system_prompt(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a gatehouse",
            prompt_text="make a gatehouse",
            display_prompt="make a gatehouse",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
        )
    )
    assert exit_code == 0
    record_id = _record_ids(repo_root)[0]
    repo = StorageRepo(repo_root)
    record_path = repo.layout.record_metadata_path(record_id)
    record = _read_json(record_path)
    record["creator"] = {
        "mode": "external_agent",
        "agent": "codex",
        "trace_available": False,
    }
    record_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    provenance_path = active_provenance_path(repo, record_id, record=record)
    provenance = _read_json(provenance_path)
    provenance["prompting"]["system_prompt_file"] = "EXTERNAL_AGENT_DATA.md"
    provenance_path.write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    captured: dict[str, object] = {}

    async def _capture_execute_single_run(*_args: object, **kwargs: object) -> RunExecutionOutcome:
        captured.update(kwargs)
        return RunExecutionOutcome(
            exit_code=1,
            run_id="run_capture",
            record_id="rec_external_parent_child",
            status="failed",
            message="captured",
        )

    outcome = asyncio.run(
        edit_record_impl(
            repo_root=repo_root,
            parent_record_id=record_id,
            edit_prompt="make the handle longer",
            record_id="rec_external_parent_child",
            execute_single_run_func=_capture_execute_single_run,
        )
    )

    assert outcome.exit_code == 1
    assert captured["system_prompt_path"] == DESIGNER_PROMPT_NAME


def test_history_api_and_revision_file_routes_traverse_lineage(
    fake_agent: None,
    tmp_path: Path,
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a hinge",
            prompt_text="make a hinge",
            display_prompt="make a hinge",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path=DESIGNER_PROMPT_NAME,
            sdk_package="sdk",
            collection="dataset",
            category_slug="hinge",
            dataset_id="ds_hinge_seed",
        )
    )
    assert exit_code == 0
    parent_record_id = _record_ids(repo_root)[0]
    first = asyncio.run(
        runner.edit_record(
            repo_root=repo_root,
            parent_record_id=parent_record_id,
            edit_prompt="make the hinge longer",
            record_id="rec_child_one",
        )
    )
    second = asyncio.run(
        runner.edit_record(
            repo_root=repo_root,
            parent_record_id=parent_record_id,
            edit_prompt="make the hinge wider",
            record_id="rec_child_two",
        )
    )
    assert first.exit_code == 0
    assert second.exit_code == 0

    repo = StorageRepo(repo_root)
    dataset_ids = {
        DatasetStore(repo).load_entry("rec_child_one")["dataset_id"],  # type: ignore[index]
        DatasetStore(repo).load_entry("rec_child_two")["dataset_id"],  # type: ignore[index]
    }
    assert len(dataset_ids) == 2

    client = TestClient(create_app(repo_root=repo_root))
    model_response = client.get(f"/api/records/{first.record_id}/files/model.py")
    assert model_response.status_code == 200
    assert "object_model" in model_response.text

    revision_trace = client.get(
        f"/api/records/{parent_record_id}/revisions/rev_000001/traces/trajectory.jsonl"
    )
    assert revision_trace.status_code == 200
    assert "assistant" in revision_trace.text

    child_history = client.get(f"/api/records/{first.record_id}/history").json()
    assert child_history["ancestors"][0]["record_id"] == parent_record_id
    assert child_history["ancestors"][0]["revision_id"] == "rev_000001"
    assert child_history["revisions"][0]["has_traces"] is True

    parent_history = client.get(f"/api/records/{parent_record_id}/history").json()
    descendant_ids = {row["record_id"] for row in parent_history["descendants"]}
    assert {"rec_child_one", "rec_child_two"} <= descendant_ids
