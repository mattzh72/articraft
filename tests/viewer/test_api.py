from __future__ import annotations

import logging
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.models import (
    CategoryRecord,
    DisplayMetadata,
    Record,
    RecordArtifacts,
    RunRecord,
    SourceRef,
)
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from storage.trajectories import compress_trajectory_file
from viewer.api.app import create_app
from viewer.api.store import ViewerStore


def test_viewer_store_staging_listing_avoids_recursive_tree_scan(
    tmp_path: Path, monkeypatch
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_live_001",
            run_mode="workbench_single",
            collection="workbench",
            created_at="2026-03-18T08:00:00Z",
            updated_at="2026-03-18T08:00:00Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="running",
            prompt_count=1,
        )
    )

    live_stage_dir = repo.layout.run_staging_dir("run_live_001") / "rec_stage_001"
    live_stage_dir.mkdir(parents=True, exist_ok=True)
    (live_stage_dir / "prompt.txt").write_text("prototype chair", encoding="utf-8")
    (live_stage_dir / "model.py").write_text("# staged model\n", encoding="utf-8")
    (live_stage_dir / "assets" / "meshes").mkdir(parents=True, exist_ok=True)
    (live_stage_dir / "assets" / "meshes" / "preview.obj").write_text("o tri\n", encoding="utf-8")
    (live_stage_dir / "traces").mkdir(parents=True, exist_ok=True)
    (live_stage_dir / "traces" / "trajectory.jsonl").write_text(
        '{"type":"message"}\n', encoding="utf-8"
    )

    newer_mtime = 1_710_750_000
    os.utime(live_stage_dir / "model.py", (newer_mtime, newer_mtime))

    def fail_rglob(self: Path, pattern: str):  # pragma: no cover - assertion helper
        raise AssertionError(f"unexpected recursive scan of {self} with {pattern}")

    monkeypatch.setattr(Path, "rglob", fail_rglob)

    entries = ViewerStore(tmp_path, ensure_search_index=False).list_staging_entries()

    assert len(entries) == 1
    assert entries[0].record_id == "rec_stage_001"
    assert entries[0].updated_at is not None


def test_viewer_store_delete_record_removes_empty_ad_hoc_category(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="internet_router",
            title="Internet Router",
            current_count=1,
            last_item_index=1,
            run_count=1,
        )
    )
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id="rec_router_001",
            created_at="2026-03-19T14:44:36Z",
            updated_at="2026-03-19T14:44:36Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="internet_router",
            source=SourceRef(run_id="run_router_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="Router",
                prompt_preview="A realistic model of an internet router.",
            ),
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
    DatasetStore(repo).promote_record(
        record_id="rec_router_001",
        dataset_id="ds_internet_router_0001",
        promoted_at="2026-03-19T14:45:00Z",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_router_001",
            run_mode="dataset_single",
            collection="dataset",
            created_at="2026-03-19T14:44:36Z",
            updated_at="2026-03-19T14:45:00Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug="internet_router",
            prompt_count=1,
        )
    )

    deleted = ViewerStore(tmp_path).delete_record("rec_router_001")

    assert deleted is True
    assert not repo.layout.record_dir("rec_router_001").exists()
    assert not repo.layout.category_metadata_path("internet_router").exists()
    assert DatasetStore(repo).list_entries() == []


def test_record_trace_system_prompt_requires_canonical_prompt_when_sha_is_recorded(
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id="rec_prompt_001",
            created_at="2026-03-22T09:00:00Z",
            updated_at="2026-03-22T09:00:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=None,
            source=SourceRef(run_id="run_prompt_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="Prompt Record",
                prompt_preview="inspect prompt fallback",
            ),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
        )
    )
    record_dir = repo.layout.record_dir("rec_prompt_001")
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_prompt_001",
            "prompting": {
                "system_prompt_file": "designer_system_prompt.txt",
                "system_prompt_sha256": "missing-prompt-sha",
            },
        },
    )
    trace_dir = repo.layout.record_traces_dir("rec_prompt_001")
    trace_dir.mkdir(parents=True, exist_ok=True)
    (trace_dir / "designer_system_prompt.txt").write_text(
        "trace-local fallback prompt\n",
        encoding="utf-8",
    )

    client = TestClient(create_app(repo_root=tmp_path))

    with caplog.at_level(logging.WARNING):
        response = client.get("/api/records/rec_prompt_001/traces/designer_system_prompt.txt")

    assert response.status_code == 404
    assert "refusing trace-local fallback" in caplog.text


def test_viewer_api_end_to_end(tmp_path: Path) -> None:
    repo_root = tmp_path
    repo = StorageRepo(repo_root)
    repo.ensure_layout()

    record_store = RecordStore(repo)
    record = Record(
        schema_version=1,
        record_id="rec_001",
        created_at="2026-03-17T19:24:12Z",
        updated_at="2026-03-17T19:25:24Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Test hinge model",
            prompt_preview="create a hinge with two linked parts",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    (repo.layout.record_dir("rec_001") / "prompt.txt").write_text(
        "create a hinge with two linked parts",
        encoding="utf-8",
    )
    repo.write_json(
        repo.layout.record_materialization_compile_report_path("rec_001"),
        {
            "schema_version": 1,
            "record_id": "rec_001",
            "status": "success",
            "metrics": {"turn_count": 3},
        },
    )
    repo.write_json(
        repo.layout.record_dir("rec_001") / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_001",
            "generation": {"provider": "openai", "model_id": "gpt-5.4"},
            "prompting": {
                "system_prompt_file": "designer_system_prompt.txt",
                "system_prompt_sha256": "prompt-sha-001",
            },
        },
    )
    repo.write_text(
        repo.layout.system_prompt_path("prompt-sha-001"),
        "shared prompt text\n",
    )
    repo.write_json(
        repo.layout.record_dir("rec_001") / "cost.json",
        {
            "total": {
                "tokens": {"total_tokens": 1000},
                "costs_usd": {"total": 0.05},
            }
        },
    )
    asset_meshes_dir = repo.layout.record_materialization_asset_meshes_dir("rec_001")
    asset_meshes_dir.mkdir(parents=True, exist_ok=True)
    (asset_meshes_dir / "part.obj").write_text(
        "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8"
    )
    trace_dir = repo.layout.record_traces_dir("rec_001")
    trace_dir.mkdir(parents=True, exist_ok=True)
    (trace_dir / "trajectory.jsonl").write_text(
        '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
        encoding="utf-8",
    )
    compress_trajectory_file(
        trace_dir / "trajectory.jsonl",
        trace_dir / "trajectory.jsonl.zst",
    )
    (trace_dir / "trajectory.jsonl").unlink()

    collections = CollectionStore(repo)
    collections.append_workbench_entry(
        record_id="rec_001",
        added_at="2026-03-17T20:20:03Z",
        label="hinge trial",
        tags=["smoke"],
    )

    dj_record = Record(
        schema_version=1,
        record_id="rec_dj_001",
        created_at="2026-03-17T19:24:13Z",
        updated_at="2026-03-17T19:25:25Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="dj_equipment",
        source=SourceRef(run_id="run_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="DJ equipment",
            prompt_preview="A realistic DJ equipment setup.",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(dj_record)
    (repo.layout.record_dir("rec_dj_001") / "prompt.txt").write_text(
        "A realistic DJ equipment setup with mixer and faders.",
        encoding="utf-8",
    )
    collections.append_workbench_entry(
        record_id="rec_dj_001",
        added_at="2026-03-17T20:20:04Z",
        label="dj trial",
        tags=["audio"],
    )

    bike_record = Record(
        schema_version=1,
        record_id="rec_bike_001",
        created_at="2026-03-17T19:24:14Z",
        updated_at="2026-03-17T19:25:26Z",
        rating=2,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Stationary exercise bike",
            prompt_preview="A realistic stationary bike with adjustable seat.",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json="cost.json",
        ),
        collections=["workbench"],
    )
    record_store.write_record(bike_record)
    (repo.layout.record_dir("rec_bike_001") / "prompt.txt").write_text(
        "A realistic stationary exercise bike with adjustable seat and handlebar structure.",
        encoding="utf-8",
    )
    repo.write_json(
        repo.layout.record_dir("rec_bike_001") / "cost.json",
        {
            "total": {
                "tokens": {"total_tokens": 1200},
                "costs_usd": {"total": 0.15},
            }
        },
    )
    collections.append_workbench_entry(
        record_id="rec_bike_001",
        added_at="2026-03-17T20:20:05Z",
        label="bike trial",
        tags=["fitness"],
    )

    datasets = DatasetStore(repo)
    datasets.promote_record(
        record_id="rec_001",
        dataset_id="hinge_dataset_001",
        promoted_at="2026-03-17T20:21:03Z",
    )
    datasets.promote_record(
        record_id="rec_dj_001",
        dataset_id="dj_dataset_001",
        promoted_at="2026-03-17T20:21:04Z",
    )

    runs = RunStore(repo)
    runs.write_run(
        RunRecord(
            schema_version=1,
            run_id="run_001",
            run_mode="workbench_single",
            collection="workbench",
            created_at="2026-03-17T19:24:12Z",
            updated_at="2026-03-17T19:25:24Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            prompt_count=1,
        )
    )
    runs.append_result(
        "run_001",
        {
            "record_id": "rec_001",
            "status": "success",
            "turn_count": 3,
            "tool_call_count": 2,
            "compile_attempt_count": 1,
        },
    )
    runs.write_run(
        RunRecord(
            schema_version=1,
            run_id="run_live_001",
            run_mode="workbench_single",
            collection="workbench",
            created_at="2026-03-18T08:00:00Z",
            updated_at="2026-03-18T08:00:00Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="running",
            prompt_count=1,
        )
    )
    live_stage_dir = repo.layout.run_staging_dir("run_live_001") / "rec_stage_001"
    live_stage_dir.mkdir(parents=True, exist_ok=True)
    (live_stage_dir / "prompt.txt").write_text(
        "prototype folding chair with a partially refined frame",
        encoding="utf-8",
    )
    (live_stage_dir / "model.py").write_text(
        "from __future__ import annotations\n\n# staged model\n",
        encoding="utf-8",
    )
    (live_stage_dir / "model.urdf").write_text(
        "<robot name='stage_preview'/>",
        encoding="utf-8",
    )
    repo.write_json(
        live_stage_dir / "cost.json",
        {
            "model_id": "gpt-5.4",
            "turns": [
                {"input_tokens": 100, "output_tokens": 200},
                {"input_tokens": 80, "output_tokens": 160},
            ],
        },
    )
    live_trace_dir = live_stage_dir / "traces"
    live_trace_dir.mkdir(parents=True, exist_ok=True)
    (live_trace_dir / "trajectory.jsonl").write_text(
        '{"type":"message","message":{"role":"assistant","content":"staging"}}\n',
        encoding="utf-8",
    )
    live_mesh_dir = live_stage_dir / "assets" / "meshes"
    live_mesh_dir.mkdir(parents=True, exist_ok=True)
    (live_mesh_dir / "preview.obj").write_text(
        "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8"
    )

    client = TestClient(create_app(repo_root=repo_root))

    assert client.get("/health").json()["status"] == "ok"

    root_response = client.get("/")
    assert root_response.status_code == 200

    bootstrap = client.get("/api/bootstrap").json()
    assert bootstrap["repo_root"] == repo_root.resolve().as_posix()
    assert len(bootstrap["workbench_entries"]) == 3
    assert len(bootstrap["dataset_entries"]) == 2
    assert len(bootstrap["staging_entries"]) == 1
    assert len(bootstrap["runs"]) == 2
    assert bootstrap["staging_entries"][0]["run_id"] == "run_live_001"
    assert bootstrap["staging_entries"][0]["record_id"] == "rec_stage_001"
    assert bootstrap["staging_entries"][0]["status"] == "running"
    assert (
        bootstrap["staging_entries"][0]["title"]
        == "prototype folding chair with a partially refined frame"
    )
    assert (
        bootstrap["staging_entries"][0]["prompt_preview"]
        == "prototype folding chair with a partially refined frame"
    )
    assert bootstrap["staging_entries"][0]["turn_count"] == 2
    assert bootstrap["staging_entries"][0]["has_checkpoint_urdf"] is True
    assert bootstrap["staging_entries"][0]["checkpoint_updated_at"] is not None
    assert bootstrap["staging_entries"][0]["model_script_updated_at"] is not None
    assert bootstrap["staging_entries"][0]["has_traces"] is True

    workbench = client.get("/api/collections/workbench").json()
    workbench_by_id = {
        item["record"]["record_id"]: item for item in workbench if item.get("record")
    }
    assert workbench_by_id["rec_001"]["record"]["rating"] is None

    dataset = client.get("/api/collections/dataset").json()
    assert [item["dataset_id"] for item in dataset] == ["dj_dataset_001", "hinge_dataset_001"]

    categories = client.get("/api/categories").json()
    assert categories == [
        {"slug": "dj_equipment", "title": "Dj Equipment"},
        {"slug": "hinges", "title": "Hinges"},
    ]

    staging = client.get("/api/staging").json()
    assert len(staging) == 1
    assert staging[0]["record_id"] == "rec_stage_001"
    assert staging[0]["staging_dir"] == "data/cache/runs/run_live_001/staging/rec_stage_001"
    assert staging[0]["checkpoint_updated_at"] is not None
    assert staging[0]["model_script_updated_at"] is not None

    run_summaries = client.get("/api/runs").json()
    run_summary_by_id = {item["run_id"]: item for item in run_summaries}
    assert run_summary_by_id["run_001"]["success_count"] == 1
    assert run_summary_by_id["run_live_001"]["status"] == "running"
    assert run_summary_by_id["run_live_001"]["success_count"] == 0

    run_detail = client.get("/api/runs/run_001").json()
    assert run_detail["run"]["run_id"] == "run_001"
    assert run_detail["results"][0]["record_id"] == "rec_001"
    assert run_detail["records"][0]["summary"]["title"] == "Test hinge model"
    assert run_detail["records"][0]["summary"]["rating"] is None

    live_run_detail = client.get("/api/runs/run_live_001").json()
    assert live_run_detail["run"]["run_id"] == "run_live_001"
    assert live_run_detail["results"] == []
    assert live_run_detail["records"] == []

    search_results = client.get("/api/records/search?q=dj&source=workbench").json()
    assert [item["record_id"] for item in search_results] == ["rec_dj_001"]

    dataset_category_search = client.get(
        "/api/records/search?q=dj&source=dataset&category=dj_equipment"
    ).json()
    assert [item["record_id"] for item in dataset_category_search] == ["rec_dj_001"]

    mismatched_dataset_category_search = client.get(
        "/api/records/search?q=dj&source=dataset&category=hinges"
    ).json()
    assert mismatched_dataset_category_search == []

    multi_category_search = client.get(
        "/api/records/search?q=dj&source=dataset&category=hinges&category=dj_equipment"
    ).json()
    assert [item["record_id"] for item in multi_category_search] == ["rec_dj_001"]

    cost_range_search = client.get(
        "/api/records/search?q=hinge&source=workbench&cost_min=0.04&cost_max=0.06"
    ).json()
    assert [item["record_id"] for item in cost_range_search] == ["rec_001"]

    cost_range_excludes_missing = client.get(
        "/api/records/search?q=dj&source=workbench&cost_min=0.00&cost_max=0.10"
    ).json()
    assert cost_range_excludes_missing == []

    update_rating = client.put("/api/records/rec_001/rating", json={"rating": 4})
    assert update_rating.status_code == 200
    assert update_rating.json()["rating"] == 4

    updated_workbench = client.get("/api/collections/workbench").json()
    updated_workbench_by_id = {
        item["record"]["record_id"]: item for item in updated_workbench if item.get("record")
    }
    assert updated_workbench_by_id["rec_001"]["record"]["rating"] == 4
    persisted_record = repo.read_json(repo.layout.record_metadata_path("rec_001"))
    assert persisted_record["rating"] == 4

    promote_response = client.post(
        "/api/records/rec_bike_001/promote",
        json={"category_title": "Exercise Bikes"},
    )
    assert promote_response.status_code == 200
    assert promote_response.json()["record_id"] == "rec_bike_001"
    assert promote_response.json()["dataset_id"] == "ds_exercise_bikes_0001"
    assert promote_response.json()["category_slug"] == "exercise_bikes"

    promoted_bike_record = repo.read_json(repo.layout.record_metadata_path("rec_bike_001"))
    assert promoted_bike_record["category_slug"] == "exercise_bikes"
    assert promoted_bike_record["collections"] == ["dataset"]
    assert (
        repo.read_json(repo.layout.category_metadata_path("exercise_bikes"))["title"]
        == "Exercise Bikes"
    )

    bootstrap_after_promote = client.get("/api/bootstrap").json()
    assert len(bootstrap_after_promote["workbench_entries"]) == 2
    assert [item["dataset_id"] for item in bootstrap_after_promote["dataset_entries"]] == [
        "dj_dataset_001",
        "ds_exercise_bikes_0001",
        "hinge_dataset_001",
    ]

    stats = client.get("/api/stats")
    assert stats.status_code == 200
    stats_payload = stats.json()
    assert stats_payload["total_records"] == 3
    assert stats_payload["total_runs"] == 2
    assert stats_payload["total_cost_usd"] == 0.2
    assert stats_payload["category_counts"] == {
        "dj_equipment": 1,
        "exercise_bikes": 1,
        "hinges": 1,
    }
    assert stats_payload["category_stats"]["hinges"] == {
        "count": 1,
        "average_rating": 4.0,
        "average_cost_usd": 0.05,
    }
    assert stats_payload["category_stats"]["dj_equipment"] == {
        "count": 1,
        "average_rating": None,
        "average_cost_usd": None,
    }
    assert stats_payload["category_stats"]["exercise_bikes"] == {
        "count": 1,
        "average_rating": 2.0,
        "average_cost_usd": 0.15,
    }
    invalid_rating = client.put("/api/records/rec_001/rating", json={"rating": 7})
    assert invalid_rating.status_code == 422

    trace_file = client.get("/api/records/rec_001/traces/conversation.jsonl")
    assert trace_file.status_code == 200
    assert '"role":"assistant"' in trace_file.text
    trajectory_file = client.get("/api/records/rec_001/traces/trajectory.jsonl")
    assert trajectory_file.status_code == 200
    assert trajectory_file.text == trace_file.text
    prompt_trace = client.get("/api/records/rec_001/traces/designer_system_prompt.txt")
    assert prompt_trace.status_code == 200
    assert prompt_trace.text == "shared prompt text\n"

    mesh_file = client.get("/api/records/rec_001/files/assets/meshes/part.obj")
    assert mesh_file.status_code == 200
    assert "v 1 0 0" in mesh_file.text

    staging_urdf = client.get("/api/staging/run_live_001/rec_stage_001/files/model.urdf")
    assert staging_urdf.status_code == 200
    assert "stage_preview" in staging_urdf.text

    staging_mesh = client.get(
        "/api/staging/run_live_001/rec_stage_001/files/assets/meshes/preview.obj"
    )
    assert staging_mesh.status_code == 200
    assert "v 1 0 0" in staging_mesh.text

    staging_prompt = client.get("/api/staging/run_live_001/rec_stage_001/text/prompt.txt")
    assert staging_prompt.status_code == 200
    assert staging_prompt.json()["record_id"] == "rec_stage_001"
    assert "folding chair" in staging_prompt.json()["content"]

    staging_trace = client.get("/api/staging/run_live_001/rec_stage_001/traces/conversation.jsonl")
    assert staging_trace.status_code == 200
    assert '"content":"staging"' in staging_trace.text
    staging_trajectory = client.get(
        "/api/staging/run_live_001/rec_stage_001/traces/trajectory.jsonl"
    )
    assert staging_trajectory.status_code == 200
    assert staging_trajectory.text == staging_trace.text

    delete_staging_response = client.delete("/api/staging/run_live_001/rec_stage_001")
    assert delete_staging_response.status_code == 200
    assert delete_staging_response.json() == {
        "status": "deleted",
        "run_id": "run_live_001",
        "record_id": "rec_stage_001",
    }
    assert not (repo.layout.run_staging_dir("run_live_001") / "rec_stage_001").exists()

    delete_response = client.delete("/api/records/rec_001")
    assert delete_response.status_code == 200
    assert delete_response.json() == {"status": "deleted", "record_id": "rec_001"}
    assert not repo.layout.record_dir("rec_001").exists()
    assert not (repo.layout.run_staging_dir("run_001") / "rec_001").exists()

    bootstrap_after_delete = client.get("/api/bootstrap").json()
    assert len(bootstrap_after_delete["workbench_entries"]) == 1
    assert [item["record_id"] for item in bootstrap_after_delete["dataset_entries"]] == [
        "rec_dj_001",
        "rec_bike_001",
    ]
    assert len(bootstrap_after_delete["staging_entries"]) == 0
    assert repo.read_json(repo.layout.dataset_manifest_path()) == {
        "generated": [
            {"name": "dj_dataset_001", "record_id": "rec_dj_001"},
            {"name": "ds_exercise_bikes_0001", "record_id": "rec_bike_001"},
        ]
    }

    missing = client.get("/api/runs/run_missing")
    assert missing.status_code == 404


def test_search_rebuilds_stale_index_when_new_workbench_record_is_added(tmp_path: Path) -> None:
    repo_root = tmp_path
    repo = StorageRepo(repo_root)
    repo.ensure_layout()

    record_store = RecordStore(repo)
    collections = CollectionStore(repo)

    record_store.write_record(
        Record(
            schema_version=1,
            record_id="rec_bottle_001",
            created_at="2026-03-21T15:40:00Z",
            updated_at="2026-03-21T15:40:00Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=None,
            source=SourceRef(run_id="run_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="Swing-top bottle",
                prompt_preview="A realistic swing-top bottle.",
            ),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["workbench"],
        )
    )
    (repo.layout.record_dir("rec_bottle_001") / "prompt.txt").write_text(
        "A realistic swing top bottle with a wire mechanism.",
        encoding="utf-8",
    )
    collections.append_workbench_entry(
        record_id="rec_bottle_001",
        added_at="2026-03-21T15:40:00Z",
    )

    client = TestClient(create_app(repo_root=repo_root))

    initial_search = client.get("/api/records/search?q=bottle&source=workbench").json()
    assert [item["record_id"] for item in initial_search] == ["rec_bottle_001"]

    record_store.write_record(
        Record(
            schema_version=1,
            record_id="rec_bottle_002",
            created_at="2026-03-21T16:50:41Z",
            updated_at="2026-03-21T16:50:41Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=None,
            source=SourceRef(run_id="run_002"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="OCC bottle",
                prompt_preview="A classic occ bottle with articulated cap.",
            ),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["workbench"],
        )
    )
    (repo.layout.record_dir("rec_bottle_002") / "prompt.txt").write_text(
        "A classic occ bottle with articulated cap.",
        encoding="utf-8",
    )
    collections.append_workbench_entry(
        record_id="rec_bottle_002",
        added_at="2026-03-21T16:50:41Z",
    )

    updated_search = client.get("/api/records/search?q=bottle&source=workbench").json()
    assert [item["record_id"] for item in updated_search] == [
        "rec_bottle_002",
        "rec_bottle_001",
    ]


def test_viewer_api_promote_uses_category_slug_not_display_title(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    CategoryStore(repo).save(
        CategoryRecord(
            schema_version=1,
            slug="screwin_light_bulb_with_socket",
            title="Screw-in light bulb with socket",
            current_count=0,
            last_item_index=0,
            run_count=0,
        )
    )
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id="rec_light_001",
            created_at="2026-03-19T14:44:36Z",
            updated_at="2026-03-19T14:44:36Z",
            rating=None,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=None,
            source=SourceRef(run_id="run_light_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="Swivel bulb",
                prompt_preview="A screw-in light bulb with socket articulation.",
            ),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=["workbench"],
        )
    )
    CollectionStore(repo).append_workbench_entry(
        record_id="rec_light_001",
        added_at="2026-03-19T14:44:36Z",
    )
    RunStore(repo).write_run(
        RunRecord(
            schema_version=1,
            run_id="run_light_001",
            run_mode="workbench",
            collection="workbench",
            created_at="2026-03-19T14:44:36Z",
            updated_at="2026-03-19T14:44:36Z",
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
            status="success",
            category_slug=None,
            prompt_count=1,
        )
    )

    client = TestClient(create_app(repo_root=tmp_path))
    promote_response = client.post(
        "/api/records/rec_light_001/promote",
        json={
            "category_slug": "screwin_light_bulb_with_socket",
            "category_title": "Screw-in light bulb with socket",
        },
    )

    assert promote_response.status_code == 200
    assert promote_response.json()["category_slug"] == "screwin_light_bulb_with_socket"
    assert promote_response.json()["dataset_id"] == "ds_screwin_light_bulb_with_socket_0001"

    promoted_record = repo.read_json(repo.layout.record_metadata_path("rec_light_001"))
    assert promoted_record["category_slug"] == "screwin_light_bulb_with_socket"
    assert promoted_record["collections"] == ["dataset"]

    category = repo.read_json(repo.layout.category_metadata_path("screwin_light_bulb_with_socket"))
    assert category["title"] == "Screw-in light bulb with socket"
    assert not repo.layout.category_metadata_path("screw_in_light_bulb_with_socket").exists()


def test_viewer_api_ensures_record_assets_on_demand(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_lazy_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_lazy_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Lazy compiled model",
            prompt_preview="compile this record when the viewer opens it",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    (repo.layout.record_dir("rec_lazy_001") / "prompt.txt").write_text(
        "compile this record when the viewer opens it",
        encoding="utf-8",
    )
    (repo.layout.record_dir("rec_lazy_001") / "model.py").write_text(
        "from __future__ import annotations\n",
        encoding="utf-8",
    )
    repo.write_json(
        repo.layout.record_dir("rec_lazy_001") / "compile_report.json",
        {
            "schema_version": 1,
            "record_id": "rec_lazy_001",
            "status": "draft",
            "urdf_path": "model.urdf",
            "warnings": [],
            "checks_run": [],
            "metrics": {},
        },
    )
    repo.write_json(
        repo.layout.record_dir("rec_lazy_001") / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_lazy_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[Path] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append(script_path)
        assert ignore_geom_qc is False
        assert run_checks is False
        assert target == "visual"
        meshes_dir = script_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text(
            "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='lazy'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            warnings=["warning: lazy compile"],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    client = TestClient(create_app(repo_root=tmp_path))

    urdf_response = client.get("/api/records/rec_lazy_001/files/model.urdf")
    assert urdf_response.status_code == 200
    assert "robot name='lazy'" in urdf_response.text
    assert compile_calls == [repo.layout.record_dir("rec_lazy_001") / "model.py"]

    mesh_response = client.get("/api/records/rec_lazy_001/files/assets/meshes/part.obj")
    assert mesh_response.status_code == 200
    assert "v 1 0 0" in mesh_response.text
    assert compile_calls == [repo.layout.record_dir("rec_lazy_001") / "model.py"]

    compile_report = repo.read_json(
        repo.layout.record_materialization_compile_report_path("rec_lazy_001")
    )
    assert compile_report["status"] == "success"
    assert compile_report["warnings"] == [{"code": "warning", "message": "warning: lazy compile"}]

    persisted_record = repo.read_json(repo.layout.record_metadata_path("rec_lazy_001"))
    assert "derived_assets" not in persisted_record


def test_viewer_api_persists_compiled_urdf_for_nonblocking_geometry_qc(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_qc_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_qc_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="QC warning model",
            prompt_preview="record that compiles with non-blocking geometry QC findings",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_qc_001")
    (record_dir / "prompt.txt").write_text("qc warning record", encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_qc_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[Path] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append(script_path)
        assert ignore_geom_qc is False
        assert run_checks is False
        assert target == "visual"
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='qc'>"
                "<link name='base'>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            warnings=[
                "URDF compile warning (collision, non-blocking): isolated parts detected "
                "(not contacting any other part in the checked pose)."
            ],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    client = TestClient(create_app(repo_root=tmp_path))

    urdf_response = client.get("/api/records/rec_qc_001/files/model.urdf")
    assert urdf_response.status_code == 200
    assert "robot name='qc'" in urdf_response.text
    assert compile_calls == [record_dir / "model.py"]

    compile_report = repo.read_json(
        repo.layout.record_materialization_compile_report_path("rec_qc_001")
    )
    assert compile_report["status"] == "success"
    assert any(
        "URDF compile warning (collision, non-blocking): isolated parts detected" in item["message"]
        for item in compile_report["warnings"]
    )

    persisted_record = repo.read_json(repo.layout.record_metadata_path("rec_qc_001"))
    assert "derived_assets" not in persisted_record


def test_viewer_api_serves_materialized_assets_when_shadow_files_exist(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_shadowed_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_shadowed_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Shadowed assets model",
            prompt_preview="record with both canonical and stray shadow asset paths",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_shadowed_001")
    (record_dir / "prompt.txt").write_text("shadowed assets", encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")
    repo.layout.record_materialization_urdf_path("rec_shadowed_001").parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    repo.layout.record_materialization_urdf_path("rec_shadowed_001").write_text(
        "<robot name='shadowed'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    repo.write_json(
        repo.layout.record_materialization_compile_report_path("rec_shadowed_001"),
        {
            "schema_version": 1,
            "record_id": "rec_shadowed_001",
            "status": "success",
            "urdf_path": "model.urdf",
            "warnings": [],
            "checks_run": ["compile_urdf"],
            "metrics": {},
        },
    )
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_shadowed_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )
    shadow_meshes_dir = record_dir / "meshes"
    shadow_meshes_dir.mkdir(parents=True, exist_ok=True)
    (shadow_meshes_dir / "part.obj").write_text("shadow mesh\n", encoding="utf-8")
    canonical_meshes_dir = repo.layout.record_materialization_asset_meshes_dir("rec_shadowed_001")
    canonical_meshes_dir.mkdir(parents=True, exist_ok=True)
    (canonical_meshes_dir / "part.obj").write_text("canonical mesh\n", encoding="utf-8")

    client = TestClient(create_app(repo_root=tmp_path))

    mesh_response = client.get("/api/records/rec_shadowed_001/files/assets/meshes/part.obj")
    assert mesh_response.status_code == 200
    assert mesh_response.text == "canonical mesh\n"


def test_viewer_api_rebuilds_missing_assets_even_with_successful_urdf(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_missing_assets_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_missing_assets_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Missing assets model",
            prompt_preview="record with urdf but no meshes",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_missing_assets_001")
    (record_dir / "prompt.txt").write_text("record with urdf but no meshes", encoding="utf-8")
    (record_dir / "model.py").write_text(
        "from __future__ import annotations\n",
        encoding="utf-8",
    )
    (record_dir / "model.urdf").write_text("<robot name='stale'/>", encoding="utf-8")
    repo.write_json(
        record_dir / "compile_report.json",
        {
            "schema_version": 1,
            "record_id": "rec_missing_assets_001",
            "status": "success",
            "urdf_path": "model.urdf",
            "warnings": [],
            "checks_run": ["compile_urdf"],
            "metrics": {},
        },
    )
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_missing_assets_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[Path] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append(script_path)
        assert ignore_geom_qc is False
        assert run_checks is False
        assert target == "visual"
        meshes_dir = script_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text(
            "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='rebuilt'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    client = TestClient(create_app(repo_root=tmp_path))

    mesh_response = client.get("/api/records/rec_missing_assets_001/files/assets/meshes/part.obj")
    assert mesh_response.status_code == 200
    assert "v 1 0 0" in mesh_response.text
    assert compile_calls == [record_dir / "model.py"]


def test_viewer_api_rebuilds_missing_assets_for_assets_prefixed_mesh_paths(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_assets_prefix_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_assets_prefix_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Assets-prefixed model",
            prompt_preview="record with assets-prefixed mesh path",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_assets_prefix_001")
    (record_dir / "prompt.txt").write_text(
        "record with assets-prefixed mesh path", encoding="utf-8"
    )
    (record_dir / "model.py").write_text(
        "from __future__ import annotations\n",
        encoding="utf-8",
    )
    (record_dir / "model.urdf").write_text(
        "<robot name='stale'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    repo.write_json(
        record_dir / "compile_report.json",
        {
            "schema_version": 1,
            "record_id": "rec_assets_prefix_001",
            "status": "success",
            "urdf_path": "model.urdf",
            "warnings": [],
            "checks_run": ["compile_urdf"],
            "metrics": {},
        },
    )
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_assets_prefix_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[Path] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append(script_path)
        assert ignore_geom_qc is False
        assert run_checks is False
        assert target == "visual"
        meshes_dir = script_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text(
            "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='rebuilt'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    client = TestClient(create_app(repo_root=tmp_path))

    mesh_response = client.get("/api/records/rec_assets_prefix_001/files/assets/meshes/part.obj")
    assert mesh_response.status_code == 200
    assert "v 1 0 0" in mesh_response.text
    assert compile_calls == [record_dir / "model.py"]


def test_viewer_store_force_materialize_clears_stale_derived_outputs(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_force_assets_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_force_assets_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Force materialize model",
            prompt_preview="record with stale derived assets",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_force_assets_001")
    (record_dir / "prompt.txt").write_text("force assets", encoding="utf-8")
    (record_dir / "model.py").write_text(
        "from __future__ import annotations\n",
        encoding="utf-8",
    )
    (record_dir / "model.urdf").write_text("<robot name='stale'/>", encoding="utf-8")
    repo.write_json(
        record_dir / "compile_report.json",
        {
            "schema_version": 1,
            "record_id": "rec_force_assets_001",
            "status": "success",
            "urdf_path": "model.urdf",
            "warnings": [],
            "checks_run": ["compile_urdf"],
            "metrics": {},
        },
    )

    canonical_meshes_dir = repo.layout.record_asset_meshes_dir("rec_force_assets_001")
    canonical_meshes_dir.mkdir(parents=True, exist_ok=True)
    (canonical_meshes_dir / "stale.obj").write_text("old mesh\n", encoding="utf-8")

    canonical_glb_dir = repo.layout.record_asset_glb_dir("rec_force_assets_001")
    canonical_glb_dir.mkdir(parents=True, exist_ok=True)
    (canonical_glb_dir / "stale.glb").write_bytes(b"old glb")

    canonical_viewer_dir = repo.layout.record_asset_viewer_dir("rec_force_assets_001")
    canonical_viewer_dir.mkdir(parents=True, exist_ok=True)
    (canonical_viewer_dir / "stale.json").write_text('{"stale": true}\n', encoding="utf-8")

    compile_calls: list[Path] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append(script_path)
        assert ignore_geom_qc is False
        assert run_checks is False
        assert target == "full"

        meshes_dir = script_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "fresh.obj").write_text(
            "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )

        glb_dir = script_path.parent / "assets" / "glb"
        glb_dir.mkdir(parents=True, exist_ok=True)
        (glb_dir / "fresh.glb").write_bytes(b"fresh glb")

        viewer_dir = script_path.parent / "assets" / "viewer"
        viewer_dir.mkdir(parents=True, exist_ok=True)
        (viewer_dir / "fresh.json").write_text('{"fresh": true}\n', encoding="utf-8")

        return SimpleNamespace(
            urdf_xml=(
                "<robot name='fresh'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/fresh.obj'/></geometry></visual>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    viewer_store = ViewerStore(tmp_path)
    result = viewer_store.materialize_record_assets("rec_force_assets_001", force=True)

    assert result.compiled is True
    assert compile_calls == [record_dir / "model.py"]
    assert (
        repo.layout.record_materialization_asset_meshes_dir("rec_force_assets_001") / "fresh.obj"
    ).exists()
    assert not (
        repo.layout.record_materialization_asset_meshes_dir("rec_force_assets_001") / "stale.obj"
    ).exists()
    assert (
        repo.layout.record_materialization_asset_glb_dir("rec_force_assets_001") / "fresh.glb"
    ).exists()
    assert not (
        repo.layout.record_materialization_asset_glb_dir("rec_force_assets_001") / "stale.glb"
    ).exists()
    assert (
        repo.layout.record_materialization_asset_viewer_dir("rec_force_assets_001") / "fresh.json"
    ).exists()
    assert not (
        repo.layout.record_materialization_asset_viewer_dir("rec_force_assets_001") / "stale.json"
    ).exists()


def test_viewer_store_visual_materialize_can_be_upgraded_to_full(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_visual_upgrade_001",
        created_at="2026-03-19T10:00:00Z",
        updated_at="2026-03-19T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_visual_upgrade_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Visual then full model",
            prompt_preview="record upgraded from visual-only to full compile",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_visual_upgrade_001")
    (record_dir / "prompt.txt").write_text("upgrade visual compile", encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_visual_upgrade_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[tuple[Path, bool, str]] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append((script_path, run_checks, target))
        meshes_dir = script_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text(
            "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
            encoding="utf-8",
        )
        if target == "visual":
            return SimpleNamespace(
                urdf_xml=(
                    "<robot name='visual'>"
                    "<link name='base'>"
                    "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                    "</link>"
                    "</robot>"
                ),
                warnings=[],
            )
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='full'>"
                "<link name='base'>"
                "<visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual>"
                "<collision><geometry><box size='1 1 1'/></geometry></collision>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    viewer_store = ViewerStore(tmp_path)

    visual_result = viewer_store.materialize_record_assets(
        "rec_visual_upgrade_001",
        force=True,
        target="visual",
    )
    assert visual_result.compiled is True
    compile_report = repo.read_json(
        repo.layout.record_materialization_compile_report_path("rec_visual_upgrade_001")
    )
    assert compile_report["metrics"]["compile_level"] == "visual"
    assert "<collision" not in repo.layout.record_materialization_urdf_path(
        "rec_visual_upgrade_001"
    ).read_text(encoding="utf-8")

    full_result = viewer_store.materialize_record_assets("rec_visual_upgrade_001", target="full")
    assert full_result.compiled is True
    compile_report = repo.read_json(
        repo.layout.record_materialization_compile_report_path("rec_visual_upgrade_001")
    )
    assert compile_report["metrics"]["compile_level"] == "full"
    assert "<collision" in repo.layout.record_materialization_urdf_path(
        "rec_visual_upgrade_001"
    ).read_text(encoding="utf-8")
    assert compile_calls == [
        (record_dir / "model.py", False, "visual"),
        (record_dir / "model.py", False, "full"),
    ]


def test_viewer_store_full_materialize_can_enable_validation(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_full_validate_001",
        created_at="2026-03-20T10:00:00Z",
        updated_at="2026-03-20T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_full_validate_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Validated full model",
            prompt_preview="record compiled with validation enabled",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_full_validate_001")
    (record_dir / "prompt.txt").write_text("validated full compile", encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_full_validate_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    compile_calls: list[tuple[Path, bool, bool, str]] = []

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        compile_calls.append((script_path, ignore_geom_qc, run_checks, target))
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='validated'>"
                "<link name='base'>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual>"
                "<collision><geometry><box size='1 1 1'/></geometry></collision>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fake_compile,
    )

    viewer_store = ViewerStore(tmp_path)

    result = viewer_store.materialize_record_assets(
        "rec_full_validate_001",
        force=True,
        validate=True,
        ignore_geom_qc=False,
        target="full",
    )

    assert result.compiled is True
    assert compile_calls == [
        (record_dir / "model.py", False, True, "full"),
    ]
    compile_report = repo.read_json(
        repo.layout.record_materialization_compile_report_path("rec_full_validate_001")
    )
    assert compile_report["metrics"]["compile_level"] == "full"
    assert compile_report["metrics"]["validation_level"] == "full"


def test_viewer_store_can_skip_nested_compile_timeout(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    record_store = RecordStore(repo)

    record = Record(
        schema_version=1,
        record_id="rec_direct_compile_001",
        created_at="2026-03-20T10:00:00Z",
        updated_at="2026-03-20T10:00:00Z",
        rating=None,
        kind="generated_model",
        prompt_kind="single_prompt",
        category_slug="hinges",
        source=SourceRef(run_id="run_direct_compile_001"),
        sdk_package="sdk",
        provider="openai",
        model_id="gpt-5.4",
        display=DisplayMetadata(
            title="Direct compile model",
            prompt_preview="record compiled without nested timeout wrapper",
        ),
        artifacts=RecordArtifacts(
            prompt_txt="prompt.txt",
            prompt_series_json=None,
            model_py="model.py",
            provenance_json="provenance.json",
            cost_json=None,
        ),
        collections=["workbench"],
    )
    record_store.write_record(record)
    record_dir = repo.layout.record_dir("rec_direct_compile_001")
    (record_dir / "prompt.txt").write_text("direct compile", encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")
    repo.write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 1,
            "record_id": "rec_direct_compile_001",
            "materialization": {
                "fingerprint_inputs": {
                    "model_py_sha256": None,
                    "model_urdf_sha256": None,
                }
            },
        },
    )

    direct_compile_calls: list[tuple[Path, bool, str]] = []

    def fake_direct_compile(
        script_path: Path,
        *,
        sdk_package: str = "sdk",
        ignore_geom_qc: bool = False,
        run_checks: bool = True,
        target: str = "full",
    ) -> SimpleNamespace:
        direct_compile_calls.append((script_path, run_checks, target))
        return SimpleNamespace(
            urdf_xml=(
                "<robot name='direct'>"
                "<link name='base'>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual>"
                "<collision><geometry><box size='1 1 1'/></geometry></collision>"
                "</link>"
                "</robot>"
            ),
            warnings=[],
        )

    def fail_if_timeout_wrapper_called(*args, **kwargs):
        raise AssertionError("timeout wrapper should not be used in direct bulk compile mode")

    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report",
        fake_direct_compile,
    )
    monkeypatch.setattr(
        "agent.compiler.compile_urdf_report_maybe_timeout",
        fail_if_timeout_wrapper_called,
    )

    viewer_store = ViewerStore(tmp_path)
    result = viewer_store.materialize_record_assets(
        "rec_direct_compile_001",
        force=True,
        target="full",
        use_compile_timeout=False,
    )

    assert result.compiled is True
    assert direct_compile_calls == [(record_dir / "model.py", False, "full")]
