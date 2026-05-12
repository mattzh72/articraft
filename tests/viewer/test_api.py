from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from storage import dataset_workflow
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
from viewer.api.app import create_app
from viewer.api.store import _effective_rating, _within_rating_filter


def _patch_dataset_tokens(monkeypatch: pytest.MonkeyPatch, *tokens: str) -> None:
    iterator = iter(tokens)
    monkeypatch.setattr(dataset_workflow, "new_dataset_token", lambda: next(iterator))


def _write_record(
    repo: StorageRepo,
    *,
    record_id: str,
    title: str,
    prompt: str,
    category_slug: str | None = "hinges",
    collections: list[str] | None = None,
    rating: int | None = None,
    source_run_id: str = "run_001",
) -> None:
    record_dir = repo.layout.record_dir(record_id)
    RecordStore(repo).write_record(
        Record(
            schema_version=1,
            record_id=record_id,
            created_at="2026-03-18T08:00:00Z",
            updated_at="2026-03-18T08:00:00Z",
            rating=rating,
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug=category_slug,
            source=SourceRef(run_id=source_run_id),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(title=title, prompt_preview=prompt),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                provenance_json="provenance.json",
                cost_json=None,
            ),
            collections=collections or ["workbench"],
        )
    )
    (record_dir / "prompt.txt").write_text(prompt, encoding="utf-8")
    (record_dir / "model.py").write_text("from __future__ import annotations\n", encoding="utf-8")


def test_effective_rating_filter_uses_bucket_ranges() -> None:
    assert _effective_rating(None, None) is None
    assert _effective_rating(4, None) == 4.0
    assert _effective_rating(5, 4) == 4.5
    assert _within_rating_filter(4.5, ["4"]) is True
    assert _within_rating_filter(4.5, ["5"]) is False


def test_viewer_api_smoke_bootstrap_browse_search_and_assets(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    _write_record(
        repo,
        record_id="rec_hinge_001",
        title="Test hinge model",
        prompt="create a hinge with two linked parts",
        category_slug="hinges",
        collections=["workbench"],
    )
    CollectionStore(repo).append_workbench_entry(
        record_id="rec_hinge_001",
        added_at="2026-03-18T08:01:00Z",
    )
    _write_record(
        repo,
        record_id="rec_latch_001",
        title="Cabinet latch",
        prompt="create a cabinet latch with a catch plate",
        category_slug="latches",
        collections=["workbench"],
    )
    CollectionStore(repo).append_workbench_entry(
        record_id="rec_latch_001",
        added_at="2026-03-18T08:02:00Z",
    )
    repo.write_json(
        repo.layout.record_materialization_compile_report_path("rec_hinge_001"),
        {
            "schema_version": 1,
            "record_id": "rec_hinge_001",
            "status": "success",
            "metrics": {},
        },
    )
    asset_meshes_dir = repo.layout.record_materialization_asset_meshes_dir("rec_hinge_001")
    asset_meshes_dir.mkdir(parents=True, exist_ok=True)
    (asset_meshes_dir / "part.obj").write_text(
        "o tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )

    _write_record(
        repo,
        record_id="rec_dj_001",
        title="DJ equipment",
        prompt="A realistic DJ equipment setup with mixer and faders.",
        category_slug="dj_equipment",
        collections=["dataset"],
    )
    DatasetStore(repo).promote_record(
        record_id="rec_dj_001",
        dataset_id="dj_dataset_001",
        promoted_at="2026-03-18T08:02:00Z",
        category_slug="dj_equipment",
    )
    _write_record(
        repo,
        record_id="rec_knob_001",
        title="Knurled knob",
        prompt="A knurled control knob with a pointer indicator.",
        category_slug="knobs",
        collections=["dataset"],
    )
    DatasetStore(repo).promote_record(
        record_id="rec_knob_001",
        dataset_id="knob_dataset_001",
        promoted_at="2026-03-18T08:03:00Z",
        category_slug="knobs",
    )

    client = TestClient(create_app(repo_root=tmp_path))

    assert client.get("/health").json()["status"] == "ok"

    bootstrap = client.get("/api/bootstrap").json()
    assert {entry["record_id"] for entry in bootstrap["workbench_entries"]} == {
        "rec_hinge_001",
        "rec_latch_001",
    }
    assert {entry["dataset_id"] for entry in bootstrap["dataset_entries"]} == {
        "dj_dataset_001",
        "knob_dataset_001",
    }

    dataset_browse = client.get("/api/records/browse?source=dataset&limit=1").json()
    assert dataset_browse["total"] == 2
    assert len(dataset_browse["record_ids"]) == 1
    assert set(dataset_browse["facets"]["categories"]) == {"dj_equipment", "knobs"}

    dataset_browse_ids = client.get("/api/records/browse/ids?source=dataset").json()
    assert dataset_browse_ids["total"] == 2
    assert set(dataset_browse_ids["record_ids"]) == {"rec_dj_001", "rec_knob_001"}

    workbench_browse = client.get("/api/records/browse?source=workbench&limit=1").json()
    assert workbench_browse["total"] == 2
    assert len(workbench_browse["record_ids"]) == 1

    workbench_browse_ids = client.get("/api/records/browse/ids?source=workbench").json()
    assert workbench_browse_ids["total"] == 2
    assert set(workbench_browse_ids["record_ids"]) == {"rec_hinge_001", "rec_latch_001"}

    search_results = client.get("/api/records/search?q=hinge&source=workbench").json()
    assert [item["record_id"] for item in search_results] == ["rec_hinge_001"]

    mesh_file = client.get("/api/records/rec_hinge_001/files/assets/meshes/part.obj")
    assert mesh_file.status_code == 200
    assert "v 1 0 0" in mesh_file.text


def test_viewer_api_promote_uses_category_slug_not_display_title(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_dataset_tokens(monkeypatch, "0001")
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
    _write_record(
        repo,
        record_id="rec_light_001",
        title="Swivel bulb",
        prompt="A screw-in light bulb with socket articulation.",
        category_slug=None,
        collections=["workbench"],
        source_run_id="run_light_001",
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
    assert not repo.layout.category_metadata_path("screw_in_light_bulb_with_socket").exists()


def test_viewer_api_ensures_record_assets_on_demand(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_record(
        repo,
        record_id="rec_lazy_001",
        title="Lazy compiled model",
        prompt="compile this record when the viewer opens it",
        category_slug="hinges",
        source_run_id="run_lazy_001",
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

    monkeypatch.setattr("agent.compiler.compile_urdf_report", fake_compile)
    monkeypatch.setattr("agent.compiler.compile_urdf_report_maybe_timeout", fake_compile)

    client = TestClient(create_app(repo_root=tmp_path))

    urdf_response = client.get("/api/records/rec_lazy_001/files/model.urdf")
    assert urdf_response.status_code == 200
    assert "robot name='lazy'" in urdf_response.text
    assert compile_calls == [repo.layout.record_dir("rec_lazy_001") / "model.py"]

    mesh_response = client.get("/api/records/rec_lazy_001/files/assets/meshes/part.obj")
    assert mesh_response.status_code == 200
    assert "v 1 0 0" in mesh_response.text
    assert compile_calls == [repo.layout.record_dir("rec_lazy_001") / "model.py"]
