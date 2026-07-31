from __future__ import annotations

import pytest

from storage.materialize import (
    build_materialization_summary,
    build_model_source_snapshot,
    compile_report_matches_model_source_snapshot,
    ensure_record_artifacts_exist,
    infer_materialization_status,
)
from storage.repo import StorageRepo


def test_infer_materialization_status_marks_primitive_only_success_as_available(
    tmp_path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_primitive"
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )

    status = infer_materialization_status(repo, record_id)

    assert status == "available"


def test_infer_materialization_status_keeps_mesh_backed_urdf_missing_without_assets(
    tmp_path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_mesh"
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/base.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )

    status = infer_materialization_status(repo, record_id)

    assert status == "missing"


def test_ensure_record_artifacts_exist_raises_for_missing_canonical_files(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_broken"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "record.json").write_text(
        '{"artifacts":{"model_py":"model.py","provenance_json":"provenance.json"}}',
        encoding="utf-8",
    )
    (record_dir / "provenance.json").write_text("{}", encoding="utf-8")

    with pytest.raises(FileNotFoundError, match="missing canonical artifact"):
        ensure_record_artifacts_exist(
            repo,
            record_id,
            required=("model_py", "provenance_json"),
        )


def test_compile_report_matches_model_source_snapshot_round_trips(tmp_path) -> None:
    model_path = tmp_path / "model.py"
    model_path.write_text("object_model = None\n", encoding="utf-8")

    report = {"metrics": build_model_source_snapshot(model_path=model_path)}

    assert compile_report_matches_model_source_snapshot(report, model_path=model_path) is True

    model_path.write_text("UPDATED = True\nobject_model = None\n", encoding="utf-8")

    assert compile_report_matches_model_source_snapshot(report, model_path=model_path) is False


def test_build_materialization_summary_reports_visual_mesh_footprint(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_summary"
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    meshes_dir = repo.layout.record_materialization_asset_meshes_dir(record_id)
    collision_dir = meshes_dir / "collision"
    collision_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir.mkdir(parents=True, exist_ok=True)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/base.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (meshes_dir / "base.obj").write_bytes(b"v 0 0 0\n")
    (collision_dir / "skip.obj").write_bytes(b"v 1 1 1\n")

    summary = build_materialization_summary(repo, record_id)

    assert summary["materialization_status"] == "available"
    assert summary["visual_mesh_file_count"] == 1
    assert summary["visual_mesh_bytes"] == len("v 0 0 0\n".encode("utf-8"))
