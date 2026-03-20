from __future__ import annotations

import json

from rich.console import Console

from scripts.compile_all_records import (
    _build_summary_table,
    _collect_candidates,
    _format_bulk_compile_error,
    _should_suppress_bulk_compile_error,
)
from storage.repo import StorageRepo


def test_non_strict_bulk_compile_relabels_geometry_qc_failures() -> None:
    error = (
        "Failed to compile assets for rec_barrier_gate_0001: RuntimeError: RuntimeError: "
        "URDF compile failure (collision, blocking): isolated parts detected "
        "(not contacting any other part in the checked pose).\n"
        "- part='base_housing' nearest_part='gate_arm' approx_gap=0.004m pose_index=0 "
        "pose=(arm_hinge=0) backend=fcl"
    )

    formatted = _format_bulk_compile_error(
        "rec_barrier_gate_0001",
        error,
        strict=False,
    )

    assert formatted.startswith("Geometry QC issue (collision, non-blocking in compile-all):")
    assert "isolated parts detected" in formatted
    assert "URDF compile failure" not in formatted
    assert "(collision, blocking)" not in formatted


def test_strict_bulk_compile_preserves_geometry_failure_wording() -> None:
    error = (
        "Failed to compile assets for rec_barrier_gate_0001: RuntimeError: RuntimeError: "
        "URDF compile failure (collision, blocking): isolated parts detected "
        "(not contacting any other part in the checked pose)."
    )

    formatted = _format_bulk_compile_error(
        "rec_barrier_gate_0001",
        error,
        strict=True,
    )

    assert formatted.startswith("URDF compile failure (collision, blocking):")


def test_non_strict_bulk_compile_suppresses_geometry_qc_detail_rows() -> None:
    error = (
        "Failed to compile assets for rec_barrier_gate_0001: RuntimeError: RuntimeError: "
        "URDF compile failure (collision, blocking): isolated parts detected "
        "(not contacting any other part in the checked pose)."
    )

    assert _should_suppress_bulk_compile_error(
        "rec_barrier_gate_0001",
        error,
        strict=False,
    )


def test_strict_bulk_compile_keeps_geometry_qc_detail_rows() -> None:
    error = (
        "Failed to compile assets for rec_barrier_gate_0001: RuntimeError: RuntimeError: "
        "URDF compile failure (collision, blocking): isolated parts detected "
        "(not contacting any other part in the checked pose)."
    )

    assert not _should_suppress_bulk_compile_error(
        "rec_barrier_gate_0001",
        error,
        strict=True,
    )


def test_non_strict_summary_uses_issue_label() -> None:
    table = _build_summary_table(
        scanned=10,
        skipped_missing_script=1,
        candidates=2,
        compiled=1,
        failed=1,
        failure_label="Non-blocking issues",
    )
    console = Console(record=True, width=120)
    console.print(table)

    rendered = console.export_text()

    assert "Non-blocking issues" in rendered


def test_collect_candidates_skips_primitive_only_success_records_without_assets(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_primitive"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    (record_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps(
            {
                "artifacts": {
                    "model_py": "model.py",
                    "model_urdf": "model.urdf",
                    "compile_report_json": "compile_report.json",
                }
            }
        ),
        encoding="utf-8",
    )
    (record_dir / "compile_report.json").write_text(
        json.dumps({"status": "success"}),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(tmp_path, force=False)

    assert skipped_missing_script == 0
    assert candidates == []


def test_collect_candidates_queues_mesh_backed_records_missing_assets(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_mesh_missing_assets"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    (record_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps(
            {
                "artifacts": {
                    "model_py": "model.py",
                    "model_urdf": "model.urdf",
                    "compile_report_json": "compile_report.json",
                }
            }
        ),
        encoding="utf-8",
    )
    (record_dir / "compile_report.json").write_text(
        json.dumps({"status": "success"}),
        encoding="utf-8",
    )

    candidates, _ = _collect_candidates(tmp_path, force=False)

    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "missing generated assets"


def test_collect_candidates_queues_broken_success_records_missing_model_script(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_missing_model"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "record.json").write_text(
        json.dumps(
            {
                "artifacts": {
                    "model_py": "model.py",
                    "model_urdf": "model.urdf",
                    "compile_report_json": "compile_report.json",
                }
            }
        ),
        encoding="utf-8",
    )
    (record_dir / "compile_report.json").write_text(
        json.dumps({"status": "success"}),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(tmp_path, force=False)

    assert skipped_missing_script == 0
    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "missing model.py"
