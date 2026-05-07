from __future__ import annotations

import errno
import json

import pytest
from rich.console import Console

from scripts.compile_all_records import (
    CompileCandidate,
    _build_concurrency_message,
    _build_multiprocessing_message,
    _build_summary_table,
    _collect_candidates,
    _effective_mem_per_worker_gb,
    _format_bulk_compile_error,
    _is_open_file_limit_error,
    _limit_candidates,
    _resolve_worker_count,
    _should_suppress_bulk_compile_error,
    _sort_candidates_for_compile,
    _spawn_initial_workers,
)
from storage.materialize import (
    build_compile_fingerprint_from_inputs,
    build_compile_fingerprint_inputs,
    build_model_source_snapshot,
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


def test_summary_uses_failure_label() -> None:
    table = _build_summary_table(
        scanned=10,
        skipped_missing_script=1,
        candidates=2,
        compiled=1,
        failed=1,
        failure_label="Failed",
    )
    console = Console(record=True, width=120)
    console.print(table)

    rendered = console.export_text()

    assert "Failed" in rendered


def test_multiprocessing_message_includes_override_and_effective_method(monkeypatch) -> None:
    monkeypatch.setenv("ARTICRAFT_MP_START_METHOD", "spawn")

    message = _build_multiprocessing_message(
        start_method="spawn",
        preload_modules=("agent.compiler", "viewer.api.store"),
    )

    assert "ARTICRAFT_MP_START_METHOD=spawn" in message
    assert "effective start method=spawn" in message


def test_multiprocessing_message_shows_preload_for_forkserver(monkeypatch) -> None:
    monkeypatch.delenv("ARTICRAFT_MP_START_METHOD", raising=False)

    message = _build_multiprocessing_message(
        start_method="forkserver",
        preload_modules=("agent.compiler", "cadquery"),
    )

    assert "ARTICRAFT_MP_START_METHOD=<unset>" in message
    assert "forkserver preload=[agent.compiler, cadquery]" in message


def test_multiprocessing_message_renders_literal_brackets() -> None:
    console = Console(record=True, width=160)
    console.print(
        _build_multiprocessing_message(
            start_method="fork",
            preload_modules=("agent.compiler", "viewer.api.store", "cadquery"),
        ),
        markup=False,
    )

    rendered = console.export_text()

    assert "parent preload=[agent.compiler, viewer.api.store, cadquery]" in rendered


def test_effective_mem_per_worker_uses_lower_visual_default() -> None:
    assert _effective_mem_per_worker_gb(target="visual", requested_mem_per_worker_gb=3.0) == 1.5


def test_effective_mem_per_worker_preserves_explicit_override_for_visual() -> None:
    assert _effective_mem_per_worker_gb(target="visual", requested_mem_per_worker_gb=2.25) == 2.25


def test_effective_mem_per_worker_preserves_full_default() -> None:
    assert _effective_mem_per_worker_gb(target="full", requested_mem_per_worker_gb=3.0) == 3.0


def test_build_concurrency_message_for_visual_auto_shows_throughput_mode(monkeypatch) -> None:
    monkeypatch.setattr("scripts.compile_all_records._logical_cpu_count", lambda: 14)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    message = _build_concurrency_message(
        requested="auto",
        resolved_workers=264,
        target="visual",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=1.5,
    )

    assert "Using 264 workers" in message
    assert "target=visual" in message
    assert "queued records=264" in message
    assert "auto mode=throughput-first" in message
    assert "hard cap 512" in message
    assert "budget" not in message


def test_build_concurrency_message_for_full_auto_shows_throughput_mode(monkeypatch) -> None:
    monkeypatch.setattr("scripts.compile_all_records._logical_cpu_count", lambda: 14)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    message = _build_concurrency_message(
        requested="auto",
        resolved_workers=264,
        target="full",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=3.0,
    )

    assert "Using 264 workers" in message
    assert "target=full" in message
    assert "queued records=264" in message
    assert "auto mode=throughput-first" in message
    assert "hard cap 512" in message
    assert "budget" not in message


def test_build_concurrency_message_for_max_shows_explicit_fanout(monkeypatch) -> None:
    monkeypatch.setattr("scripts.compile_all_records._logical_cpu_count", lambda: 14)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    message = _build_concurrency_message(
        requested="max",
        resolved_workers=20,
        target="full",
        candidate_count=20,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=3.0,
    )

    assert "Using 20 workers" in message
    assert "explicit max fan-out" in message
    assert "budget" not in message


def test_build_concurrency_message_includes_open_file_cap(monkeypatch) -> None:
    from scripts.compile_all_records import OpenFileWorkerCap

    monkeypatch.setattr("scripts.compile_all_records._logical_cpu_count", lambda: 14)
    monkeypatch.setattr(
        "scripts.compile_all_records._open_file_worker_cap",
        lambda: OpenFileWorkerCap(
            worker_cap=23,
            soft_limit=256,
            open_files=18,
            reserve_files=64,
            per_worker_budget=8,
        ),
    )

    message = _build_concurrency_message(
        requested="auto",
        resolved_workers=23,
        target="visual",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=1.5,
    )

    assert "open files soft limit 256" in message
    assert "open now 18" in message
    assert "fd cap 23" in message


def test_is_open_file_limit_error_matches_emfile_and_enfile() -> None:
    assert _is_open_file_limit_error(OSError(errno.EMFILE, "Too many open files"))
    assert _is_open_file_limit_error(OSError(errno.ENFILE, "File table overflow"))
    assert not _is_open_file_limit_error(OSError(errno.EINVAL, "Invalid argument"))


def test_sort_candidates_for_compile_prefers_heavier_records() -> None:
    candidates = [
        CompileCandidate(
            "rec_light", "missing model.urdf", estimated_mesh_bytes=10, mesh_file_count=1
        ),
        CompileCandidate(
            "rec_heavy", "missing model.urdf", estimated_mesh_bytes=100, mesh_file_count=2
        ),
        CompileCandidate(
            "rec_medium", "missing model.urdf", estimated_mesh_bytes=50, mesh_file_count=4
        ),
    ]

    ordered = _sort_candidates_for_compile(candidates)

    assert [candidate.record_id for candidate in ordered] == [
        "rec_heavy",
        "rec_medium",
        "rec_light",
    ]


def test_limit_candidates_applies_after_heavy_first_sort() -> None:
    candidates = [
        CompileCandidate(
            "rec_light", "missing model.urdf", estimated_mesh_bytes=10, mesh_file_count=1
        ),
        CompileCandidate(
            "rec_heavy", "missing model.urdf", estimated_mesh_bytes=100, mesh_file_count=2
        ),
        CompileCandidate(
            "rec_medium", "missing model.urdf", estimated_mesh_bytes=50, mesh_file_count=4
        ),
    ]

    limited = _limit_candidates(candidates, limit=2)

    assert [candidate.record_id for candidate in limited] == [
        "rec_heavy",
        "rec_medium",
    ]


def test_resolve_worker_count_auto_for_visual_uses_throughput_first_fanout(monkeypatch) -> None:
    monkeypatch.setattr(
        "scripts.compile_all_records._logical_cpu_count",
        lambda: 12,
    )
    monkeypatch.setattr("scripts.compile_all_records._auto_worker_memory_cap", lambda **_: None)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    resolved = _resolve_worker_count(
        "auto",
        candidate_count=264,
        reserve_mem_gb=1.0,
        mem_per_worker_gb=2.0,
        target="visual",
    )

    assert resolved == 264


def test_resolve_worker_count_max_means_maximum_fanout(monkeypatch) -> None:
    monkeypatch.setattr(
        "scripts.compile_all_records._logical_cpu_count",
        lambda: 12,
    )
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    resolved = _resolve_worker_count(
        "max",
        candidate_count=20,
        reserve_mem_gb=1.0,
        mem_per_worker_gb=2.0,
        target="visual",
    )

    assert resolved == 20


def test_resolve_worker_count_auto_for_full_uses_throughput_first_fanout(monkeypatch) -> None:
    monkeypatch.setattr("scripts.compile_all_records._auto_worker_memory_cap", lambda **_: None)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    resolved = _resolve_worker_count(
        "auto",
        candidate_count=20,
        reserve_mem_gb=1.0,
        mem_per_worker_gb=2.0,
        target="full",
    )

    assert resolved == 20


def test_resolve_worker_count_auto_for_full_uses_open_file_cap(monkeypatch) -> None:
    from scripts.compile_all_records import OpenFileWorkerCap

    monkeypatch.setattr("scripts.compile_all_records._auto_worker_memory_cap", lambda **_: None)
    monkeypatch.setattr(
        "scripts.compile_all_records._open_file_worker_cap",
        lambda: OpenFileWorkerCap(
            worker_cap=23,
            soft_limit=256,
            open_files=18,
            reserve_files=64,
            per_worker_budget=8,
        ),
    )

    resolved = _resolve_worker_count(
        "auto",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=2.0,
        target="full",
    )

    assert resolved == 23


def test_resolve_worker_count_auto_respects_memory_cap(monkeypatch) -> None:
    monkeypatch.setattr("scripts.compile_all_records._auto_worker_memory_cap", lambda **_: 7)
    monkeypatch.setattr("scripts.compile_all_records._open_file_worker_cap", lambda: None)

    resolved = _resolve_worker_count(
        "auto",
        candidate_count=264,
        reserve_mem_gb=1.0,
        mem_per_worker_gb=2.0,
        target="full",
    )

    assert resolved == 7


def test_resolve_worker_count_visual_auto_is_clamped_by_open_file_limit(monkeypatch) -> None:
    from scripts.compile_all_records import OpenFileWorkerCap

    monkeypatch.setattr(
        "scripts.compile_all_records._logical_cpu_count",
        lambda: 12,
    )
    monkeypatch.setattr("scripts.compile_all_records._auto_worker_memory_cap", lambda **_: None)
    monkeypatch.setattr(
        "scripts.compile_all_records._open_file_worker_cap",
        lambda: OpenFileWorkerCap(
            worker_cap=23,
            soft_limit=256,
            open_files=18,
            reserve_files=64,
            per_worker_budget=8,
        ),
    )

    resolved = _resolve_worker_count(
        "auto",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=1.5,
        target="visual",
    )

    assert resolved == 23


def test_resolve_worker_count_explicit_numeric_is_clamped_by_open_file_limit(monkeypatch) -> None:
    from scripts.compile_all_records import OpenFileWorkerCap

    monkeypatch.setattr(
        "scripts.compile_all_records._open_file_worker_cap",
        lambda: OpenFileWorkerCap(
            worker_cap=23,
            soft_limit=256,
            open_files=18,
            reserve_files=64,
            per_worker_budget=8,
        ),
    )

    resolved = _resolve_worker_count(
        "264",
        candidate_count=264,
        reserve_mem_gb=2.0,
        mem_per_worker_gb=1.5,
        target="visual",
    )

    assert resolved == 23


def test_spawn_initial_workers_backs_off_after_open_file_limit(monkeypatch, tmp_path) -> None:
    calls = {"count": 0}

    def fake_spawn_worker(*args, **kwargs):
        calls["count"] += 1
        if calls["count"] == 1:
            return object()
        raise OSError(errno.EMFILE, "Too many open files")

    monkeypatch.setattr("scripts.compile_all_records._spawn_worker", fake_spawn_worker)

    workers, warning = _spawn_initial_workers(
        object(),  # type: ignore[arg-type]
        requested_worker_count=5,
        repo_root=tmp_path,
        strict=False,
        target="visual",
        result_queue=object(),
    )

    assert len(workers) == 1
    assert warning is not None
    assert "continuing with 1 active workers instead of the requested 5" in warning


def test_spawn_initial_workers_raises_if_no_worker_can_start(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(
        "scripts.compile_all_records._spawn_worker",
        lambda *args, **kwargs: (_ for _ in ()).throw(OSError(errno.EMFILE, "Too many open files")),
    )

    with pytest.raises(
        RuntimeError, match="Open-file limit reached before any bulk compile worker"
    ):
        _spawn_initial_workers(
            object(),  # type: ignore[arg-type]
            requested_worker_count=5,
            repo_root=tmp_path,
            strict=False,
            target="visual",
            result_queue=object(),
        )


def test_collect_candidates_skips_primitive_only_success_records_without_assets(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_primitive"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=record_dir / "model.py")
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "full",
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                },
            }
        ),
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
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    (materialization_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps({"status": "success"}),
        encoding="utf-8",
    )

    candidates, _ = _collect_candidates(tmp_path, force=False)

    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "missing generated assets"


def test_collect_candidates_trusts_current_available_compile_report_by_default(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_cached_available"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    model_path = record_dir / "model.py"
    model_path.write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=model_path)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "visual",
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                    "materialization_status": "available",
                    **build_model_source_snapshot(model_path=model_path),
                },
            }
        ),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(
        tmp_path,
        force=False,
        target="visual",
    )

    assert skipped_missing_script == 0
    assert candidates == []


def test_collect_candidates_verify_assets_detects_stale_available_compile_report(
    tmp_path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_stale_available"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    model_path = record_dir / "model.py"
    model_path.write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=model_path)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='mesh'><link name='base'><visual><geometry><mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "visual",
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                    "materialization_status": "available",
                    **build_model_source_snapshot(model_path=model_path),
                },
            }
        ),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(
        tmp_path,
        force=False,
        target="visual",
        verify_assets=True,
    )

    assert skipped_missing_script == 0
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


def test_collect_candidates_uses_record_artifacts_model_py_path(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_custom_model_path"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "generated_model.py").write_text("object_model = None\n", encoding="utf-8")
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "generated_model.py"}}),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(tmp_path, force=False)

    assert skipped_missing_script == 0
    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "missing model.urdf"


def test_collect_candidates_visual_target_skips_legacy_full_compile_reports(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_visual_skip"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=record_dir / "model.py")
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                },
            }
        ),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(
        tmp_path,
        force=False,
        target="visual",
    )

    assert skipped_missing_script == 0
    assert candidates == []


def test_collect_candidates_full_target_queues_visual_only_reports_for_upgrade(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_visual_upgrade"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "model.py").write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=record_dir / "model.py")
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "visual",
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                },
            }
        ),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(
        tmp_path,
        force=False,
        target="full",
    )

    assert skipped_missing_script == 0
    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "compile level is visual"


def test_collect_candidates_queues_records_when_compile_inputs_change(tmp_path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_changed_inputs"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    model_path = record_dir / "model.py"
    model_path.write_text("object_model = None\n", encoding="utf-8")
    old_fingerprint_inputs = build_compile_fingerprint_inputs(model_path=model_path)
    model_path.write_text("UPDATED = True\nobject_model = None\n", encoding="utf-8")
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "full",
                    "fingerprint_inputs": old_fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        old_fingerprint_inputs
                    ),
                },
            }
        ),
        encoding="utf-8",
    )

    candidates, skipped_missing_script = _collect_candidates(tmp_path, force=False, target="full")

    assert skipped_missing_script == 0
    assert len(candidates) == 1
    assert candidates[0].record_id == record_id
    assert candidates[0].reason == "compile inputs changed"


def test_collect_candidates_uses_cached_model_source_snapshot_to_skip_rehash(
    tmp_path, monkeypatch
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()

    record_id = "rec_snapshot_fast_path"
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    materialization_dir = repo.layout.record_materialization_dir(record_id)
    materialization_dir.mkdir(parents=True, exist_ok=True)
    model_path = record_dir / "model.py"
    model_path.write_text("object_model = None\n", encoding="utf-8")
    fingerprint_inputs = build_compile_fingerprint_inputs(model_path=model_path)
    (materialization_dir / "model.urdf").write_text(
        "<robot name='primitive'><link name='base'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"artifacts": {"model_py": "model.py"}}),
        encoding="utf-8",
    )
    (materialization_dir / "compile_report.json").write_text(
        json.dumps(
            {
                "status": "success",
                "metrics": {
                    "compile_level": "full",
                    "fingerprint_inputs": fingerprint_inputs,
                    "materialization_fingerprint": build_compile_fingerprint_from_inputs(
                        fingerprint_inputs
                    ),
                    **build_model_source_snapshot(model_path=model_path),
                },
            }
        ),
        encoding="utf-8",
    )

    def _unexpected_hash(**kwargs):
        raise AssertionError(
            "build_compile_fingerprint_inputs should not run for unchanged sources"
        )

    monkeypatch.setattr(
        "scripts.compile_all_records.build_compile_fingerprint_inputs", _unexpected_hash
    )

    candidates, skipped_missing_script = _collect_candidates(tmp_path, force=False, target="full")

    assert skipped_missing_script == 0
    assert candidates == []
