from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from agent import batch_runner
from storage.datasets import DatasetStore
from storage.repo import StorageRepo


def _make_row() -> batch_runner.BatchRowSpec:
    return batch_runner.BatchRowSpec(
        csv_row_number=1,
        prompt_index=1,
        row_id="row_0001",
        category_slug="hinge",
        category_title="Hinge",
        prompt="make a hinge",
        provider="openai",
        model_id="gpt-5.4",
        thinking_level="high",
        max_turns=12,
        max_cost_usd=None,
        sdk_package="sdk",
        scaffold_mode="lite",
        post_success_design_audit=True,
        label=None,
    )


def _make_allocation() -> batch_runner.BatchRowAllocation:
    return batch_runner.BatchRowAllocation(
        row_id="row_0001",
        category_slug="hinge",
        dataset_id="ds_hinge_0001",
        record_id="rec_hinge_0001",
        prompt_index=1,
    )


def _make_config(tmp_path: Path) -> batch_runner.BatchRunConfig:
    row = _make_row()
    allocation = _make_allocation()
    return batch_runner.BatchRunConfig(
        repo_root=tmp_path,
        spec_path=tmp_path / "data" / "batch_specs" / "hinge.csv",
        batch_spec_id="hinge",
        run_id="run_123",
        rows=[row],
        allocations={row.row_id: allocation},
        concurrency=1,
        local_work_concurrency=1,
        system_prompt_path="designer_system_prompt.txt",
        scaffold_mode="lite",
        sdk_docs_mode="full",
        max_cost_usd=None,
        resume=False,
        resume_policy="failed_or_pending",
        keep_awake=False,
        pause_file=tmp_path / ".paused",
        pause_poll_seconds=1.0,
        keyboard_pause_enabled=False,
        qc_blurb_text=None,
        post_success_design_audit=True,
    )


@pytest.mark.parametrize(
    ("resume", "latest_status", "resume_policy", "has_persisted_success", "action", "final_status"),
    [
        (False, "pending", "failed_or_pending", False, "run", "pending"),
        (True, "failed", "failed_only", False, "run", "failed"),
        (True, "pending", "failed_only", False, "skip", "pending"),
        (True, "success", "all", False, "run", "success"),
        (True, "running", "failed_or_pending", False, "run", "running"),
        (True, "failed", "failed_or_pending", True, "reuse_success", "success"),
    ],
)
def test_resume_decision_table(
    resume: bool,
    latest_status: batch_runner.BatchRowStatus,
    resume_policy: batch_runner.ResumePolicy,
    has_persisted_success: bool,
    action: batch_runner.ResumeAction,
    final_status: batch_runner.BatchRowStatus,
) -> None:
    decision = batch_runner._resume_decision(
        resume=resume,
        latest_status=latest_status,
        resume_policy=resume_policy,
        has_persisted_success=has_persisted_success,
    )

    assert decision.action == action
    assert decision.latest_status == final_status
    assert decision.should_queue() is (action == "run")


def test_resume_signature_and_mismatch_field() -> None:
    row = _make_row()
    existing = {
        "category_slug": "hinge",
        "prompt": "make a hinge",
        "provider": "openai",
        "model_id": "gpt-5.4",
        "thinking_level": "high",
        "max_turns": 12,
        "max_cost_usd": None,
        "sdk_package": "sdk",
        "scaffold_mode": "lite",
        "post_success_design_audit": True,
    }

    assert row.resume_signature() == (
        "hinge",
        "make a hinge",
        "openai",
        "gpt-5.4",
        "high",
        12,
        None,
        "sdk",
        "lite",
        True,
    )
    assert batch_runner._resume_signature_mismatch_field(existing, row) is None

    existing["model_id"] = "gpt-5.3"
    assert batch_runner._resume_signature_mismatch_field(existing, row) == "model_id"

    existing["model_id"] = "gpt-5.4"
    existing["max_cost_usd"] = 1.25
    assert batch_runner._resume_signature_mismatch_field(existing, row) == "max_cost_usd"


def test_resume_signature_mismatch_field_preserves_false_bool_normalization() -> None:
    row = replace(_make_row(), post_success_design_audit=False)
    existing = {
        "category_slug": "hinge",
        "prompt": "make a hinge",
        "provider": "openai",
        "model_id": "gpt-5.4",
        "thinking_level": "high",
        "max_turns": 12,
        "max_cost_usd": None,
        "sdk_package": "sdk",
        "scaffold_mode": "lite",
        "post_success_design_audit": False,
    }

    assert batch_runner._resume_signature_mismatch_field(existing, row) is None


def test_batch_result_record_serialization_preserves_running_and_terminal_shapes(
    tmp_path: Path,
) -> None:
    config = _make_config(tmp_path)
    row = _make_row()
    allocation = _make_allocation()

    running_payload = batch_runner._build_batch_result_record(
        config=config,
        row=row,
        allocation=allocation,
        status="running",
        message=None,
        staging_dir=tmp_path / "data" / "runs" / "run_123" / "staging" / "rec_hinge_0001",
    ).to_dict()
    assert running_payload == {
        "row_id": "row_0001",
        "record_id": "rec_hinge_0001",
        "dataset_id": "ds_hinge_0001",
        "category_slug": "hinge",
        "prompt_index": 1,
        "status": "running",
        "message": None,
        "staging_dir": "data/runs/run_123/staging/rec_hinge_0001",
    }

    success_payload = batch_runner._build_batch_result_record(
        config=config,
        row=row,
        allocation=allocation,
        status="success",
        message=None,
        staging_dir=tmp_path / "data" / "runs" / "run_123" / "staging" / "rec_hinge_0001",
        turn_count=3,
        tool_call_count=4,
        compile_attempt_count=1,
        record_dir=tmp_path / "data" / "records" / "rec_hinge_0001",
    ).to_dict()
    assert success_payload == {
        "row_id": "row_0001",
        "record_id": "rec_hinge_0001",
        "dataset_id": "ds_hinge_0001",
        "category_slug": "hinge",
        "prompt_index": 1,
        "status": "success",
        "message": None,
        "staging_dir": "data/runs/run_123/staging/rec_hinge_0001",
        "turn_count": 3,
        "tool_call_count": 4,
        "compile_attempt_count": 1,
        "record_dir": "data/records/rec_hinge_0001",
    }

    failed_payload = batch_runner._build_batch_result_record(
        config=config,
        row=row,
        allocation=allocation,
        status="failed",
        message="synthetic failure",
        staging_dir=tmp_path / "data" / "runs" / "run_123" / "staging" / "rec_hinge_0001",
    ).to_dict()
    assert failed_payload == {
        "row_id": "row_0001",
        "record_id": "rec_hinge_0001",
        "dataset_id": "ds_hinge_0001",
        "category_slug": "hinge",
        "prompt_index": 1,
        "status": "failed",
        "message": "synthetic failure",
        "staging_dir": "data/runs/run_123/staging/rec_hinge_0001",
        "turn_count": None,
        "tool_call_count": None,
        "compile_attempt_count": None,
        "record_dir": None,
    }


def test_write_row_state_serializes_attempts(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    row = _make_row()
    allocation = _make_allocation()
    attempt = batch_runner.BatchAttemptRecord(
        timestamp="2026-03-26T12:00:00Z",
        provider="openai",
        model_id="gpt-5.4",
        thinking_level="high",
        max_turns=12,
        max_cost_usd=None,
        sdk_package="sdk",
        post_success_design_audit=True,
        success=False,
        message="synthetic failure",
        turn_count=1,
        tool_call_count=1,
        compile_attempt_count=1,
    )

    payload = batch_runner._write_row_state(
        repo,
        run_id="run_123",
        row=row,
        allocation=allocation,
        latest_status="failed",
        attempt=attempt,
        current_attempt_started_at="2026-03-26T11:59:00Z",
        latest_error_message="synthetic failure",
    )

    assert payload == {
        "row_id": "row_0001",
        "prompt_index": 1,
        "category_slug": "hinge",
        "dataset_id": "ds_hinge_0001",
        "record_id": "rec_hinge_0001",
        "latest_status": "failed",
        "latest_error_message": "synthetic failure",
        "current_attempt_started_at": "2026-03-26T11:59:00Z",
        "attempt_count": 1,
        "attempts": [attempt.to_dict()],
        "last_updated": payload["last_updated"],
    }


def test_overall_batch_status_marks_missing_or_failed_rows_as_failed() -> None:
    assert (
        batch_runner._overall_batch_status({"row_1": "success"}, expected_row_count=1) == "success"
    )
    assert (
        batch_runner._overall_batch_status({"row_1": "success"}, expected_row_count=2) == "failed"
    )
    assert (
        batch_runner._overall_batch_status(
            {"row_1": "success", "row_2": "failed"},
            expected_row_count=2,
        )
        == "failed"
    )


def test_read_total_cost_prefers_all_in_total(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_artifacts"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "cost.json").write_text(
        """
        {
          "total": {"costs_usd": {"total": 0.75}},
          "all_in_total": {"costs_usd": {"total": 0.91}}
        }
        """.strip(),
        encoding="utf-8",
    )

    assert batch_runner._read_total_cost(run_dir) == pytest.approx(0.91)


def test_build_allocations_reuses_dataset_id_index(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    rows = [
        replace(_make_row(), row_id=f"row_{index:04d}", prompt_index=index, prompt=f"hinge {index}")
        for index in range(1, 4)
    ]

    build_calls = 0
    original_builder = DatasetStore._build_dataset_id_index_from_records

    def wrapped_builder(self: DatasetStore) -> dict[str, str]:
        nonlocal build_calls
        build_calls += 1
        return original_builder(self)

    monkeypatch.setattr(DatasetStore, "_build_dataset_id_index_from_records", wrapped_builder)

    allocations = batch_runner._build_allocations(rows, repo=repo)

    assert len(allocations) == 3
    assert build_calls == 1
