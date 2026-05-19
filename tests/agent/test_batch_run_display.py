from __future__ import annotations

from io import StringIO

from rich.console import Console

import agent.tui.batch_run as batch_run_module
from agent.tui.batch_run import BatchRunDisplay


def _make_display() -> tuple[BatchRunDisplay, StringIO]:
    buffer = StringIO()
    console = Console(file=buffer, force_terminal=False, color_system=None, width=180)
    display = BatchRunDisplay(
        console=console,
        experiment_name="hinge_batch",
        total_runs=20,
        concurrency=4,
        model_id="gpt-5.4",
        enabled=True,
    )
    return display, buffer


def test_add_compaction_event_shows_row_label_and_cost() -> None:
    display, buffer = _make_display()
    display.add_run("rec_hinge_0001", "make a hinge")

    display.add_compaction_event(
        "rec_hinge_0001",
        trigger="compile_plateau",
        estimated_saved_next_input_tokens=90_000,
        billed_cost=0.00045,
        previous_response_id_cleared=True,
    )

    output = buffer.getvalue()
    assert "compact [#001/20] rec_hinge_0001" in output
    assert "compile plateau" in output
    assert "saved≈90.0K" in output
    assert "billed=$0.000450" in output
    assert "prev=cleared" in output


def test_add_maintenance_event_shows_cache_lifecycle_line() -> None:
    display, buffer = _make_display()
    display.add_run("rec_hinge_0001", "make a hinge")

    display.add_maintenance_event(
        "rec_hinge_0001",
        {"kind": "cache_create", "cache_name": "cachedContents/cache_1"},
        billed_cost=0.0,
    )

    output = buffer.getvalue()
    assert "maint   [#001/20] rec_hinge_0001" in output
    assert "cache create" in output
    assert "cachedContents/cache_1" in output


def test_run_lines_show_provider_model_and_thinking_metadata() -> None:
    display, buffer = _make_display()
    display.add_run(
        "rec_camcorder_0001",
        "make a camcorder",
        row_id="camcorder_claude_high",
        provider="anthropic",
        model_id="claude-opus-4-7",
        thinking_level="high",
    )

    display.start_run("rec_camcorder_0001")
    display.update_run_progress("rec_camcorder_0001", turn=1, cost=0.0)
    display.complete_run("rec_camcorder_0001", success=True, cost=0.01)

    output = buffer.getvalue()
    assert "camcorder_claude_high" in output
    assert "anthropic" in output
    assert "claude-opus-4-7" in output
    assert "thinking=high" in output


def test_resume_preserved_rows_do_not_affect_rate_or_eta(monkeypatch) -> None:
    now = 100.0

    def fake_time() -> float:
        return now

    monkeypatch.setattr(batch_run_module.time, "time", fake_time)

    display, buffer = _make_display()
    display.add_run("rec_hinge_0001", "preserved hinge")
    display.add_run("rec_hinge_0002", "fresh hinge")

    display.complete_run("rec_hinge_0001", success=True, cost=0.0)
    now = 102.0
    display.start_run("rec_hinge_0002")
    now = 104.0
    display.complete_run("rec_hinge_0002", success=True, cost=0.01)
    display.stop()

    output = buffer.getvalue()
    assert "rate=15.00 runs/min" in output
    assert "eta=1m 12s" in output
    assert "rate=30.00 runs/min" not in output
    assert "avg=4.0s" in output
