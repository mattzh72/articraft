from __future__ import annotations

import logging
import time
from io import StringIO

from rich.console import Console

from agent.tui.single_run import LLMWaitAwareStreamHandler, SingleRunDisplay


def _make_display() -> tuple[SingleRunDisplay, StringIO]:
    buffer = StringIO()
    console = Console(file=buffer, force_terminal=False, color_system=None, width=160)
    display = SingleRunDisplay(
        console=console,
        model_id="gpt-5.4",
        thinking_level="high",
        max_turns=30,
        enabled=True,
    )
    return display, buffer


def test_start_shows_run_header() -> None:
    display, buffer = _make_display()

    display.start()

    output = buffer.getvalue()
    assert "run gpt-5.4" in output


def test_add_llm_call_shows_context_pressure() -> None:
    display, buffer = _make_display()

    display.add_llm_call(
        {
            "prompt_tokens": 245_000,
            "cached_tokens": 50_000,
            "candidates_tokens": 5_000,
            "total_tokens": 250_000,
        },
        0.0125,
        2.5,
        context_pressure={
            "prompt_tokens": 245_000,
            "max_context_tokens": 400_000,
            "pressure_ratio": 245_000 / 400_000,
            "remaining_context_tokens": 155_000,
            "cached_tokens": 50_000,
            "output_tokens": 5_000,
            "total_tokens": 250_000,
        },
    )

    output = buffer.getvalue()
    assert "250.0K tokens" in output
    assert "ctx=245.0K/400.0K (61%)" in output
    assert "left=155.0K" in output
    assert "out=5.0K" in output
    assert "cache=50.0K (20%)" in output


def test_add_tool_call_shows_success_result_and_compilation() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="apply_patch",
        args={"input": "*** Begin Patch\n*** Update File: CURRENT_FILE\n@@"},
        success=True,
        duration=0.01,
        result="Patch applied successfully (2 hunks)",
        compilation={"status": "success", "error": None},
    )

    output = buffer.getvalue()
    assert "tool    apply_patch ✓" in output
    assert "result: Patch applied successfully (2 hunks)" in output
    assert "compilation.status: success" in output


def test_add_tool_call_shows_compilation_error_details() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="apply_patch",
        args={},
        success=True,
        duration=0.01,
        result="Patch applied successfully (1 hunks)",
        compilation={"status": "error", "error": "Syntax error: invalid syntax (line 12)"},
    )

    output = buffer.getvalue()
    assert "compilation.status: error" in output
    assert "compilation.error: Syntax error: invalid syntax (line 12)" in output


def test_add_tool_call_shows_all_find_examples_titles() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="find_examples",
        args={"query": "pcb assembly", "limit": 3},
        success=True,
        duration=0.02,
        result=[
            {"title": "Raspberry Pi 3 Model B Assembly", "path": "sdk/_examples/cadquery/foo.md"},
            {"title": "Parametric Pin Header", "path": "sdk/_examples/cadquery/bar.md"},
            {"title": "RJ45 Surface-mount Jack", "path": "sdk/_examples/cadquery/baz.md"},
        ],
    )

    output = buffer.getvalue()
    assert "tool    find_examples ✓" in output
    assert "titles:" in output
    assert "- Raspberry Pi 3 Model B Assembly" in output
    assert "- Parametric Pin Header" in output
    assert "- RJ45 Surface-mount Jack" in output
    assert "result: [{" not in output


def test_add_tool_call_marks_weakly_relevant_find_example_titles() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="find_examples",
        args={"query": "support bracket", "limit": 2},
        success=True,
        duration=0.02,
        result=[
            {
                "title": "PiTray Clip",
                "path": "sdk/_examples/cadquery/pitray_clip.md",
                "match_quality": "weakly_relevant",
            }
        ],
    )

    output = buffer.getvalue()
    assert "- PiTray Clip [weakly relevant]" in output


def test_add_tool_call_shows_read_file_path_before_slice_args() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="read_file",
        args={"path": "docs/sdk/references/geometry/mesh-geometry.md", "offset": 1, "limit": 200},
        success=True,
        duration=0.02,
        result='{"result":"L10: runtime"}',
    )

    output = buffer.getvalue()
    assert "tool    read_file ✓" in output
    assert "path: docs/sdk/references/geometry/mesh-geometry.md" in output
    assert "offset: 1" in output
    assert "limit: 200" in output
    assert "... (1 more)" not in output


def test_add_tool_call_renders_compile_model_as_compile_event() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="compile_model",
        args={},
        success=True,
        duration=0.25,
        result=(
            "<compile_signals>\n"
            "<summary>\n"
            "status=success failures=0 warnings=0 notes=0\n"
            "Compile passed cleanly.\n"
            "</summary>\n"
            "</compile_signals>"
        ),
        compilation={"status": "success", "error": None},
    )

    output = buffer.getvalue()
    assert "compile ✓" in output
    assert "tool    compile_model" not in output


def test_add_tool_call_renders_compile_model_warning_details() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="compile_model",
        args={},
        success=True,
        duration=0.25,
        result=(
            "<compile_signals>\n"
            "<summary>\n"
            "status=success failures=0 warnings=1 notes=0\n"
            "Compile passed with warnings.\n"
            "</summary>\n\n"
            "<warnings>\n"
            "- [geometry_overlap] Geometry overlap check reported overlaps.\n"
            "warn_if_overlaps(): pair=('base','door') depth=(0.001,0.002,0.003)\n"
            "</warnings>\n"
            "</compile_signals>"
        ),
        compilation={"status": "success", "error": None},
    )

    output = buffer.getvalue()
    assert "compile ✓" in output
    assert "status=success failures=0 warnings=1 notes=0" in output
    assert "Compile passed with warnings." in output
    assert "[geometry_overlap] Geometry overlap check reported overlaps." in output
    assert "warn_if_overlaps(): pair=('base','door') depth=(0.001,0.002,0.003)" in output


def test_add_tool_call_renders_compile_model_failure_details() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="compile_model",
        args={},
        success=True,
        duration=0.25,
        result=(
            "<compile_signals>\n"
            "<summary>\n"
            "status=failure failures=1 warnings=0 notes=0\n"
            "Primary issue: ValueError: bad loft\n"
            "</summary>\n\n"
            "<failures>\n"
            "- [compile_runtime] ValueError: bad loft\n"
            "Traceback line 1\n"
            "Traceback line 2\n"
            "</failures>\n"
            "\n<response_rules>\n"
            "- Failures are blocking and should be investigated.\n"
            "</response_rules>\n"
            "</compile_signals>"
        ),
        compilation={"status": "error", "error": "Primary issue: ValueError: bad loft"},
    )

    output = buffer.getvalue()
    assert "compile ✗" in output
    assert "status=failure failures=1 warnings=0 notes=0" in output
    assert "Primary issue: ValueError: bad loft" in output
    assert "[compile_runtime] ValueError: bad loft" in output
    assert "Traceback line 1" in output
    assert "Traceback line 2" in output
    assert "Failures are blocking and should be investigated." in output


def test_add_tool_call_renders_all_compile_warnings_without_truncating_count() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="compile_model",
        args={},
        success=True,
        duration=0.25,
        result=(
            "<compile_signals>\n"
            "<summary>\n"
            "status=success failures=0 warnings=4 notes=0\n"
            "Compile passed with warnings.\n"
            "</summary>\n\n"
            "<warnings>\n"
            "- [warning_1] first warning\n"
            "- [warning_2] second warning\n"
            "- [warning_3] third warning\n"
            "- [warning_4] fourth warning\n"
            "</warnings>\n"
            "</compile_signals>"
        ),
        compilation={"status": "success", "error": None},
    )

    output = buffer.getvalue()
    assert "[warning_1] first warning" in output
    assert "[warning_2] second warning" in output
    assert "[warning_3] third warning" in output
    assert "[warning_4] fourth warning" in output


def test_add_compaction_event_shows_saved_tokens_and_billed_cost_in_summary() -> None:
    display, buffer = _make_display()

    display.add_compaction_event(
        trigger="hard_pressure",
        estimated_saved_next_input_tokens=210_000,
        billed_cost=0.00125,
        previous_response_id_cleared=True,
        usage_total_tokens=1_800,
    )
    display.stop()

    output = buffer.getvalue()
    assert "compact hard pressure" in output
    assert "saved≈210.0K" in output
    assert "billed=$0.001250" in output
    assert "prev=cleared" in output
    assert "1.8K tokens" in output
    assert "$0.001250" in output


def test_add_maintenance_event_shows_cache_lifecycle_line() -> None:
    display, buffer = _make_display()

    display.add_maintenance_event(
        {
            "kind": "cache_create",
            "cache_name": "cachedContents/cache_1",
        },
        billed_cost=0.0,
    )

    output = buffer.getvalue()
    assert "maint   cache create" in output
    assert "cachedContents/cache_1" in output


class _TTYBuffer(StringIO):
    def isatty(self) -> bool:
        return True


def test_logging_clears_live_wait_line_before_emitting() -> None:
    buffer = _TTYBuffer()
    console = Console(file=buffer, force_terminal=False, color_system=None, width=160)
    display = SingleRunDisplay(
        console=console,
        model_id="gpt-5.4",
        thinking_level="high",
        max_turns=30,
        enabled=True,
    )
    logger = logging.getLogger("tests.single_run_display.wait_line")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    handler = LLMWaitAwareStreamHandler(stream=buffer)
    handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
    logger.handlers = [handler]
    try:
        display.start_llm_wait()
        time.sleep(0.05)
        logger.info("external log")
        time.sleep(0.05)
        display.stop_llm_wait()
    finally:
        logger.handlers = []

    output = buffer.getvalue()
    assert "INFO: external log" in output
    assert "\r" in output
