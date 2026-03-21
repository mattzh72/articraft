from __future__ import annotations

from io import StringIO

from rich.console import Console

from agent.tui.single_run import SingleRunDisplay


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
