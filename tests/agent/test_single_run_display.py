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


def test_add_tool_call_shows_all_find_examples_titles() -> None:
    display, buffer = _make_display()

    display.add_tool_call(
        tool_name="find_examples",
        args={"query": "pcb assembly", "limit": 3},
        success=True,
        duration=0.02,
        result=[
            {"title": "Raspberry Pi 3 Model B Assembly", "path": "sdk/_examples/hybrid/foo.md"},
            {"title": "Parametric Pin Header", "path": "sdk/_examples/hybrid/bar.md"},
            {"title": "RJ45 Surface-mount Jack", "path": "sdk/_examples/hybrid/baz.md"},
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
                "path": "sdk/_examples/hybrid/pitray_clip.md",
                "match_quality": "weakly_relevant",
            }
        ],
    )

    output = buffer.getvalue()
    assert "- PiTray Clip [weakly relevant]" in output


def test_add_small_context_handoff_shows_full_message() -> None:
    display, buffer = _make_display()

    display.add_small_context_handoff(
        reset_count=2,
        carryover_message={
            "role": "user",
            "content": (
                "<small_context_resume>\n"
                "- The last patch compiled cleanly.\n"
                "- Re-read the current file before editing.\n"
                "</small_context_resume>"
            ),
        },
    )

    output = buffer.getvalue()
    assert "handoff next agent  reset #2" in output
    assert "FULL handoff message:" in output
    assert "role: user" in output
    assert "<small_context_resume>" in output
    assert "- The last patch compiled cleanly." in output
    assert "- Re-read the current file before editing." in output
