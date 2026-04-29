from __future__ import annotations

import re

from agent.prompts.compile import compile_prompt_variant, find_stale_prompts
from agent.prompts.spec import iter_prompt_variants

REQUIRED_TAGS = (
    "<role>",
    "<tools>",
    "<modeling>",
    "<link_naming>",
)
DISALLOWED_FRAGMENTS = (
    "expect_aabb_",
    "expect_joint_motion_axis(",
    "Prefer **AABB-based intent checks**",
)


def _opening_tag_count(text: str) -> int:
    return len(re.findall(r"^<(?!(?:/|compile_signals>))[a-z_]+>$", text, flags=re.MULTILINE))


def _assert_shared_contract(text: str, *, allow_process: bool = False) -> None:
    for tag in REQUIRED_TAGS:
        assert tag in text
    assert _opening_tag_count(text) >= 4
    assert "<compile_signals>" in text

    # Three hard requirements
    assert "NO FLOATING PARTS" in text
    assert "NO UNINTENTIONAL OVERLAPS" in text
    assert "REALISTIC GEOMETRY" in text
    assert "Assign plausible colors and materials" in text
    assert "buttons, knobs, switches, keys, levers, pedals" in text
    assert "small local hidden overlap is acceptable" in text
    assert "Keep intentional overlap local and element-scoped when possible" in text

    # Workflow guidance is intentionally omitted from the compiled system prompts.
    if not allow_process:
        assert "<process>" not in text
    assert "Start with a short context pass:" not in text
    assert "preloaded SDK quickstart/router" not in text
    assert "Read only the specific `docs/` references needed for the next change" not in text
    assert "read the full file once, not a small slice" not in text
    assert "Do not re-read it if it is already in context." not in text
    assert "Prefer evidence over introspection." not in text
    assert "Start with the smallest coherent backbone or subassembly" not in text
    assert "Expand one coherent region at a time" not in text
    assert "Treat `compile_model` and `probe_model` as feedback tools." not in text
    assert "Prefer the smallest action that gives decisive evidence." not in text
    assert (
        "If the cause is obvious from `model.py` and compile output, fix it directly." not in text
    )
    assert "When a spatial issue is ambiguous, use `probe_model` to gather evidence." not in text
    assert "After a first ambiguous repair does not resolve the issue" not in text
    assert "Always run `compile_model` on the latest revision before concluding." not in text
    assert "PHASE 1" not in text

    # Core tool references
    assert "probe_model" in text
    assert "find_examples" in text
    assert "read-only Python inspection" in text
    assert "Never answer with code directly in the assistant response." in text
    assert "Do not ask the user for feedback, confirmation, or permission to continue." in text

    assert "Do not provide `file_path`" not in text
    assert "missing exact geometry" not in text
    assert "means a gap, not an overlap" not in text

    for fragment in DISALLOWED_FRAGMENTS:
        assert fragment not in text


def test_prompt_outputs_are_current() -> None:
    stale = find_stale_prompts()
    assert not stale, f"Generated prompts are stale: {[variant.output.name for variant in stale]}"

    compiled_by_name = {
        variant.output.name: compile_prompt_variant(variant) for variant in iter_prompt_variants()
    }

    openai_text = compiled_by_name["designer_system_prompt_openai.txt"]
    _assert_shared_contract(openai_text)
    assert (
        "Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`."
        in openai_text
    )
    assert "write_code" not in openai_text
    assert "FREEFORM tool" in openai_text
    assert "Prefer several small `apply_patch` edits over one giant patch" in openai_text
    assert "searches curated SDK examples for patterns" in openai_text
    assert "[weakly relevant]" in openai_text
    assert (
        "Prefer Articraft-native primitives and placement helpers when they represent the form credibly."
        in openai_text
    )
    assert (
        "Use CadQuery only for the advanced parts that need lower-level shape control"
        in openai_text
    )
    assert (
        "Mix approaches freely; do not switch the whole object to CadQuery unless the whole object needs it."
        in openai_text
    )
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in openai_text
    assert "Accepted intentional cases include proxy nesting" in openai_text
    assert "Pair every `ctx.allow_overlap(...)` with at least one exact proof check" in openai_text
    assert (
        "When you no longer need tools, conclude instead of continuing to reflect in text."
        not in openai_text
    )

    gemini_text = compiled_by_name["designer_system_prompt_gemini.txt"]
    _assert_shared_contract(gemini_text)
    assert (
        "Available tools: `read_file`, `replace`, `write_file`, `compile_model`, `probe_model`, and `find_examples`."
        in gemini_text
    )
    assert 'Use `read_file(path="model.py")` for the current editable code section' in gemini_text
    assert "write_code" not in gemini_text
    assert "Prefer small exact `replace` edits over broad rewrites" in gemini_text
    assert "searches curated SDK examples for patterns" in gemini_text
    assert "[weakly relevant]" in gemini_text
    assert (
        "Prefer Articraft-native primitives and placement helpers when they represent the form credibly."
        in gemini_text
    )
    assert (
        "Use CadQuery only for the advanced parts that need lower-level shape control"
        in gemini_text
    )
    assert (
        "Mix approaches freely; do not switch the whole object to CadQuery unless the whole object needs it."
        in gemini_text
    )
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in gemini_text
    assert "Accepted intentional cases include proxy nesting" in gemini_text
    assert "Pair every `ctx.allow_overlap(...)` with at least one exact proof check" in gemini_text
    assert (
        "When you no longer need tools, conclude instead of continuing to reflect in text."
        in gemini_text
    )
    assert (
        "After a clean compile on the latest revision, conclude immediately unless you can name one specific unresolved defect."
        in gemini_text
    )

    openrouter_text = compiled_by_name["designer_system_prompt_openrouter.txt"]
    _assert_shared_contract(openrouter_text, allow_process=True)
    assert "<process>" in openrouter_text
    assert "Work evidence-first. Before editing, read `model.py`" in openrouter_text
    assert "use `find_examples` for one or two relevant construction patterns" in openrouter_text
    assert (
        "Do not keep planning in assistant text. Once you know the next concrete step, use a tool."
        in openrouter_text
    )
    assert (
        "Available tools: `read_file`, `replace`, `write_file`, `compile_model`, `probe_model`, and `find_examples`."
        in openrouter_text
    )
    assert "write_code" not in openrouter_text
    assert "Prefer small exact `replace` edits over broad rewrites" in openrouter_text
