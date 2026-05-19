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
CORE_CONCEPTS = (
    "NO FLOATING PARTS",
    "NO UNINTENTIONAL OVERLAPS",
    "REALISTIC GEOMETRY",
    "Assign plausible colors and materials",
    "Do not remove, cap, fuse, or simplify prompt-critical visible geometry",
    "Prefer CadQuery for visible geometry",
    "probe_model",
    "find_examples",
    "Never answer with code directly in the assistant response.",
)


def _opening_tag_count(text: str) -> int:
    return len(re.findall(r"^<(?!(?:/|compile_signals>))[a-z_]+>$", text, flags=re.MULTILINE))


def _assert_shared_contract(text: str, *, allow_process: bool = False) -> None:
    for tag in REQUIRED_TAGS:
        assert tag in text
    assert _opening_tag_count(text) >= 4
    assert "<compile_signals>" in text

    for concept in CORE_CONCEPTS:
        assert concept in text

    if not allow_process:
        assert "<process>" not in text
    assert "PHASE 1" not in text

    assert "Do not provide `file_path`" not in text

    for fragment in DISALLOWED_FRAGMENTS:
        assert fragment not in text


def _assert_tool_capabilities(
    text: str, expected: set[str], *, absent: set[str] = frozenset()
) -> None:
    for tool_name in expected:
        assert tool_name in text
    for tool_name in absent:
        assert tool_name not in text


def test_prompt_outputs_are_current() -> None:
    stale = find_stale_prompts()
    assert not stale, f"Generated prompts are stale: {[variant.output.name for variant in stale]}"

    compiled_by_name = {
        variant.output.name: compile_prompt_variant(variant) for variant in iter_prompt_variants()
    }

    openai_text = compiled_by_name["designer_system_prompt_openai.txt"]
    _assert_shared_contract(openai_text)
    _assert_tool_capabilities(
        openai_text,
        {"read_file", "apply_patch", "compile_model", "probe_model", "find_examples"},
        absent={"write_code", "replace", "write_file"},
    )
    assert "FREEFORM tool" in openai_text

    gemini_text = compiled_by_name["designer_system_prompt_gemini.txt"]
    _assert_shared_contract(gemini_text)
    _assert_tool_capabilities(
        gemini_text,
        {"read_file", "replace", "write_file", "compile_model", "probe_model", "find_examples"},
        absent={"write_code", "apply_patch"},
    )

    openrouter_text = compiled_by_name["designer_system_prompt_openrouter.txt"]
    _assert_shared_contract(openrouter_text, allow_process=True)
    assert "<process>" in openrouter_text
    _assert_tool_capabilities(
        openrouter_text,
        {"read_file", "replace", "write_file", "compile_model", "probe_model", "find_examples"},
        absent={"write_code", "apply_patch"},
    )

    anthropic_text = compiled_by_name["designer_system_prompt_anthropic.txt"]
    _assert_shared_contract(anthropic_text, allow_process=True)
    assert "<process>" in anthropic_text
    _assert_tool_capabilities(
        anthropic_text,
        {"read_file", "replace", "write_file", "compile_model", "probe_model", "find_examples"},
        absent={"write_code", "apply_patch"},
    )
