from __future__ import annotations

import re

from agent.prompts.compile import compile_prompt_variant, find_stale_prompts
from agent.prompts.spec import iter_prompt_variants

REQUIRED_TAGS = (
    "<role>",
    "<process>",
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


def _assert_shared_contract(text: str) -> None:
    for tag in REQUIRED_TAGS:
        assert tag in text
    assert _opening_tag_count(text) >= 5
    assert "<compile_signals>" in text

    # Three hard requirements
    assert "NO FLOATING PARTS" in text
    assert "NO UNINTENTIONAL OVERLAPS" in text
    assert "REALISTIC GEOMETRY" in text

    # Shared process stub stays compact
    assert "Start with a short evidence-gathering pass before the first edit:" in text
    assert "preloaded SDK quickstart/router" in text
    assert "load only the specific `docs/` references needed for that next change" in text
    assert "Gather context for the next coherent edit, not for the whole object up front." in text
    assert "Start with the smallest coherent backbone or subassembly" in text
    assert "Expand one coherent region at a time" in text
    assert "Always run `compile_model` on the latest revision before concluding." in text
    assert "PHASE 1" not in text

    # Core tool references
    assert "probe_model" in text
    assert "find_examples" in text
    assert "inspection-only" in text

    # SDK docs deference
    assert "Read mounted SDK docs as needed" in text
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
        "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
        in openai_text
    )
    assert "write_code" not in openai_text
    assert "FREEFORM tool" in openai_text
    assert "Prefer several small `apply_patch` edits over one giant patch" in openai_text
    assert "Build one coherent part or subassembly at a time" in openai_text
    assert "lexical search over curated examples for the active SDK" in openai_text
    assert "[weakly relevant]" in openai_text
    assert "Prefer Articraft-native primitives and placement helpers" in openai_text
    assert (
        "Use CadQuery-backed geometry only for the specific parts that need lower-level control"
        in openai_text
    )
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in openai_text
    assert "Treat `compile_model` as the full validation pass." in openai_text

    gemini_text = compiled_by_name["designer_system_prompt_gemini.txt"]
    _assert_shared_contract(gemini_text)
    assert (
        "Use ONLY `read_file`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`"
        in gemini_text
    )
    assert 'old_string=""' in gemini_text
    assert "write_code" not in gemini_text
    assert "Prefer small exact `edit_code` replacements over broad rewrites" in gemini_text
    assert "lexical search over curated examples for the active SDK" in gemini_text
    assert "[weakly relevant]" in gemini_text
    assert "Prefer Articraft-native primitives and placement helpers" in gemini_text
    assert (
        "Use CadQuery-backed geometry only for the specific parts that need lower-level control"
        in gemini_text
    )
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in gemini_text
