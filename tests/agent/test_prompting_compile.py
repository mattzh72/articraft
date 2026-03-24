from __future__ import annotations

import re

from agent.prompts.compile import compile_prompt_variant, find_stale_prompts
from agent.prompts.spec import iter_prompt_variants

REQUIRED_TAGS = (
    "<role>",
    "<tool_contract>",
    "<modeling_charter>",
    "<verification_contract>",
    "<repair_strategy>",
)
DISALLOWED_FRAGMENTS = (
    "expect_aabb_",
    "expect_joint_motion_axis(",
    "Prefer **AABB-based intent checks**",
)


def _opening_tag_count(text: str) -> int:
    return len(re.findall(r"^<(?!(?:/|compile_signals>))[a-z_]+>$", text, flags=re.MULTILINE))


def _assert_shared_contract(text: str, *, budget: int) -> None:
    for tag in REQUIRED_TAGS:
        assert tag in text
    assert _opening_tag_count(text) == 5
    assert "<compile_signals>" in text
    assert 'Do not optimize for "passing" in the abstract' in text
    assert "Use the injected SDK docs for exact helper signatures" in text
    assert "The model is not done until every applicable visual coverage category is proved" in text
    assert "hero features are present" in text
    assert "connected/seated" in text
    assert "important parts are in the right place" in text
    assert "each new visible form or mechanism has a matching assertion" in text
    assert "complexity is a feature, not a bug" in text
    assert "Do not cap a visible opening with a solid placeholder primitive." in text
    assert "Broad `warn_if_*` checks are sensors, not proof." in text
    assert "keep the scaffolded hard gates" in text
    assert "If the object has a mounted subassembly" in text
    assert "If a warning-tier heuristic fires, investigate it with `probe_model`" in text
    assert "If support/floating is ambiguous, use `probe_model` helpers first" in text
    assert "Deprecated as default scaffold heuristics in new generated code" in text
    for fragment in DISALLOWED_FRAGMENTS:
        assert fragment not in text
    assert len(text.splitlines()) <= budget


def test_prompt_outputs_are_current() -> None:
    stale = find_stale_prompts()
    assert not stale, f"Generated prompts are stale: {[variant.output.name for variant in stale]}"

    compiled_by_name = {
        variant.output.name: compile_prompt_variant(variant) for variant in iter_prompt_variants()
    }

    openai_text = compiled_by_name["designer_system_prompt_openai.txt"]
    _assert_shared_contract(openai_text, budget=132)
    assert "Use ONLY `read_file`, `apply_patch`, `probe_model`, and `find_examples`" in openai_text
    assert "write_code" not in openai_text
    assert (
        "probe_model` runs short Python snippets against the current bound model for "
        "inspection-only geometry diagnosis" in openai_text
    )
    assert "Use `probe_model` only for non-mutating inspection." in openai_text
    assert "find_examples` is lexical search over curated base SDK examples" in openai_text
    assert "stale or deprecated SDK code" in openai_text
    assert "inspiration/reference only" in openai_text
    assert "Author visuals only; do not author collision geometry in `sdk`." in openai_text
    assert "silhouette-critical shells, ducts, nacelles, blades, petals" in openai_text

    openai_hybrid_text = compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    _assert_shared_contract(openai_hybrid_text, budget=142)
    assert (
        "Use ONLY `read_file`, `apply_patch`, `probe_model`, and `find_examples`"
        in openai_hybrid_text
    )
    assert (
        "probe_model` runs short Python snippets against the current bound model for "
        "inspection-only geometry diagnosis" in openai_hybrid_text
    )
    assert "Use `probe_model` only for non-mutating inspection." in openai_hybrid_text
    assert (
        "find_examples` is lexical search over curated hybrid/CadQuery examples"
        in openai_hybrid_text
    )
    assert "stale or deprecated SDK code" in openai_hybrid_text
    assert "inspiration/reference only" in openai_hybrid_text
    assert "Import from `sdk_hybrid`, not `sdk`" in openai_hybrid_text
    assert (
        "`section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable"
        in openai_hybrid_text
    )
    assert "silhouette-critical shells, ducts, nacelles, blades, petals" in openai_hybrid_text

    gemini_text = compiled_by_name["designer_system_prompt_gemini.txt"]
    _assert_shared_contract(gemini_text, budget=132)
    assert "Use ONLY `read_code`, `edit_code`, `probe_model`, and `find_examples`" in gemini_text
    assert 'old_string=""' in gemini_text
    assert "write_code" not in gemini_text
    assert (
        "probe_model` runs short Python snippets against the current bound model for "
        "inspection-only geometry diagnosis" in gemini_text
    )
    assert "Use `probe_model` only for non-mutating inspection." in gemini_text
    assert "find_examples` is lexical search over curated base SDK examples" in gemini_text
    assert "stale or deprecated SDK code" in gemini_text
    assert "inspiration/reference only" in gemini_text
    assert "Author visuals only; do not author collision geometry in `sdk`." in gemini_text

    gemini_hybrid_text = compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    _assert_shared_contract(gemini_hybrid_text, budget=142)
    assert (
        "Use ONLY `read_code`, `edit_code`, `probe_model`, and `find_examples`"
        in gemini_hybrid_text
    )
    assert (
        "probe_model` runs short Python snippets against the current bound model for "
        "inspection-only geometry diagnosis" in gemini_hybrid_text
    )
    assert "Use `probe_model` only for non-mutating inspection." in gemini_hybrid_text
    assert (
        "find_examples` is lexical search over curated hybrid/CadQuery examples"
        in gemini_hybrid_text
    )
    assert "stale or deprecated SDK code" in gemini_hybrid_text
    assert "inspiration/reference only" in gemini_hybrid_text
    assert "Import from `sdk_hybrid`, not `sdk`" in gemini_hybrid_text
    assert (
        "`section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable"
        in gemini_hybrid_text
    )
