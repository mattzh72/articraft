from __future__ import annotations

import re

from agent.prompts.compile import compile_prompt_variant, find_stale_prompts
from agent.prompts.spec import iter_prompt_variants

REQUIRED_TAGS = (
    "<role>",
    "<process>",
    "<tools>",
    "<modeling>",
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
    assert _opening_tag_count(text) == 4
    assert "<compile_signals>" in text

    # Three hard requirements
    assert "NO FLOATING PARTS" in text
    assert "NO UNINTENTIONAL OVERLAPS" in text
    assert "REALISTIC GEOMETRY" in text

    # Phased iterative workflow
    assert "PHASE 1" in text
    assert "PHASE 2" in text
    assert "PHASE 3" in text
    assert "PHASE 4" in text
    assert "Do NOT write all geometry in one giant edit" in text
    assert "one part or subassembly at a time" in text
    assert "simple envelope geometry or minimal sketches/extrusions" in text
    assert "smallest coherent backbone or subassembly" in text
    assert "Do not scaffold the entire object in one pass" in text
    assert "Upgrade each region from envelope geometry to realistic geometry" in text
    assert "keep its learned dimensions/joints/attachments" in text

    # Core tool references
    assert "probe_model" in text
    assert "find_examples" in text
    assert "inspection-only" in text

    # Probe helpers
    assert "find_floating_parts(...)" in text
    assert "`summary(...)`, `dims(...)`, `projection(...)`" in text

    # Object-first pattern
    assert "object-first" in text
    assert "resolve `part(...)`, `joint(...)`, `visual(...)` locals once" in text

    # Testing guidance
    assert "expect_contact" in text
    assert "expect_gap" in text
    assert "expect_overlap" in text
    assert "expect_within" in text
    assert "exactly one root part" in text
    assert "Use `run_tests()` for prompt-specific exact checks" in text
    assert "treat that name as a contract" in text
    assert "real 3D interpenetration" in text
    assert "projected footprint check" in text

    # Repair rules
    assert "Classify before patching" in text
    assert "2 repair turns" in text

    # Geometry guidance
    assert "Never cap a visible opening with a solid placeholder" in text

    # SDK docs deference
    assert "SDK docs" in text
    assert "Do not provide `file_path`" not in text
    assert "missing exact geometry" not in text
    assert "means a gap, not an overlap" not in text

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
    _assert_shared_contract(openai_text, budget=112)
    assert (
        "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
        in openai_text
    )
    assert "write_code" not in openai_text
    assert "FREEFORM tool" in openai_text
    assert "lexical search over curated examples for the active SDK" in openai_text
    assert "[weakly relevant]" in openai_text
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in openai_text
    assert "Use `compile_model` explicitly to run full compile + QC" in openai_text
    assert "`compile_model` automatically checks model validity" in openai_text

    openai_hybrid_text = compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    _assert_shared_contract(openai_hybrid_text, budget=120)
    assert (
        "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
        in openai_hybrid_text
    )
    assert "FREEFORM tool" in openai_hybrid_text
    assert "lexical search over curated examples for the active SDK" in openai_hybrid_text
    assert "[weakly relevant]" in openai_hybrid_text
    assert "Import from `sdk_hybrid`, not `sdk`" in openai_hybrid_text
    assert (
        "`section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable"
        in openai_hybrid_text
    )
    assert "`compile_model` automatically checks model validity" in openai_hybrid_text

    gemini_text = compiled_by_name["designer_system_prompt_gemini.txt"]
    _assert_shared_contract(gemini_text, budget=113)
    assert (
        "Use ONLY `read_code`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`"
        in gemini_text
    )
    assert 'old_string=""' in gemini_text
    assert "write_code" not in gemini_text
    assert "lexical search over curated examples for the active SDK" in gemini_text
    assert "[weakly relevant]" in gemini_text
    assert "Author visual geometry only; do not author collision geometry in `sdk`." in gemini_text

    gemini_hybrid_text = compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    _assert_shared_contract(gemini_hybrid_text, budget=120)
    assert (
        "Use ONLY `read_code`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`"
        in gemini_hybrid_text
    )
    assert "lexical search over curated examples for the active SDK" in gemini_hybrid_text
    assert "[weakly relevant]" in gemini_hybrid_text
    assert "Import from `sdk_hybrid`, not `sdk`" in gemini_hybrid_text
    assert (
        "`section_loft(...)`, `repair_loft(...)`, and `partition_shell(...)` are unavailable"
        in gemini_hybrid_text
    )
