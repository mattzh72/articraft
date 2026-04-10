from __future__ import annotations

from agent.prompts.compile import compile_prompt_variant
from agent.prompts.spec import iter_prompt_variants


def test_compiled_openai_prompt_keeps_compact_contract_and_visual_test_gate() -> None:
    variant = next(
        variant
        for variant in iter_prompt_variants()
        if variant.output.name == "designer_system_prompt_openai.txt"
    )
    compiled = compile_prompt_variant(variant)
    generated = variant.output.read_text(encoding="utf-8")

    for text in (compiled, generated):
        assert "<role>" in text
        assert "<process>" in text
        assert "<tools>" in text
        assert "<modeling>" in text

        # Three hard requirements are prominent
        assert "NO FLOATING PARTS" in text
        assert "NO UNINTENTIONAL OVERLAPS" in text
        assert "REALISTIC GEOMETRY" in text
        assert "ARTICULATE THE PRIMARY MECHANISMS" in text

        # Compact shared workflow stub
        assert "Start with a short context pass:" in text
        assert "preloaded SDK quickstart/router" in text
        assert "read the full file once, not a small slice" in text
        assert "Start with the smallest coherent backbone or subassembly" in text
        assert "Expand one coherent region at a time" in text
        assert "Always run `compile_model` on the latest revision before concluding." in text
        assert "PHASE 1" not in text

        # Tool contract
        assert (
            "Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`."
            in text
        )
        assert "FREEFORM tool" in text
        assert "inspection-only" in text
        assert "lexical search over curated examples for the active SDK" in text
        assert "Prefer several small `apply_patch` edits over one giant patch" in text
        assert "Treat `compile_model` as the full validation pass." in text
        assert "Prefer the smallest action that gives decisive evidence." in text
        assert (
            "If the cause is obvious from `model.py` and `compile_model` output, fix it directly."
            in text
        )

        # Modeling section stays as SDK-specific deltas
        assert (
            "Read mounted SDK docs as needed for placement, probe patterns, exact signatures, and testing guidance."
            in text
        )
        assert "Match the visible construction logic of the object." in text
        assert "Preserve correct joint origins, axes, limits, and articulation behavior." in text
        assert "Author visual geometry only; do not author collision geometry in `sdk`." in text

        # No disallowed fragments
        assert "expect_aabb_" not in text
        assert "expect_joint_motion_axis(" not in text
        assert len(text.splitlines()) <= 80
