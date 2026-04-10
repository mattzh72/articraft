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

        # Compact shared workflow stub
        assert "Start with a short evidence-gathering pass before the first edit:" in text
        assert "preloaded SDK quickstart/router" in text
        assert "Start with the smallest coherent backbone or subassembly" in text
        assert "Expand one coherent region at a time" in text
        assert "Always run `compile_model` on the latest revision before concluding." in text
        assert "PHASE 1" not in text

        # Tool contract
        assert (
            "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
            in text
        )
        assert "FREEFORM tool" in text
        assert "inspection-only" in text
        assert "lexical search over curated examples for the active SDK" in text
        assert "Prefer several small `apply_patch` edits over one giant patch" in text
        assert "Treat `compile_model` as the full validation pass." in text

        # Modeling section stays as SDK-specific deltas
        assert "Read mounted SDK docs as needed" in text
        assert "Prefer Articraft-native primitives and placement helpers" in text
        assert (
            "Use CadQuery-backed geometry only for the specific parts that need lower-level control"
            in text
        )
        assert "Author visual geometry only; do not author collision geometry in `sdk`." in text

        # No disallowed fragments
        assert "expect_aabb_" not in text
        assert "expect_joint_motion_axis(" not in text
        assert len(text.splitlines()) <= 80
