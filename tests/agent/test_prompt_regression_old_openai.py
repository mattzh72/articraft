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

        # Phased iterative workflow
        assert "PHASE 1" in text
        assert "PHASE 2" in text
        assert "PHASE 3" in text
        assert "PHASE 4" in text
        assert "Do NOT write all geometry in one giant edit" in text
        assert "simple envelope geometry or minimal sketches/extrusions" in text
        assert "Upgrade each region from envelope geometry to realistic geometry" in text
        assert "keep its learned dimensions/joints/attachments" in text

        # Tool contract
        assert (
            "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
            in text
        )
        assert "FREEFORM tool" in text
        assert "inspection-only" in text
        assert "lexical search over curated base SDK examples" in text
        assert "Use `compile_model` explicitly to run full compile + QC" in text

        # Probe and testing
        assert "find_floating_parts(...)" in text
        assert "object-first" in text
        assert "resolve `part(...)`, `joint(...)`, `visual(...)` locals once" in text
        assert "exactly one root part" in text
        assert "Use `run_tests()` for prompt-specific exact checks" in text
        assert "Never cap a visible opening with a solid placeholder" in text

        # No disallowed fragments
        assert "expect_aabb_" not in text
        assert "expect_joint_motion_axis(" not in text
        assert len(text.splitlines()) <= 112
