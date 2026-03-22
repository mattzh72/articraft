from __future__ import annotations

from agent.prompts.compile import compile_prompt_variant
from agent.prompts.spec import iter_prompt_variants


def test_compiled_openai_prompt_keeps_tool_policy_and_design_charter() -> None:
    variant = next(
        variant
        for variant in iter_prompt_variants()
        if variant.output.name == "designer_system_prompt_openai.txt"
    )
    compiled = compile_prompt_variant(variant)
    generated = variant.output.read_text(encoding="utf-8")

    for text in (compiled, generated):
        assert "<role>" in text
        assert "<tool_contract>" in text
        assert "<feedback_strategy>" in text
        assert "<modeling_priority>" in text
        assert "Use ONLY `read_file` and `apply_patch`" in text
        assert "write_code" not in text
        assert "Use code, compile output, QC, and tests as your sensors" in text
        assert "<compile_signals>" in text
        assert 'Do not optimize for "passing" in the abstract' in text
        assert "prompt-named visible features" in text
        assert "wrong geometric representation" in text
        assert "wrong overall composition" in text
        assert "warn_if_articulation_origin_near_geometry" in text
        assert "warn_if_part_geometry_disconnected" in text
        assert "deliberately dumb static sensors" in text
        assert "semantic regression" in text
        assert "Do not preserve a scaffold just because parts of it compile" in text
        assert "silhouette-critical visible forms" in text
        assert "Plain primitives are acceptable for hidden structure" in text
        assert "If the real object should be hollow, thin-walled, or cavity-bearing" in text
        assert "Do not omit important internal structures" in text
        assert "most defining 3-6" in text
