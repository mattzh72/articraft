from __future__ import annotations

from agent.prompts.compile import compile_prompt_variant, find_stale_prompts
from agent.prompts.spec import iter_prompt_variants


def test_prompt_outputs_are_current() -> None:
    stale = find_stale_prompts()
    assert not stale, f"Generated prompts are stale: {[variant.output.name for variant in stale]}"

    compiled_by_name = {
        variant.output.name: compile_prompt_variant(variant) for variant in iter_prompt_variants()
    }

    assert "apply_patch" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "write_code" not in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "<tool_contract>" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "<feedback_strategy>" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "<failure_diagnosis>" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "<compile_signals>" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "<modeling_priority>" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "wrong geometric representation" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert "prompt-named visible features" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert (
        "warn_if_articulation_origin_near_geometry"
        in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert (
        "warn_if_part_geometry_disconnected"
        in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert (
        "deliberately dumb static sensors" in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert "semantic regression" in compiled_by_name["designer_system_prompt_openai.txt"]
    assert (
        "silhouette-critical visible forms" in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert (
        "Plain primitives are acceptable for hidden structure"
        in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert (
        "If the real object should be hollow, thin-walled, or cavity-bearing"
        in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert (
        "Do not omit important internal structures"
        in compiled_by_name["designer_system_prompt_openai.txt"]
    )
    assert "sdk_hybrid" in compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    assert "full base SDK" not in compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    assert "section_loft(...)" in compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    assert "find_examples" in compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    assert "lexical, not semantic" in compiled_by_name["designer_system_prompt_openai_hybrid.txt"]
    assert "edit_code" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "read_code" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "write_code" not in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "<feedback_strategy>" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "<compile_signals>" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "wrong geometric representation" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert "prompt-named visible features" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert (
        "warn_if_articulation_origin_near_geometry"
        in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert (
        "warn_if_part_geometry_disconnected"
        in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert (
        "deliberately dumb static sensors" in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert "semantic regression" in compiled_by_name["designer_system_prompt_gemini.txt"]
    assert (
        "silhouette-critical visible forms" in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert (
        "Plain primitives are acceptable for hidden structure"
        in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert (
        "If the real object should be hollow, thin-walled, or cavity-bearing"
        in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert (
        "Do not omit important internal structures"
        in compiled_by_name["designer_system_prompt_gemini.txt"]
    )
    assert "sdk_hybrid" in compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    assert "full base SDK" not in compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    assert "section_loft(...)" in compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    assert "find_examples" in compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
    assert "lexical, not semantic" in compiled_by_name["designer_system_prompt_gemini_hybrid.txt"]
