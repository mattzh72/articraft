from __future__ import annotations

from pathlib import Path

from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    load_prompt_section_text,
    load_system_prompt_text,
    provider_system_prompt_suffix,
    resolve_system_prompt_path,
)
from agent.tools import (
    build_first_turn_runtime_guidance,
    build_initial_user_content,
    prepend_runtime_guidance,
)


def test_system_prompt_resolution_variants() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    resolved = resolve_system_prompt_path(
        str(Path("agent/prompts/generated") / DESIGNER_PROMPT_NAME),
        provider="openai",
        repo_root=repo_root,
    )
    assert resolved.name == OPENAI_DESIGNER_PROMPT_NAME

    loaded_path, loaded_text = load_system_prompt_text(
        str(Path("agent/prompts/generated") / DESIGNER_PROMPT_NAME),
        provider="openai",
        repo_root=repo_root,
    )
    assert loaded_path == resolved
    assert loaded_text == resolved.read_text(encoding="utf-8")

    gemini_resolved = resolve_system_prompt_path(
        str(Path("agent/prompts/generated") / DESIGNER_PROMPT_NAME),
        provider="gemini",
        repo_root=repo_root,
    )
    assert gemini_resolved.name == GEMINI_DESIGNER_PROMPT_NAME


def test_provider_system_prompt_suffixes() -> None:
    assert provider_system_prompt_suffix("openai") == ""
    assert provider_system_prompt_suffix("gemini") == ""


def test_first_turn_runtime_guidance_is_provider_specific() -> None:
    openai_guidance = build_first_turn_runtime_guidance("openai")
    gemini_guidance = build_first_turn_runtime_guidance("gemini")

    assert "read_file" in openai_guidance
    assert "apply_patch" in openai_guidance
    assert "docs/..." in openai_guidance
    assert "read_code" in gemini_guidance
    assert "read_file" in gemini_guidance
    assert "edit_code" in gemini_guidance
    assert "multiple independent read-only lookups" in gemini_guidance


def test_prepend_runtime_guidance_supports_text_only_content() -> None:
    content = prepend_runtime_guidance(
        "make a hinge",
        runtime_guidance_text="<runtime_task_guidance>\n- Stay incremental.\n</runtime_task_guidance>",
    )

    assert isinstance(content, str)
    assert content.startswith("<runtime_task_guidance>")
    assert content.endswith("make a hinge")


def test_build_initial_user_content_can_append_runtime_guidance_for_multimodal(
    tmp_path: Path,
) -> None:
    image_path = tmp_path / "reference.png"
    image_path.write_bytes(b"\x89PNG\r\n\x1a\n")

    content = build_initial_user_content(
        "make a lamp",
        image_path=image_path,
        runtime_guidance_text="<runtime_task_guidance>\n- Stay incremental.\n</runtime_task_guidance>",
    )

    assert content == [
        {
            "type": "input_text",
            "text": "<runtime_task_guidance>\n- Stay incremental.\n</runtime_task_guidance>",
        },
        {"type": "input_text", "text": "make a lamp"},
        {"type": "input_image", "image_path": str(image_path), "detail": "high"},
    ]


def test_compaction_prompt_section_loads_from_prompt_assets() -> None:
    prompt_path, prompt_text = load_prompt_section_text("gemini_compaction.md")

    assert prompt_path.name == "gemini_compaction.md"
    assert "Return JSON only." in prompt_text
