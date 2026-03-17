from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
    HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    load_system_prompt_text,
    provider_system_prompt_suffix,
    resolve_system_prompt_path,
)


def main() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    resolved = resolve_system_prompt_path(
        str(Path("agent/prompting/generated") / DESIGNER_PROMPT_NAME),
        provider="openai",
        repo_root=repo_root,
    )
    assert resolved.name == OPENAI_DESIGNER_PROMPT_NAME

    loaded_path, loaded_text = load_system_prompt_text(
        str(Path("agent/prompting/generated") / DESIGNER_PROMPT_NAME),
        provider="openai",
        repo_root=repo_root,
    )
    assert loaded_path == resolved
    assert loaded_text == resolved.read_text(encoding="utf-8")

    hybrid_resolved = resolve_system_prompt_path(
        str(Path("agent/prompting/generated") / DESIGNER_PROMPT_NAME),
        provider="openai",
        sdk_package="sdk_hybrid",
        repo_root=repo_root,
    )
    assert hybrid_resolved.name == HYBRID_OPENAI_DESIGNER_PROMPT_NAME

    gemini_resolved = resolve_system_prompt_path(
        str(Path("agent/prompting/generated") / DESIGNER_PROMPT_NAME),
        provider="gemini",
        repo_root=repo_root,
    )
    assert gemini_resolved.name == GEMINI_DESIGNER_PROMPT_NAME

    gemini_hybrid_resolved = resolve_system_prompt_path(
        str(Path("agent/prompting/generated") / DESIGNER_PROMPT_NAME),
        provider="gemini",
        sdk_package="sdk_hybrid",
        repo_root=repo_root,
    )
    assert gemini_hybrid_resolved.name == HYBRID_GEMINI_DESIGNER_PROMPT_NAME

    assert provider_system_prompt_suffix("openai") == ""
    assert provider_system_prompt_suffix("gemini") == ""


if __name__ == "__main__":
    main()
