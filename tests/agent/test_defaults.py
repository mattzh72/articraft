from __future__ import annotations

from agent.defaults import (
    DEFAULT_MAX_TURNS,
    GEMINI_3_FLASH_DEFAULT_MAX_TURNS,
    default_max_turns_for_model,
)


def test_default_max_turns_for_model_uses_gemini_3_flash_override() -> None:
    assert default_max_turns_for_model("gemini-3-flash-preview") == (
        GEMINI_3_FLASH_DEFAULT_MAX_TURNS
    )
    assert default_max_turns_for_model("gemini-3.0-flash") == GEMINI_3_FLASH_DEFAULT_MAX_TURNS


def test_default_max_turns_for_model_keeps_baseline_for_other_models() -> None:
    assert default_max_turns_for_model("gpt-5.4") == DEFAULT_MAX_TURNS
    assert default_max_turns_for_model("gemini-3.1-pro-preview") == DEFAULT_MAX_TURNS
    assert default_max_turns_for_model("gemini-2.5-flash") == DEFAULT_MAX_TURNS
