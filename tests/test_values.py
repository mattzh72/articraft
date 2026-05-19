from __future__ import annotations

import pytest

from articraft.values import (
    ProviderName,
    ThinkingLevel,
    infer_provider_from_model_id,
    normalize_provider_name,
    normalize_thinking_level,
    provider_reasoning_level,
    reasoning_level_alias,
)


@pytest.mark.parametrize(
    ("model_id", "provider"),
    [
        ("gpt-5.5-2026-04-23", ProviderName.OPENAI),
        ("claude-sonnet-4-5", ProviderName.ANTHROPIC),
        ("gemini-3-flash-preview", ProviderName.GEMINI),
        ("openai/gpt-5.4", ProviderName.OPENROUTER),
    ],
)
def test_infer_provider_from_model_id(model_id: str, provider: ProviderName) -> None:
    assert infer_provider_from_model_id(model_id) is provider


def test_normalize_provider_name_rejects_unknown_provider() -> None:
    with pytest.raises(ValueError, match="Unsupported provider"):
        normalize_provider_name("local")


def test_thinking_level_helpers_keep_public_med_spelling() -> None:
    assert normalize_thinking_level("medium") is ThinkingLevel.MED
    assert provider_reasoning_level("med") == "medium"
    assert reasoning_level_alias("med") == "medium"
