from __future__ import annotations

from enum import StrEnum


class ProviderName(StrEnum):
    ANTHROPIC = "anthropic"
    GEMINI = "gemini"
    OPENAI = "openai"
    OPENROUTER = "openrouter"


class ThinkingLevel(StrEnum):
    LOW = "low"
    MED = "med"
    HIGH = "high"


PROVIDER_VALUES = tuple(provider.value for provider in ProviderName)
PROVIDER_VALUE_SET = frozenset(PROVIDER_VALUES)
THINKING_LEVEL_VALUES = tuple(level.value for level in ThinkingLevel)
THINKING_LEVEL_VALUE_SET = frozenset(THINKING_LEVEL_VALUES)


def normalize_provider_name(provider: str | ProviderName | None) -> ProviderName:
    value = str(provider or ProviderName.OPENAI).strip().lower()
    try:
        return ProviderName(value)
    except ValueError as exc:
        raise ValueError(f"Unsupported provider: {provider}") from exc


def infer_provider_from_model_id(model_id: str | None) -> ProviderName | None:
    model_norm = (model_id or "").strip().lower()
    if not model_norm:
        return None
    if model_norm.startswith(("gpt-", "o1", "o3", "o4")):
        return ProviderName.OPENAI
    if model_norm.startswith("claude-"):
        return ProviderName.ANTHROPIC
    if model_norm.startswith("gemini-"):
        return ProviderName.GEMINI
    if "/" in model_norm or model_norm.startswith("openrouter/"):
        return ProviderName.OPENROUTER
    return None


def normalize_thinking_level(
    thinking_level: str | ThinkingLevel | None,
    *,
    default: ThinkingLevel = ThinkingLevel.HIGH,
) -> ThinkingLevel:
    value = str(thinking_level or default).strip().lower()
    if value == "medium":
        value = ThinkingLevel.MED.value
    try:
        return ThinkingLevel(value)
    except ValueError:
        return default


def provider_reasoning_level(
    thinking_level: str | ThinkingLevel | None,
    *,
    default: ThinkingLevel = ThinkingLevel.HIGH,
    medium_value: str = "medium",
) -> str:
    level = normalize_thinking_level(thinking_level, default=default)
    if level is ThinkingLevel.MED:
        return medium_value
    return level.value


def reasoning_level_alias(value: str | None) -> str:
    normalized = (value or "").strip().lower()
    if normalized == ThinkingLevel.MED.value:
        return "medium"
    return normalized
