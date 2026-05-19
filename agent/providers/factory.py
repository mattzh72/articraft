from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from agent.providers.anthropic import (
    DEFAULT_ANTHROPIC_MODEL,
    AnthropicLLM,
    anthropic_api_key_from_env,
)
from agent.providers.base import ProviderClient
from agent.providers.gemini import GeminiLLM, gemini_client_config_from_env
from agent.providers.openai import DEFAULT_OPENAI_MODEL, OpenAILLM, openai_api_key_from_env
from agent.providers.openrouter import (
    DEFAULT_OPENROUTER_MODEL,
    OpenRouterLLM,
    openrouter_api_key_from_env,
)
from articraft.values import (
    PROVIDER_VALUE_SET,
    ProviderName,
)
from articraft.values import (
    infer_provider_from_model_id as _infer_provider_from_model_id,
)
from articraft.values import (
    normalize_provider_name as _normalize_provider_name,
)

DEFAULT_GEMINI_MODEL = "gemini-3.1-pro-preview"
SUPPORTED_PROVIDERS = PROVIDER_VALUE_SET


@dataclass(slots=True, frozen=True)
class ProviderConfig:
    provider: str = "openai"
    model_id: str | None = None
    thinking_level: str = "high"
    openai_transport: str = "http"
    openai_reasoning_summary: str | None = "auto"
    openai_prompt_cache_key: str | None = None
    openai_prompt_cache_retention: str | None = None

    @property
    def normalized_provider(self) -> str:
        return normalize_provider_name(self.provider)


@dataclass(slots=True, frozen=True)
class ProviderConstructors:
    anthropic: Callable[..., ProviderClient] = AnthropicLLM
    gemini: Callable[..., ProviderClient] = GeminiLLM
    openai: Callable[..., ProviderClient] = OpenAILLM
    openrouter: Callable[..., ProviderClient] = OpenRouterLLM


def normalize_provider_name(provider: str | None) -> str:
    return _normalize_provider_name(provider).value


def infer_provider_from_model_id(model_id: str | None) -> str | None:
    provider = _infer_provider_from_model_id(model_id)
    return provider.value if provider is not None else None


def default_model_id(config: ProviderConfig) -> str:
    if config.model_id:
        return config.model_id
    provider = _normalize_provider_name(config.provider)
    if provider is ProviderName.ANTHROPIC:
        return DEFAULT_ANTHROPIC_MODEL
    if provider is ProviderName.GEMINI:
        return DEFAULT_GEMINI_MODEL
    if provider is ProviderName.OPENROUTER:
        return DEFAULT_OPENROUTER_MODEL
    if provider is ProviderName.OPENAI:
        return DEFAULT_OPENAI_MODEL
    raise ValueError(f"Unsupported provider: {config.provider}")


def create_provider_client(
    config: ProviderConfig,
    *,
    dry_run: bool = False,
    constructors: ProviderConstructors | None = None,
) -> ProviderClient:
    provider = _normalize_provider_name(config.provider)
    model_id = default_model_id(config)
    provider_constructors = constructors or ProviderConstructors()
    if provider is ProviderName.ANTHROPIC:
        return provider_constructors.anthropic(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider is ProviderName.GEMINI:
        return provider_constructors.gemini(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider is ProviderName.OPENROUTER:
        return provider_constructors.openrouter(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider is ProviderName.OPENAI:
        return provider_constructors.openai(
            model_id=model_id,
            thinking_level=config.thinking_level,
            reasoning_summary=config.openai_reasoning_summary,
            transport=config.openai_transport,
            prompt_cache_key=config.openai_prompt_cache_key,
            prompt_cache_retention=config.openai_prompt_cache_retention,
            dry_run=dry_run,
        )
    raise ValueError(f"Unsupported provider: {config.provider}")


def validate_provider_credentials(provider: str) -> None:
    provider_norm = _normalize_provider_name(provider)
    if provider_norm is ProviderName.ANTHROPIC:
        if not anthropic_api_key_from_env():
            raise ValueError(
                "Anthropic credentials are required. Set ANTHROPIC_API_KEY or ANTHROPIC_API_KEYS."
            )
        return
    if provider_norm is ProviderName.GEMINI:
        gemini_client_config_from_env()
        return
    if provider_norm is ProviderName.OPENROUTER:
        if not openrouter_api_key_from_env():
            raise ValueError(
                "OpenRouter credentials are required. "
                "Set OPENROUTER_API_KEY or OPENROUTER_API_KEYS."
            )
        return
    if provider_norm is ProviderName.OPENAI and not openai_api_key_from_env():
        raise ValueError("OpenAI credentials are required. Set OPENAI_API_KEY or OPENAI_API_KEYS.")
