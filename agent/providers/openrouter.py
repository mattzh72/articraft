"""
OpenRouter LLM wrapper using the OpenAI-compatible Chat Completions API.

This module intentionally avoids importing `openai` at import-time so the rest
of the repo can be used without OpenRouter/OpenAI SDK credentials installed.
"""

from __future__ import annotations

import logging
import os
from typing import Any

from agent.providers import _shared as provider_shared
from agent.providers._shared import (
    create_openai_compatible_client,
    env_keys_from_values,
    random_env_key,
    should_retry_transient_http_exception,
)
from agent.providers._shared import (
    env_float as _env_float,
)
from agent.providers._shared import (
    env_int as _env_int,
)
from agent.providers._shared import (
    get_value as _get,
)
from agent.providers._shared import (
    serialize_json_value as _serialize_json_value,
)
from agent.providers.chat_completions import OpenAICompatibleChatCompletionsMixin
from articraft.values import reasoning_level_alias

logger = logging.getLogger(__name__)
_async_retry = provider_shared.async_retry

DEFAULT_OPENROUTER_MODEL = "tencent/hy3-preview:free"
OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"
DEFAULT_OPENROUTER_CONTEXT_TOKENS = 262_144
DEFAULT_OPENROUTER_MAX_TOKENS = 262_144
DEFAULT_OPENROUTER_OUTPUT_SAFETY_TOKENS = 1_024


def openrouter_api_keys_from_env(env: dict[str, str] | None = None) -> list[str]:
    return env_keys_from_values(
        primary_name="OPENROUTER_API_KEY",
        pool_name="OPENROUTER_API_KEYS",
        env=env,
    )


def openrouter_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    return random_env_key(
        primary_name="OPENROUTER_API_KEY",
        pool_name="OPENROUTER_API_KEYS",
        env=env,
    )


class OpenRouterLLM(OpenAICompatibleChatCompletionsMixin):
    """OpenRouter Chat Completions client for tool-calling workflows."""

    provider_name = "openrouter"
    provider_label = "OpenRouter"
    base_url = OPENROUTER_BASE_URL
    supports_image_content = True
    assistant_extra_content_key = "openrouter"
    assistant_extra_fields = ("reasoning", "reasoning_details")
    logger = logger

    def __init__(
        self,
        model_id: str = DEFAULT_OPENROUTER_MODEL,
        *,
        thinking_level: str = "high",
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.thinking_level = thinking_level
        self.reasoning = _reasoning_config_from_thinking_level(thinking_level)
        self.max_tokens = _env_int(
            "OPENROUTER_MAX_TOKENS",
            _env_int("OPENROUTER_MAX_COMPLETION_TOKENS", DEFAULT_OPENROUTER_MAX_TOKENS),
        )
        self.context_tokens = _env_int(
            "OPENROUTER_CONTEXT_TOKENS",
            DEFAULT_OPENROUTER_CONTEXT_TOKENS,
        )
        self.output_safety_tokens = _env_int(
            "OPENROUTER_OUTPUT_SAFETY_TOKENS",
            DEFAULT_OPENROUTER_OUTPUT_SAFETY_TOKENS,
        )
        self.request_timeout_seconds = _env_float("OPENROUTER_REQUEST_TIMEOUT_SECONDS", 900.0)
        self.max_attempts = max(1, int(_env_float("OPENROUTER_MAX_ATTEMPTS", 4)))
        self.retry_base_seconds = _env_float("OPENROUTER_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("OPENROUTER_RETRY_MAX_SECONDS", 20.0)

        if dry_run:
            self._client = None
            self._client_is_async = False
            return

        api_key = openrouter_api_key_from_env()
        if not api_key:
            raise ValueError(
                "OpenRouter credentials not found. Set OPENROUTER_API_KEY or OPENROUTER_API_KEYS."
            )

        default_headers = _openrouter_default_headers()
        extra_kwargs: dict[str, Any] = {}
        if default_headers:
            extra_kwargs["default_headers"] = default_headers
        self._client, self._client_is_async = create_openai_compatible_client(
            provider_label="OpenRouter",
            api_key=api_key,
            base_url=OPENROUTER_BASE_URL,
            timeout_seconds=self.request_timeout_seconds,
            extra_client_kwargs=extra_kwargs,
        )

    def _chat_extra_body(self) -> dict[str, Any]:
        return {"reasoning": self.reasoning}

    def _response_extra_content(self, message: Any) -> tuple[str | None, dict[str, Any] | None]:
        reasoning = _get(message, "reasoning")
        reasoning_details = _serialize_json_value(_get(message, "reasoning_details"))

        thought_summary = _reasoning_details_summary(reasoning_details)
        if not thought_summary and isinstance(reasoning, str):
            thought_summary = reasoning.strip()

        extra_content = None
        if reasoning_details or reasoning:
            extra_content = {
                "openrouter": {
                    "reasoning": reasoning,
                    "reasoning_details": reasoning_details,
                }
            }
        return thought_summary or None, extra_content

    def _should_retry_exception(self, exc: BaseException) -> bool:
        return _should_retry_openrouter_exception(exc)


def _reasoning_config_from_thinking_level(thinking_level: str) -> dict[str, Any]:
    level = reasoning_level_alias(thinking_level)
    if level in {"0", "false", "none", "off", "disabled"}:
        return {"enabled": False}
    reasoning: dict[str, Any] = {"enabled": True}
    if level in {"minimal", "low", "medium", "high", "xhigh"}:
        reasoning["effort"] = level
    return reasoning


def _reasoning_details_summary(reasoning_details: Any) -> str:
    if not isinstance(reasoning_details, list):
        return ""
    fragments: list[str] = []
    for item in reasoning_details:
        if not isinstance(item, dict):
            continue
        text = item.get("summary") or item.get("text")
        if isinstance(text, str) and text.strip():
            fragments.append(text.strip())
    return "\n".join(fragments).strip()


def _openrouter_default_headers() -> dict[str, str]:
    headers: dict[str, str] = {}
    referer = os.environ.get("OPENROUTER_HTTP_REFERER") or os.environ.get("OPENROUTER_SITE_URL")
    title = os.environ.get("OPENROUTER_APP_TITLE") or os.environ.get("OPENROUTER_SITE_NAME")
    if referer and referer.strip():
        headers["HTTP-Referer"] = referer.strip()
    if title and title.strip():
        headers["X-OpenRouter-Title"] = title.strip()
    return headers


def _should_retry_openrouter_exception(exc: BaseException) -> bool:
    return should_retry_transient_http_exception(exc)
