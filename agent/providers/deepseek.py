"""
DeepSeek LLM wrapper using the OpenAI-compatible Chat Completions API.

This module intentionally avoids importing `openai` at import-time so the rest
of the repo can be used without DeepSeek/OpenAI SDK credentials installed.
"""

from __future__ import annotations

import logging
from typing import Any

from agent.providers import _shared as provider_shared
from agent.providers._shared import (
    create_openai_compatible_client,
    get_value,
    random_env_key,
    should_retry_transient_http_exception,
)
from agent.providers._shared import (
    env_float as _env_float,
)
from agent.providers._shared import (
    env_int as _env_int,
)
from agent.providers.chat_completions import (
    OpenAICompatibleChatCompletionsMixin,
)
from agent.providers.chat_completions import (
    convert_chat_messages as _base_convert_chat_messages,
)
from articraft.values import reasoning_level_alias

logger = logging.getLogger(__name__)
_async_retry = provider_shared.async_retry

DEFAULT_DEEPSEEK_MODEL = "deepseek-v4-pro"
DEEPSEEK_BASE_URL = "https://api.deepseek.com/v1"
DEFAULT_DEEPSEEK_CONTEXT_TOKENS = 1_000_000
DEFAULT_DEEPSEEK_MAX_TOKENS = 8_192
DEFAULT_DEEPSEEK_OUTPUT_SAFETY_TOKENS = 1_024


def deepseek_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    return random_env_key(primary_name="DEEPSEEK_API_KEY", pool_name="DEEPSEEK_API_KEY", env=env)


def _thinking_config_from_thinking_level(thinking_level: str) -> dict[str, str]:
    level = reasoning_level_alias(thinking_level)
    if level in {"0", "false", "none", "off", "disabled"}:
        return {"type": "disabled"}
    return {"type": "enabled"}


def _reasoning_effort_from_thinking_level(thinking_level: str) -> str | None:
    level = reasoning_level_alias(thinking_level)
    if level in {"0", "false", "none", "off", "disabled"}:
        return None
    if level in {"xhigh", "max"}:
        return "max"
    if level in {"minimal", "low", "medium", "high"}:
        return "high"
    return None


class DeepSeekLLM(OpenAICompatibleChatCompletionsMixin):
    """DeepSeek Chat Completions client for tool-calling workflows."""

    provider_name = "deepseek"
    provider_label = "DeepSeek"
    base_url = DEEPSEEK_BASE_URL
    assistant_extra_content_key = "deepseek"
    assistant_extra_fields = ("reasoning_content",)
    logger = logger

    def __init__(
        self,
        model_id: str = DEFAULT_DEEPSEEK_MODEL,
        *,
        thinking_level: str = "high",
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.thinking_level = thinking_level
        self.thinking = _thinking_config_from_thinking_level(thinking_level)
        self.reasoning_effort = _reasoning_effort_from_thinking_level(thinking_level)
        self.max_tokens = _env_int("DEEPSEEK_MAX_TOKENS", DEFAULT_DEEPSEEK_MAX_TOKENS)
        self.context_tokens = _env_int("DEEPSEEK_CONTEXT_TOKENS", DEFAULT_DEEPSEEK_CONTEXT_TOKENS)
        self.output_safety_tokens = _env_int(
            "DEEPSEEK_OUTPUT_SAFETY_TOKENS", DEFAULT_DEEPSEEK_OUTPUT_SAFETY_TOKENS
        )
        self.request_timeout_seconds = _env_float("DEEPSEEK_REQUEST_TIMEOUT_SECONDS", 900.0)
        self.max_attempts = max(1, int(_env_float("DEEPSEEK_MAX_ATTEMPTS", 4)))
        self.retry_base_seconds = _env_float("DEEPSEEK_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("DEEPSEEK_RETRY_MAX_SECONDS", 20.0)

        if dry_run:
            self._client = None
            self._client_is_async = False
            return

        api_key = deepseek_api_key_from_env()
        if not api_key:
            raise ValueError("DeepSeek credentials not found. Set DEEPSEEK_API_KEY.")

        self._client, self._client_is_async = create_openai_compatible_client(
            provider_label="DeepSeek",
            api_key=api_key,
            base_url=DEEPSEEK_BASE_URL,
            timeout_seconds=self.request_timeout_seconds,
        )

    def _chat_extra_body(self) -> dict[str, Any]:
        return {"thinking": self.thinking}

    def _chat_payload_extra_fields(self) -> dict[str, Any]:
        if not self.reasoning_effort:
            return {}
        return {"reasoning_effort": self.reasoning_effort}

    def _response_extra_content(self, message: Any) -> tuple[str | None, dict[str, Any] | None]:
        reasoning_content = get_value(message, "reasoning_content")
        if not isinstance(reasoning_content, str) or not reasoning_content.strip():
            return None, None
        return (
            reasoning_content.strip(),
            {"deepseek": {"reasoning_content": reasoning_content}},
        )

    def _should_retry_exception(self, exc: BaseException) -> bool:
        return _should_retry_deepseek_exception(exc)


def _should_retry_deepseek_exception(exc: BaseException) -> bool:
    return should_retry_transient_http_exception(exc)


def _convert_chat_messages(messages: list[dict]) -> list[dict[str, Any]]:
    return _base_convert_chat_messages(
        messages,
        assistant_extra_content_key="deepseek",
        assistant_extra_fields=("reasoning_content",),
        include_images=False,
    )
