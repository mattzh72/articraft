"""
DashScope OpenAI-compatible Chat Completions provider.

DashScope's compatible endpoint uses the OpenAI SDK surface, but Qwen models
are served through chat.completions rather than OpenAI's Responses API.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

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
from agent.providers.chat_completions import (
    OpenAICompatibleChatCompletionsMixin,
)
from agent.providers.chat_completions import (
    convert_chat_messages as _base_convert_chat_messages,
)
from articraft.values import reasoning_level_alias

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


DEFAULT_DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_DASHSCOPE_MODEL = "qwen3.6-flash"
DEFAULT_DASHSCOPE_CONTEXT_TOKENS = 1_000_000
DEFAULT_DASHSCOPE_MAX_TOKENS = 64_000
DEFAULT_DASHSCOPE_OUTPUT_SAFETY_TOKENS = 1_024
_DASHSCOPE_THINKING_BUDGET_BY_LEVEL = {
    "low": 16_000,
    "medium": 32_000,
}
logger = logging.getLogger(__name__)


def _load_cwd_dotenv_override() -> None:
    dotenv_path = Path.cwd() / ".env"
    if dotenv_path.exists():
        # Do not clobber already-exported credentials from the caller shell/script.
        load_dotenv(dotenv_path=dotenv_path, override=False)


def dashscope_api_keys_from_env(env: dict[str, str] | None = None) -> list[str]:
    return env_keys_from_values(
        primary_name="DASHSCOPE_API_KEY",
        pool_name="DASHSCOPE_API_KEYS",
        env=env,
    )


def dashscope_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    return random_env_key(
        primary_name="DASHSCOPE_API_KEY",
        pool_name="DASHSCOPE_API_KEYS",
        env=env,
    )


class DashScopeLLM(OpenAICompatibleChatCompletionsMixin):
    """DashScope Chat Completions client for Qwen tool-calling workflows."""

    provider_name = "dashscope"
    provider_label = "DashScope"
    supports_image_content = True
    assistant_extra_content_key = "dashscope"
    assistant_extra_fields = ("reasoning_content",)
    logger = logger

    def __init__(
        self,
        model_id: str | None = None,
        *,
        thinking_level: str = "high",
        dry_run: bool = False,
    ):
        _load_cwd_dotenv_override()
        self.model_id = model_id or os.environ.get("DASHSCOPE_MODEL") or DEFAULT_DASHSCOPE_MODEL
        self.base_url = (os.environ.get("DASHSCOPE_BASE_URL") or DEFAULT_DASHSCOPE_BASE_URL).rstrip(
            "/"
        )
        self.thinking_level = thinking_level
        self.max_tokens = _env_int(
            "DASHSCOPE_MAX_TOKENS",
            _env_int("DASHSCOPE_MAX_COMPLETION_TOKENS", DEFAULT_DASHSCOPE_MAX_TOKENS),
        )
        self.context_tokens = _env_int("DASHSCOPE_CONTEXT_TOKENS", DEFAULT_DASHSCOPE_CONTEXT_TOKENS)
        self.output_safety_tokens = _env_int(
            "DASHSCOPE_OUTPUT_SAFETY_TOKENS",
            DEFAULT_DASHSCOPE_OUTPUT_SAFETY_TOKENS,
        )
        self.request_timeout_seconds = _env_float("DASHSCOPE_REQUEST_TIMEOUT_SECONDS", 900.0)
        self.max_attempts = max(1, int(_env_float("DASHSCOPE_MAX_ATTEMPTS", 4)))
        self.retry_base_seconds = _env_float("DASHSCOPE_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("DASHSCOPE_RETRY_MAX_SECONDS", 20.0)
        self.extra_body = _dashscope_extra_body(thinking_level)

        if dry_run:
            self._client = None
            self._client_is_async = False
            return

        api_key = dashscope_api_key_from_env()
        if not api_key:
            raise ValueError(
                "DashScope credentials not found. Set DASHSCOPE_API_KEY or DASHSCOPE_API_KEYS."
            )

        self._client, self._client_is_async = create_openai_compatible_client(
            provider_label="DashScope",
            api_key=api_key,
            base_url=self.base_url,
            timeout_seconds=self.request_timeout_seconds,
        )

    def _chat_extra_body(self) -> dict[str, Any]:
        return dict(self.extra_body)

    def _response_extra_content(self, message: Any) -> tuple[str | None, dict[str, Any] | None]:
        from agent.providers._shared import get_value

        reasoning_content = get_value(message, "reasoning_content")
        if not isinstance(reasoning_content, str) or not reasoning_content.strip():
            return None, None
        return (
            reasoning_content.strip(),
            {"dashscope": {"reasoning_content": reasoning_content}},
        )

    def _should_retry_exception(self, exc: BaseException) -> bool:
        return _should_retry_dashscope_exception(exc)


def _dashscope_extra_body(thinking_level: str) -> dict[str, Any]:
    level = reasoning_level_alias(thinking_level)
    raw = os.environ.get("DASHSCOPE_ENABLE_THINKING")
    if raw is None:
        enable_thinking = level not in {"0", "false", "none", "off", "disabled"}
    else:
        enable_thinking = raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}
    extra: dict[str, Any] = {"enable_thinking": enable_thinking}

    if enable_thinking:
        budget = os.environ.get("DASHSCOPE_THINKING_BUDGET")
        if budget and budget.strip():
            try:
                extra["thinking_budget"] = int(budget.strip().replace("_", ""))
            except ValueError:
                pass
        elif level in _DASHSCOPE_THINKING_BUDGET_BY_LEVEL:
            extra["thinking_budget"] = _DASHSCOPE_THINKING_BUDGET_BY_LEVEL[level]
    return extra


def _should_retry_dashscope_exception(exc: BaseException) -> bool:
    return should_retry_transient_http_exception(exc)


def _convert_dashscope_chat_messages(messages: list[dict]) -> list[dict[str, Any]]:
    return _base_convert_chat_messages(
        messages,
        assistant_extra_content_key="dashscope",
        assistant_extra_fields=("reasoning_content",),
        include_images=True,
    )
