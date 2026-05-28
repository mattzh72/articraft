"""
DashScope OpenAI-compatible Chat Completions provider.

DashScope's compatible endpoint uses the OpenAI SDK surface, but Qwen models
are served through chat.completions rather than OpenAI's Responses API.
"""

from __future__ import annotations

import asyncio
import copy
import logging
import os
import random
from pathlib import Path
from typing import Any

from agent.providers.base import (
    ContextWindowPressure,
    PrepareRequestResult,
    build_context_window_pressure,
)
from agent.providers.openrouter import (
    OpenRouterLLM,
    _async_retry,
    _convert_chat_messages,
    _convert_tools,
    _estimate_prompt_tokens,
    _should_retry_openrouter_exception,
)

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


DEFAULT_DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_DASHSCOPE_MODEL = "qwen3.6-flash"
DEFAULT_DASHSCOPE_CONTEXT_TOKENS = 262_144
DEFAULT_DASHSCOPE_MAX_TOKENS = 0
DEFAULT_DASHSCOPE_OUTPUT_SAFETY_TOKENS = 1_024
logger = logging.getLogger(__name__)


def _load_cwd_dotenv_override() -> None:
    dotenv_path = Path.cwd() / ".env"
    if dotenv_path.exists():
        # Do not clobber already-exported credentials from the caller shell/script.
        load_dotenv(dotenv_path=dotenv_path, override=False)


def dashscope_api_keys_from_env(env: dict[str, str] | None = None) -> list[str]:
    values = os.environ if env is None else env

    def _split(raw: str | None) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    primary_key = values.get("DASHSCOPE_API_KEY")
    if primary_key and primary_key.strip():
        keys.append(primary_key.strip())
    keys.extend(_split(values.get("DASHSCOPE_API_KEYS")))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)
    return unique_keys


def dashscope_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    keys = dashscope_api_keys_from_env(env)
    return random.choice(keys) if keys else None


class DashScopeLLM(OpenRouterLLM):
    """DashScope Chat Completions client for Qwen tool-calling workflows."""

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

        client_kwargs: dict[str, Any] = {
            "api_key": api_key,
            "base_url": self.base_url,
            "max_retries": 0,
        }
        if self.request_timeout_seconds and self.request_timeout_seconds > 0:
            client_kwargs["timeout"] = float(self.request_timeout_seconds)

        try:
            from openai import AsyncOpenAI  # type: ignore

            self._client = AsyncOpenAI(**client_kwargs)
            self._client_is_async = True
        except Exception:
            try:
                from openai import OpenAI  # type: ignore
            except Exception as exc:  # pragma: no cover
                raise RuntimeError(
                    "DashScope provider selected but the `openai` package is not installed. "
                    "Install it (e.g. `uv add openai`), then try again."
                ) from exc

            self._client = OpenAI(**client_kwargs)
            self._client_is_async = False

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        payload = self._build_chat_payload(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
        )
        payload["base_url"] = self.base_url
        return payload

    def context_window_pressure(self, usage: dict[str, int]) -> ContextWindowPressure:
        return build_context_window_pressure(
            provider="dashscope",
            usage=usage,
            max_context_tokens=self.context_tokens,
        )

    async def prepare_next_request(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
        completed_turns: int,
        consecutive_compile_failure_count: int = 0,
        last_compile_failure_sig: str | None = None,
    ) -> PrepareRequestResult:
        return PrepareRequestResult()

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        if self._client is None:
            raise RuntimeError("DashScope transport is unavailable in dry_run mode")

        request_payload = self._build_chat_payload(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
        )

        async def _request_once() -> Any:
            request_coro = self._chat_completion(request_payload)
            if self.request_timeout_seconds and self.request_timeout_seconds > 0:
                return await asyncio.wait_for(
                    request_coro,
                    timeout=float(self.request_timeout_seconds),
                )
            return await request_coro

        response = await _async_retry(
            _request_once,
            max_attempts=self.max_attempts,
            should_retry=_should_retry_openrouter_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=logger,
            context="dashscope[chat.completions]",
        )
        return self._convert_response(response)

    def _build_chat_payload(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        chat_messages: list[dict[str, Any]] = []
        if system_prompt.strip():
            chat_messages.append({"role": "system", "content": system_prompt})
        chat_messages.extend(_convert_chat_messages(messages))

        payload: dict[str, Any] = {
            "model": self.model_id,
            "messages": chat_messages,
        }
        if self.extra_body:
            payload["extra_body"] = copy.deepcopy(self.extra_body)
        converted_tools = _convert_tools(tools)
        if converted_tools:
            payload["tools"] = converted_tools
        max_tokens = self._request_max_tokens(payload)
        if max_tokens > 0:
            payload["max_tokens"] = max_tokens
        return payload

    def _request_max_tokens(self, payload: dict[str, Any]) -> int:
        if self.max_tokens <= 0:
            return 0
        if self.context_tokens <= 0:
            return self.max_tokens
        estimated_prompt_tokens = _estimate_prompt_tokens(payload)
        available = (
            self.context_tokens - estimated_prompt_tokens - max(0, self.output_safety_tokens)
        )
        if available <= 0:
            return min(self.max_tokens, 16)
        return min(self.max_tokens, available)


def _dashscope_extra_body(thinking_level: str) -> dict[str, Any]:
    raw = os.environ.get("DASHSCOPE_ENABLE_THINKING")
    if raw is None:
        enable_thinking = True
    else:
        enable_thinking = raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}
    extra: dict[str, Any] = {"enable_thinking": enable_thinking}

    budget = os.environ.get("DASHSCOPE_THINKING_BUDGET")
    if budget and budget.strip():
        try:
            extra["thinking_budget"] = int(budget.strip().replace("_", ""))
        except ValueError:
            pass
    return extra


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw.strip().replace("_", ""))
    except Exception:
        return default
