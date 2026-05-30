"""
DeepSeek LLM wrapper using the OpenAI-compatible Chat Completions API.

This module intentionally avoids importing `openai` at import-time so the rest
of the repo can be used without DeepSeek/OpenAI SDK credentials installed.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import random
import uuid
from typing import Any

from agent.providers.base import (
    ContextWindowPressure,
    PrepareRequestResult,
    build_context_window_pressure,
)
from articraft.values import reasoning_level_alias

logger = logging.getLogger(__name__)

DEFAULT_DEEPSEEK_MODEL = "deepseek-v4-pro"
DEEPSEEK_BASE_URL = "https://api.deepseek.com/v1"
DEFAULT_DEEPSEEK_CONTEXT_TOKENS = 1_000_000
DEFAULT_DEEPSEEK_MAX_TOKENS = 8_192
DEFAULT_DEEPSEEK_OUTPUT_SAFETY_TOKENS = 1_024


def deepseek_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    values = os.environ if env is None else env
    key = values.get("DEEPSEEK_API_KEY")
    return key.strip() if key and key.strip() else None


def _thinking_config_from_thinking_level(thinking_level: str) -> dict[str, Any]:
    level = reasoning_level_alias(thinking_level)
    if level in {"0", "false", "none", "off", "disabled"}:
        return {"type": "disabled"}
    thinking: dict[str, Any] = {"type": "enabled"}
    if level in {"minimal", "low", "medium", "high", "xhigh"}:
        thinking["effort"] = level
    return thinking


class DeepSeekLLM:
    """DeepSeek Chat Completions client for tool-calling workflows."""

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
        else:
            api_key = deepseek_api_key_from_env()
            if not api_key:
                raise ValueError("DeepSeek credentials not found. Set DEEPSEEK_API_KEY.")

            client_kwargs: dict[str, Any] = {
                "api_key": api_key,
                "base_url": DEEPSEEK_BASE_URL,
                "max_retries": 0,
            }
            if self.request_timeout_seconds and self.request_timeout_seconds > 0:
                client_kwargs["timeout"] = float(self.request_timeout_seconds)

            self._client: Any
            self._client_is_async: bool
            try:
                from openai import AsyncOpenAI  # type: ignore

                self._client = AsyncOpenAI(**client_kwargs)
                self._client_is_async = True
            except Exception:
                try:
                    from openai import OpenAI  # type: ignore
                except Exception as exc:  # pragma: no cover
                    raise RuntimeError(
                        "DeepSeek provider selected but the `openai` package is not installed. "
                        "Install it (e.g. `uv add openai`), then try again."
                    ) from exc

                self._client = OpenAI(**client_kwargs)
                self._client_is_async = False

    # ── ProviderClient protocol ─────────────────────────────────────────────

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
        payload["base_url"] = DEEPSEEK_BASE_URL
        return payload

    def context_window_pressure(self, usage: dict[str, int]) -> ContextWindowPressure:
        return build_context_window_pressure(
            provider="deepseek",
            usage=usage,
            max_context_tokens=DEFAULT_DEEPSEEK_CONTEXT_TOKENS,
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
            raise RuntimeError("DeepSeek transport is unavailable in dry_run mode")

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
            should_retry=_should_retry_deepseek_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=logger,
            context="deepseek[chat.completions]",
        )
        return self._convert_response(response)

    async def close(self) -> None:
        client = getattr(self, "_client", None)
        if client is None:
            return None
        close_method = getattr(client, "close", None)
        if close_method is None:
            return None
        result = close_method()
        if asyncio.iscoroutine(result):
            await result
        return None

    # ── Chat Completions helpers ──────────────────────────────────────────

    async def _chat_completion(self, request_payload: dict[str, Any]) -> Any:
        payload = dict(request_payload)
        extra_body = payload.pop("extra_body", None)
        if self._client_is_async:
            if extra_body is None:
                return await self._client.chat.completions.create(**payload)
            return await self._client.chat.completions.create(**payload, extra_body=extra_body)
        if extra_body is None:
            return await asyncio.to_thread(self._client.chat.completions.create, **payload)
        return await asyncio.to_thread(
            self._client.chat.completions.create, **payload, extra_body=extra_body
        )

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
            "extra_body": {"thinking": self.thinking},
        }
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

    def _convert_response(self, response: Any) -> dict[str, Any]:
        choice = _first_choice(response)
        message = _get(choice, "message")
        if message is None:
            return {}

        content = _get(message, "content")
        tool_calls = _serialize_tool_calls(_get(message, "tool_calls"))
        reasoning_content = _get(message, "reasoning_content")
        usage = _extract_usage(response)

        result: dict[str, Any] = {
            "content": content if isinstance(content, str) else "",
            "tool_calls": tool_calls,
        }
        if isinstance(reasoning_content, str) and reasoning_content.strip():
            result["thought_summary"] = reasoning_content.strip()
            result["extra_content"] = {
                "deepseek": {
                    "reasoning_content": reasoning_content,
                }
            }
        if usage:
            result["usage"] = usage
        return result


# ── Message conversion helpers ────────────────────────────────────────────


def _convert_chat_messages(messages: list[dict]) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for message in messages:
        if not isinstance(message, dict):
            continue
        role = message.get("role")
        if role == "user":
            converted.append({"role": "user", "content": _convert_message_content(message)})
            continue
        if role == "assistant":
            assistant: dict[str, Any] = {"role": "assistant"}
            content = message.get("content")
            assistant["content"] = content if isinstance(content, str) else None
            tool_calls = message.get("tool_calls")
            if tool_calls:
                assistant["tool_calls"] = tool_calls
            extra_content = message.get("extra_content")
            if isinstance(extra_content, dict):
                deepseek = extra_content.get("deepseek")
                if isinstance(deepseek, dict):
                    reasoning_content = deepseek.get("reasoning_content")
                    if reasoning_content:
                        assistant["reasoning_content"] = reasoning_content
            converted.append(assistant)
            continue
        if role == "tool":
            tool_message: dict[str, Any] = {
                "role": "tool",
                "tool_call_id": message.get("tool_call_id") or message.get("call_id"),
                "content": message.get("content") or "",
            }
            name = message.get("name")
            if isinstance(name, str) and name:
                tool_message["name"] = name
            converted.append(tool_message)
    return converted


def _convert_message_content(message: dict[str, Any]) -> Any:
    content = message.get("content")
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return ""

    parts: list[dict[str, Any]] = []
    for part in content:
        if not isinstance(part, dict):
            continue
        part_type = part.get("type")
        if part_type in {"input_text", "text"}:
            text = part.get("text")
            if isinstance(text, str) and text:
                parts.append({"type": "text", "text": text})
    return parts or ""


def _convert_tools(tools: list[dict]) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for tool in tools or []:
        if not isinstance(tool, dict) or tool.get("type") != "function":
            continue
        func = tool.get("function") if isinstance(tool.get("function"), dict) else None
        if not func:
            continue
        converted.append(
            {
                "type": "function",
                "function": {
                    "name": func.get("name", ""),
                    "description": func.get("description", ""),
                    "parameters": func.get("parameters") or {"type": "object"},
                },
            }
        )
    return converted


def _first_choice(response: Any) -> Any:
    choices = _get(response, "choices")
    if isinstance(choices, list) and choices:
        return choices[0]
    return None


def _serialize_tool_calls(tool_calls: Any) -> list[dict[str, Any]]:
    if not tool_calls:
        return []
    serialized: list[dict[str, Any]] = []
    for tool_call in tool_calls:
        item = _serialize_json_value(tool_call)
        if not isinstance(item, dict):
            continue
        call_id = item.get("id") or f"call_{uuid.uuid4().hex}"
        function = item.get("function") if isinstance(item.get("function"), dict) else {}
        serialized.append(
            {
                "id": call_id,
                "type": item.get("type") or "function",
                "function": {
                    "name": str(function.get("name") or ""),
                    "arguments": str(function.get("arguments") or ""),
                },
            }
        )
    return serialized


def _extract_usage(response: Any) -> dict[str, int] | None:
    usage = _get(response, "usage")
    if usage is None:
        return None

    prompt_tokens = _get(usage, "prompt_tokens")
    completion_tokens = _get(usage, "completion_tokens")
    total_tokens = _get(usage, "total_tokens")
    cached_tokens = _get(usage, "cached_tokens")
    if not isinstance(cached_tokens, int):
        details = _get(usage, "prompt_tokens_details")
        cached_tokens = _get(details, "cached_tokens") if details is not None else None
    completion_details = _get(usage, "completion_tokens_details")
    reasoning_tokens = (
        _get(completion_details, "reasoning_tokens") if completion_details is not None else None
    )

    cleaned: dict[str, int] = {}
    if isinstance(prompt_tokens, int):
        cleaned["prompt_tokens"] = prompt_tokens
    if isinstance(completion_tokens, int):
        cleaned["candidates_tokens"] = completion_tokens
    if isinstance(total_tokens, int):
        cleaned["total_tokens"] = total_tokens
    if isinstance(cached_tokens, int):
        cleaned["cached_tokens"] = cached_tokens
    if isinstance(reasoning_tokens, int):
        cleaned["reasoning_tokens"] = reasoning_tokens
    return cleaned or None


def _serialize_json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list):
        return [_serialize_json_value(item) for item in value]
    if isinstance(value, tuple):
        return [_serialize_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _serialize_json_value(child) for key, child in value.items()}
    dump = getattr(value, "model_dump", None)
    if callable(dump):
        return dump(mode="json", exclude_none=True, warnings="none")
    dump_json = getattr(value, "model_dump_json", None)
    if callable(dump_json):
        return json.loads(dump_json(exclude_none=True))
    attrs = getattr(value, "__dict__", None)
    if isinstance(attrs, dict):
        return {
            str(key): _serialize_json_value(child)
            for key, child in attrs.items()
            if not key.startswith("_")
        }
    return str(value)


def _estimate_prompt_tokens(payload: dict[str, Any]) -> int:
    messages = payload.get("messages")
    tools = payload.get("tools")
    text_size = _json_text_size(messages) + _json_text_size(tools)
    structural_overhead = 128
    if isinstance(messages, list):
        structural_overhead += len(messages) * 16
    if isinstance(tools, list):
        structural_overhead += len(tools) * 128
    return max(1, (text_size + 2) // 3 + structural_overhead)


def _json_text_size(value: Any) -> int:
    if value is None:
        return 0
    try:
        return len(json.dumps(value, ensure_ascii=False, separators=(",", ":")))
    except Exception:
        return len(str(value))


def _get(obj: Any, field: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(field)
    return getattr(obj, field, None)


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


# ── Retry helpers ─────────────────────────────────────────────────────────


def _extract_http_status(exc: BaseException) -> int | None:
    status = getattr(exc, "status_code", None) or getattr(exc, "status", None)
    if isinstance(status, int) and 100 <= status <= 599:
        return status
    response = getattr(exc, "response", None)
    if response is not None:
        status = getattr(response, "status_code", None) or getattr(response, "status", None)
        if isinstance(status, int) and 100 <= status <= 599:
            return status
    return None


def _should_retry_deepseek_exception(exc: BaseException) -> bool:
    if isinstance(exc, (asyncio.TimeoutError, TimeoutError)):
        return True
    if isinstance(exc, json.JSONDecodeError):
        return True
    status = _extract_http_status(exc)
    if status is not None:
        if status in {408, 409, 425, 429, 500, 502, 503, 504}:
            return True
        if 400 <= status < 500:
            return False
        if status >= 500:
            return True
    message = str(exc).lower()
    return any(
        needle in message
        for needle in (
            "timeout",
            "timed out",
            "connection error",
            "connection reset",
            "connection aborted",
            "server disconnected",
            "protocol error",
            "temporarily unavailable",
            "rate limit",
            "bad gateway",
            "service unavailable",
        )
    )


def _format_retry_exception(exc: BaseException) -> str:
    status = _extract_http_status(exc)
    message = str(exc).strip()
    summary = type(exc).__name__
    if status is not None:
        summary += f" (HTTP {status})"
    if message:
        return f"{summary}: {message}"
    return f"{summary}: {repr(exc)}"


async def _async_retry(
    operation: Any,
    *,
    max_attempts: int,
    should_retry: Any,
    base_delay: float,
    max_delay: float,
    logger: logging.Logger,
    context: str,
    sleep_fn: Any = asyncio.sleep,
    rng: Any = random.random,
) -> Any:
    if max_attempts <= 0:
        raise ValueError("max_attempts must be >= 1")

    attempt = 0
    while True:
        try:
            return await operation()
        except Exception as exc:
            attempt += 1
            if attempt >= max_attempts or not should_retry(exc):
                raise

            cap = min(max_delay, base_delay * (2 ** (attempt - 1)))
            delay = max(0.0, float(rng()) * cap)
            logger.warning(
                "%s failed (attempt %s/%s), retrying in %.2fs: %s",
                context,
                attempt,
                max_attempts,
                delay,
                _format_retry_exception(exc),
            )
            await sleep_fn(delay)
