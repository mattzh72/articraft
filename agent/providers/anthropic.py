"""
Anthropic Claude Messages API provider.

This module uses the official Anthropic Python SDK for transport while keeping
the repository's provider-normalized payload and response handling. It preserves
Anthropic-native message blocks in conversation history because extended
thinking with tool use requires round-tripping thinking blocks exactly.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import mimetypes
import os
import random
import socket
import urllib.error
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from agent.providers.base import (
    ContextWindowPressure,
    PrepareRequestResult,
    build_context_window_pressure,
)
from articraft.values import reasoning_level_alias

try:
    from dotenv import load_dotenv  # type: ignore
except ImportError:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


logger = logging.getLogger(__name__)

ANTHROPIC_BASE_URL = "https://api.anthropic.com"
ANTHROPIC_VERSION = "2023-06-01"
DEFAULT_ANTHROPIC_MODEL = "claude-opus-4-7"
DEFAULT_ANTHROPIC_CONTEXT_TOKENS = 1_000_000
DEFAULT_ANTHROPIC_MAX_TOKENS = 16_000
DEFAULT_ANTHROPIC_OUTPUT_SAFETY_TOKENS = 1_024
ANTHROPIC_PROMPT_CACHE_ENV = "ANTHROPIC_PROMPT_CACHE"
ANTHROPIC_PROMPT_CACHE_TTL_ENV = "ANTHROPIC_PROMPT_CACHE_TTL"
_ADAPTIVE_THINKING_MODELS = (
    "claude-mythos-preview",
    "claude-opus-4-7",
    "claude-opus-4-6",
    "claude-sonnet-4-6",
)


def _load_cwd_dotenv_override() -> None:
    dotenv_path = Path.cwd() / ".env"
    if dotenv_path.exists():
        load_dotenv(dotenv_path=dotenv_path, override=True)


@dataclass(slots=True)
class AnthropicAPIError(RuntimeError):
    status_code: int | None
    error_type: str | None
    message: str
    request_id: str | None = None
    retry_after: float | None = None

    def __str__(self) -> str:
        parts = ["Anthropic API error"]
        if self.status_code is not None:
            parts.append(f"HTTP {self.status_code}")
        if self.error_type:
            parts.append(self.error_type)
        if self.request_id:
            parts.append(f"request_id={self.request_id}")
        return f"{' '.join(parts)}: {self.message}"


def anthropic_api_keys_from_env(env: dict[str, str] | None = None) -> list[str]:
    values = os.environ if env is None else env

    def _split(raw: str | None) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    primary_key = values.get("ANTHROPIC_API_KEY")
    if primary_key and primary_key.strip():
        keys.append(primary_key.strip())
    keys.extend(_split(values.get("ANTHROPIC_API_KEYS")))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)
    return unique_keys


def anthropic_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    keys = anthropic_api_keys_from_env(env)
    return random.choice(keys) if keys else None


class AnthropicLLM:
    """Anthropic Messages API client for tool-calling workflows."""

    def __init__(
        self,
        model_id: str = DEFAULT_ANTHROPIC_MODEL,
        *,
        thinking_level: str = "high",
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.thinking_level = thinking_level
        self.max_tokens = _env_int("ANTHROPIC_MAX_TOKENS", DEFAULT_ANTHROPIC_MAX_TOKENS)
        self.context_tokens = _env_int(
            "ANTHROPIC_CONTEXT_TOKENS",
            DEFAULT_ANTHROPIC_CONTEXT_TOKENS,
        )
        self.output_safety_tokens = _env_int(
            "ANTHROPIC_OUTPUT_SAFETY_TOKENS",
            DEFAULT_ANTHROPIC_OUTPUT_SAFETY_TOKENS,
        )
        self.request_timeout_seconds = _env_float("ANTHROPIC_REQUEST_TIMEOUT_SECONDS", 900.0)
        self.max_attempts = max(1, int(_env_float("ANTHROPIC_MAX_ATTEMPTS", 4)))
        self.retry_base_seconds = _env_float("ANTHROPIC_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("ANTHROPIC_RETRY_MAX_SECONDS", 20.0)
        self.base_url = os.environ.get("ANTHROPIC_BASE_URL", ANTHROPIC_BASE_URL).rstrip("/")
        self.api_key: str | None = None
        self._client: Any = None
        if not dry_run:
            _load_cwd_dotenv_override()
            self.api_key = anthropic_api_key_from_env()
            if not self.api_key:
                raise ValueError(
                    "Anthropic credentials not found. Set ANTHROPIC_API_KEY or ANTHROPIC_API_KEYS."
                )
            self._client = self._create_sdk_client(self.api_key)

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        payload = self._build_messages_payload(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
        )
        payload["base_url"] = self.base_url
        payload["anthropic_version"] = ANTHROPIC_VERSION
        return payload

    def context_window_pressure(self, usage: dict[str, int]) -> ContextWindowPressure:
        return build_context_window_pressure(
            provider="anthropic",
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
            raise RuntimeError("Anthropic transport is unavailable in dry_run mode")

        payload = self._build_messages_payload(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
        )

        async def _request_once() -> Any:
            return await self._create_message(payload)

        response = await _async_retry(
            _request_once,
            max_attempts=self.max_attempts,
            should_retry=_should_retry_anthropic_exception,
            retry_after=_retry_after_from_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=logger,
            context="anthropic[messages]",
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

    def _create_sdk_client(self, api_key: str) -> Any:
        try:
            from anthropic import AsyncAnthropic  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "Anthropic provider selected but the `anthropic` package is not installed. "
                "Install it (e.g. `uv add anthropic`), then try again."
            ) from exc

        client_kwargs: dict[str, Any] = {
            "api_key": api_key,
            "base_url": self.base_url,
            # Disable SDK-managed retries so request timing and retry policy are
            # controlled by this wrapper rather than hidden nested backoff.
            "max_retries": 0,
        }
        if self.request_timeout_seconds and self.request_timeout_seconds > 0:
            client_kwargs["timeout"] = float(self.request_timeout_seconds)
        else:
            client_kwargs["timeout"] = None
        return AsyncAnthropic(**client_kwargs)

    async def _create_message(self, payload: dict[str, Any]) -> Any:
        if self._client is None:
            raise RuntimeError("Anthropic transport is unavailable in dry_run mode")
        try:
            return await self._client.messages.create(**payload)
        except Exception as exc:
            normalized = _api_error_from_sdk_exception(exc)
            if normalized is exc:
                raise
            raise normalized from exc

    def _build_messages_payload(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        converted_messages = _convert_messages(messages)
        converted_tools = _convert_tools(tools)
        cache_control = _prompt_cache_control()
        if cache_control is not None:
            _apply_prompt_cache_controls(
                messages=converted_messages,
                tools=converted_tools,
                cache_control=cache_control,
            )
        payload: dict[str, Any] = {
            "model": self.model_id,
            "max_tokens": self._request_max_tokens(
                system_prompt=system_prompt,
                messages=converted_messages,
                tools=converted_tools,
            ),
            "messages": converted_messages,
        }
        if system_prompt.strip():
            if cache_control is None:
                payload["system"] = system_prompt
            else:
                payload["system"] = [
                    {
                        "type": "text",
                        "text": system_prompt,
                        "cache_control": dict(cache_control),
                    }
                ]
        if converted_tools:
            payload["tools"] = converted_tools
        if cache_control is not None:
            payload["cache_control"] = dict(cache_control)
        thinking = _thinking_config_for_model(self.model_id, self.thinking_level)
        if thinking is not None:
            payload["thinking"] = thinking
        effort = _effort_from_thinking_level(self.thinking_level)
        if effort is not None:
            payload["output_config"] = {"effort": effort}
        return payload

    def _request_max_tokens(
        self,
        *,
        system_prompt: str,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> int:
        if self.max_tokens <= 0:
            return DEFAULT_ANTHROPIC_MAX_TOKENS
        if self.context_tokens <= 0:
            return self.max_tokens
        estimated_prompt_tokens = _estimate_prompt_tokens(
            {"system": system_prompt, "messages": messages, "tools": tools}
        )
        available = (
            self.context_tokens - estimated_prompt_tokens - max(0, self.output_safety_tokens)
        )
        if available <= 0:
            return min(self.max_tokens, 16)
        return max(1, min(self.max_tokens, available))

    def _convert_response(self, response: Any) -> dict[str, Any]:
        return _convert_response(response)


def _convert_messages(messages: list[dict]) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    index = 0
    while index < len(messages):
        message = messages[index]
        if not isinstance(message, dict):
            index += 1
            continue
        role = message.get("role")
        if role == "user":
            converted.append({"role": "user", "content": _convert_user_content(message)})
            index += 1
            continue
        if role == "assistant":
            content = _convert_assistant_content(message)
            if content:
                converted.append({"role": "assistant", "content": content})
            index += 1
            continue
        if role == "tool":
            tool_results: list[dict[str, Any]] = []
            while index < len(messages):
                tool_message = messages[index]
                if not isinstance(tool_message, dict) or tool_message.get("role") != "tool":
                    break
                result = _convert_tool_result(tool_message)
                if result:
                    tool_results.append(result)
                index += 1
            if tool_results:
                converted.append({"role": "user", "content": tool_results})
            continue
        index += 1
    return converted


def _convert_user_content(message: dict[str, Any]) -> str | list[dict[str, Any]]:
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
            continue
        if part_type == "input_image":
            image = _convert_image_part(part)
            if image:
                parts.append(image)
    return parts or ""


def _convert_assistant_content(message: dict[str, Any]) -> list[dict[str, Any]]:
    extra_content = message.get("extra_content")
    if isinstance(extra_content, dict):
        anthropic = extra_content.get("anthropic")
        if isinstance(anthropic, dict):
            raw_content = anthropic.get("content")
            if isinstance(raw_content, list) and raw_content:
                return [_serialize_json_value(block) for block in raw_content if block is not None]

    parts: list[dict[str, Any]] = []
    content = message.get("content")
    if isinstance(content, str) and content:
        parts.append({"type": "text", "text": content})
    for tool_call in message.get("tool_calls") or []:
        block = _tool_call_to_tool_use_block(tool_call)
        if block:
            parts.append(block)
    return parts


def _tool_call_to_tool_use_block(tool_call: Any) -> dict[str, Any] | None:
    if not isinstance(tool_call, dict):
        return None
    function = tool_call.get("function")
    if not isinstance(function, dict):
        return None
    name = function.get("name")
    if not isinstance(name, str) or not name:
        return None
    arguments = function.get("arguments")
    if isinstance(arguments, str):
        try:
            tool_input = json.loads(arguments) if arguments.strip() else {}
        except json.JSONDecodeError:
            tool_input = {"input": arguments}
    elif isinstance(arguments, dict):
        tool_input = arguments
    else:
        tool_input = {}
    return {
        "type": "tool_use",
        "id": str(tool_call.get("id") or f"toolu_{uuid.uuid4().hex}"),
        "name": name,
        "input": tool_input,
    }


def _convert_tool_result(message: dict[str, Any]) -> dict[str, Any] | None:
    tool_use_id = message.get("tool_call_id") or message.get("call_id")
    if not isinstance(tool_use_id, str) or not tool_use_id:
        return None
    content = message.get("content")
    result: dict[str, Any] = {
        "type": "tool_result",
        "tool_use_id": tool_use_id,
        "content": content if isinstance(content, str) else json.dumps(content),
    }
    if _tool_result_is_error(content):
        result["is_error"] = True
    return result


def _tool_result_is_error(content: Any) -> bool:
    if isinstance(content, str):
        try:
            payload = json.loads(content)
        except json.JSONDecodeError:
            return False
    elif isinstance(content, dict):
        payload = content
    else:
        return False
    error = payload.get("error") if isinstance(payload, dict) else None
    return isinstance(error, str) and bool(error.strip())


def _convert_tools(tools: list[dict]) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for tool in tools or []:
        if not isinstance(tool, dict) or tool.get("type") != "function":
            continue
        func = tool.get("function") if isinstance(tool.get("function"), dict) else None
        if not func:
            continue
        name = func.get("name")
        if not isinstance(name, str) or not name:
            continue
        converted.append(
            {
                "name": name,
                "description": func.get("description") or "",
                "input_schema": func.get("parameters") or {"type": "object"},
            }
        )
    return converted


def _prompt_cache_control() -> dict[str, str] | None:
    enabled = os.environ.get(ANTHROPIC_PROMPT_CACHE_ENV, "1").strip().lower()
    if enabled in {"0", "false", "no", "off", "disabled"}:
        return None

    cache_control = {"type": "ephemeral"}
    ttl = os.environ.get(ANTHROPIC_PROMPT_CACHE_TTL_ENV, "").strip().lower()
    if ttl in {"1h", "1hr", "1hour", "hour"}:
        cache_control["ttl"] = "1h"
    elif ttl in {"", "5m", "5min", "5mins", "5minute", "5minutes"}:
        pass
    else:
        logger.warning(
            "Ignoring unsupported %s=%r; expected 5m or 1h",
            ANTHROPIC_PROMPT_CACHE_TTL_ENV,
            ttl,
        )
    return cache_control


def _apply_prompt_cache_controls(
    *,
    messages: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    cache_control: dict[str, str],
) -> None:
    if tools:
        tools[-1]["cache_control"] = dict(cache_control)

    # The harness sends the reusable SDK documentation as the first user
    # message, followed by the per-record task prompt. Cache that stable prefix
    # explicitly so different records can share it; automatic caching then handles
    # the growing conversation after the variable task prompt.
    if messages and _is_workspace_docs_message(messages[0]):
        _mark_message_content_for_cache(messages[0], cache_control)


def _is_workspace_docs_message(message: dict[str, Any]) -> bool:
    if message.get("role") != "user":
        return False
    content = message.get("content")
    if isinstance(content, str):
        return "# Workspace Documentation (read-only)" in content
    if isinstance(content, list):
        return any(
            isinstance(part, dict)
            and part.get("type") == "text"
            and isinstance(part.get("text"), str)
            and "# Workspace Documentation (read-only)" in part["text"]
            for part in content
        )
    return False


def _mark_message_content_for_cache(
    message: dict[str, Any],
    cache_control: dict[str, str],
) -> None:
    content = message.get("content")
    if isinstance(content, str):
        if content.strip():
            message["content"] = [
                {"type": "text", "text": content, "cache_control": dict(cache_control)}
            ]
        return
    if not isinstance(content, list):
        return

    for part in reversed(content):
        if not isinstance(part, dict):
            continue
        if part.get("type") == "text" and isinstance(part.get("text"), str) and part["text"]:
            part["cache_control"] = dict(cache_control)
            return
        if part.get("type") in {"image", "document", "tool_result", "tool_use"}:
            part["cache_control"] = dict(cache_control)
            return


def _convert_image_part(part: dict[str, Any]) -> dict[str, Any] | None:
    image_url = part.get("image_url")
    image_path = part.get("image_path")
    if isinstance(image_url, str) and image_url:
        source = _image_url_to_source(image_url)
        return {"type": "image", "source": source} if source else None
    if isinstance(image_path, str) and image_path:
        return {"type": "image", "source": _image_path_to_base64_source(image_path)}
    return None


def _image_url_to_source(image_url: str) -> dict[str, Any] | None:
    if image_url.startswith(("http://", "https://")):
        return {"type": "url", "url": image_url}
    if image_url.startswith("data:"):
        header, sep, data = image_url.partition(",")
        if sep != ",":
            return None
        mime_type = header.removeprefix("data:").split(";", 1)[0] or "application/octet-stream"
        return {"type": "base64", "media_type": mime_type, "data": data}
    return None


def _image_path_to_base64_source(image_path: str) -> dict[str, Any]:
    path = Path(image_path).expanduser().resolve()
    mime_type, _ = mimetypes.guess_type(path.name)
    if not mime_type:
        mime_type = "application/octet-stream"
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return {"type": "base64", "media_type": mime_type, "data": data}


def _convert_response(response: Any) -> dict[str, Any]:
    content = _get(response, "content") or []
    text_fragments: list[str] = []
    thinking_fragments: list[str] = []
    tool_calls: list[dict[str, Any]] = []
    serialized_content: list[Any] = []

    if isinstance(content, list):
        for block in content:
            item = _serialize_json_value(block)
            if isinstance(item, dict):
                serialized_content.append(item)
                block_type = item.get("type")
                if block_type == "text":
                    text = item.get("text")
                    if isinstance(text, str) and text:
                        text_fragments.append(text)
                    continue
                if block_type in {"thinking", "redacted_thinking"}:
                    thinking = item.get("thinking") or item.get("summary")
                    if isinstance(thinking, str) and thinking.strip():
                        thinking_fragments.append(thinking.strip())
                    continue
                if block_type == "tool_use":
                    name = item.get("name")
                    tool_input = item.get("input")
                    tool_calls.append(
                        {
                            "id": str(item.get("id") or f"toolu_{uuid.uuid4().hex}"),
                            "type": "function",
                            "function": {
                                "name": str(name or ""),
                                "arguments": json.dumps(
                                    tool_input if isinstance(tool_input, dict) else {},
                                    ensure_ascii=False,
                                    separators=(",", ":"),
                                ),
                            },
                        }
                    )

    usage = _extract_usage(response)
    result: dict[str, Any] = {
        "content": "\n".join(text_fragments).strip(),
        "tool_calls": tool_calls,
    }
    if thinking_fragments:
        result["thought_summary"] = "\n".join(thinking_fragments).strip()
    if serialized_content:
        result["extra_content"] = {
            "anthropic": {
                "content": serialized_content,
                "stop_reason": _get(response, "stop_reason"),
                "stop_sequence": _get(response, "stop_sequence"),
            }
        }
    if usage:
        result["usage"] = usage
    return result


def _extract_usage(response: Any) -> dict[str, int] | None:
    usage = _get(response, "usage")
    if usage is None:
        return None

    input_tokens = _get(usage, "input_tokens")
    output_tokens = _get(usage, "output_tokens")
    cache_creation = _get(usage, "cache_creation_input_tokens")
    cache_read = _get(usage, "cache_read_input_tokens")
    cache_creation_detail = _get(usage, "cache_creation")

    prompt_tokens = sum(
        value for value in (input_tokens, cache_creation, cache_read) if isinstance(value, int)
    )
    cleaned: dict[str, int] = {}
    if prompt_tokens:
        cleaned["prompt_tokens"] = prompt_tokens
    if isinstance(output_tokens, int):
        cleaned["candidates_tokens"] = output_tokens
    if prompt_tokens or isinstance(output_tokens, int):
        cleaned["total_tokens"] = prompt_tokens + (
            output_tokens if isinstance(output_tokens, int) else 0
        )
    if isinstance(cache_read, int):
        cleaned["cached_tokens"] = cache_read
    if isinstance(cache_creation, int):
        cleaned["cache_creation_input_tokens"] = cache_creation
    if isinstance(cache_read, int):
        cleaned["cache_read_input_tokens"] = cache_read
    if cache_creation_detail is not None:
        cache_creation_5m = _get(cache_creation_detail, "ephemeral_5m_input_tokens")
        cache_creation_1h = _get(cache_creation_detail, "ephemeral_1h_input_tokens")
        if isinstance(cache_creation_5m, int):
            cleaned["cache_creation_5m_input_tokens"] = cache_creation_5m
        if isinstance(cache_creation_1h, int):
            cleaned["cache_creation_1h_input_tokens"] = cache_creation_1h
    return cleaned or None


def _thinking_config_for_model(model_id: str, thinking_level: str) -> dict[str, str] | None:
    level = reasoning_level_alias(thinking_level)
    if level in {"0", "false", "none", "off", "disabled"}:
        return None
    model_norm = model_id.strip().lower()
    if model_norm.startswith(_ADAPTIVE_THINKING_MODELS):
        return {"type": "adaptive"}
    return None


def _effort_from_thinking_level(thinking_level: str) -> str | None:
    level = reasoning_level_alias(thinking_level)
    if level in {"low", "medium", "high", "xhigh", "max"}:
        return level
    return None


def _api_error_from_sdk_exception(exc: BaseException) -> BaseException:
    if isinstance(exc, AnthropicAPIError):
        return exc

    try:
        import anthropic  # type: ignore
    except Exception:  # pragma: no cover
        return exc

    if isinstance(exc, anthropic.APIStatusError):
        body = getattr(exc, "body", None)
        error = body.get("error") if isinstance(body, dict) else None
        message = error.get("message") if isinstance(error, dict) else None
        error_type = getattr(exc, "type", None)
        response = getattr(exc, "response", None)
        headers = getattr(response, "headers", {}) if response is not None else {}
        request_id = getattr(exc, "request_id", None)
        if not isinstance(request_id, str):
            request_id = headers.get("request-id") if hasattr(headers, "get") else None
        return AnthropicAPIError(
            status_code=getattr(exc, "status_code", None),
            error_type=error_type if isinstance(error_type, str) else None,
            message=(
                message
                if isinstance(message, str) and message
                else (getattr(exc, "message", None) or str(exc))
            ),
            request_id=request_id,
            retry_after=(
                _parse_retry_after(headers.get("retry-after")) if hasattr(headers, "get") else None
            ),
        )

    if isinstance(exc, anthropic.APITimeoutError):
        return AnthropicAPIError(
            status_code=None,
            error_type="timeout_error",
            message=getattr(exc, "message", None) or str(exc),
        )

    if isinstance(exc, anthropic.APIConnectionError):
        return AnthropicAPIError(
            status_code=None,
            error_type="connection_error",
            message=getattr(exc, "message", None) or str(exc),
        )

    if isinstance(exc, anthropic.APIError):
        return AnthropicAPIError(
            status_code=None,
            error_type="api_error",
            message=getattr(exc, "message", None) or str(exc),
        )

    return exc


def _should_retry_anthropic_exception(exc: BaseException) -> bool:
    if isinstance(exc, (asyncio.TimeoutError, TimeoutError, socket.timeout)):
        return True
    if isinstance(exc, urllib.error.URLError):
        return True
    if isinstance(exc, json.JSONDecodeError):
        return True
    if isinstance(exc, AnthropicAPIError):
        if exc.error_type == "invalid_json_response":
            return True
        if exc.retry_after is not None:
            return True
        status = exc.status_code
        if status in {408, 409, 425, 429, 500, 502, 503, 504, 529}:
            return True
        if status is not None and 400 <= status < 500:
            return False
        if status is not None and status >= 500:
            return True
    message = str(exc).lower()
    return any(
        needle in message
        for needle in (
            "timeout",
            "timed out",
            "connection error",
            "connection reset",
            "server disconnected",
            "temporarily unavailable",
            "rate limit",
            "overloaded",
            "bad gateway",
            "service unavailable",
        )
    )


def _retry_after_from_exception(exc: BaseException) -> float | None:
    if isinstance(exc, AnthropicAPIError):
        return exc.retry_after
    return None


def _parse_retry_after(value: str | None) -> float | None:
    if not value:
        return None
    try:
        parsed = float(value.strip())
    except ValueError:
        return None
    return parsed if parsed >= 0 else None


def _format_retry_exception(exc: BaseException) -> str:
    if isinstance(exc, AnthropicAPIError):
        return str(exc)
    return f"{type(exc).__name__}: {str(exc).strip() or repr(exc)}"


async def _async_retry(
    operation: Any,
    *,
    max_attempts: int,
    should_retry: Any,
    retry_after: Any,
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

            explicit_delay = retry_after(exc)
            if isinstance(explicit_delay, (int, float)) and explicit_delay >= 0:
                delay = (
                    min(max_delay, float(explicit_delay))
                    if max_delay > 0
                    else float(explicit_delay)
                )
            else:
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
            if delay > 0:
                await sleep_fn(delay)
            else:
                await sleep_fn(0)


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
    text_size = _json_text_size(payload)
    structural_overhead = 256
    messages = payload.get("messages")
    tools = payload.get("tools")
    if isinstance(messages, list):
        structural_overhead += len(messages) * 24
    if isinstance(tools, list):
        structural_overhead += len(tools) * 128
    return max(1, (text_size + 3) // 4 + structural_overhead)


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
