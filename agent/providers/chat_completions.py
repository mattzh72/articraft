from __future__ import annotations

import asyncio
import base64
import copy
import logging
import mimetypes
import uuid
from pathlib import Path
from typing import Any

from agent.providers._shared import (
    async_retry,
    estimate_prompt_tokens,
    get_value,
    serialize_json_value,
    should_retry_transient_http_exception,
)
from agent.providers.base import (
    ContextWindowPressure,
    PrepareRequestResult,
    build_context_window_pressure,
)


class OpenAICompatibleChatCompletionsMixin:
    provider_name = "chat"
    base_url: str
    supports_image_content = False
    assistant_extra_content_key: str | None = None
    assistant_extra_fields: tuple[str, ...] = ()

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
            provider=self.provider_name,
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
            raise RuntimeError(f"{self.provider_label} transport is unavailable in dry_run mode")

        request_payload = self._build_chat_payload(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
        )

        async def request_once() -> Any:
            request_coro = self._chat_completion(request_payload)
            if self.request_timeout_seconds and self.request_timeout_seconds > 0:
                return await asyncio.wait_for(
                    request_coro,
                    timeout=float(self.request_timeout_seconds),
                )
            return await request_coro

        response = await async_retry(
            request_once,
            max_attempts=self.max_attempts,
            should_retry=self._should_retry_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=getattr(self, "logger", logging.getLogger(__name__)),
            context=f"{self.provider_name}[chat.completions]",
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
            self._client.chat.completions.create,
            **payload,
            extra_body=extra_body,
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
        chat_messages.extend(
            convert_chat_messages(
                messages,
                assistant_extra_content_key=self.assistant_extra_content_key,
                assistant_extra_fields=self.assistant_extra_fields,
                include_images=self.supports_image_content,
            )
        )

        payload: dict[str, Any] = {
            "model": self.model_id,
            "messages": chat_messages,
        }
        extra_body = self._chat_extra_body()
        if extra_body:
            payload["extra_body"] = copy.deepcopy(extra_body)
        payload.update(self._chat_payload_extra_fields())

        converted_tools = convert_tools(tools)
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
        estimated_prompt_tokens = estimate_prompt_tokens(payload)
        available = (
            self.context_tokens - estimated_prompt_tokens - max(0, self.output_safety_tokens)
        )
        if available <= 0:
            return min(self.max_tokens, 16)
        return min(self.max_tokens, available)

    def _convert_response(self, response: Any) -> dict[str, Any]:
        choice = first_choice(response)
        message = get_value(choice, "message")
        if message is None:
            return {}

        content = get_value(message, "content")
        result: dict[str, Any] = {
            "content": content if isinstance(content, str) else "",
            "tool_calls": serialize_tool_calls(get_value(message, "tool_calls")),
        }
        thought_summary, extra_content = self._response_extra_content(message)
        if thought_summary:
            result["thought_summary"] = thought_summary
        if extra_content:
            result["extra_content"] = extra_content

        usage = extract_usage(response)
        if usage:
            result["usage"] = usage
        return result

    @property
    def provider_label(self) -> str:
        return self.provider_name.title()

    def _chat_extra_body(self) -> dict[str, Any] | None:
        return None

    def _chat_payload_extra_fields(self) -> dict[str, Any]:
        return {}

    def _response_extra_content(self, message: Any) -> tuple[str | None, dict[str, Any] | None]:
        return None, None

    def _should_retry_exception(self, exc: BaseException) -> bool:
        return should_retry_transient_http_exception(exc)


def convert_chat_messages(
    messages: list[dict],
    *,
    assistant_extra_content_key: str | None,
    assistant_extra_fields: tuple[str, ...],
    include_images: bool,
) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for message in messages:
        if not isinstance(message, dict):
            continue
        role = message.get("role")
        if role == "user":
            converted.append(
                {
                    "role": "user",
                    "content": convert_message_content(message, include_images=include_images),
                }
            )
            continue
        if role == "assistant":
            assistant: dict[str, Any] = {"role": "assistant"}
            content = message.get("content")
            assistant["content"] = content if isinstance(content, str) else None
            tool_calls = message.get("tool_calls")
            if tool_calls:
                assistant["tool_calls"] = tool_calls
            extra_content = message.get("extra_content")
            if assistant_extra_content_key and isinstance(extra_content, dict):
                provider_extra = extra_content.get(assistant_extra_content_key)
                if isinstance(provider_extra, dict):
                    for field in assistant_extra_fields:
                        value = provider_extra.get(field)
                        if value:
                            assistant[field] = value
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


def convert_message_content(message: dict[str, Any], *, include_images: bool = False) -> Any:
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
        if include_images and part_type == "input_image":
            image = convert_image_part(part)
            if image:
                parts.append(image)
    return parts or ""


def convert_image_part(part: dict[str, Any]) -> dict[str, Any] | None:
    detail = part.get("detail")
    url = part.get("image_url")
    if not isinstance(url, str) or not url:
        image_path = part.get("image_path")
        if isinstance(image_path, str) and image_path:
            url = image_path_to_data_url(image_path)
    if not isinstance(url, str) or not url:
        return None

    image_url: dict[str, Any] = {"url": url}
    if isinstance(detail, str) and detail:
        image_url["detail"] = detail
    return {"type": "image_url", "image_url": image_url}


def convert_tools(tools: list[dict]) -> list[dict[str, Any]]:
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


def first_choice(response: Any) -> Any:
    choices = get_value(response, "choices")
    if isinstance(choices, list) and choices:
        return choices[0]
    return None


def serialize_tool_calls(tool_calls: Any) -> list[dict[str, Any]]:
    if not tool_calls:
        return []
    serialized: list[dict[str, Any]] = []
    for tool_call in tool_calls:
        item = serialize_json_value(tool_call)
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


def extract_usage(response: Any) -> dict[str, int] | None:
    usage = get_value(response, "usage")
    if usage is None:
        return None

    prompt_tokens = get_value(usage, "prompt_tokens")
    completion_tokens = get_value(usage, "completion_tokens")
    total_tokens = get_value(usage, "total_tokens")
    cached_tokens = get_value(usage, "cached_tokens")
    if not isinstance(cached_tokens, int):
        details = get_value(usage, "prompt_tokens_details")
        cached_tokens = get_value(details, "cached_tokens") if details is not None else None
    completion_details = get_value(usage, "completion_tokens_details")
    reasoning_tokens = (
        get_value(completion_details, "reasoning_tokens")
        if completion_details is not None
        else None
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


def image_path_to_data_url(image_path: str) -> str:
    path = Path(image_path).expanduser().resolve()
    mime_type, _ = mimetypes.guess_type(path.name)
    if not mime_type:
        mime_type = "application/octet-stream"
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{data}"
