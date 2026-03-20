"""
OpenAI LLM wrapper using the Responses API with tool calling support.

This module intentionally avoids importing `openai` at import-time so the rest of
the repo can be used without OpenAI installed.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import mimetypes
import os
import random
import time
import uuid
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlparse, urlunparse

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


logger = logging.getLogger(__name__)


def openai_api_keys_from_env(env: dict[str, str] | None = None) -> list[str]:
    values = os.environ if env is None else env

    def _split(raw: str | None) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    primary_key = values.get("OPENAI_API_KEY")
    if primary_key and primary_key.strip():
        keys.append(primary_key.strip())
    keys.extend(_split(values.get("OPENAI_API_KEYS")))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)
    return unique_keys


def openai_api_key_from_env(env: dict[str, str] | None = None) -> str | None:
    keys = openai_api_keys_from_env(env)
    return random.choice(keys) if keys else None


class OpenAILLM:
    """Minimal OpenAI Responses API client for tool-calling workflows."""

    def __init__(
        self,
        model_id: str = "gpt-5.4",
        *,
        thinking_level: str = "high",
        reasoning_summary: Optional[str] = "auto",
        transport: str = "http",
        store: Optional[bool] = None,
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.reasoning_effort = _effort_from_thinking_level(thinking_level)
        self.reasoning_summary = _normalize_reasoning_summary(reasoning_summary)
        self.transport = _normalize_transport(transport)
        self.store = self.transport == "websocket" if store is None else bool(store)
        # Hard timeout for a single OpenAI request. Set to 0/negative to disable.
        self.request_timeout_seconds = _env_float("OPENAI_REQUEST_TIMEOUT_SECONDS", 900.0)
        self.max_attempts = max(1, int(_env_float("OPENAI_MAX_ATTEMPTS", 4)))
        self.retry_base_seconds = _env_float("OPENAI_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("OPENAI_RETRY_MAX_SECONDS", 20.0)
        self.websocket_open_timeout_seconds = _env_float(
            "OPENAI_WEBSOCKET_OPEN_TIMEOUT_SECONDS", 20.0
        )
        # Rotate the socket before the documented 60 minute server cap.
        self.websocket_max_connection_age_seconds = _env_float(
            "OPENAI_WEBSOCKET_MAX_CONNECTION_AGE_SECONDS", 3300.0
        )
        self._api_key: Optional[str] = None
        if dry_run:
            self._client = None
            self._client_is_async = False
        else:
            load_dotenv()
            api_key = openai_api_key_from_env()
            if not api_key:
                raise ValueError(
                    "OpenAI credentials not found. Set OPENAI_API_KEY or OPENAI_API_KEYS."
                )
            self._api_key = api_key

            self._client: Any
            self._client_is_async: bool
            try:
                from openai import AsyncOpenAI  # type: ignore

                self._client = AsyncOpenAI(api_key=api_key)
                self._client_is_async = True
            except Exception:
                try:
                    from openai import OpenAI  # type: ignore
                except Exception as exc:  # pragma: no cover
                    raise RuntimeError(
                        "OpenAI provider selected but the `openai` package is not installed. "
                        "Install it (e.g. `uv add openai`), then try again."
                    ) from exc

                self._client = OpenAI(api_key=api_key)
                self._client_is_async = False

        base_url = os.environ.get("OPENAI_BASE_URL")
        client_base_url = getattr(getattr(self, "_client", None), "base_url", None)
        if not base_url and client_base_url is not None:
            base_url = str(client_base_url)
        self._responses_websocket_url = _responses_websocket_url(base_url)

        # Responses API expects a list of structured items; we keep the canonical
        # conversation in this format to avoid lossy conversions.
        self._input_items: list[dict[str, Any]] = []
        self._last_message_count: int = 0
        self._previous_response_id: Optional[str] = None
        self._websocket: Any = None
        self._websocket_opened_at: Optional[float] = None

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        """
        Build the exact request payload that would be sent to the OpenAI Responses API,
        without making any network calls.
        """

        converted_tools = self._convert_tools(tools)

        # Build the canonical Responses input items exactly like generate_with_tools.
        self._input_items = []
        self._last_message_count = 0
        self._previous_response_id = None
        self._append_new_inputs(messages)

        return self._build_request_payload(
            system_prompt=system_prompt,
            tools=converted_tools,
            input_items=self._input_items,
        )

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        converted_tools = self._convert_tools(tools)
        new_input_items = self._append_new_inputs(messages)

        request_payload = self._build_request_payload(
            system_prompt=system_prompt,
            tools=converted_tools,
            input_items=self._input_items,
        )
        incremental_request_payload = self._build_request_payload(
            system_prompt=system_prompt,
            tools=converted_tools,
            input_items=new_input_items,
            previous_response_id=self._previous_response_id,
        )
        fallback_request_payload = self._build_request_payload(
            system_prompt=system_prompt,
            tools=converted_tools,
            input_items=self._input_items,
        )

        async def _request_once() -> Any:
            try:
                return await self._request_with_transport(
                    request_payload=request_payload,
                    incremental_request_payload=incremental_request_payload,
                    fallback_request_payload=fallback_request_payload,
                )
            except Exception:
                reasoning = request_payload.get("reasoning")
                if not (isinstance(reasoning, dict) and "summary" in reasoning):
                    raise
                return await self._request_with_transport(
                    request_payload=_payload_without_reasoning_summary(request_payload),
                    incremental_request_payload=_payload_without_reasoning_summary(
                        incremental_request_payload
                    ),
                    fallback_request_payload=_payload_without_reasoning_summary(
                        fallback_request_payload
                    ),
                )

        response = await _async_retry(
            _request_once,
            max_attempts=self.max_attempts,
            should_retry=_should_retry_openai_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=logger,
            context=f"openai[{self.transport}]",
        )

        # Persist the full response items as the source of truth for future turns.
        self._input_items.extend(self._serialize_response_output(response))
        self._previous_response_id = self._extract_response_id(response)

        return self._convert_response(response)

    async def close(self) -> None:
        await self._close_websocket()
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

    def _build_request_payload(
        self,
        *,
        system_prompt: str,
        tools: list[dict[str, Any]],
        input_items: list[dict[str, Any]],
        previous_response_id: Optional[str] = None,
    ) -> dict[str, Any]:
        request_payload: dict[str, Any] = {
            "model": self.model_id,
            "instructions": system_prompt,
            "tools": tools,
            "parallel_tool_calls": True,
            "input": input_items,
            "store": self.store,
            # Include encrypted reasoning so reasoning items can be safely carried
            # over across tool-calling turns during full-context fallbacks.
            "include": ["reasoning.encrypted_content"],
        }

        reasoning: dict[str, Any] = {"effort": self.reasoning_effort}
        if self.reasoning_summary:
            reasoning["summary"] = self.reasoning_summary
        request_payload["reasoning"] = reasoning
        if previous_response_id:
            request_payload["previous_response_id"] = previous_response_id
        return request_payload

    async def _request_with_http(self, request_payload: dict[str, Any]) -> Any:
        if self._client is None:
            raise RuntimeError("OpenAI HTTP transport is unavailable in dry_run mode")
        if self._client_is_async:
            return await self._client.responses.create(**request_payload)
        return await asyncio.to_thread(self._client.responses.create, **request_payload)

    async def _request_with_transport(
        self,
        *,
        request_payload: dict[str, Any],
        incremental_request_payload: dict[str, Any],
        fallback_request_payload: dict[str, Any],
    ) -> Any:
        if self.transport == "websocket":
            request_coro = self._request_with_websocket(
                request_payload=incremental_request_payload,
                fallback_request_payload=fallback_request_payload,
            )
        else:
            request_coro = self._request_with_http(request_payload)

        if self.request_timeout_seconds and self.request_timeout_seconds > 0:
            return await asyncio.wait_for(
                request_coro,
                timeout=float(self.request_timeout_seconds),
            )
        return await request_coro

    async def _request_with_websocket(
        self,
        *,
        request_payload: dict[str, Any],
        fallback_request_payload: dict[str, Any],
    ) -> Any:
        try:
            return await self._send_websocket_request(request_payload=request_payload)
        except _OpenAIWebSocketError as exc:
            if exc.code == "websocket_connection_limit_reached":
                try:
                    return await self._send_websocket_request(
                        request_payload=request_payload, force_reconnect=True
                    )
                except _OpenAIWebSocketError as retry_exc:
                    if retry_exc.code == "previous_response_not_found":
                        return await self._send_websocket_request(
                            request_payload=fallback_request_payload, force_reconnect=True
                        )
                    raise
            if exc.code == "previous_response_not_found":
                return await self._send_websocket_request(
                    request_payload=fallback_request_payload, force_reconnect=True
                )
            raise

    async def _send_websocket_request(
        self,
        *,
        request_payload: dict[str, Any],
        force_reconnect: bool = False,
    ) -> Any:
        websocket = await self._ensure_websocket_connection(force_reconnect=force_reconnect)
        event = {"type": "response.create", **request_payload}
        await websocket.send(json.dumps(event))
        return await self._receive_websocket_response(websocket)

    async def _ensure_websocket_connection(self, *, force_reconnect: bool = False) -> Any:
        if (
            not force_reconnect
            and self._websocket is not None
            and not getattr(self._websocket, "closed", False)
            and not self._websocket_connection_is_stale()
        ):
            return self._websocket

        await self._close_websocket()
        self._websocket = await self._open_websocket_connection()
        self._websocket_opened_at = time.monotonic()
        return self._websocket

    async def _open_websocket_connection(self) -> Any:
        if not self._api_key:
            raise RuntimeError("OpenAI websocket transport requires a real API key")
        try:
            import websockets  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "OpenAI websocket transport requires the `websockets` package."
            ) from exc

        return await websockets.connect(
            self._responses_websocket_url,
            additional_headers={"Authorization": f"Bearer {self._api_key}"},
            open_timeout=self.websocket_open_timeout_seconds,
            max_size=None,
        )

    def _websocket_connection_is_stale(self) -> bool:
        if self._websocket_opened_at is None:
            return False
        max_age = self.websocket_max_connection_age_seconds
        if max_age <= 0:
            return False
        return (time.monotonic() - self._websocket_opened_at) >= max_age

    async def _receive_websocket_response(self, websocket: Any) -> Any:
        while True:
            raw_message = await websocket.recv()
            if isinstance(raw_message, bytes):
                raw_message = raw_message.decode("utf-8")
            event = json.loads(raw_message)
            event_type = event.get("type")

            if event_type == "error":
                raise _OpenAIWebSocketError.from_event(event)

            if event_type == "response.completed":
                response_payload = event.get("response")
                if response_payload is None:
                    raise _OpenAIWebSocketError(
                        code="missing_response",
                        message="response.completed event did not include a response payload",
                    )
                return self._coerce_response_object(response_payload)

            if event_type == "response.incomplete":
                response_payload = event.get("response")
                if response_payload is None:
                    raise _OpenAIWebSocketError(
                        code="missing_response",
                        message="response.incomplete event did not include a response payload",
                    )
                return self._coerce_response_object(response_payload)

            if event_type == "response.failed":
                response_payload = event.get("response")
                raise _OpenAIWebSocketError(
                    code="response_failed",
                    message=_response_error_message(response_payload)
                    or "OpenAI response failed over websocket",
                    response=response_payload,
                )

    def _coerce_response_object(self, response_payload: Any) -> Any:
        if not isinstance(response_payload, dict):
            return response_payload
        try:
            from openai.types.responses.response import Response  # type: ignore

            return Response.model_validate(response_payload)
        except Exception:
            return response_payload

    async def _close_websocket(self) -> None:
        websocket = self._websocket
        self._websocket = None
        self._websocket_opened_at = None
        if websocket is None:
            return None
        close_method = getattr(websocket, "close", None)
        if close_method is None:
            return None
        result = close_method()
        if asyncio.iscoroutine(result):
            await result
        return None

    def _append_new_inputs(self, messages: list[dict]) -> list[dict[str, Any]]:
        """
        Convert new harness messages to Responses API input items.

        After the first model call, we ignore assistant messages from `messages`
        to avoid duplicating assistant outputs (we already store the model's
        canonical `response.output` items in `_input_items`).
        """

        new_items: list[dict[str, Any]] = []
        start = 0 if not self._input_items else self._last_message_count
        for message in messages[start:]:
            if not isinstance(message, dict):
                continue

            role = message.get("role")
            if role in {"user", "assistant"}:
                if self._input_items and role == "assistant":
                    continue
                content_parts = self._convert_message_content(message.get("content"))
                if content_parts:
                    item = {"role": role, "content": content_parts}
                    self._input_items.append(item)
                    new_items.append(item)

                # Best-effort: if an assistant message includes tool_calls (e.g. when
                # resuming from external history), add them as tool call items.
                for call in message.get("tool_calls") or []:
                    if not isinstance(call, dict):
                        continue
                    call_id = call.get("id") or f"call_{uuid.uuid4().hex}"
                    if call.get("type") == "custom":
                        custom = call.get("custom") if isinstance(call.get("custom"), dict) else {}
                        item = {
                            "type": "custom_tool_call",
                            "call_id": call_id,
                            "name": custom.get("name", ""),
                            "input": custom.get("input", ""),
                        }
                        self._input_items.append(item)
                        new_items.append(item)
                    else:
                        func = (
                            call.get("function") if isinstance(call.get("function"), dict) else {}
                        )
                        item = {
                            "type": "function_call",
                            "call_id": call_id,
                            "name": func.get("name", ""),
                            "arguments": func.get("arguments", ""),
                        }
                        self._input_items.append(item)
                        new_items.append(item)

            elif role == "tool":
                call_id = message.get("tool_call_id") or message.get("call_id")
                if not call_id:
                    continue
                output = message.get("content")
                if output is None:
                    output = ""
                if not isinstance(output, str):
                    try:
                        output = json.dumps(output)
                    except Exception:
                        output = str(output)
                if message.get("tool_type") == "custom":
                    item = {"type": "custom_tool_call_output", "call_id": call_id, "output": output}
                    self._input_items.append(item)
                    new_items.append(item)
                else:
                    item = {"type": "function_call_output", "call_id": call_id, "output": output}
                    self._input_items.append(item)
                    new_items.append(item)

        self._last_message_count = len(messages)
        return new_items

    def _convert_message_content(self, content: Any) -> list[dict[str, Any]]:
        if isinstance(content, str):
            if not content.strip():
                return []
            return [{"type": "input_text", "text": content}]

        if not isinstance(content, list):
            return []

        parts: list[dict[str, Any]] = []
        for part in content:
            if not isinstance(part, dict):
                continue

            part_type = part.get("type")
            if part_type in {"input_text", "text"}:
                text = part.get("text")
                if isinstance(text, str) and text.strip():
                    parts.append({"type": "input_text", "text": text})
                continue

            if part_type == "input_image":
                image_part = self._convert_image_part(part)
                if image_part:
                    parts.append(image_part)

        return parts

    def _convert_image_part(self, part: dict[str, Any]) -> Optional[dict[str, Any]]:
        detail = part.get("detail")

        if isinstance(part.get("image_url"), str) and part["image_url"]:
            item: dict[str, Any] = {
                "type": "input_image",
                "image_url": part["image_url"],
            }
            if isinstance(detail, str) and detail:
                item["detail"] = detail
            return item

        if isinstance(part.get("file_id"), str) and part["file_id"]:
            item = {
                "type": "input_image",
                "file_id": part["file_id"],
            }
            if isinstance(detail, str) and detail:
                item["detail"] = detail
            return item

        image_path = part.get("image_path")
        if isinstance(image_path, str) and image_path:
            item = {
                "type": "input_image",
                "image_url": _image_path_to_data_url(image_path),
            }
            if isinstance(detail, str) and detail:
                item["detail"] = detail
            return item

        return None

    def _convert_tools(self, tools: list[dict]) -> list[dict[str, Any]]:
        """
        Convert existing OpenAI-style function tool schemas into Responses API format.

        Supported input formats:
          {"type":"function","function":{"name":...,"description":...,"parameters":...}}
          {"type":"custom","name":...,"description":...,"format":{...}}
        """

        converted: list[dict[str, Any]] = []
        for tool in tools or []:
            if not isinstance(tool, dict):
                continue
            tool_type = tool.get("type")
            if tool_type == "custom":
                fmt = tool.get("format") if isinstance(tool.get("format"), dict) else None
                if not fmt:
                    continue
                converted.append(
                    {
                        "type": "custom",
                        "name": tool.get("name", ""),
                        "description": tool.get("description", ""),
                        "format": {
                            "type": fmt.get("type", ""),
                            "syntax": fmt.get("syntax", ""),
                            "definition": fmt.get("definition", ""),
                        },
                    }
                )
                continue

            if tool_type == "function":
                func = tool.get("function") if isinstance(tool.get("function"), dict) else None
                if not func:
                    continue
                converted.append(
                    {
                        "type": "function",
                        "name": func.get("name", ""),
                        "description": func.get("description", ""),
                        "parameters": func.get("parameters")
                        or {"type": "object", "properties": {}},
                        # Codex-style tools use non-strict schemas so optional fields
                        # do not need to be listed in `required`.
                        "strict": False,
                    }
                )
        return converted

    def _serialize_response_output(self, response: Any) -> list[dict[str, Any]]:
        output = (
            response.get("output")
            if isinstance(response, dict)
            else getattr(response, "output", None)
        )
        if output is None:
            return []

        serialized: list[dict[str, Any]] = []
        for item in output:
            if item is None:
                continue
            if isinstance(item, dict):
                serialized.append(item)
                continue

            dump_json = getattr(item, "model_dump_json", None)
            if callable(dump_json):
                serialized.append(json.loads(dump_json(exclude_none=True)))
                continue

            dump = getattr(item, "model_dump", None)
            if callable(dump):
                serialized.append(dump(exclude_none=True))
                continue

            # Best-effort fallback for tests / duck-typed objects.
            item_type = getattr(item, "type", None)
            record: dict[str, Any] = {"type": item_type} if item_type else {}
            for attr in (
                "role",
                "content",
                "name",
                "arguments",
                "input",
                "output",
                "call_id",
                "id",
                "summary",
            ):
                value = getattr(item, attr, None)
                if value is not None:
                    record[attr] = value
            if record:
                serialized.append(record)

        return serialized

    def _extract_usage(self, response: Any) -> Optional[dict[str, int]]:
        usage = (
            response.get("usage")
            if isinstance(response, dict)
            else getattr(response, "usage", None)
        )
        if usage is None:
            return None

        def get(field: str) -> Any:
            if isinstance(usage, dict):
                return usage.get(field)
            return getattr(usage, field, None)

        def get_details() -> Any:
            for name in ("input_tokens_details", "prompt_tokens_details", "input_token_details"):
                details = get(name)
                if details is not None:
                    return details
            return None

        def get_from(obj: Any, field: str) -> Any:
            if obj is None:
                return None
            if isinstance(obj, dict):
                return obj.get(field)
            return getattr(obj, field, None)

        input_tokens = get("input_tokens")
        output_tokens = get("output_tokens")
        total_tokens = get("total_tokens")
        cached_tokens = get("cached_tokens")
        if not isinstance(cached_tokens, int):
            details = get_details()
            for field in (
                "cached_tokens",
                "cached_input_tokens",
                "cache_read_tokens",
                "cache_read_input_tokens",
            ):
                value = get_from(details, field)
                if isinstance(value, int):
                    cached_tokens = value
                    break

        cleaned: dict[str, int] = {}
        if isinstance(input_tokens, int):
            cleaned["prompt_tokens"] = input_tokens
        if isinstance(output_tokens, int):
            cleaned["candidates_tokens"] = output_tokens
        if isinstance(total_tokens, int):
            cleaned["total_tokens"] = total_tokens
        if isinstance(cached_tokens, int):
            cleaned["cached_tokens"] = cached_tokens

        return cleaned or None

    def _convert_response(self, response: Any) -> dict[str, Any]:
        text_fragments: list[str] = []
        reasoning_fragments: list[str] = []
        tool_calls: list[dict[str, Any]] = []
        usage = self._extract_usage(response)

        output = (
            response.get("output")
            if isinstance(response, dict)
            else getattr(response, "output", None)
        )
        output = output or []
        for item in output:
            if item is None:
                continue

            item_type = item.get("type") if isinstance(item, dict) else getattr(item, "type", None)
            if item_type == "reasoning":
                summary = (
                    item.get("summary")
                    if isinstance(item, dict)
                    else getattr(item, "summary", None)
                )
                reasoning_text = _extract_reasoning_summary(summary)
                if reasoning_text:
                    reasoning_fragments.append(reasoning_text)
                continue

            if item_type == "message":
                content = (
                    item.get("content")
                    if isinstance(item, dict)
                    else getattr(item, "content", None)
                )
                text = _extract_message_text(content)
                if text:
                    text_fragments.append(text)
                continue

            if item_type == "function_call":
                name = item.get("name") if isinstance(item, dict) else getattr(item, "name", None)
                arguments = (
                    item.get("arguments")
                    if isinstance(item, dict)
                    else getattr(item, "arguments", None)
                )
                call_id = (
                    item.get("call_id")
                    if isinstance(item, dict)
                    else getattr(item, "call_id", None)
                )
                tool_calls.append(
                    {
                        "id": call_id or f"call_{uuid.uuid4().hex}",
                        "type": "function",
                        "function": {
                            "name": str(name or ""),
                            "arguments": str(arguments or ""),
                        },
                    }
                )
                continue

            if item_type == "custom_tool_call":
                name = item.get("name") if isinstance(item, dict) else getattr(item, "name", None)
                input_text = (
                    item.get("input") if isinstance(item, dict) else getattr(item, "input", None)
                )
                call_id = (
                    item.get("call_id")
                    if isinstance(item, dict)
                    else getattr(item, "call_id", None)
                )
                tool_calls.append(
                    {
                        "id": call_id or f"call_{uuid.uuid4().hex}",
                        "type": "custom",
                        "custom": {
                            "name": str(name or ""),
                            "input": str(input_text or ""),
                        },
                    }
                )
                continue

        result: dict[str, Any] = {
            "content": "\n".join(text_fragments).strip(),
            "tool_calls": tool_calls,
        }
        if reasoning_fragments:
            result["thought_summary"] = "\n".join(reasoning_fragments).strip()
        if usage:
            result["usage"] = usage
        return result

    def _extract_response_id(self, response: Any) -> Optional[str]:
        response_id = (
            response.get("id") if isinstance(response, dict) else getattr(response, "id", None)
        )
        if isinstance(response_id, str) and response_id:
            return response_id
        return None


def _extract_reasoning_summary(summary: Any) -> str:
    if not summary:
        return ""
    if isinstance(summary, str):
        return summary.strip()
    parts: list[str] = []
    if isinstance(summary, list):
        for item in summary:
            if isinstance(item, str):
                if item.strip():
                    parts.append(item.strip())
                continue
            text = item.get("text") if isinstance(item, dict) else getattr(item, "text", None)
            if isinstance(text, str) and text.strip():
                parts.append(text.strip())
    return " ".join(parts).strip()


def _normalize_reasoning_summary(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.lower() in {"0", "false", "none", "off"}:
        return None
    return text


def _extract_message_text(content: Any) -> str:
    if not content:
        return ""
    if isinstance(content, str):
        return content
    fragments: list[str] = []
    if isinstance(content, list):
        for part in content:
            if part is None:
                continue
            part_type = part.get("type") if isinstance(part, dict) else getattr(part, "type", None)
            if part_type in {"output_text", "input_text", "text"}:
                text = part.get("text") if isinstance(part, dict) else getattr(part, "text", None)
                if isinstance(text, str) and text:
                    fragments.append(text)
                continue
            text = part.get("text") if isinstance(part, dict) else getattr(part, "text", None)
            if isinstance(text, str) and text:
                fragments.append(text)
    return "\n".join(fragments).strip()


def _effort_from_thinking_level(thinking_level: str) -> str:
    level = (thinking_level or "").strip().lower()
    if level == "med":
        level = "medium"
    if level not in {"low", "medium", "high"}:
        level = "medium"
    return level


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def _image_path_to_data_url(image_path: str) -> str:
    path = Path(image_path).expanduser().resolve()
    mime_type, _ = mimetypes.guess_type(path.name)
    if not mime_type:
        mime_type = "application/octet-stream"
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{data}"


def _normalize_transport(transport: str) -> str:
    value = (transport or "http").strip().lower()
    if value not in {"http", "websocket"}:
        raise ValueError(f"Unsupported OpenAI transport: {transport}")
    return value


def _responses_websocket_url(base_url: Optional[str]) -> str:
    root = (base_url or "https://api.openai.com/v1/").strip()
    parsed = urlparse(root)
    scheme = "wss" if parsed.scheme == "https" else "ws"
    path = parsed.path.rstrip("/")
    if not path:
        path = "/v1"
    if not path.endswith("/responses"):
        if not path.endswith("/v1"):
            path = f"{path}/v1"
        path = f"{path}/responses"
    return urlunparse((scheme, parsed.netloc, path, "", "", ""))


def _payload_without_reasoning_summary(request_payload: dict[str, Any]) -> dict[str, Any]:
    updated = dict(request_payload)
    reasoning = updated.get("reasoning")
    if not isinstance(reasoning, dict) or "summary" not in reasoning:
        return updated
    new_reasoning = dict(reasoning)
    new_reasoning.pop("summary", None)
    updated["reasoning"] = new_reasoning
    return updated


def _extract_http_status(exc: BaseException) -> Optional[int]:
    status = getattr(exc, "status_code", None) or getattr(exc, "status", None)
    if isinstance(status, int) and 100 <= status <= 599:
        return status
    response = getattr(exc, "response", None)
    if response is not None:
        status = getattr(response, "status_code", None) or getattr(response, "status", None)
        if isinstance(status, int) and 100 <= status <= 599:
            return status
    return None


def _should_retry_openai_exception(exc: BaseException) -> bool:
    if isinstance(exc, (asyncio.TimeoutError, TimeoutError)):
        return True

    status = _extract_http_status(exc)
    if status is not None:
        if status in {408, 409, 425, 429, 500, 502, 503, 504}:
            return True
        if 400 <= status < 500:
            return False
        if status >= 500:
            return True

    if isinstance(exc, _OpenAIWebSocketError):
        if exc.code == "previous_response_not_found":
            return False
        if exc.code in {
            "websocket_connection_limit_reached",
            "rate_limit_exceeded",
            "server_error",
            "overloaded",
            "temporarily_unavailable",
        }:
            return True

    message = str(exc).lower()
    if any(
        needle in message
        for needle in (
            "overloaded",
            "temporarily unavailable",
            "service unavailable",
            "timeout",
            "timed out",
            "deadline exceeded",
            "rate limit",
            "too many requests",
            "connection reset",
            "connection aborted",
            "connection refused",
            "connection error",
            "internal error",
            "backend error",
        )
    ):
        return True

    if any(
        needle in message
        for needle in (
            "api key",
            "unauthorized",
            "permission denied",
            "forbidden",
            "invalid",
            "malformed",
            "not found",
        )
    ):
        return False

    return True


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
    fn: Any,
    *,
    max_attempts: int,
    should_retry: Any,
    base_delay: float,
    max_delay: float,
    sleep_fn: Any = asyncio.sleep,
    rng: Any = random.random,
    logger: Optional[logging.Logger] = None,
    context: str = "openai",
) -> Any:
    if max_attempts <= 0:
        raise ValueError("max_attempts must be >= 1")

    attempt = 0
    while True:
        try:
            return await fn()
        except Exception as exc:
            attempt += 1
            if attempt >= max_attempts or not bool(should_retry(exc)):
                raise

            cap = min(max_delay, base_delay * (2 ** (attempt - 1)))
            delay = max(0.0, float(rng()) * cap)
            if logger:
                logger.warning(
                    "%s failed (attempt %s/%s), retrying in %.2fs: %s",
                    context,
                    attempt,
                    max_attempts,
                    delay,
                    _format_retry_exception(exc),
                )
            await sleep_fn(delay)


def _response_error_message(response_payload: Any) -> str:
    if response_payload is None:
        return ""
    error = (
        response_payload.get("error")
        if isinstance(response_payload, dict)
        else getattr(response_payload, "error", None)
    )
    if error is None:
        return ""
    if isinstance(error, str):
        return error.strip()
    if isinstance(error, dict):
        message = error.get("message")
        if isinstance(message, str) and message.strip():
            return message.strip()
        return json.dumps(error)
    message = getattr(error, "message", None)
    if isinstance(message, str) and message.strip():
        return message.strip()
    code = getattr(error, "code", None)
    if isinstance(code, str) and code.strip():
        return code.strip()
    return str(error)


class _OpenAIWebSocketError(RuntimeError):
    def __init__(
        self,
        *,
        code: Optional[str],
        message: str,
        status: Optional[int] = None,
        response: Any = None,
    ):
        self.code = code
        self.status = status
        self.response = response
        super().__init__(message)

    @classmethod
    def from_event(cls, event: dict[str, Any]) -> "_OpenAIWebSocketError":
        error = event.get("error")
        if isinstance(error, dict):
            message = error.get("message")
            if not isinstance(message, str) or not message.strip():
                message = json.dumps(error)
            code = error.get("code")
        else:
            message = str(error or "OpenAI websocket error")
            code = None
        status = event.get("status")
        return cls(
            code=str(code) if code is not None else None,
            message=message,
            status=int(status) if isinstance(status, int) else None,
        )
