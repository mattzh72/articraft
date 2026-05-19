"""
Gemini LLM wrapper with tool calling support.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import os
import random
import threading
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from agent.prompts import load_prompt_section_text
from agent.providers import gemini_codec
from agent.providers.base import (
    CompactionEvent,
    ContextWindowPressure,
    PrepareRequestResult,
    ProviderTraceEvent,
    build_context_window_pressure,
)
from agent.providers.compaction_policy import decide_compaction
from articraft.values import reasoning_level_alias

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


_logger = logging.getLogger(__name__)

_GEMINI_DANGER_ZONE_TOKENS = 700_000
_GEMINI_PREFIX_CACHE_TTL_SECONDS = 3600
_GEMINI_PREFIX_CACHE_REFRESH_WINDOW_SECONDS = 300
_GEMINI_COMPACTION_PROMPT_FILE = "gemini_compaction.md"
DEFAULT_GEMINI_COMPACTION_MODEL = "gemini-3-flash-preview"


def _load_cwd_dotenv_override() -> None:
    dotenv_path = Path.cwd() / ".env"
    if dotenv_path.exists():
        load_dotenv(dotenv_path=dotenv_path, override=True)


class GeminiLLM:
    """Minimal Gemini client for tool-calling workflows."""

    def __init__(
        self,
        model_id: str = "gemini-3.1-pro-preview",
        thinking_level: str = "high",
        *,
        compaction_model_id: str | None = None,
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.compaction_model_id = (compaction_model_id or DEFAULT_GEMINI_COMPACTION_MODEL).strip()
        self.thinking_level = thinking_level
        self.include_thoughts = _env_bool("GEMINI_INCLUDE_THOUGHTS", True)
        # Hard timeout for a single Gemini request. This protects the harness from
        # hanging indefinitely when the underlying client stalls.
        # Set to 0/negative to disable.
        self.request_timeout_seconds = _env_float("GEMINI_REQUEST_TIMEOUT_SECONDS", 300.0)
        self.max_retries = _env_int("GEMINI_MAX_RETRIES", 6)
        self.retry_base_seconds = _env_float("GEMINI_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("GEMINI_RETRY_MAX_SECONDS", 20.0)
        if dry_run:
            self.client = None
            self._clients: list[Any] = []
            self._client_lock = threading.Lock()
            self._next_client_index = 0
        else:
            _load_cwd_dotenv_override()
            config = gemini_client_config_from_env()
            from google import genai  # type: ignore

            if config.backend == "api_key":
                self._clients = [genai.Client(api_key=key) for key in config.api_keys]
            else:
                self._clients = [
                    genai.Client(vertexai=True, project=config.project, location=config.location)
                ]
            self._client_lock = threading.Lock()
            self._next_client_index = random.randrange(len(self._clients))
            # Keep the legacy attribute for compatibility with any external callers.
            self.client = self._clients[0]
        self._reset_request_state()

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """
        Build the exact request arguments that would be sent to the Gemini client,
        without making any network calls.
        """
        self._reset_request_state()
        self._append_new_contents(messages)
        contents = self._request_contents()
        config = self._build_generate_config(
            system_prompt=system_prompt,
            tools=tools,
            cached_content_name=None,
        )

        return {
            "model": self.model_id,
            "contents": [c.model_dump(mode="json", exclude_none=True) for c in contents],
            "config": config.model_dump(mode="json", exclude_none=True),
        }

    def context_window_pressure(self, usage: dict[str, int]) -> ContextWindowPressure:
        return build_context_window_pressure(
            provider="gemini",
            usage=usage,
            max_context_tokens=_context_window_tokens_for_model(self.model_id),
            hard_pressure_tokens=_hard_pressure_threshold_for_model(self.model_id),
        )

    async def prepare_next_request(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
        completed_turns: int,
        consecutive_compile_failure_count: int = 0,
        last_compile_failure_sig: Optional[str] = None,
    ) -> PrepareRequestResult:
        result = PrepareRequestResult()
        self._append_new_contents(messages)
        if consecutive_compile_failure_count <= 0 or not last_compile_failure_sig:
            self._last_soft_compaction_failure_sig = None

        result.maintenance_events.extend(
            await self._ensure_prefix_cache(
                system_prompt=system_prompt,
                tools=tools,
                trace_events=result.trace_events,
            )
        )

        turn_number = completed_turns + 1
        if turn_number <= 2:
            return result

        prompt_tokens = self._last_prompt_tokens()
        hard_threshold = _hard_pressure_threshold_for_model(self.model_id)
        if (
            hard_threshold is None
            and prompt_tokens is not None
            and not self._unsupported_threshold_event_emitted
        ):
            self._unsupported_threshold_event_emitted = True
            result.trace_events.append(
                ProviderTraceEvent(
                    event_type="compaction_skipped",
                    payload={
                        "reason": "unsupported_threshold_model",
                        "model_id": self.model_id,
                        "prompt_tokens": prompt_tokens,
                        "turn_before_request": completed_turns,
                    },
                )
            )

        immutable_prefix_contents, compactable_contents, raw_tail_contents = (
            self._partition_compaction_contents()
        )
        # Keep the trigger policy shared across providers so pressure bands,
        # cache-aware thresholds, and cooldown behavior stay aligned.
        decision = decide_compaction(
            prompt_tokens=prompt_tokens,
            cached_tokens=self._last_cached_tokens(),
            hard_threshold=hard_threshold,
            consecutive_compile_failure_count=consecutive_compile_failure_count,
            last_compile_failure_sig=last_compile_failure_sig,
            last_soft_compaction_failure_sig=self._last_soft_compaction_failure_sig,
            compactable_item_count=len(compactable_contents),
            turn_number=turn_number,
            last_soft_compaction_turn_number=self._last_soft_compaction_turn_number,
            last_soft_compaction_prompt_tokens=self._last_soft_compaction_prompt_tokens,
        )
        trigger = decision.trigger
        if trigger is None:
            if consecutive_compile_failure_count > 0 and last_compile_failure_sig:
                result.trace_events.append(
                    ProviderTraceEvent(
                        event_type="compaction_skipped",
                        payload={
                            "reason": decision.reason,
                            "model_id": self.model_id,
                            "prompt_tokens": prompt_tokens,
                            "cached_tokens": self._last_cached_tokens(),
                            "turn_before_request": completed_turns,
                            "failure_streak": consecutive_compile_failure_count,
                            "pressure_ratio": decision.pressure_ratio,
                            "cache_ratio": decision.cache_ratio,
                            "soft_failure_threshold": decision.soft_failure_threshold,
                            "min_compactable_items": decision.min_compactable_items,
                            "hard_trigger_tokens": decision.hard_trigger_tokens,
                        },
                    )
                )
            return result

        if not compactable_contents:
            result.trace_events.append(
                ProviderTraceEvent(
                    event_type="compaction_skipped",
                    payload={
                        "reason": "nothing_to_compact",
                        "trigger": trigger,
                        "model_id": self.model_id,
                        "turn_before_request": completed_turns,
                    },
                )
            )
            return result

        before_item_count = self._effective_message_count()
        before_tokens: int | None = None
        after_tokens: int | None = None
        estimate_error = "Gemini preflight token counting is disabled by design."

        summary_content, usage = await self._compact_messages(
            system_prompt=system_prompt,
            messages=compactable_contents,
        )
        self._contents = [
            *immutable_prefix_contents,
            summary_content,
            *raw_tail_contents,
        ]
        self._last_response_start_index = len(immutable_prefix_contents) + 1
        self._last_compaction_turn_number = turn_number
        if trigger == "compile_plateau":
            self._last_soft_compaction_failure_sig = last_compile_failure_sig
            self._last_soft_compaction_turn_number = turn_number
            self._last_soft_compaction_prompt_tokens = prompt_tokens

        event = CompactionEvent(
            turn_before_request=completed_turns,
            trigger=trigger,
            model_id=self.model_id,
            usage=usage,
            before_next_input_tokens=before_tokens,
            after_next_input_tokens=after_tokens,
            estimated_saved_next_input_tokens=(
                before_tokens - after_tokens
                if isinstance(before_tokens, int) and isinstance(after_tokens, int)
                else None
            ),
            before_item_count=before_item_count,
            after_item_count=self._effective_message_count(),
            previous_response_id_cleared=False,
            guardrails={
                "min_turns_satisfied": True,
                "hard_trigger_tokens": decision.hard_trigger_tokens,
                "pressure_ratio": decision.pressure_ratio,
                "cache_ratio": decision.cache_ratio,
                "soft_failure_threshold": decision.soft_failure_threshold,
                "raw_tail_preserved": True,
            },
            estimate_error=estimate_error,
        )
        if trigger == "compile_plateau" and isinstance(after_tokens, int):
            self._last_soft_compaction_prompt_tokens = after_tokens
        result.compaction_event = event
        result.trace_events.append(
            ProviderTraceEvent(event_type="compaction", payload=event.to_dict())
        )
        return result

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """Generate a response with optional tool calls."""
        self._append_new_contents(messages)
        contents = self._request_contents()
        config = self._build_generate_config(
            system_prompt=system_prompt,
            tools=tools,
            cached_content_name=self._active_cached_content_name(),
        )

        async def _call() -> Any:
            client = self._next_client()
            coro = client.aio.models.generate_content(
                model=self.model_id,
                contents=contents,
                config=config,
            )
            if self.request_timeout_seconds and self.request_timeout_seconds > 0:
                return await asyncio.wait_for(coro, timeout=float(self.request_timeout_seconds))
            return await coro

        response = await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.generate_content(model={self.model_id})",
        )

        response_start_index = len(self._contents)
        history_content = self._history_content_from_response(response)
        if history_content is not None:
            self._contents.append(history_content)
            self._last_response_start_index = response_start_index
        self._last_usage = self._extract_usage(response)
        return self._convert_response(response)

    async def close(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        if self._cached_content_name:
            cache_name = self._cached_content_name
            try:
                await self._delete_prefix_cache(cache_name)
                events.append(
                    {
                        "kind": "cache_delete",
                        "type": "cache_delete",
                        "model_id": self.model_id,
                        "cache_name": cache_name,
                        "usage": None,
                        "accounting_status": "not_billed_v1",
                    }
                )
            except Exception as exc:
                _logger.exception("Failed to delete Gemini cached content: %s", cache_name)
                events.append(
                    {
                        "kind": "cache_delete",
                        "type": "cache_delete",
                        "model_id": self.model_id,
                        "cache_name": cache_name,
                        "usage": None,
                        "accounting_status": "not_billed_v1",
                        "error": str(exc),
                    }
                )
            finally:
                self._cached_content_name = None
                self._cached_content_expire_time = None
                self._cached_prefix_digest = None
        return events

    def _reset_request_state(self) -> None:
        self._contents: list[Any] = []
        self._last_message_count = 0
        self._last_response_start_index: int | None = None
        self._last_usage: Optional[dict[str, int]] = None
        self._last_compaction_turn_number: int | None = None
        self._last_soft_compaction_failure_sig: str | None = None
        self._last_soft_compaction_turn_number: int | None = None
        self._last_soft_compaction_prompt_tokens: int | None = None
        self._unsupported_threshold_event_emitted = False
        self._cached_content_name: str | None = None
        self._cached_content_expire_time: datetime | None = None
        self._cached_prefix_digest: str | None = None

    def _append_new_contents(self, messages: list[dict]) -> None:
        if not messages:
            self._contents = []
            self._last_message_count = 0
            self._last_response_start_index = None
            return

        if self._last_message_count > len(messages):
            self._contents = []
            self._last_message_count = 0
            self._last_response_start_index = None

        latest_assistant_content_index: int | None = None
        start = 0 if not self._contents else self._last_message_count
        for message in messages[start:]:
            if not isinstance(message, dict):
                continue
            role = message.get("role")
            if role == "assistant" and self._last_response_start_index is not None:
                continue
            content = self._convert_message(message)
            if content is None:
                continue
            if role == "assistant":
                latest_assistant_content_index = len(self._contents)
            self._contents.append(content)

        self._last_message_count = len(messages)
        if self._last_response_start_index is None and latest_assistant_content_index is not None:
            self._last_response_start_index = latest_assistant_content_index

    def _immutable_prefix_count(self) -> int:
        count = 0
        for content in self._contents:
            if count >= 2:
                break
            if not gemini_codec.is_standard_user_content(content):
                break
            count += 1
        return count

    def _partition_compaction_contents(
        self,
    ) -> tuple[list[Any], list[Any], list[Any]]:
        immutable_prefix_count = self._immutable_prefix_count()
        raw_tail_start = self._last_response_start_index
        if raw_tail_start is None:
            raw_tail_start = len(self._contents)
        raw_tail_start = max(
            immutable_prefix_count,
            min(raw_tail_start, len(self._contents)),
        )
        immutable_prefix_contents = list(self._contents[:immutable_prefix_count])
        compactable_contents = list(self._contents[immutable_prefix_count:raw_tail_start])
        raw_tail_contents = list(self._contents[raw_tail_start:])
        return immutable_prefix_contents, compactable_contents, raw_tail_contents

    def _request_contents(self) -> list[Any]:
        request_contents = list(self._contents)
        if not self._active_cached_content_name():
            return request_contents
        return request_contents[self._immutable_prefix_count() :]

    def _effective_message_count(self) -> int:
        return len(self._request_contents())

    def _last_prompt_tokens(self) -> int | None:
        if not isinstance(self._last_usage, dict):
            return None
        prompt_tokens = self._last_usage.get("prompt_tokens")
        return prompt_tokens if isinstance(prompt_tokens, int) else None

    def _last_cached_tokens(self) -> int | None:
        if not isinstance(self._last_usage, dict):
            return None
        cached_tokens = self._last_usage.get("cached_tokens")
        return cached_tokens if isinstance(cached_tokens, int) else None

    def _extract_cache_usage(self, cache: Any) -> dict[str, int] | None:
        if isinstance(cache, dict) and isinstance(cache.get("usage_metadata"), dict):
            usage = self._extract_usage({"usage": cache["usage_metadata"]})
        else:
            usage = self._extract_usage(cache)
        return usage if usage else None

    def _active_cached_content_name(self) -> str | None:
        if not self._cached_content_name or not self._cached_content_expire_time:
            return None
        now = datetime.now(timezone.utc)
        if self._cached_content_expire_time <= now:
            self._cached_content_name = None
            self._cached_content_expire_time = None
            self._cached_prefix_digest = None
            return None
        return self._cached_content_name

    def _build_generate_config(
        self,
        *,
        system_prompt: str,
        tools: list[dict],
        cached_content_name: str | None,
    ) -> Any:
        from google.genai import types  # type: ignore

        tool_declarations = self._convert_tools(tools)
        return types.GenerateContentConfig(
            system_instruction=None if cached_content_name else system_prompt,
            tools=(
                [types.Tool(function_declarations=tool_declarations)]
                if tool_declarations and not cached_content_name
                else None
            ),
            cached_content=cached_content_name,
            temperature=0.7,
            thinking_config=self._build_thinking_config(),
        )

    async def _ensure_prefix_cache(
        self,
        *,
        system_prompt: str,
        tools: list[dict],
        trace_events: list[ProviderTraceEvent],
    ) -> list[dict[str, Any]]:
        return []

    async def _create_prefix_cache(
        self,
        *,
        system_prompt: str,
        tools: list[dict],
        messages: list[Any],
    ) -> Any:
        if not self._clients:
            raise RuntimeError("Gemini cached content requires a real API client")
        from google.genai import types  # type: ignore

        tool_declarations = self._convert_tools(tools)
        cache_config = types.CreateCachedContentConfig(
            contents=list(messages),
            system_instruction=system_prompt,
            tools=[types.Tool(function_declarations=tool_declarations)]
            if tool_declarations
            else None,
            ttl=f"{_GEMINI_PREFIX_CACHE_TTL_SECONDS}s",
            display_name=f"articraft-{uuid.uuid4().hex[:8]}",
        )

        async def _call() -> Any:
            client = self._next_client()
            return await client.aio.caches.create(model=self.model_id, config=cache_config)

        return await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.caches.create(model={self.model_id})",
        )

    async def _refresh_prefix_cache(self, cache_name: str) -> Any:
        if not self._clients:
            raise RuntimeError("Gemini cache refresh requires a real API client")
        from google.genai import types  # type: ignore

        async def _call() -> Any:
            client = self._next_client()
            return await client.aio.caches.update(
                name=cache_name,
                config=types.UpdateCachedContentConfig(ttl=f"{_GEMINI_PREFIX_CACHE_TTL_SECONDS}s"),
            )

        return await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.caches.update(model={self.model_id})",
        )

    async def _delete_prefix_cache(self, cache_name: str) -> None:
        if not self._clients:
            return None

        async def _call() -> Any:
            client = self._next_client()
            return await client.aio.caches.delete(name=cache_name)

        await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.caches.delete(model={self.model_id})",
        )
        return None

    async def _compact_messages(
        self,
        *,
        system_prompt: str,
        messages: list[Any],
    ) -> tuple[Any, dict[str, int] | None]:
        if not self._clients:
            raise RuntimeError("Gemini compaction requires a real API client")
        from google.genai import types  # type: ignore

        compact_input = {
            "system_prompt": system_prompt,
            "messages": [
                gemini_codec.content_for_compaction_prompt(message) for message in messages
            ],
        }
        prompt_text = _gemini_compaction_prompt_text()
        compaction_config = types.GenerateContentConfig(
            system_instruction=prompt_text,
            temperature=0,
            response_mime_type="application/json",
            response_schema={
                "type": "object",
                "properties": {
                    "task_requirements": {"type": "array", "items": {"type": "string"}},
                    "constraints": {"type": "array", "items": {"type": "string"}},
                    "tool_findings": {"type": "array", "items": {"type": "string"}},
                    "compile_state": {"type": "array", "items": {"type": "string"}},
                    "next_steps": {"type": "array", "items": {"type": "string"}},
                },
                "required": [
                    "task_requirements",
                    "constraints",
                    "tool_findings",
                    "compile_state",
                    "next_steps",
                ],
            },
        )

        async def _call() -> Any:
            client = self._next_client()
            return await client.aio.models.generate_content(
                model=self.compaction_model_id,
                contents=json.dumps(compact_input, ensure_ascii=False),
                config=compaction_config,
            )

        response = await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.compact(model={self.compaction_model_id})",
        )
        parsed = gemini_codec.parse_json_response_text(response)
        return gemini_codec.content_from_text_message(
            gemini_codec.render_compaction_summary_message(parsed)
        ), self._extract_usage(response)

    def _next_client(self) -> Any:
        if not self._clients:
            raise RuntimeError("Gemini client is not initialized")
        with self._client_lock:
            client = self._clients[self._next_client_index]
            self._next_client_index = (self._next_client_index + 1) % len(self._clients)
        return client

    def _build_thinking_config(self) -> Optional[Any]:
        try:
            from google.genai import types  # type: ignore
        except Exception:
            return None

        level = (self.thinking_level or "").strip().lower()
        if not level:
            return None

        model_id = (self.model_id or "").lower()
        if "2.5" in model_id:
            budget = self._thinking_budget_from_level(level, model_id)
            try:
                return types.ThinkingConfig(
                    thinking_budget=budget,
                    include_thoughts=self.include_thoughts,
                )
            except TypeError:
                return types.ThinkingConfig(thinking_budget=budget)

        normalized_level = self._normalize_thinking_level(level, model_id)
        try:
            return types.ThinkingConfig(
                thinking_level=normalized_level,
                include_thoughts=self.include_thoughts,
            )
        except TypeError:
            return types.ThinkingConfig(thinking_level=normalized_level)

    def _normalize_thinking_level(self, level: str, model_id: str) -> str:
        level = reasoning_level_alias(level)

        if level not in {"minimal", "low", "medium", "high"}:
            level = "high"

        if "flash" not in model_id:
            if level == "medium":
                level = "high"
            elif level == "minimal":
                level = "low"

        return level

    def _thinking_budget_from_level(self, level: str, model_id: str) -> int:
        level = reasoning_level_alias(level)
        min_budget, max_budget = self._thinking_budget_range(model_id)

        if level in {"low", "minimal"}:
            budget = min_budget
        elif level == "medium":
            budget = (min_budget + max_budget) // 2
        elif level in {"dynamic", "auto"}:
            budget = -1
        elif level in {"off", "none", "disable", "disabled", "0"}:
            budget = 0
        else:
            budget = max_budget

        if budget == 0 and min_budget > 0:
            budget = min_budget

        return budget

    def _thinking_budget_range(self, model_id: str) -> tuple[int, int]:
        if "flash-lite" in model_id or "flash_lite" in model_id:
            return 512, 24576
        if "flash" in model_id:
            return 0, 24576
        return 128, 32768

    def _convert_tools(self, tools: list[dict]) -> list[Any]:
        from google.genai.types import FunctionDeclaration  # type: ignore

        declarations: list[FunctionDeclaration] = []
        for tool in tools or []:
            func_def = tool.get("function") if isinstance(tool, dict) else None
            if not func_def:
                continue

            parameters = func_def.get("parameters")
            if parameters and isinstance(parameters, dict):
                parameters = {
                    key: value for key, value in parameters.items() if key != "additionalProperties"
                }

            declarations.append(
                FunctionDeclaration(
                    name=func_def.get("name", ""),
                    description=func_def.get("description", ""),
                    parameters=parameters,
                )
            )
        return declarations

    def _convert_message(self, message: dict) -> Any | None:
        return gemini_codec.convert_message(message)

    def _convert_user_content_parts(self, content: list[dict[str, Any]]) -> list[Any]:
        return gemini_codec.convert_user_content_parts(content)

    def _convert_input_image_part(self, item: dict[str, Any]) -> Any:
        return gemini_codec.convert_input_image_part(item)

    def _extract_usage(self, response: Any) -> Optional[dict[str, int]]:
        return gemini_codec.extract_usage(response)

    def _convert_response(self, response: Any) -> dict:
        return gemini_codec.convert_response(response)

    def _history_content_from_response(self, response: Any) -> Any | None:
        return gemini_codec.history_content_from_response(response)


def _hard_pressure_threshold_for_model(model_id: str) -> int | None:
    normalized = (model_id or "").strip().lower()
    if normalized.startswith("gemini-3") or normalized.startswith("gemini-2.5"):
        return _GEMINI_DANGER_ZONE_TOKENS
    return None


def _context_window_tokens_for_model(model_id: str) -> int | None:
    override = _env_int("GEMINI_CONTEXT_WINDOW_TOKENS", 0)
    if override > 0:
        return override

    normalized = (model_id or "").strip().lower()
    if normalized.startswith("gemini-3") or normalized.startswith("gemini-2.5"):
        return 1_000_000
    return None


def _prefix_digest(system_prompt: str, tools: list[dict], messages: list[dict[str, Any]]) -> str:
    payload = {
        "system_prompt": system_prompt,
        "tools": tools,
        "messages": messages,
    }
    raw = json.dumps(payload, sort_keys=True, ensure_ascii=False, default=str)
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def _isoformat_or_none(value: datetime | None) -> str | None:
    if value is None:
        return None
    return value.astimezone(timezone.utc).isoformat()


def _seconds_until(value: datetime) -> float:
    return (value - datetime.now(timezone.utc)).total_seconds()


def _parse_cache_expire_time(cache: Any) -> datetime | None:
    raw = getattr(cache, "expire_time", None)
    if raw is None and isinstance(cache, dict):
        raw = cache.get("expire_time")
    if isinstance(raw, datetime):
        return raw.astimezone(timezone.utc)
    if isinstance(raw, str) and raw.strip():
        candidate = raw.strip().replace("Z", "+00:00")
        try:
            parsed = datetime.fromisoformat(candidate)
        except ValueError:
            return None
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return parsed.astimezone(timezone.utc)
    return None


def _gemini_compaction_prompt_text() -> str:
    _, text = load_prompt_section_text(_GEMINI_COMPACTION_PROMPT_FILE)
    return text


@dataclass(frozen=True)
class GeminiClientConfig:
    backend: str
    api_keys: tuple[str, ...] = ()
    project: Optional[str] = None
    location: str = "global"


def gemini_api_keys_from_env(env: Optional[dict[str, str]] = None) -> list[str]:
    values = os.environ if env is None else env

    def _split(raw: Optional[str]) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    keys.extend(_split(values.get("GEMINI_API_KEYS")))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)

    return unique_keys


def gemini_client_config_from_env(env: Optional[dict[str, str]] = None) -> GeminiClientConfig:
    values = os.environ if env is None else env

    api_keys = gemini_api_keys_from_env(values)
    project = values.get("GOOGLE_CLOUD_PROJECT", "").strip() or _project_from_credentials_file(
        values.get("GOOGLE_APPLICATION_CREDENTIALS")
    )
    location = values.get("GOOGLE_CLOUD_LOCATION", "").strip() or "global"

    if project:
        return GeminiClientConfig(backend="vertex", project=project, location=location)

    if api_keys:
        return GeminiClientConfig(backend="api_key", api_keys=tuple(api_keys))

    raise ValueError(
        "Gemini configuration not found. Set GEMINI_API_KEYS for the Gemini API, "
        "or configure Vertex AI with GOOGLE_CLOUD_PROJECT. "
        "Authentication should come from GOOGLE_APPLICATION_CREDENTIALS. "
        "GOOGLE_CLOUD_LOCATION defaults to global."
    )


def _project_from_credentials_file(raw_path: Optional[str]) -> Optional[str]:
    if not raw_path:
        return None
    try:
        payload = json.loads(Path(raw_path).expanduser().read_text(encoding="utf-8"))
    except Exception:
        return None
    project = payload.get("project_id")
    return project.strip() if isinstance(project, str) and project.strip() else None


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw.strip())
    except Exception:
        return default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    return default


def _extract_http_status(exc: BaseException) -> Optional[int]:
    """
    Best-effort extraction of an HTTP status code from Gemini / google-genai errors.
    """

    for attr in ("status_code", "http_status", "status", "code"):
        value = getattr(exc, attr, None)
        try:
            if callable(value):
                value = value()
        except Exception:
            value = None

        if isinstance(value, int) and 100 <= value <= 599:
            return value

    response = getattr(exc, "response", None)
    status = getattr(response, "status_code", None) or getattr(response, "status", None)
    if isinstance(status, int) and 100 <= status <= 599:
        return status

    return None


def _should_retry_gemini_exception(exc: BaseException) -> bool:
    """
    Default retry policy for Gemini calls.

    - Retries transient 4xx/5xx responses (e.g. 429/503 overload).
    - Avoids retrying obvious auth/config errors (e.g. 401/403).
    - For unknown failures, prefers retry (caller still has max_attempts cap).
    """

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
            "resource exhausted",
            "connection reset",
            "connection aborted",
            "connection refused",
            "internal error",
            "backend error",
        )
    ):
        return True

    if any(
        needle in message
        for needle in (
            "not supported in gemini api",
            "api key",
            "unauthorized",
            "permission denied",
            "forbidden",
            "invalid argument",
            "not found",
        )
    ):
        return False

    return True


def _format_retry_exception(exc: BaseException) -> str:
    """Build a readable retry error summary, even for blank exception messages."""
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
    context: str = "gemini",
) -> Any:
    """
    Retry an async callable with exponential backoff + full jitter.
    """

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
