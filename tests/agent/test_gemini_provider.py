from __future__ import annotations

import asyncio
import base64
import json
from datetime import datetime, timezone
from types import SimpleNamespace

from agent.providers.gemini import (
    GeminiLLM,
    _gemini_compaction_prompt_text,
    _should_retry_gemini_exception,
)


def _seed_gemini_messages() -> list[dict[str, str]]:
    return [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "task"},
        {"role": "assistant", "content": "older assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_older",
            "content": "older tool result",
        },
        {"role": "assistant", "content": "latest assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_latest",
            "content": "latest tool result",
        },
    ]


def _content_text(content: object) -> str:
    parts = getattr(content, "parts", None) or []
    texts = [getattr(part, "text", None) for part in parts]
    return "\n".join(text for text in texts if isinstance(text, str))


def test_gemini_context_window_pressure_reports_prompt_fraction() -> None:
    provider = GeminiLLM(model_id="gemini-3.1-pro-preview", dry_run=True)

    pressure = provider.context_window_pressure(
        {
            "prompt_tokens": 250_000,
            "cached_tokens": 100_000,
            "candidates_tokens": 25_000,
            "total_tokens": 275_000,
        }
    )

    assert pressure.max_context_tokens == 1_000_000
    assert pressure.prompt_tokens == 250_000
    assert pressure.remaining_context_tokens == 750_000
    assert pressure.pressure_ratio == 0.25
    assert pressure.output_tokens == 25_000


def test_prepare_next_request_skips_prefix_cache_event() -> None:
    provider = GeminiLLM(dry_run=True)

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[
                {"role": "user", "content": "sdk docs"},
                {"role": "user", "content": "task"},
            ],
            tools=[],
            completed_turns=0,
        )
    )

    assert result.compaction_event is None
    assert result.maintenance_events == []
    assert result.trace_events == []
    assert provider._cached_content_name is None


def test_prepare_next_request_compacts_for_hard_pressure_and_preserves_raw_tail() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 700_000}
    provider._cached_content_name = "cachedContents/cache_1"
    provider._cached_content_expire_time = datetime(2099, 4, 4, 12, 0, tzinfo=timezone.utc)

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[object]
    ) -> tuple[object, dict[str, int]]:
        assert [_content_text(message) for message in messages] == ["old assistant", ""]
        function_response = getattr(messages[1].parts[0], "function_response", None)
        assert function_response is not None
        assert function_response.name == "compile_model"
        return (
            provider._convert_message(
                {"role": "user", "content": "[System-generated compaction summary]\nsummary"}
            ),
            {"prompt_tokens": 10_000, "candidates_tokens": 500, "total_tokens": 10_500},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._compact_messages = fake_compact_messages  # type: ignore[method-assign]

    messages = [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "task"},
        {"role": "assistant", "content": "old assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_old",
            "content": "old tool result",
        },
        {"role": "assistant", "content": "latest assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_latest",
            "content": "latest tool result",
        },
        {"role": "user", "content": "fix latest"},
    ]

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=2,
        )
    )

    assert result.compaction_event is not None
    assert result.compaction_event.trigger == "hard_pressure"
    assert result.compaction_event.before_next_input_tokens is None
    assert result.compaction_event.after_next_input_tokens is None
    assert result.compaction_event.estimated_saved_next_input_tokens is None
    assert (
        result.compaction_event.estimate_error
        == "Gemini preflight token counting is disabled by design."
    )

    request_contents = provider._request_contents()
    assert _content_text(request_contents[0]).startswith("[System-generated compaction summary]")
    assert _content_text(request_contents[1]) == "latest assistant"
    assert getattr(request_contents[2].parts[0], "function_response").name == "compile_model"
    assert _content_text(request_contents[3]) == "fix latest"


def test_request_contents_preserves_full_history_without_compaction_summary() -> None:
    provider = GeminiLLM(dry_run=True)
    messages = [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "task"},
        {"role": "assistant", "content": "old assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_old",
            "content": "old tool result",
        },
        {"role": "assistant", "content": "latest assistant"},
        {
            "role": "tool",
            "name": "compile_model",
            "tool_call_id": "call_latest",
            "content": "latest tool result",
        },
        {"role": "user", "content": "fix latest"},
    ]

    provider._append_new_contents(messages)

    request_contents = provider._request_contents()

    assert [_content_text(content) for content in request_contents] == [
        "sdk docs",
        "task",
        "old assistant",
        "",
        "latest assistant",
        "",
        "fix latest",
    ]
    assert getattr(request_contents[3].parts[0], "function_response").name == "compile_model"
    assert getattr(request_contents[5].parts[0], "function_response").name == "compile_model"


def test_prepare_next_request_soft_compaction_resets_after_compile_streak_reset() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 500_000}

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[object]
    ) -> tuple[object, dict[str, int]]:
        return (
            provider._convert_message(
                {"role": "user", "content": "[System-generated compaction summary]\nsummary"}
            ),
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._compact_messages = fake_compact_messages  # type: ignore[method-assign]

    messages = _seed_gemini_messages()

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig",
        )
    )
    second = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=4,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig",
        )
    )
    reset = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=5,
            consecutive_compile_failure_count=0,
            last_compile_failure_sig=None,
        )
    )
    provider._contents = []
    provider._last_message_count = 0
    provider._last_response_start_index = None
    third = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=6,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig",
        )
    )

    assert first.compaction_event is not None
    assert second.compaction_event is None
    assert reset.compaction_event is None
    assert third.compaction_event is not None


def test_prepare_next_request_skips_soft_compaction_under_low_pressure() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 100_000}

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=_seed_gemini_messages(),
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=6,
            last_compile_failure_sig="compile_sig",
        )
    )

    assert result.compaction_event is None
    assert result.trace_events[-1].payload["reason"] == "soft_pressure_too_low"


def test_prepare_next_request_requires_extra_failure_when_cache_hits_are_high() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 500_000, "cached_tokens": 350_000}

    compact_calls = 0

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[object]
    ) -> tuple[object, dict[str, int]]:
        nonlocal compact_calls
        compact_calls += 1
        return (
            provider._convert_message(
                {"role": "user", "content": "[System-generated compaction summary]\nsummary"}
            ),
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._compact_messages = fake_compact_messages  # type: ignore[method-assign]

    skipped = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=_seed_gemini_messages(),
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig_a",
        )
    )
    compacted = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=_seed_gemini_messages(),
            tools=[],
            completed_turns=3,
            consecutive_compile_failure_count=5,
            last_compile_failure_sig="compile_sig_b",
        )
    )

    assert skipped.compaction_event is None
    assert skipped.trace_events[-1].payload["reason"] == "compile_plateau_below_threshold"
    assert compacted.compaction_event is not None
    assert compacted.compaction_event.trigger == "compile_plateau"
    assert compact_calls == 1


def test_prepare_next_request_soft_compaction_respects_cooldown() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 500_000}

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    compact_calls = 0

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[object]
    ) -> tuple[object, dict[str, int]]:
        nonlocal compact_calls
        compact_calls += 1
        return (
            provider._convert_message(
                {"role": "user", "content": "[System-generated compaction summary]\nsummary"}
            ),
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._compact_messages = fake_compact_messages  # type: ignore[method-assign]

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=_seed_gemini_messages(),
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig_a",
        )
    )
    provider._last_usage = {"prompt_tokens": 500_000}
    provider._contents = []
    provider._last_message_count = 0
    provider._last_response_start_index = None
    skipped = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=_seed_gemini_messages(),
            tools=[],
            completed_turns=3,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="compile_sig_b",
        )
    )

    assert first.compaction_event is not None
    assert skipped.compaction_event is None
    assert skipped.trace_events[-1].payload["reason"] == "soft_compaction_cooldown"
    assert compact_calls == 1


def test_prepare_next_request_emits_skip_for_unsupported_threshold_model() -> None:
    provider = GeminiLLM(model_id="gemini-1.5-pro", dry_run=True)
    provider._last_usage = {"prompt_tokens": 800_000}

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[
                {"role": "user", "content": "sdk docs"},
                {"role": "user", "content": "task"},
            ],
            tools=[],
            completed_turns=2,
        )
    )

    assert result.compaction_event is None
    assert [event.event_type for event in result.trace_events] == ["compaction_skipped"]


def test_gemini_compaction_prompt_is_loaded_from_prompt_asset() -> None:
    prompt_text = _gemini_compaction_prompt_text()
    assert "Return JSON only." in prompt_text
    assert "Do not invent missing details." in prompt_text
    assert "return an empty array" in prompt_text


def test_close_emits_cache_delete_event() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._cached_content_name = "cachedContents/cache_1"

    async def fake_delete_prefix_cache(cache_name: str) -> None:
        assert cache_name == "cachedContents/cache_1"

    provider._delete_prefix_cache = fake_delete_prefix_cache  # type: ignore[method-assign]

    events = asyncio.run(provider.close())

    assert events == [
        {
            "kind": "cache_delete",
            "type": "cache_delete",
            "model_id": provider.model_id,
            "cache_name": "cachedContents/cache_1",
            "usage": None,
            "accounting_status": "not_billed_v1",
        }
    ]


def test_convert_message_prefers_raw_google_content_for_replay() -> None:
    provider = GeminiLLM(dry_run=True)
    signature = base64.b64encode(b"sig").decode("ascii")

    content = provider._convert_message(
        {
            "role": "assistant",
            "extra_content": {
                "google": {
                    "content": {
                        "role": "model",
                        "parts": [
                            {
                                "type": "text",
                                "text": "internal summary",
                                "thought": True,
                                "thought_signature": signature,
                            },
                            {
                                "type": "function_call",
                                "name": "compile_model",
                                "arguments": {},
                                "thought_signature": signature,
                            },
                        ],
                    },
                    "parts": [
                        {
                            "type": "text",
                            "text": "internal summary",
                            "thought": True,
                            "thought_signature": signature,
                        },
                        {
                            "type": "function_call",
                            "name": "compile_model",
                            "arguments": {},
                            "thought_signature": signature,
                        },
                    ],
                }
            },
        }
    )

    thought_part, tool_part = content.parts
    assert content.role == "model"
    assert thought_part.text == "internal summary"
    assert thought_part.thought is True
    assert thought_part.thought_signature == b"sig"
    assert tool_part.function_call.name == "compile_model"
    assert tool_part.thought_signature == b"sig"


def test_append_new_contents_ignores_harness_assistant_after_canonical_model_turn() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._append_new_contents(
        [
            {"role": "user", "content": "sdk docs"},
            {"role": "user", "content": "task"},
        ]
    )
    provider._contents.append(
        provider._convert_message(
            {
                "role": "assistant",
                "extra_content": {
                    "google": {
                        "content": {
                            "role": "model",
                            "parts": [
                                {
                                    "type": "text",
                                    "text": "raw model reply",
                                    "thought": False,
                                }
                            ],
                        }
                    }
                },
            }
        )
    )
    provider._last_response_start_index = 2

    provider._append_new_contents(
        [
            {"role": "user", "content": "sdk docs"},
            {"role": "user", "content": "task"},
            {"role": "assistant", "thought_summary": "internal summary"},
            {
                "role": "tool",
                "name": "compile_model",
                "tool_call_id": "call_latest",
                "content": '{"result": "ok"}',
            },
        ]
    )

    assert len(provider._contents) == 4
    assert _content_text(provider._contents[2]) == "raw model reply"
    assert getattr(provider._contents[3].parts[0], "function_response").name == "compile_model"


def test_convert_response_serializes_replace_function_call_args() -> None:
    provider = GeminiLLM(dry_run=True)

    response = SimpleNamespace(
        candidates=[
            SimpleNamespace(
                content=SimpleNamespace(
                    parts=[
                        SimpleNamespace(
                            function_call=SimpleNamespace(
                                id="call_edit",
                                name="replace",
                                args={
                                    "old_string": '"draft_model"',
                                    "new_string": '"draft_model_v2"',
                                },
                            ),
                            thought_signature=None,
                        )
                    ]
                )
            )
        ]
    )

    converted = provider._convert_response(response)

    assert converted["content"] == ""
    assert len(converted["tool_calls"]) == 1
    tool_call = converted["tool_calls"][0]
    assert tool_call["id"] == "call_edit"
    assert tool_call["function"]["name"] == "replace"
    assert json.loads(tool_call["function"]["arguments"]) == {
        "old_string": '"draft_model"',
        "new_string": '"draft_model_v2"',
    }


def test_convert_response_serializes_full_google_content_for_replay() -> None:
    provider = GeminiLLM(dry_run=True)
    signature = b"sig"

    response = SimpleNamespace(
        candidates=[
            SimpleNamespace(
                content=SimpleNamespace(
                    parts=[
                        SimpleNamespace(
                            text="internal summary",
                            thought=True,
                            thought_signature=signature,
                            function_call=None,
                        ),
                        SimpleNamespace(
                            text="final answer",
                            thought=False,
                            thought_signature=None,
                            function_call=None,
                        ),
                        SimpleNamespace(
                            text=None,
                            thought=False,
                            thought_signature=signature,
                            function_call=SimpleNamespace(
                                id="call_compile",
                                name="compile_model",
                                args={},
                            ),
                        ),
                    ]
                )
            )
        ]
    )

    converted = provider._convert_response(response)

    assert converted["thought_summary"] == "internal summary"
    assert converted["content"] == "final answer"
    assert converted["extra_content"] == {
        "google": {
            "content": {
                "role": "model",
                "parts": [
                    {
                        "type": "text",
                        "text": "internal summary",
                        "thought": True,
                        "thought_signature": base64.b64encode(signature).decode("ascii"),
                    },
                    {
                        "type": "text",
                        "text": "final answer",
                        "thought": False,
                        "thought_signature": None,
                    },
                    {
                        "type": "function_call",
                        "id": "call_compile",
                        "name": "compile_model",
                        "arguments": {},
                        "thought_signature": base64.b64encode(signature).decode("ascii"),
                    },
                ],
            }
        }
    }


def test_history_content_from_response_preserves_thought_parts() -> None:
    provider = GeminiLLM(dry_run=True)
    signature = b"sig"

    response = SimpleNamespace(
        candidates=[
            SimpleNamespace(
                content=SimpleNamespace(
                    role="model",
                    parts=[
                        SimpleNamespace(
                            text="internal summary",
                            thought=True,
                            thought_signature=signature,
                            function_call=None,
                        ),
                        SimpleNamespace(
                            text="final answer",
                            thought=False,
                            thought_signature=None,
                            function_call=None,
                        ),
                    ],
                )
            )
        ]
    )

    history_content = provider._history_content_from_response(response)

    assert history_content is response.candidates[0].content


def test_prepare_next_request_never_attempts_prefix_cache_creation() -> None:
    provider = GeminiLLM(dry_run=True)
    create_calls = 0

    async def fake_create_prefix_cache(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> dict:
        nonlocal create_calls
        create_calls += 1
        return {"name": "cachedContents/cache_1"}

    provider._create_prefix_cache = fake_create_prefix_cache  # type: ignore[method-assign]

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[
                {"role": "user", "content": "sdk docs"},
                {"role": "user", "content": "task"},
            ],
            tools=[],
            completed_turns=0,
        )
    )
    second = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[
                {"role": "user", "content": "sdk docs"},
                {"role": "user", "content": "task"},
            ],
            tools=[],
            completed_turns=1,
        )
    )

    assert create_calls == 0
    assert first.trace_events == []
    assert second.trace_events == []


def test_unsupported_gemini_api_value_error_is_not_retried() -> None:
    exc = ValueError("system_instruction parameter is not supported in Gemini API.")
    assert _should_retry_gemini_exception(exc) is False
