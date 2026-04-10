from __future__ import annotations

import asyncio
import base64
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


def test_prepare_next_request_creates_prefix_cache_event() -> None:
    provider = GeminiLLM(dry_run=True)

    async def fake_count_prefix_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        assert system_prompt == "system"
        assert len(messages) == 2
        return 5000

    async def fake_create_prefix_cache(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> dict:
        return {
            "name": "cachedContents/cache_1",
            "expire_time": "2026-04-02T12:00:00Z",
            "usage_metadata": {"total_token_count": 5000, "cached_content_token_count": 5000},
        }

    provider._count_prefix_tokens = fake_count_prefix_tokens  # type: ignore[method-assign]
    provider._create_prefix_cache = fake_create_prefix_cache  # type: ignore[method-assign]

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
    assert result.maintenance_events[0]["kind"] == "cache_create"
    assert result.maintenance_events[0]["cache_name"] == "cachedContents/cache_1"
    assert provider._cached_content_name == "cachedContents/cache_1"


def test_prepare_next_request_compacts_for_hard_pressure_and_preserves_raw_tail() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 700_000}
    provider._cached_content_name = "cachedContents/cache_1"
    provider._cached_content_expire_time = datetime(2099, 4, 4, 12, 0, tzinfo=timezone.utc)

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    counted_messages: list[list[str]] = []
    token_counts = iter([750_000, 150_000])

    async def fake_count_request_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        counted_messages.append([str(message.get("content")) for message in messages])
        return next(token_counts)

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[dict]
    ) -> tuple[dict, dict[str, int]]:
        assert [message.get("content") for message in messages] == [
            "old assistant",
            "old tool result",
        ]
        return (
            {"role": "user", "content": "[System-generated compaction summary]\nsummary"},
            {"prompt_tokens": 10_000, "candidates_tokens": 500, "total_tokens": 10_500},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
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
    assert result.compaction_event.before_next_input_tokens == 750_000
    assert result.compaction_event.after_next_input_tokens == 150_000
    assert result.compaction_event.estimated_saved_next_input_tokens == 600_000
    assert counted_messages == [
        ["latest assistant", "latest tool result", "fix latest"],
        [
            "[System-generated compaction summary]\nsummary",
            "latest assistant",
            "latest tool result",
            "fix latest",
        ],
    ]

    request_messages = provider._request_messages()
    assert request_messages[0]["content"].startswith("[System-generated compaction summary]")
    assert [message.get("content") for message in request_messages[1:]] == [
        "latest assistant",
        "latest tool result",
        "fix latest",
    ]


def test_prepare_next_request_soft_compaction_resets_after_compile_streak_reset() -> None:
    provider = GeminiLLM(dry_run=True)
    provider._last_usage = {"prompt_tokens": 500_000}

    async def fake_ensure_prefix_cache(
        *, system_prompt: str, tools: list[dict], trace_events: list
    ) -> list[dict]:
        return []

    async def fake_count_request_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        return 500_000

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[dict]
    ) -> tuple[dict, dict[str, int]]:
        return (
            {"role": "user", "content": "[System-generated compaction summary]\nsummary"},
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
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

    async def fake_count_request_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        return 500_000

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[dict]
    ) -> tuple[dict, dict[str, int]]:
        nonlocal compact_calls
        compact_calls += 1
        return (
            {"role": "user", "content": "[System-generated compaction summary]\nsummary"},
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
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

    token_counts = iter([500_000, 420_000])
    compact_calls = 0

    async def fake_count_request_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        return next(token_counts)

    async def fake_compact_messages(
        *, system_prompt: str, messages: list[dict]
    ) -> tuple[dict, dict[str, int]]:
        nonlocal compact_calls
        compact_calls += 1
        return (
            {"role": "user", "content": "[System-generated compaction summary]\nsummary"},
            {"prompt_tokens": 1_000, "candidates_tokens": 50, "total_tokens": 1_050},
        )

    provider._ensure_prefix_cache = fake_ensure_prefix_cache  # type: ignore[method-assign]
    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
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


def test_convert_message_preserves_thought_flag_when_replaying_google_parts() -> None:
    provider = GeminiLLM(dry_run=True)
    signature = base64.b64encode(b"sig").decode("ascii")

    content = provider._convert_message(
        {
            "role": "assistant",
            "extra_content": {
                "google": {
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
                    ]
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


def test_count_generate_request_tokens_uses_sdk_count_tokens() -> None:
    provider = GeminiLLM(dry_run=True)
    captured: dict[str, object] = {}

    class _FakeModels:
        async def count_tokens(
            self, *, model: str, contents: list[object], config: object
        ) -> object:
            captured["model"] = model
            captured["contents"] = contents
            captured["config"] = config
            return SimpleNamespace(total_tokens=321)

    provider._clients = [SimpleNamespace(aio=SimpleNamespace(models=_FakeModels()))]

    total_tokens = asyncio.run(
        provider._count_generate_request_tokens(
            system_prompt="system",
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "compile_model",
                        "description": "Compile the model.",
                        "parameters": {"type": "object", "properties": {}},
                    },
                }
            ],
            messages=[{"role": "user", "content": "task"}],
            cached_content_name=None,
            context="test",
        )
    )

    config = captured["config"]
    assert total_tokens == 321
    assert captured["model"] == provider.model_id
    assert len(captured["contents"]) == 1
    assert getattr(config, "system_instruction") == "system"
    assert len(getattr(config, "tools") or []) == 1


def test_prepare_next_request_disables_prefix_token_count_after_unsupported_error() -> None:
    provider = GeminiLLM(dry_run=True)
    count_attempts = 0

    class _UnsupportedCountTokensError(RuntimeError):
        status_code = 400

    async def fake_count_prefix_tokens(
        *, system_prompt: str, tools: list[dict], messages: list[dict]
    ) -> int:
        nonlocal count_attempts
        count_attempts += 1
        raise _UnsupportedCountTokensError(
            '400 INVALID_ARGUMENT. {"error": {"message": "Invalid JSON payload received. '
            'Unknown name \\"systemInstruction\\": Cannot find field.\\n'
            'Invalid JSON payload received. Unknown name \\"tools\\": Cannot find field.\\n'
            'Invalid JSON payload received. Unknown name \\"generationConfig\\": Cannot find field."}}'
        )

    provider._count_prefix_tokens = fake_count_prefix_tokens  # type: ignore[method-assign]

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

    assert count_attempts == 1
    assert first.trace_events[0].payload["reason"] == "token_counting_disabled"
    assert second.trace_events[0].payload["reason"] == "token_counting_disabled"


def test_unsupported_gemini_api_value_error_is_not_retried() -> None:
    exc = ValueError("system_instruction parameter is not supported in Gemini API.")
    assert _should_retry_gemini_exception(exc) is False
