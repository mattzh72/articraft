from __future__ import annotations

import asyncio
import logging
import sys
from types import SimpleNamespace

import pytest

from agent.providers.openai import (
    DEFAULT_OPENAI_COMPACTION_MODEL,
    OpenAILLM,
    _OpenAIWebSocketError,
    openai_api_key_from_env,
    openai_api_keys_from_env,
)
from agent.tools import build_tool_registry


class _FakeOpenAIError(RuntimeError):
    def __init__(self, message: str, *, status_code: int) -> None:
        self.status_code = status_code
        super().__init__(message)


def _seed_openai_compaction_provider(
    *,
    prompt_tokens: int,
    cached_tokens: int = 0,
) -> OpenAILLM:
    provider = OpenAILLM(dry_run=True)
    provider._input_items = [
        {"role": "user", "content": [{"type": "input_text", "text": "docs"}]},
        {"role": "user", "content": [{"type": "input_text", "text": "task"}]},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "old"}],
        },
        {
            "type": "function_call",
            "call_id": "call_old",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_old", "output": "old output"},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "latest"}],
        },
        {
            "type": "function_call",
            "call_id": "call_latest",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_latest", "output": "latest output"},
    ]
    provider._last_response_start_index = 5
    provider._last_usage = {
        "prompt_tokens": prompt_tokens,
        "cached_tokens": cached_tokens,
    }
    return provider


def test_openai_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = openai_api_keys_from_env(
        {
            "OPENAI_API_KEY": "sk-primary",
            "OPENAI_API_KEYS": "sk-primary,\nsk-secondary, sk-third, sk-secondary",
        }
    )

    assert keys == ["sk-primary", "sk-secondary", "sk-third"]


def test_openai_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = openai_api_key_from_env({"OPENAI_API_KEYS": "sk-first,sk-second"})

    assert key in {"sk-first", "sk-second"}


def test_generate_with_tools_only_retries_without_reasoning_summary_for_supported_error(
    caplog: pytest.LogCaptureFixture,
) -> None:
    provider = OpenAILLM(dry_run=True)
    provider.max_attempts = 1
    request_payloads: list[dict] = []

    async def fake_request_with_transport(
        *,
        request_payload: dict,
        incremental_request_payload: dict,
        fallback_request_payload: dict,
    ) -> dict:
        request_payloads.append(request_payload)
        if len(request_payloads) == 1:
            raise _FakeOpenAIError("Unsupported parameter: reasoning.summary", status_code=400)
        return {}

    provider._request_with_transport = fake_request_with_transport  # type: ignore[method-assign]

    with caplog.at_level(logging.WARNING):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="be precise",
                messages=[{"role": "user", "content": "build a box"}],
                tools=[],
            )
        )

    assert len(request_payloads) == 2
    assert request_payloads[0]["reasoning"]["summary"] == "auto"
    assert "summary" not in request_payloads[1]["reasoning"]
    assert "rejected reasoning.summary" in caplog.text


def test_generate_with_tools_does_not_hide_unrelated_openai_errors() -> None:
    provider = OpenAILLM(dry_run=True)
    provider.max_attempts = 1
    attempts = 0

    async def fake_request_with_transport(
        *,
        request_payload: dict,
        incremental_request_payload: dict,
        fallback_request_payload: dict,
    ) -> dict:
        nonlocal attempts
        attempts += 1
        raise _FakeOpenAIError("Unauthorized", status_code=401)

    provider._request_with_transport = fake_request_with_transport  # type: ignore[method-assign]

    with pytest.raises(_FakeOpenAIError, match="Unauthorized"):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="be precise",
                messages=[{"role": "user", "content": "build a box"}],
                tools=[],
            )
        )

    assert attempts == 1


def test_request_with_websocket_logs_full_context_fallback(
    caplog: pytest.LogCaptureFixture,
) -> None:
    provider = OpenAILLM(dry_run=True, transport="websocket")
    provider._previous_response_id = "resp_prev"
    provider._input_items = [{"type": "message"}]
    calls: list[tuple[dict, bool]] = []

    async def fake_send_websocket_request(
        *, request_payload: dict, force_reconnect: bool = False
    ) -> dict:
        calls.append((request_payload, force_reconnect))
        if len(calls) == 1:
            raise _OpenAIWebSocketError(
                code="previous_response_not_found",
                message="missing prior response",
            )
        return {"output": []}

    provider._send_websocket_request = fake_send_websocket_request  # type: ignore[method-assign]

    with caplog.at_level(logging.WARNING):
        response = asyncio.run(
            provider._request_with_websocket(
                request_payload={"previous_response_id": "resp_prev"},
                fallback_request_payload={"input": [{"type": "message"}]},
            )
        )

    assert response == {"output": []}
    assert calls == [
        ({"previous_response_id": "resp_prev"}, False),
        ({"input": [{"type": "message"}]}, True),
    ]
    assert "full-context fallback triggered" in caplog.text


def test_openai_client_disables_sdk_retries_and_uses_request_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    class _FakeAsyncOpenAI:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    monkeypatch.setenv("OPENAI_API_KEY", "sk-primary")
    monkeypatch.setenv("OPENAI_REQUEST_TIMEOUT_SECONDS", "37")
    monkeypatch.setitem(sys.modules, "openai", SimpleNamespace(AsyncOpenAI=_FakeAsyncOpenAI))

    provider = OpenAILLM(dry_run=False)

    assert provider._client_is_async is True
    assert captured == {
        "api_key": "sk-primary",
        "max_retries": 0,
        "timeout": 37.0,
    }


def test_openai_default_request_timeout_is_30_minutes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("OPENAI_REQUEST_TIMEOUT_SECONDS", raising=False)

    provider = OpenAILLM(dry_run=True)

    assert provider.request_timeout_seconds == 1800.0


def test_openai_default_compaction_model_is_mini() -> None:
    provider = OpenAILLM(dry_run=True)

    assert DEFAULT_OPENAI_COMPACTION_MODEL == "gpt-5.4-mini"
    assert provider.compaction_model_id == "gpt-5.4-mini"


def test_convert_tools_normalizes_function_schemas_for_responses_strict_mode() -> None:
    provider = OpenAILLM(dry_run=True)
    registry = build_tool_registry("openai", sdk_package="sdk")

    converted = provider._convert_tools(registry.get_tool_schemas())

    read_file = next(tool for tool in converted if tool.get("name") == "read_file")
    assert read_file["strict"] is True
    assert read_file["parameters"]["required"] == ["path", "offset", "limit"]
    assert read_file["parameters"]["properties"]["path"]["type"] == "string"
    assert read_file["parameters"]["properties"]["offset"]["type"] == ["integer", "null"]
    assert read_file["parameters"]["properties"]["limit"]["type"] == ["integer", "null"]
    assert read_file["parameters"]["additionalProperties"] is False

    probe_model = next(tool for tool in converted if tool.get("name") == "probe_model")
    assert probe_model["parameters"]["required"] == ["code", "timeout_ms", "include_stdout"]
    assert probe_model["parameters"]["properties"]["code"]["type"] == "string"
    assert probe_model["parameters"]["properties"]["timeout_ms"]["type"] == ["integer", "null"]
    assert probe_model["parameters"]["properties"]["include_stdout"]["type"] == [
        "boolean",
        "null",
    ]

    compile_model = next(tool for tool in converted if tool.get("name") == "compile_model")
    assert compile_model["parameters"] == {
        "type": "object",
        "properties": {},
        "required": [],
        "additionalProperties": False,
    }


def test_openai_sync_client_disables_sdk_retries_and_uses_request_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured: dict[str, object] = {}

    class _FakeOpenAI:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    class _MissingAsyncOpenAI:
        def __init__(self, **kwargs: object) -> None:
            raise RuntimeError("async unavailable")

    monkeypatch.setenv("OPENAI_API_KEY", "sk-primary")
    monkeypatch.setenv("OPENAI_REQUEST_TIMEOUT_SECONDS", "42")
    monkeypatch.setitem(
        sys.modules,
        "openai",
        SimpleNamespace(AsyncOpenAI=_MissingAsyncOpenAI, OpenAI=_FakeOpenAI),
    )

    provider = OpenAILLM(dry_run=False)

    assert provider._client_is_async is False
    assert captured == {
        "api_key": "sk-primary",
        "max_retries": 0,
        "timeout": 42.0,
    }


def test_prepare_next_request_compacts_for_hard_pressure_and_clears_previous_response_id() -> None:
    provider = OpenAILLM(dry_run=True)
    provider._input_items = [
        {"role": "user", "content": [{"type": "input_text", "text": "docs"}]},
        {"role": "user", "content": [{"type": "input_text", "text": "task"}]},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "old"}],
        },
        {
            "type": "function_call",
            "call_id": "call_old",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_old", "output": "old output"},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "latest"}],
        },
        {
            "type": "function_call",
            "call_id": "call_latest",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_latest", "output": "latest output"},
        {"role": "user", "content": [{"type": "input_text", "text": "fix compile"}]},
    ]
    provider._last_response_start_index = 5
    provider._previous_response_id = "resp_prev"
    provider._last_usage = {"prompt_tokens": 272_000}

    async def fake_count_request_tokens(request_payload: dict) -> int:
        return 300_000 if len(request_payload["input"]) == 9 else 90_000

    async def fake_compact_inputs(
        *,
        system_prompt: str,
        input_items: list[dict[str, object]],
    ) -> dict:
        assert system_prompt == "system"
        assert input_items == provider._input_items[2:5]
        return {
            "id": "resp_cmp",
            "output": [{"type": "compaction", "id": "cmp_1"}],
            "usage": {"input_tokens": 18_000, "output_tokens": 900, "total_tokens": 18_900},
        }

    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
    provider._compact_inputs = fake_compact_inputs  # type: ignore[method-assign]

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
        )
    )

    assert result.compaction_event is not None
    assert result.compaction_event.trigger == "hard_pressure"
    assert result.compaction_event.before_next_input_tokens == 300_000
    assert result.compaction_event.after_next_input_tokens == 90_000
    assert result.compaction_event.estimated_saved_next_input_tokens == 210_000
    assert provider._previous_response_id is None
    assert provider._input_items == [
        {"role": "user", "content": [{"type": "input_text", "text": "docs"}]},
        {"role": "user", "content": [{"type": "input_text", "text": "task"}]},
        {"type": "compaction", "id": "cmp_1"},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "latest"}],
        },
        {
            "type": "function_call",
            "call_id": "call_latest",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_latest", "output": "latest output"},
        {"role": "user", "content": [{"type": "input_text", "text": "fix compile"}]},
    ]
    assert [event.event_type for event in result.trace_events] == ["compaction"]


def test_prepare_next_request_compacts_once_for_compile_plateau_signature() -> None:
    provider = _seed_openai_compaction_provider(prompt_tokens=210_000)

    compact_calls = 0

    async def fake_count_request_tokens(request_payload: dict) -> int:
        return len(request_payload["input"]) * 100

    async def fake_compact_inputs(
        *,
        system_prompt: str,
        input_items: list[dict[str, object]],
    ) -> dict:
        nonlocal compact_calls
        compact_calls += 1
        return {
            "id": "resp_cmp",
            "output": [{"type": "compaction", "id": f"cmp_{compact_calls}"}],
            "usage": {"input_tokens": 1_000, "output_tokens": 250, "total_tokens": 1_250},
        }

    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
    provider._compact_inputs = fake_compact_inputs  # type: ignore[method-assign]

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="sig_a",
        )
    )
    second = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=4,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="sig_a",
        )
    )

    assert first.compaction_event is not None
    assert first.compaction_event.trigger == "compile_plateau"
    assert second.compaction_event is None
    assert compact_calls == 1


def test_prepare_next_request_skips_compile_plateau_under_low_pressure() -> None:
    provider = _seed_openai_compaction_provider(prompt_tokens=10_000)

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=6,
            last_compile_failure_sig="sig_a",
        )
    )

    assert result.compaction_event is None
    assert [event.event_type for event in result.trace_events] == ["compaction_skipped"]
    assert result.trace_events[0].payload["reason"] == "soft_pressure_too_low"


def test_prepare_next_request_requires_extra_failure_when_cache_hits_are_high() -> None:
    provider = _seed_openai_compaction_provider(prompt_tokens=210_000, cached_tokens=150_000)

    compact_calls = 0

    async def fake_count_request_tokens(request_payload: dict) -> int:
        return len(request_payload["input"]) * 100

    async def fake_compact_inputs(
        *,
        system_prompt: str,
        input_items: list[dict[str, object]],
    ) -> dict:
        nonlocal compact_calls
        compact_calls += 1
        return {
            "id": "resp_cmp",
            "output": [{"type": "compaction", "id": f"cmp_{compact_calls}"}],
            "usage": {"input_tokens": 1_000, "output_tokens": 250, "total_tokens": 1_250},
        }

    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
    provider._compact_inputs = fake_compact_inputs  # type: ignore[method-assign]

    skipped = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="sig_a",
        )
    )
    compacted = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=3,
            consecutive_compile_failure_count=5,
            last_compile_failure_sig="sig_b",
        )
    )

    assert skipped.compaction_event is None
    assert skipped.trace_events[-1].payload["reason"] == "compile_plateau_below_threshold"
    assert compacted.compaction_event is not None
    assert compacted.compaction_event.trigger == "compile_plateau"
    assert compact_calls == 1


def test_prepare_next_request_soft_compaction_respects_cooldown() -> None:
    provider = _seed_openai_compaction_provider(prompt_tokens=210_000)

    compact_calls = 0
    token_counts = iter([210_000, 180_000])

    async def fake_count_request_tokens(request_payload: dict) -> int:
        return next(token_counts)

    async def fake_compact_inputs(
        *,
        system_prompt: str,
        input_items: list[dict[str, object]],
    ) -> dict:
        nonlocal compact_calls
        compact_calls += 1
        return {
            "id": "resp_cmp",
            "output": [{"type": "compaction", "id": f"cmp_{compact_calls}"}],
            "usage": {"input_tokens": 1_000, "output_tokens": 250, "total_tokens": 1_250},
        }

    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
    provider._compact_inputs = fake_compact_inputs  # type: ignore[method-assign]

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="sig_a",
        )
    )
    provider._input_items = _seed_openai_compaction_provider(prompt_tokens=200_000)._input_items
    provider._last_response_start_index = 5
    provider._last_usage = {"prompt_tokens": 200_000}
    skipped = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=3,
            consecutive_compile_failure_count=4,
            last_compile_failure_sig="sig_b",
        )
    )

    assert first.compaction_event is not None
    assert skipped.compaction_event is None
    assert skipped.trace_events[-1].payload["reason"] == "soft_compaction_cooldown"
    assert compact_calls == 1


def test_prepare_next_request_survives_token_count_failures() -> None:
    provider = OpenAILLM(dry_run=True)
    provider._input_items = [
        {"role": "user", "content": [{"type": "input_text", "text": "docs"}]},
        {"role": "user", "content": [{"type": "input_text", "text": "task"}]},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "old"}],
        },
        {
            "type": "function_call",
            "call_id": "call_old",
            "name": "compile_model",
            "arguments": "{}",
        },
        {"type": "function_call_output", "call_id": "call_old", "output": "old output"},
        {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": "latest"}],
        },
    ]
    provider._last_response_start_index = 5
    provider._last_usage = {"prompt_tokens": 272_000}

    async def fake_count_request_tokens(request_payload: dict) -> int:
        raise RuntimeError("token count unavailable")

    async def fake_compact_inputs(
        *,
        system_prompt: str,
        input_items: list[dict[str, object]],
    ) -> dict:
        return {
            "id": "resp_cmp",
            "output": [{"type": "compaction", "id": "cmp_1"}],
            "usage": {"input_tokens": 5_000, "output_tokens": 250, "total_tokens": 5_250},
        }

    provider._count_request_tokens = fake_count_request_tokens  # type: ignore[method-assign]
    provider._compact_inputs = fake_compact_inputs  # type: ignore[method-assign]

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
        )
    )

    assert result.compaction_event is not None
    assert result.compaction_event.before_next_input_tokens is None
    assert result.compaction_event.after_next_input_tokens is None
    assert result.compaction_event.estimate_error == "token count unavailable"


def test_prepare_next_request_logs_unsupported_threshold_model_once() -> None:
    provider = OpenAILLM(model_id="gpt-4o", dry_run=True)
    provider._last_usage = {"prompt_tokens": 123_456}

    first = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=2,
        )
    )
    second = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=3,
        )
    )

    assert [event.event_type for event in first.trace_events] == ["compaction_skipped"]
    assert first.trace_events[0].payload["reason"] == "unsupported_threshold_model"
    assert second.trace_events == []
