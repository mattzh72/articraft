from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

from agent.providers.openrouter import (
    DEFAULT_OPENROUTER_MAX_TOKENS,
    DEFAULT_OPENROUTER_MODEL,
    OpenRouterLLM,
    _async_retry,
    _should_retry_openrouter_exception,
    openrouter_api_key_from_env,
    openrouter_api_keys_from_env,
)


def test_openrouter_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = openrouter_api_keys_from_env(
        {
            "OPENROUTER_API_KEY": "sk-or-primary",
            "OPENROUTER_API_KEYS": "sk-or-primary,\nsk-or-secondary, sk-or-secondary",
        }
    )

    assert keys == ["sk-or-primary", "sk-or-secondary"]


def test_openrouter_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = openrouter_api_key_from_env({"OPENROUTER_API_KEYS": "sk-or-first,sk-or-second"})

    assert key in {"sk-or-first", "sk-or-second"}


def test_openrouter_default_model_is_tencent_free_preview() -> None:
    provider = OpenRouterLLM(dry_run=True)

    assert provider.model_id == DEFAULT_OPENROUTER_MODEL


def test_openrouter_context_window_pressure_uses_configured_context_tokens() -> None:
    provider = OpenRouterLLM(dry_run=True)
    provider.context_tokens = 262_144

    pressure = provider.context_window_pressure(
        {
            "prompt_tokens": 131_072,
            "candidates_tokens": 4_096,
            "total_tokens": 135_168,
        }
    )

    assert pressure.max_context_tokens == 262_144
    assert pressure.prompt_tokens == 131_072
    assert pressure.remaining_context_tokens == 131_072
    assert pressure.pressure_ratio == 0.5
    assert pressure.output_tokens == 4_096


def test_openrouter_request_preview_uses_chat_completions_shape() -> None:
    provider = OpenRouterLLM(dry_run=True)
    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "compile_model",
                    "description": "Compile",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
            }
        ],
    )

    assert payload["base_url"] == "https://openrouter.ai/api/v1"
    assert payload["model"] == "tencent/hy3-preview:free"
    assert payload["messages"] == [
        {"role": "system", "content": "system"},
        {"role": "user", "content": "task"},
    ]
    assert payload["tools"][0]["type"] == "function"
    assert payload["extra_body"]["reasoning"] == {"enabled": True, "effort": "high"}
    assert 0 < payload["max_tokens"] < DEFAULT_OPENROUTER_MAX_TOKENS


def test_openrouter_response_preserves_reasoning_details_for_next_turn() -> None:
    provider = OpenRouterLLM(dry_run=True)
    response = SimpleNamespace(
        choices=[
            SimpleNamespace(
                message=SimpleNamespace(
                    content="I will compile.",
                    reasoning=None,
                    reasoning_details=[
                        {"type": "reasoning.summary", "summary": "Planned tool use", "index": 0}
                    ],
                    tool_calls=[
                        SimpleNamespace(
                            id="call_1",
                            type="function",
                            function=SimpleNamespace(
                                name="compile_model",
                                arguments="{}",
                            ),
                        )
                    ],
                )
            )
        ],
        usage=SimpleNamespace(
            prompt_tokens=11,
            completion_tokens=7,
            total_tokens=18,
            prompt_tokens_details=SimpleNamespace(cached_tokens=3),
            completion_tokens_details=SimpleNamespace(reasoning_tokens=2),
        ),
    )

    converted = provider._convert_response(response)

    assert converted["content"] == "I will compile."
    assert converted["tool_calls"][0]["id"] == "call_1"
    assert converted["thought_summary"] == "Planned tool use"
    assert converted["extra_content"]["openrouter"]["reasoning_details"] == [
        {"type": "reasoning.summary", "summary": "Planned tool use", "index": 0}
    ]
    assert converted["usage"] == {
        "prompt_tokens": 11,
        "candidates_tokens": 7,
        "total_tokens": 18,
        "cached_tokens": 3,
        "reasoning_tokens": 2,
    }

    next_payload = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {"role": "user", "content": "task"},
            {
                "role": "assistant",
                "content": converted["content"],
                "tool_calls": converted["tool_calls"],
                "extra_content": converted["extra_content"],
            },
            {"role": "tool", "tool_call_id": "call_1", "content": "{}"},
        ],
        tools=[],
    )

    assert next_payload["messages"][2]["reasoning_details"] == [
        {"type": "reasoning.summary", "summary": "Planned tool use", "index": 0}
    ]


def test_openrouter_prepare_next_request_is_noop() -> None:
    provider = OpenRouterLLM(dry_run=True)

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[],
            tools=[],
            completed_turns=5,
        )
    )

    assert result.compaction_event is None
    assert result.trace_events == []


def test_openrouter_retries_malformed_json_response() -> None:
    provider = OpenRouterLLM(dry_run=True)
    provider._client = object()
    provider.max_attempts = 2
    provider.retry_base_seconds = 0
    provider.retry_max_seconds = 0
    attempts = 0

    async def fake_chat_completion(request_payload: dict) -> SimpleNamespace:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise json.JSONDecodeError("Expecting value", "\n" * 48, 264)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(content="ok", tool_calls=[]))],
            usage=None,
        )

    provider._chat_completion = fake_chat_completion  # type: ignore[method-assign]

    response = asyncio.run(
        provider.generate_with_tools(
            system_prompt="system",
            messages=[{"role": "user", "content": "task"}],
            tools=[],
        )
    )

    assert attempts == 2
    assert response["content"] == "ok"


def test_openrouter_retry_predicate_treats_json_decode_errors_as_transient() -> None:
    exc = json.JSONDecodeError("Expecting value", "\n" * 48, 264)

    assert _should_retry_openrouter_exception(exc) is True


def test_openrouter_async_retry_uses_exponential_full_jitter() -> None:
    attempts = 0
    sleeps: list[float] = []

    async def flaky() -> str:
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise TimeoutError("timed out")
        return "ok"

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)

    result = asyncio.run(
        _async_retry(
            flaky,
            max_attempts=3,
            should_retry=lambda exc: isinstance(exc, TimeoutError),
            base_delay=2,
            max_delay=10,
            logger=SimpleNamespace(warning=lambda *args, **kwargs: None),
            context="openrouter[test]",
            sleep_fn=fake_sleep,
            rng=lambda: 0.5,
        )
    )

    assert result == "ok"
    assert attempts == 3
    assert sleeps == [1.0, 2.0]
