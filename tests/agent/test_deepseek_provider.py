from __future__ import annotations

import asyncio
import json
import logging
from types import SimpleNamespace

import pytest

from agent.providers.deepseek import (
    DEFAULT_DEEPSEEK_MODEL,
    DeepSeekLLM,
    _async_retry,
    _should_retry_deepseek_exception,
    deepseek_api_key_from_env,
)


def test_deepseek_api_key_from_env_reads_primary_key() -> None:
    key = deepseek_api_key_from_env({"DEEPSEEK_API_KEY": "sk-ds-example"})

    assert key == "sk-ds-example"


def test_deepseek_api_key_from_env_returns_none_when_missing() -> None:
    key = deepseek_api_key_from_env({})

    assert key is None


def test_deepseek_default_model_is_v4_pro() -> None:
    provider = DeepSeekLLM(dry_run=True)

    assert provider.model_id == DEFAULT_DEEPSEEK_MODEL
    assert "v4-pro" in provider.model_id


def test_deepseek_context_window_pressure_uses_1m_context_tokens() -> None:
    provider = DeepSeekLLM(dry_run=True)

    pressure = provider.context_window_pressure(
        {
            "prompt_tokens": 500_000,
            "candidates_tokens": 4_096,
            "total_tokens": 504_096,
        }
    )

    assert pressure.max_context_tokens == 1_000_000
    assert pressure.prompt_tokens == 500_000
    assert pressure.remaining_context_tokens == 500_000
    assert pressure.pressure_ratio == 0.5
    assert pressure.output_tokens == 4_096


def test_deepseek_context_window_pressure_uses_env_context_tokens(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DEEPSEEK_CONTEXT_TOKENS", "200000")
    provider = DeepSeekLLM(dry_run=True)

    pressure = provider.context_window_pressure({"prompt_tokens": 100_000})

    assert pressure.max_context_tokens == 200_000
    assert pressure.remaining_context_tokens == 100_000


def test_deepseek_request_preview_uses_chat_completions_shape() -> None:
    provider = DeepSeekLLM(dry_run=True)
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

    assert payload["base_url"] == "https://api.deepseek.com/v1"
    assert payload["model"] == "deepseek-v4-pro"
    assert payload["messages"] == [
        {"role": "system", "content": "system"},
        {"role": "user", "content": "task"},
    ]
    assert "tools" in payload
    assert "extra_body" in payload
    assert payload["extra_body"]["thinking"] == {"type": "enabled"}
    assert payload["reasoning_effort"] == "high"


def test_deepseek_thinking_can_be_disabled() -> None:
    provider = DeepSeekLLM(model_id="deepseek-v4-pro", thinking_level="off", dry_run=True)
    payload = provider.build_request_preview(
        system_prompt="test",
        messages=[{"role": "user", "content": "task"}],
        tools=[],
    )

    thinking = payload["extra_body"]["thinking"]
    assert thinking["type"] == "disabled"
    assert "reasoning_effort" not in payload


def test_deepseek_reasoning_effort_maps_supported_levels() -> None:
    for level, expected_effort in [
        ("low", "high"),
        ("med", "high"),
        ("high", "high"),
        ("xhigh", "max"),
    ]:
        provider = DeepSeekLLM(model_id="deepseek-v4-pro", thinking_level=level, dry_run=True)
        payload = provider.build_request_preview(
            system_prompt="test",
            messages=[{"role": "user", "content": "task"}],
            tools=[],
        )
        thinking = payload["extra_body"]["thinking"]
        assert thinking["type"] == "enabled"
        assert "effort" not in thinking
        assert payload["reasoning_effort"] == expected_effort


def test_deepseek_chat_response_has_no_reasoning_content() -> None:
    provider = DeepSeekLLM(dry_run=True)
    response = SimpleNamespace(
        choices=[
            SimpleNamespace(
                message=SimpleNamespace(
                    content="I will build a latch.",
                    tool_calls=None,
                    reasoning_content=None,
                )
            )
        ],
        usage=None,
    )

    result = provider._convert_response(response)

    assert result["content"] == "I will build a latch."
    assert result["tool_calls"] == []
    assert "thought_summary" not in result


def test_deepseek_reasoner_response_preserves_reasoning_content() -> None:
    provider = DeepSeekLLM(dry_run=True)
    response = SimpleNamespace(
        choices=[
            SimpleNamespace(
                message=SimpleNamespace(
                    content="I will build a latch.",
                    tool_calls=None,
                    reasoning_content="Let me think about the structure first.",
                )
            )
        ],
        usage=None,
    )

    result = provider._convert_response(response)

    assert result["content"] == "I will build a latch."
    assert result["thought_summary"] == "Let me think about the structure first."
    assert result["extra_content"]["deepseek"]["reasoning_content"] == (
        "Let me think about the structure first."
    )


def test_deepseek_reasoner_response_round_trips_reasoning_content_in_next_payload() -> None:
    from agent.providers.deepseek import _convert_chat_messages

    messages = [
        {
            "role": "assistant",
            "content": "Done.",
            "extra_content": {
                "deepseek": {
                    "reasoning_content": "Planning complete.",
                }
            },
        },
    ]

    converted = _convert_chat_messages(messages)

    assert converted[0]["role"] == "assistant"
    assert converted[0]["content"] == "Done."
    assert converted[0].get("reasoning_content") == "Planning complete."


def test_deepseek_retry_predicate_treats_json_decode_errors_as_transient() -> None:
    assert _should_retry_deepseek_exception(json.JSONDecodeError("msg", "doc", 0)) is True


def test_deepseek_retry_predicate_treats_429_as_transient() -> None:
    exc = Exception("Rate limited")
    exc.status_code = 429  # type: ignore[attr-defined]
    assert _should_retry_deepseek_exception(exc) is True


def test_deepseek_retry_predicate_treats_400_as_non_transient() -> None:
    exc = Exception("Bad request")
    exc.status_code = 400  # type: ignore[attr-defined]
    assert _should_retry_deepseek_exception(exc) is False


def test_deepseek_async_retry_uses_exponential_full_jitter() -> None:
    call_count = 0
    sleeps: list[float] = []

    async def _failing_op() -> None:
        nonlocal call_count
        call_count += 1
        raise RuntimeError("transient")

    async def _fake_sleep(delay: float) -> None:
        sleeps.append(delay)

    with pytest.raises(RuntimeError, match="transient"):
        asyncio.run(
            _async_retry(
                _failing_op,
                max_attempts=3,
                should_retry=lambda _exc: True,
                base_delay=0.5,
                max_delay=20.0,
                logger=logging.getLogger("test"),
                context="test",
                sleep_fn=_fake_sleep,
                rng=lambda: 0.5,
            )
        )

    assert call_count == 3
    assert len(sleeps) == 2
    # First retry: cap = min(20, 0.5 * 2^0) = 0.5, delay = 0.5 * 0.5 = 0.25
    # Second retry: cap = min(20, 0.5 * 2^1) = 1.0, delay = 1.0 * 0.5 = 0.5
    assert sleeps[0] == 0.25
    assert sleeps[1] == 0.5
