from __future__ import annotations

import asyncio
import urllib.error
from types import SimpleNamespace

import anthropic
import httpx

from agent.providers.anthropic import (
    ANTHROPIC_VERSION,
    DEFAULT_ANTHROPIC_MODEL,
    AnthropicAPIError,
    AnthropicLLM,
    _api_error_from_sdk_exception,
    _async_retry,
    _load_cwd_dotenv_override,
    _should_retry_anthropic_exception,
    anthropic_api_key_from_env,
    anthropic_api_keys_from_env,
)


class _FakeAnthropicMessages:
    def __init__(self, responses: list[object]):
        self.responses = list(responses)
        self.calls: list[dict] = []

    async def create(self, **kwargs: object) -> object:
        self.calls.append(dict(kwargs))
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


class _FakeAnthropicClient:
    def __init__(self, responses: list[object]):
        self.messages = _FakeAnthropicMessages(responses)
        self.closed = False

    async def close(self) -> None:
        self.closed = True


def test_anthropic_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = anthropic_api_keys_from_env(
        {
            "ANTHROPIC_API_KEY": "sk-ant-primary",
            "ANTHROPIC_API_KEYS": "sk-ant-primary,\nsk-ant-secondary, sk-ant-secondary",
        }
    )

    assert keys == ["sk-ant-primary", "sk-ant-secondary"]


def test_anthropic_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = anthropic_api_key_from_env({"ANTHROPIC_API_KEYS": "sk-ant-first,sk-ant-second"})

    assert key in {"sk-ant-first", "sk-ant-second"}


def test_anthropic_provider_loads_dotenv_over_exported_key(monkeypatch, tmp_path) -> None:
    env_name = "ANTHROPIC_API_" + "KEY"
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv(env_name, "test-exported")
    (tmp_path / ".env").write_text(f"{env_name}=test-dotenv\n", encoding="utf-8")

    _load_cwd_dotenv_override()

    assert anthropic_api_key_from_env() == "test-dotenv"


def test_anthropic_default_model_is_current_opus() -> None:
    provider = AnthropicLLM(dry_run=True)

    assert provider.model_id == DEFAULT_ANTHROPIC_MODEL


def test_anthropic_request_preview_uses_messages_shape() -> None:
    provider = AnthropicLLM(dry_run=True)
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

    assert payload["base_url"] == "https://api.anthropic.com"
    assert payload["anthropic_version"] == ANTHROPIC_VERSION
    assert payload["model"] == "claude-opus-4-7"
    assert payload["cache_control"] == {"type": "ephemeral"}
    assert payload["system"] == [
        {
            "type": "text",
            "text": "system",
            "cache_control": {"type": "ephemeral"},
        }
    ]
    assert payload["thinking"] == {"type": "adaptive"}
    assert payload["output_config"] == {"effort": "high"}
    assert payload["messages"] == [{"role": "user", "content": "task"}]
    assert payload["tools"] == [
        {
            "name": "compile_model",
            "description": "Compile",
            "input_schema": {"type": "object", "properties": {}, "required": []},
            "cache_control": {"type": "ephemeral"},
        }
    ]


def test_anthropic_prompt_cache_can_be_disabled(monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_PROMPT_CACHE", "off")
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[],
    )

    assert "cache_control" not in payload
    assert payload["system"] == "system"


def test_anthropic_prompt_cache_marks_stable_workspace_docs() -> None:
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {
                "role": "user",
                "content": "\n\n# Workspace Documentation (read-only)\nshared docs",
            },
            {"role": "user", "content": "per-record prompt"},
        ],
        tools=[],
    )

    assert payload["messages"][0] == {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "\n\n# Workspace Documentation (read-only)\nshared docs",
                "cache_control": {"type": "ephemeral"},
            }
        ],
    }
    assert payload["messages"][1] == {"role": "user", "content": "per-record prompt"}


def test_anthropic_prompt_cache_supports_one_hour_ttl(monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_PROMPT_CACHE_TTL", "1h")
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[],
    )

    assert payload["cache_control"] == {"type": "ephemeral", "ttl": "1h"}
    assert payload["system"][0]["cache_control"] == {"type": "ephemeral", "ttl": "1h"}


def test_anthropic_preview_coalesces_tool_results_and_preserves_raw_blocks() -> None:
    provider = AnthropicLLM(dry_run=True)
    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {"role": "user", "content": "task"},
            {
                "role": "assistant",
                "content": "I will compile.",
                "tool_calls": [
                    {
                        "id": "toolu_1",
                        "type": "function",
                        "function": {"name": "compile_model", "arguments": "{}"},
                    },
                    {
                        "id": "toolu_2",
                        "type": "function",
                        "function": {"name": "read_file", "arguments": '{"path":"model.py"}'},
                    },
                ],
                "extra_content": {
                    "anthropic": {
                        "content": [
                            {
                                "type": "thinking",
                                "thinking": "Need compile evidence.",
                                "signature": "sig",
                            },
                            {"type": "text", "text": "I will compile."},
                            {
                                "type": "tool_use",
                                "id": "toolu_1",
                                "name": "compile_model",
                                "input": {},
                            },
                            {
                                "type": "tool_use",
                                "id": "toolu_2",
                                "name": "read_file",
                                "input": {"path": "model.py"},
                            },
                        ]
                    }
                },
            },
            {
                "role": "tool",
                "tool_call_id": "toolu_1",
                "name": "compile_model",
                "content": '{"result":"Compile passed cleanly."}',
            },
            {
                "role": "tool",
                "tool_call_id": "toolu_2",
                "name": "read_file",
                "content": '{"error":"not found"}',
            },
        ],
        tools=[],
    )

    assert payload["messages"][1]["content"][0]["type"] == "thinking"
    assert payload["messages"][1]["content"][0]["signature"] == "sig"
    assert payload["messages"][2] == {
        "role": "user",
        "content": [
            {
                "type": "tool_result",
                "tool_use_id": "toolu_1",
                "content": '{"result":"Compile passed cleanly."}',
            },
            {
                "type": "tool_result",
                "tool_use_id": "toolu_2",
                "content": '{"error":"not found"}',
                "is_error": True,
            },
        ],
    }


def test_anthropic_response_converts_text_thinking_tool_use_and_usage() -> None:
    provider = AnthropicLLM(dry_run=True)
    response = {
        "content": [
            {"type": "thinking", "thinking": "Planned compile.", "signature": "sig"},
            {"type": "text", "text": "I will compile."},
            {
                "type": "tool_use",
                "id": "toolu_1",
                "name": "compile_model",
                "input": {},
            },
        ],
        "stop_reason": "tool_use",
        "usage": {
            "input_tokens": 10,
            "cache_creation_input_tokens": 3,
            "cache_read_input_tokens": 7,
            "cache_creation": {
                "ephemeral_5m_input_tokens": 1,
                "ephemeral_1h_input_tokens": 2,
            },
            "output_tokens": 5,
        },
    }

    converted = provider._convert_response(response)

    assert converted["content"] == "I will compile."
    assert converted["thought_summary"] == "Planned compile."
    assert converted["tool_calls"] == [
        {
            "id": "toolu_1",
            "type": "function",
            "function": {"name": "compile_model", "arguments": "{}"},
        }
    ]
    assert converted["extra_content"]["anthropic"]["content"][0]["signature"] == "sig"
    assert converted["usage"] == {
        "prompt_tokens": 20,
        "candidates_tokens": 5,
        "total_tokens": 25,
        "cached_tokens": 7,
        "cache_creation_input_tokens": 3,
        "cache_read_input_tokens": 7,
        "cache_creation_5m_input_tokens": 1,
        "cache_creation_1h_input_tokens": 2,
    }


def test_anthropic_generate_uses_official_sdk_messages_create() -> None:
    response = {
        "content": [
            {"type": "thinking", "thinking": "Planned compile.", "signature": "sig"},
            {"type": "text", "text": "I will compile."},
        ],
        "usage": {
            "input_tokens": 10,
            "cache_read_input_tokens": 7,
            "output_tokens": 5,
        },
    }
    client = _FakeAnthropicClient([response])
    provider = AnthropicLLM(dry_run=True)
    provider._client = client

    converted = asyncio.run(
        provider.generate_with_tools(
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
    )

    assert converted["content"] == "I will compile."
    assert converted["thought_summary"] == "Planned compile."
    assert len(client.messages.calls) == 1
    call = client.messages.calls[0]
    assert call["model"] == "claude-opus-4-7"
    assert call["cache_control"] == {"type": "ephemeral"}
    assert call["system"] == [
        {"type": "text", "text": "system", "cache_control": {"type": "ephemeral"}}
    ]
    assert call["thinking"] == {"type": "adaptive"}
    assert call["output_config"] == {"effort": "high"}
    assert call["tools"][-1]["cache_control"] == {"type": "ephemeral"}


def test_anthropic_thinking_rejection_does_not_retry_without_thinking() -> None:
    provider = AnthropicLLM(model_id="claude-opus-4-7", dry_run=True)
    provider.max_attempts = 1
    client = _FakeAnthropicClient(
        [
            AnthropicAPIError(
                status_code=400,
                error_type="invalid_request_error",
                message="thinking is not supported for this model",
            )
        ]
    )
    provider._client = client

    try:
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "task"}],
                tools=[],
            )
        )
    except AnthropicAPIError:
        pass
    else:  # pragma: no cover - defensive assertion branch
        raise AssertionError("expected AnthropicAPIError")

    assert len(client.messages.calls) == 1
    assert client.messages.calls[0]["thinking"] == {"type": "adaptive"}
    assert client.messages.calls[0]["output_config"] == {"effort": "high"}


def test_anthropic_sdk_status_errors_are_normalized() -> None:
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    response = httpx.Response(
        429,
        headers={"request-id": "req_123", "retry-after": "2"},
        json={"error": {"type": "rate_limit_error", "message": "rate limited"}},
        request=request,
    )
    sdk_error = anthropic.RateLimitError(
        "rate limited",
        response=response,
        body=response.json(),
    )

    normalized = _api_error_from_sdk_exception(sdk_error)

    assert isinstance(normalized, AnthropicAPIError)
    assert normalized.status_code == 429
    assert normalized.error_type == "rate_limit_error"
    assert normalized.message == "rate limited"
    assert normalized.request_id == "req_123"
    assert normalized.retry_after == 2


def test_anthropic_generate_retries_normalized_sdk_rate_limit_error() -> None:
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    response = httpx.Response(
        429,
        headers={"request-id": "req_123", "retry-after": "0"},
        json={"error": {"type": "rate_limit_error", "message": "rate limited"}},
        request=request,
    )
    sdk_error = anthropic.RateLimitError(
        "rate limited",
        response=response,
        body=response.json(),
    )
    client = _FakeAnthropicClient(
        [
            sdk_error,
            {
                "content": [{"type": "text", "text": "done"}],
                "usage": {"input_tokens": 1, "output_tokens": 1},
            },
        ]
    )
    provider = AnthropicLLM(dry_run=True)
    provider._client = client
    provider.max_attempts = 2
    provider.retry_base_seconds = 0
    provider.retry_max_seconds = 0

    converted = asyncio.run(
        provider.generate_with_tools(
            system_prompt="system",
            messages=[{"role": "user", "content": "task"}],
            tools=[],
        )
    )

    assert converted["content"] == "done"
    assert len(client.messages.calls) == 2


def test_anthropic_close_closes_sdk_client() -> None:
    client = _FakeAnthropicClient([])
    provider = AnthropicLLM(dry_run=True)
    provider._client = client

    asyncio.run(provider.close())

    assert client.closed is True


def test_anthropic_thinking_rejection_from_sdk_does_not_retry_without_thinking() -> None:
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    response = httpx.Response(
        400,
        headers={"request-id": "req_123"},
        json={
            "error": {
                "type": "invalid_request_error",
                "message": "thinking is not supported for this model",
            }
        },
        request=request,
    )
    client = _FakeAnthropicClient(
        [
            anthropic.BadRequestError(
                "thinking is not supported for this model",
                response=response,
                body=response.json(),
            )
        ]
    )
    provider = AnthropicLLM(model_id="claude-opus-4-7", dry_run=True)
    provider._client = client
    provider.max_attempts = 1

    try:
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "task"}],
                tools=[],
            )
        )
    except AnthropicAPIError:
        pass
    else:  # pragma: no cover - defensive assertion branch
        raise AssertionError("expected AnthropicAPIError")

    assert len(client.messages.calls) == 1
    assert client.messages.calls[0]["thinking"] == {"type": "adaptive"}
    assert client.messages.calls[0]["output_config"] == {"effort": "high"}


def test_anthropic_retry_predicate_treats_sdk_connection_errors_as_transient() -> None:
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    exc = anthropic.APIConnectionError(request=request)

    normalized = _api_error_from_sdk_exception(exc)

    assert _should_retry_anthropic_exception(normalized) is True


def test_anthropic_retry_predicate_treats_rate_limits_as_transient() -> None:
    exc = AnthropicAPIError(
        status_code=429,
        error_type="rate_limit_error",
        message="rate limited",
        retry_after=1,
    )

    assert _should_retry_anthropic_exception(exc) is True


def test_anthropic_retry_predicate_treats_request_too_large_as_terminal() -> None:
    exc = AnthropicAPIError(
        status_code=413,
        error_type="request_too_large",
        message="too large",
    )

    assert _should_retry_anthropic_exception(exc) is False


def test_anthropic_retry_predicate_treats_url_errors_as_transient() -> None:
    assert _should_retry_anthropic_exception(urllib.error.URLError("disconnected")) is True


def test_anthropic_async_retry_honors_retry_after() -> None:
    attempts = 0
    sleeps: list[float] = []

    async def flaky() -> str:
        nonlocal attempts
        attempts += 1
        if attempts < 2:
            raise AnthropicAPIError(
                status_code=529,
                error_type="overloaded_error",
                message="overloaded",
                retry_after=3,
            )
        return "ok"

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)

    result = asyncio.run(
        _async_retry(
            flaky,
            max_attempts=2,
            should_retry=_should_retry_anthropic_exception,
            retry_after=lambda exc: exc.retry_after if isinstance(exc, AnthropicAPIError) else None,
            base_delay=1,
            max_delay=10,
            logger=SimpleNamespace(warning=lambda *args, **kwargs: None),
            context="anthropic[test]",
            sleep_fn=fake_sleep,
            rng=lambda: 0.5,
        )
    )

    assert result == "ok"
    assert sleeps == [3]
