from __future__ import annotations

import asyncio
import logging
import sys
from types import SimpleNamespace

import pytest

from agent.providers.openai import (
    OpenAILLM,
    _OpenAIWebSocketError,
    openai_api_key_from_env,
    openai_api_keys_from_env,
)


class _FakeOpenAIError(RuntimeError):
    def __init__(self, message: str, *, status_code: int) -> None:
        self.status_code = status_code
        super().__init__(message)


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
