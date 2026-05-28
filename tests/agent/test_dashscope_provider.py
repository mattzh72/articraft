from __future__ import annotations

import asyncio
from types import SimpleNamespace

from agent.providers.dashscope import (
    DEFAULT_DASHSCOPE_BASE_URL,
    DEFAULT_DASHSCOPE_MODEL,
    DashScopeLLM,
    dashscope_api_key_from_env,
    dashscope_api_keys_from_env,
)


def test_dashscope_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = dashscope_api_keys_from_env(
        {
            "DASHSCOPE_API_KEY": "sk-primary",
            "DASHSCOPE_API_KEYS": "sk-primary,\nsk-secondary, sk-secondary",
        }
    )

    assert keys == ["sk-primary", "sk-secondary"]


def test_dashscope_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = dashscope_api_key_from_env({"DASHSCOPE_API_KEYS": "sk-first,sk-second"})

    assert key in {"sk-first", "sk-second"}


def test_dashscope_default_model_is_qwen_flash() -> None:
    provider = DashScopeLLM(dry_run=True)

    assert provider.model_id == DEFAULT_DASHSCOPE_MODEL


def test_dashscope_request_preview_uses_compatible_chat_shape() -> None:
    provider = DashScopeLLM(dry_run=True)
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

    assert payload["base_url"] == DEFAULT_DASHSCOPE_BASE_URL
    assert payload["model"] == DEFAULT_DASHSCOPE_MODEL
    assert payload["messages"] == [
        {"role": "system", "content": "system"},
        {"role": "user", "content": "task"},
    ]
    assert payload["tools"][0]["type"] == "function"
    assert payload["extra_body"] == {"enable_thinking": True}
    assert "max_tokens" not in payload


def test_dashscope_generate_uses_chat_completions() -> None:
    provider = DashScopeLLM(dry_run=True)
    provider._client = object()
    provider._client_is_async = True

    captured: dict[str, object] = {}

    async def fake_chat_completion(request_payload: dict) -> SimpleNamespace:
        captured.update(request_payload)
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

    assert captured["model"] == DEFAULT_DASHSCOPE_MODEL
    assert captured["extra_body"] == {"enable_thinking": True}
    assert response["content"] == "ok"
