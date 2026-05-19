from __future__ import annotations

import pytest

from agent.providers.anthropic import AnthropicLLM
from agent.providers.gemini import GeminiLLM
from agent.providers.openai import OpenAILLM
from agent.providers.openrouter import OpenRouterLLM
from agent.tools.base import make_tool_schema


def _compile_tool_schema() -> dict:
    return make_tool_schema(
        "compile_model",
        "Compile the current model.",
        parameters={},
        required=[],
    )


def _tool_conversation() -> list[dict]:
    return [
        {"role": "user", "content": "build a latch"},
        {
            "role": "assistant",
            "content": "I will compile the current draft.",
            "tool_calls": [
                {
                    "id": "call_compile",
                    "type": "function",
                    "function": {
                        "name": "compile_model",
                        "arguments": "{}",
                    },
                }
            ],
        },
        {
            "role": "tool",
            "tool_call_id": "call_compile",
            "name": "compile_model",
            "content": '{"result":"Compile passed cleanly."}',
        },
    ]


def test_openai_preview_preserves_replayed_tool_call_chain() -> None:
    provider = OpenAILLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=_tool_conversation(),
        tools=[_compile_tool_schema()],
    )

    assert payload["tools"][0]["name"] == "compile_model"
    assert payload["input"] == [
        {
            "role": "user",
            "content": [{"type": "input_text", "text": "build a latch"}],
        },
        {
            "role": "assistant",
            "content": [{"type": "input_text", "text": "I will compile the current draft."}],
        },
        {
            "type": "function_call",
            "call_id": "call_compile",
            "name": "compile_model",
            "arguments": "{}",
        },
        {
            "type": "function_call_output",
            "call_id": "call_compile",
            "output": '{"result":"Compile passed cleanly."}',
        },
    ]


def test_gemini_preview_preserves_replayed_tool_call_chain() -> None:
    provider = GeminiLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=_tool_conversation(),
        tools=[_compile_tool_schema()],
    )

    declaration = payload["config"]["tools"][0]["function_declarations"][0]
    assert declaration["name"] == "compile_model"

    contents = payload["contents"]
    assert contents[0] == {"role": "user", "parts": [{"text": "build a latch"}]}
    assert contents[1]["role"] == "model"
    assert contents[1]["parts"][0]["text"] == "I will compile the current draft."
    assert contents[1]["parts"][1]["function_call"] == {
        "name": "compile_model",
        "args": {},
    }
    assert contents[2] == {
        "role": "user",
        "parts": [
            {
                "function_response": {
                    "name": "compile_model",
                    "response": {"result": "Compile passed cleanly."},
                }
            }
        ],
    }


def test_openrouter_preview_preserves_replayed_tool_call_chain() -> None:
    provider = OpenRouterLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=_tool_conversation(),
        tools=[_compile_tool_schema()],
    )

    assert payload["tools"][0]["function"]["name"] == "compile_model"
    assert payload["messages"][:4] == [
        {"role": "system", "content": "system"},
        {"role": "user", "content": "build a latch"},
        {
            "role": "assistant",
            "content": "I will compile the current draft.",
            "tool_calls": [
                {
                    "id": "call_compile",
                    "type": "function",
                    "function": {
                        "name": "compile_model",
                        "arguments": "{}",
                    },
                }
            ],
        },
        {
            "role": "tool",
            "tool_call_id": "call_compile",
            "content": '{"result":"Compile passed cleanly."}',
            "name": "compile_model",
        },
    ]


def test_anthropic_preview_preserves_replayed_tool_call_chain() -> None:
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=_tool_conversation(),
        tools=[_compile_tool_schema()],
    )

    assert payload["tools"][0]["name"] == "compile_model"
    assert payload["messages"] == [
        {"role": "user", "content": "build a latch"},
        {
            "role": "assistant",
            "content": [
                {"type": "text", "text": "I will compile the current draft."},
                {
                    "type": "tool_use",
                    "id": "call_compile",
                    "name": "compile_model",
                    "input": {},
                },
            ],
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "tool_result",
                    "tool_use_id": "call_compile",
                    "content": '{"result":"Compile passed cleanly."}',
                }
            ],
        },
    ]


@pytest.mark.parametrize(
    "provider",
    [
        AnthropicLLM(dry_run=True),
        OpenAILLM(dry_run=True),
        GeminiLLM(dry_run=True),
        OpenRouterLLM(dry_run=True),
    ],
)
def test_provider_previews_accept_required_tool_schema(provider: object) -> None:
    payload = provider.build_request_preview(  # type: ignore[attr-defined]
        system_prompt="system",
        messages=[{"role": "user", "content": "build a latch"}],
        tools=[_compile_tool_schema()],
    )

    serialized = str(payload)
    assert "compile_model" in serialized
    assert "Compile the current model." in serialized
