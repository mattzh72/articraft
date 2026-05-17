from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent.providers.codex_cli import CodexCliExecResult, CodexCliLLM
from agent.providers.factory import ProviderConfig, create_provider_client, default_model_id


def test_codex_cli_default_model_is_no_key_sentinel() -> None:
    assert default_model_id(ProviderConfig(provider="codex-cli")) == "codex-cli-default"


def test_codex_cli_request_preview_describes_subprocess_transport() -> None:
    provider = CodexCliLLM(dry_run=True)

    preview = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": "make a hinge"},
                    {"type": "input_image", "image_path": "/tmp/reference.png"},
                ],
            }
        ],
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "compile_model",
                    "description": "Compile current model",
                    "parameters": {"type": "object", "properties": {}},
                },
            }
        ],
    )

    assert preview["transport"] == "codex-cli"
    assert preview["model"] == "codex-cli-default"
    assert preview["command"][:2] == ["codex", "exec"]
    assert "--ask-for-approval" not in preview["command"]
    assert "--image" in preview["command"]
    assert "/tmp/reference.png" in preview["command"]
    assert "--output-schema" in preview["command"]
    assert '"name": "compile_model"' in preview["prompt"]
    assert "JSON-encoded object string" in preview["prompt"]
    assert "make a hinge" in preview["prompt"]


def test_codex_cli_generate_converts_schema_payload_to_tool_calls() -> None:
    captured: dict[str, object] = {}

    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        captured["command"] = command
        captured["prompt"] = prompt
        captured["timeout_seconds"] = timeout_seconds
        captured["output_path"] = output_path
        return CodexCliExecResult(
            returncode=0,
            stdout="tokens used\n1,234\n",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "Need to inspect current model.",
                    "tool_calls": [
                        {
                            "name": "read_file",
                            "arguments": json.dumps({"path": "model.py", "offset": 1}),
                        }
                    ],
                }
            ),
        )

    provider = CodexCliLLM(
        model_id="gpt-5.5",
        thinking_level="med",
        runner=fake_runner,
    )

    response = asyncio.run(
        provider.generate_with_tools(
            system_prompt="system",
            messages=[{"role": "user", "content": "make a hinge"}],
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "read_file",
                        "description": "Read a file",
                        "parameters": {"type": "object", "properties": {}},
                    },
                }
            ],
        )
    )

    assert captured["command"][:2] == ["codex", "exec"]
    assert "--ask-for-approval" not in captured["command"]
    assert "--model" in captured["command"]
    assert "gpt-5.5" in captured["command"]
    assert 'model_reasoning_effort="medium"' in captured["command"]
    assert "Do not edit files, run shell commands" in str(captured["prompt"])
    assert response["thought_summary"] == "Need to inspect current model."
    assert response["usage"] == {"total_tokens": 1234}
    assert response["tool_calls"][0]["function"]["name"] == "read_file"
    assert json.loads(response["tool_calls"][0]["function"]["arguments"]) == {
        "path": "model.py",
        "offset": 1,
    }
    assert response["extra_content"]["codex_cli"]["raw_response"]["tool_calls"]


def test_codex_cli_prepare_next_request_emits_trace_metadata() -> None:
    provider = CodexCliLLM(dry_run=True)

    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[{"role": "user", "content": "make a hinge"}],
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "compile_model",
                        "description": "Compile current model",
                        "parameters": {"type": "object", "properties": {}},
                    },
                }
            ],
            completed_turns=3,
            consecutive_compile_failure_count=1,
            last_compile_failure_sig="abc123",
        )
    )

    event = result.trace_events[0]
    assert event.event_type == "codex_cli_request"
    assert event.payload["provider"] == "codex-cli"
    assert event.payload["completed_turns"] == 3
    assert event.payload["tool_names"] == ["compile_model"]
    assert event.payload["prompt_chars"] > 0
    assert len(event.payload["prompt_sha256"]) == 64
    assert event.payload["consecutive_compile_failure_count"] == 1
    assert event.payload["last_compile_failure_sig"] == "abc123"


def test_codex_cli_generate_surfaces_subprocess_failure() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=2,
            stdout="",
            stderr="not logged in",
            last_message="",
        )

    provider = CodexCliLLM(runner=fake_runner)

    with pytest.raises(RuntimeError, match="not logged in"):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "make a hinge"}],
                tools=[],
            )
        )


def test_factory_creates_codex_cli_provider() -> None:
    provider = create_provider_client(ProviderConfig(provider="codex-cli"), dry_run=True)

    assert isinstance(provider, CodexCliLLM)
