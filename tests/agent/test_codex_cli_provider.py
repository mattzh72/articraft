from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent.payload_preview import build_provider_payload_preview
from agent.prompts import (
    CODEX_CLI_DESIGNER_PROMPT_NAME,
    DESIGNER_PROMPT_NAME,
    resolve_system_prompt_path,
)
from agent.providers.codex_cli import CodexCliExecResult, CodexCliLLM, _redacted_command
from agent.providers.factory import ProviderConfig, create_provider_client, default_model_id


def test_codex_cli_requires_explicit_model(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ARTICRAFT_CODEX_MODEL", raising=False)

    with pytest.raises(ValueError, match="requires an explicit model"):
        default_model_id(ProviderConfig(provider="codex-cli"))


def test_codex_cli_uses_env_model_when_configured(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ARTICRAFT_CODEX_MODEL", "codex/gpt-5.5")

    assert default_model_id(ProviderConfig(provider="codex-cli")) == "codex/gpt-5.5"


def test_codex_cli_prompt_resolution_and_payload_preview() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    codex_cli_resolved = resolve_system_prompt_path(
        DESIGNER_PROMPT_NAME,
        provider="codex-cli",
        repo_root=repo_root,
    )
    assert codex_cli_resolved.name == CODEX_CLI_DESIGNER_PROMPT_NAME

    payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="codex-cli",
        model_id="codex/gpt-5.5",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )

    assert payload["transport"] == "codex-cli"
    assert "Codex CLI behind Articraft's internal harness" in payload["prompt"]
    assert (
        "Available tools: `read_file`, `apply_patch`, `replace`, `write_file`, `compile_model`, `probe_model`, and `find_examples`."
        in payload["prompt"]
    )


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


def test_codex_cli_redacts_local_paths_from_persisted_command_metadata() -> None:
    assert _redacted_command(
        [
            "codex",
            "exec",
            "--output-schema",
            "/var/folders/private/schema.json",
            "--image",
            "/Users/example/private/reference.png",
            "--output-last-message",
            "/var/folders/private/assistant.json",
            "-",
        ]
    ) == [
        "codex",
        "exec",
        "--output-schema",
        "<path>",
        "--image",
        "<path>",
        "--output-last-message",
        "<path>",
        "-",
    ]


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
            stdout="  Tokens used: 1,234\n",
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


def test_codex_cli_output_schema_enumerates_available_tool_names() -> None:
    captured: dict[str, object] = {}

    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        schema_path = Path(command[command.index("--output-schema") + 1])
        captured["schema"] = json.loads(schema_path.read_text(encoding="utf-8"))
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "",
                    "tool_calls": [
                        {
                            "name": "compile_model",
                            "arguments": "{}",
                        }
                    ],
                }
            ),
        )

    provider = CodexCliLLM(model_id="codex/gpt-5.5", runner=fake_runner)

    asyncio.run(
        provider.generate_with_tools(
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
        )
    )

    name_schema = captured["schema"]["properties"]["tool_calls"]["items"]["properties"]["name"]
    arguments_schema = captured["schema"]["properties"]["tool_calls"]["items"]["properties"][
        "arguments"
    ]
    assert name_schema["enum"] == ["compile_model"]
    assert arguments_schema["type"] == "string"


def test_codex_cli_rejects_invalid_tool_call_schema() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "",
                    "tool_calls": [
                        {
                            "name": "compile_model",
                            "arguments": "{not json}",
                        }
                    ],
                }
            ),
        )

    provider = CodexCliLLM(model_id="codex/gpt-5.5", runner=fake_runner)

    with pytest.raises(RuntimeError, match=r"tool_call\[1\] field 'arguments' must be valid JSON"):
        asyncio.run(
            provider.generate_with_tools(
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
            )
        )


def test_codex_cli_rejects_non_object_tool_call() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "",
                    "tool_calls": ["bad"],
                }
            ),
        )

    provider = CodexCliLLM(model_id="codex/gpt-5.5", runner=fake_runner)

    with pytest.raises(RuntimeError, match=r"tool_call\[1\] must be a JSON object"):
        asyncio.run(
            provider.generate_with_tools(
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
            )
        )


def test_codex_cli_generate_reuses_seen_images_on_later_turns() -> None:
    commands: list[list[str]] = []

    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        commands.append(command)
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "Done.",
                    "thought_summary": "",
                    "tool_calls": [],
                }
            ),
        )

    provider = CodexCliLLM(runner=fake_runner)
    tools: list[dict[str, object]] = []

    asyncio.run(
        provider.generate_with_tools(
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
            tools=tools,
        )
    )
    asyncio.run(
        provider.generate_with_tools(
            system_prompt="system",
            messages=[{"role": "user", "content": "continue"}],
            tools=tools,
        )
    )

    assert commands[0].count("--image") == 1
    assert commands[1].count("--image") == 1
    assert "/tmp/reference.png" in commands[1]


def test_codex_cli_prepare_next_request_reuses_seen_images_in_trace_metadata() -> None:
    provider = CodexCliLLM(dry_run=True)

    asyncio.run(
        provider.prepare_next_request(
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
            tools=[],
            completed_turns=0,
        )
    )
    result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=[{"role": "user", "content": "continue"}],
            tools=[],
            completed_turns=1,
        )
    )

    assert result.trace_events[0].payload["image_paths"] == ["/tmp/reference.png"]


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


def test_codex_cli_compacts_old_history_before_later_request() -> None:
    prompts: list[str] = []

    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        prompts.append(prompt)
        if "Summarize Articraft Codex CLI harness history" in prompt:
            return CodexCliExecResult(
                returncode=0,
                stdout="tokens used\n321\n",
                stderr="",
                last_message=json.dumps(
                    {
                        "summary": (
                            "The object is a folding chair. Earlier attempts fixed the hinge axis; "
                            "continue from the current model.py and compile before finishing."
                        )
                    }
                ),
            )
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "Done.",
                    "thought_summary": "",
                    "tool_calls": [],
                }
            ),
        )

    provider = CodexCliLLM(runner=fake_runner)
    provider.compaction_threshold_chars = 100
    provider.compaction_tail_messages = 2
    messages = [
        {"role": "user", "content": "SDK docs"},
        {"role": "user", "content": "make a folding chair"},
        *[
            {
                "role": "tool",
                "tool_call_id": f"call_{index}",
                "name": "compile_model",
                "content": f"old compile failure {index}: hinge axis wrong",
            }
            for index in range(8)
        ],
    ]

    prepare_result = asyncio.run(
        provider.prepare_next_request(
            system_prompt="system",
            messages=messages,
            tools=[],
            completed_turns=5,
        )
    )
    response = asyncio.run(
        provider.generate_with_tools(
            system_prompt="system",
            messages=messages,
            tools=[],
        )
    )

    compaction_events = [
        event for event in prepare_result.trace_events if event.event_type == "codex_cli_compaction"
    ]
    assert compaction_events
    assert compaction_events[0].payload["usage"] == {"total_tokens": 321}
    assert response["content"] == "Done."
    assert len(prompts) == 2
    assert "old compile failure 0" in prompts[0]
    assert "<codex_cli_compacted_history>" in prompts[1]
    assert "Earlier attempts fixed the hinge axis" in prompts[1]
    assert "old compile failure 0" not in prompts[1]
    assert "old compile failure 7" in prompts[1]


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


def test_codex_cli_generate_rejects_invalid_last_message_json() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message="not json",
        )

    provider = CodexCliLLM(runner=fake_runner)

    with pytest.raises(RuntimeError, match="not valid JSON"):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "make a hinge"}],
                tools=[],
            )
        )


def test_codex_cli_generate_rejects_missing_required_response_field() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps({"content": "", "tool_calls": []}),
        )

    provider = CodexCliLLM(runner=fake_runner)

    with pytest.raises(RuntimeError, match="missing required field\\(s\\): thought_summary"):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "make a hinge"}],
                tools=[],
            )
        )


def test_codex_cli_generate_rejects_non_list_tool_calls() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "",
                    "tool_calls": {"name": "read_file", "arguments": "{}"},
                }
            ),
        )

    provider = CodexCliLLM(runner=fake_runner)

    with pytest.raises(RuntimeError, match="'tool_calls' must be a list"):
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "make a hinge"}],
                tools=[],
            )
        )


def test_codex_cli_generate_passes_tool_call_payloads_like_openai_codec() -> None:
    async def fake_runner(
        command: list[str],
        prompt: str,
        timeout_seconds: float,
        output_path: Path,
    ) -> CodexCliExecResult:
        return CodexCliExecResult(
            returncode=0,
            stdout="",
            stderr="",
            last_message=json.dumps(
                {
                    "content": "",
                    "thought_summary": "",
                    "tool_calls": [
                        {
                            "name": "delete_everything",
                            "arguments": "{not json",
                            "extra": "preserved in raw response only",
                        }
                    ],
                }
            ),
        )

    provider = CodexCliLLM(runner=fake_runner)

    with pytest.raises(RuntimeError, match=r"tool_call\[1\] field 'arguments' must be valid JSON"):
        asyncio.run(
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


def test_factory_creates_codex_cli_provider() -> None:
    provider = create_provider_client(
        ProviderConfig(provider="codex-cli", model_id="codex/gpt-5.5"),
        dry_run=True,
    )

    assert isinstance(provider, CodexCliLLM)
