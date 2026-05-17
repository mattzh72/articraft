from __future__ import annotations

import asyncio
import hashlib
import json
import os
import re
import shlex
import tempfile
import uuid
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from agent.providers.base import PrepareRequestResult, ProviderTraceEvent
from articraft.values import reasoning_level_alias

DEFAULT_CODEX_CLI_MODEL = "codex-cli-default"
DEFAULT_CODEX_CLI_TIMEOUT_SECONDS = 900.0


@dataclass(slots=True, frozen=True)
class CodexCliExecResult:
    returncode: int
    stdout: str
    stderr: str
    last_message: str


CodexCliRunner = Callable[[list[str], str, float, Path], Awaitable[CodexCliExecResult]]


class CodexCliLLM:
    """No-key Codex CLI provider used behind the normal Articraft harness loop."""

    def __init__(
        self,
        model_id: str = DEFAULT_CODEX_CLI_MODEL,
        *,
        thinking_level: str = "high",
        dry_run: bool = False,
        runner: CodexCliRunner | None = None,
    ) -> None:
        self.model_id = model_id or DEFAULT_CODEX_CLI_MODEL
        self.thinking_level = thinking_level
        self.dry_run = dry_run
        self._runner = runner or _run_codex_exec
        self.timeout_seconds = _env_float(
            "ARTICRAFT_CODEX_CLI_TIMEOUT_SECONDS",
            DEFAULT_CODEX_CLI_TIMEOUT_SECONDS,
        )

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        return {
            "transport": "codex-cli",
            "model": self.model_id,
            "thinking_level": self.thinking_level,
            "command": self._base_command(
                schema_path=Path("<schema.json>"),
                output_path=Path("<last-message.json>"),
                image_paths=_image_paths_from_messages(messages),
            ),
            "prompt": _render_codex_prompt(
                system_prompt=system_prompt,
                messages=messages,
                tools=tools,
                thinking_level=self.thinking_level,
            ),
        }

    async def prepare_next_request(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
        completed_turns: int,
        consecutive_compile_failure_count: int = 0,
        last_compile_failure_sig: str | None = None,
    ) -> PrepareRequestResult:
        prompt = _render_codex_prompt(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
            thinking_level=self.thinking_level,
        )
        event = ProviderTraceEvent(
            event_type="codex_cli_request",
            payload={
                "provider": "codex-cli",
                "model_id": self.model_id,
                "thinking_level": self.thinking_level,
                "completed_turns": completed_turns,
                "message_count": len(messages),
                "tool_names": _tool_names(tools),
                "image_paths": _image_paths_from_messages(messages),
                "prompt_sha256": hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
                "prompt_chars": len(prompt),
                "consecutive_compile_failure_count": consecutive_compile_failure_count,
                "last_compile_failure_sig": last_compile_failure_sig,
            },
        )
        return PrepareRequestResult(trace_events=[event])

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict[str, Any]:
        if self.dry_run:
            raise RuntimeError("Codex CLI transport is unavailable in dry_run mode")

        with tempfile.TemporaryDirectory(prefix="articraft_codex_cli_") as tmp:
            tmp_dir = Path(tmp)
            schema_path = tmp_dir / "assistant_turn.schema.json"
            output_path = tmp_dir / "assistant_turn.json"
            schema_path.write_text(json.dumps(_OUTPUT_SCHEMA, indent=2) + "\n", encoding="utf-8")
            command = self._base_command(
                schema_path=schema_path,
                output_path=output_path,
                image_paths=_image_paths_from_messages(messages),
            )
            prompt = _render_codex_prompt(
                system_prompt=system_prompt,
                messages=messages,
                tools=tools,
                thinking_level=self.thinking_level,
            )
            result = await self._runner(command, prompt, self.timeout_seconds, output_path)

        if result.returncode != 0:
            raise RuntimeError(_format_codex_error(result))
        payload = _parse_last_message(result.last_message)
        return _convert_payload_to_provider_response(
            payload,
            model_id=self.model_id,
            command=command,
            stdout=result.stdout,
            stderr=result.stderr,
        )

    async def close(self) -> None:
        return None

    def _base_command(
        self,
        *,
        schema_path: Path,
        output_path: Path,
        image_paths: list[str] | None = None,
    ) -> list[str]:
        binary = os.environ.get("ARTICRAFT_CODEX_CLI_BIN", "codex").strip() or "codex"
        command = [
            binary,
            "exec",
            "--skip-git-repo-check",
            "--ephemeral",
            "--ignore-rules",
            "--sandbox",
            os.environ.get("ARTICRAFT_CODEX_CLI_SANDBOX", "read-only").strip() or "read-only",
            "--color",
            "never",
            "--output-schema",
            str(schema_path),
            "--output-last-message",
            str(output_path),
            "-C",
            str(Path.cwd().resolve()),
        ]
        if _should_pass_model_id(self.model_id):
            command.extend(["--model", self.model_id])
        for image_path in image_paths or []:
            command.extend(["--image", image_path])
        reasoning = reasoning_level_alias(self.thinking_level)
        if reasoning:
            command.extend(["-c", f'model_reasoning_effort="{reasoning}"'])
        extra_args = os.environ.get("ARTICRAFT_CODEX_CLI_EXTRA_ARGS", "").strip()
        if extra_args:
            command.extend(shlex.split(extra_args))
        command.append("-")
        return command


async def _run_codex_exec(
    command: list[str],
    prompt: str,
    timeout_seconds: float,
    output_path: Path,
) -> CodexCliExecResult:
    try:
        process = await asyncio.create_subprocess_exec(
            *command,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "Codex CLI provider requires the `codex` executable. "
            "Install/login to Codex CLI or set ARTICRAFT_CODEX_CLI_BIN."
        ) from exc

    try:
        stdout_bytes, stderr_bytes = await asyncio.wait_for(
            process.communicate(prompt.encode("utf-8")),
            timeout=timeout_seconds,
        )
    except asyncio.TimeoutError:
        process.kill()
        await process.wait()
        raise RuntimeError(f"Codex CLI timed out after {timeout_seconds:.0f}s")

    last_message = output_path.read_text(encoding="utf-8") if output_path.exists() else ""
    return CodexCliExecResult(
        returncode=int(process.returncode or 0),
        stdout=stdout_bytes.decode("utf-8", errors="replace"),
        stderr=stderr_bytes.decode("utf-8", errors="replace"),
        last_message=last_message,
    )


def _render_codex_prompt(
    *,
    system_prompt: str,
    messages: list[dict],
    tools: list[dict],
    thinking_level: str,
) -> str:
    return "\n\n".join(
        [
            "You are the Codex CLI transport for Articraft's internal harness.",
            (
                "Return exactly one assistant turn as JSON matching the provided schema. "
                "Do not edit files, run shell commands, or perform work outside this JSON response. "
                "Articraft will execute any tool calls and will handle compile, retry, trace, "
                "and record persistence."
            ),
            f"Requested reasoning level: {reasoning_level_alias(thinking_level) or thinking_level}",
            "<system_prompt>\n" + system_prompt.strip() + "\n</system_prompt>",
            "<available_tools>\n"
            + json.dumps(_tool_reference(tools), indent=2, ensure_ascii=False)
            + "\n</available_tools>",
            "<conversation>\n"
            + json.dumps(_conversation_reference(messages), indent=2, ensure_ascii=False)
            + "\n</conversation>",
            (
                "Tool call rules:\n"
                "- Use tool_calls when the next Articraft action should read/edit/probe/compile.\n"
                "- Each tool call must name one available tool and provide arguments as a "
                "JSON-encoded object string.\n"
                "- Use content only when you are concluding or explaining a blocker.\n"
                "- If the latest compile succeeded and no defect remains, return no tool_calls and "
                "a concise completion message."
            ),
        ]
    )


def _conversation_reference(messages: list[dict]) -> list[dict[str, Any]]:
    rendered: list[dict[str, Any]] = []
    for message in messages:
        if not isinstance(message, dict):
            continue
        item: dict[str, Any] = {"role": str(message.get("role") or "")}
        content = message.get("content")
        item["content"] = _render_message_content(content)
        if message.get("name"):
            item["name"] = message.get("name")
        if message.get("tool_call_id"):
            item["tool_call_id"] = message.get("tool_call_id")
        tool_calls = message.get("tool_calls")
        if isinstance(tool_calls, list) and tool_calls:
            item["tool_calls"] = tool_calls
        rendered.append(item)
    return rendered


def _render_message_content(content: Any) -> Any:
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return content
    rendered: list[dict[str, Any]] = []
    for part in content:
        if not isinstance(part, dict):
            rendered.append({"type": "value", "value": part})
            continue
        part_type = str(part.get("type") or "")
        if part_type in {"input_text", "text"}:
            rendered.append({"type": "text", "text": str(part.get("text") or "")})
        elif part_type in {"input_image", "image"}:
            rendered.append(
                {
                    "type": "image",
                    "image_path": str(part.get("image_path") or part.get("path") or ""),
                    "detail": str(part.get("detail") or ""),
                }
            )
        else:
            rendered.append(dict(part))
    return rendered


def _image_paths_from_messages(messages: list[dict]) -> list[str]:
    paths: list[str] = []
    seen: set[str] = set()
    for message in messages:
        if not isinstance(message, dict):
            continue
        content = message.get("content")
        if not isinstance(content, list):
            continue
        for part in content:
            if not isinstance(part, dict):
                continue
            if part.get("type") not in {"input_image", "image"}:
                continue
            path = str(part.get("image_path") or part.get("path") or "").strip()
            if not path or path in seen:
                continue
            seen.add(path)
            paths.append(path)
    return paths


def _tool_reference(tools: list[dict]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for tool in tools:
        if not isinstance(tool, dict) or tool.get("type") != "function":
            continue
        function = tool.get("function")
        if not isinstance(function, dict):
            continue
        normalized.append(
            {
                "name": str(function.get("name") or ""),
                "description": str(function.get("description") or ""),
                "parameters": function.get("parameters") or {"type": "object"},
            }
        )
    return normalized


def _tool_names(tools: list[dict]) -> list[str]:
    return [item["name"] for item in _tool_reference(tools) if item["name"]]


def _parse_last_message(raw: str) -> dict[str, Any]:
    text = raw.strip()
    if not text:
        raise RuntimeError("Codex CLI did not write an assistant response")
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise RuntimeError("Codex CLI assistant response must be a JSON object")
    return payload


def _convert_payload_to_provider_response(
    payload: dict[str, Any],
    *,
    model_id: str,
    command: list[str],
    stdout: str,
    stderr: str,
) -> dict[str, Any]:
    tool_calls: list[dict[str, Any]] = []
    raw_tool_calls = payload.get("tool_calls")
    if isinstance(raw_tool_calls, list):
        for item in raw_tool_calls:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "").strip()
            arguments = _coerce_tool_arguments(item.get("arguments"))
            tool_calls.append(
                {
                    "id": str(item.get("id") or f"call_codex_{uuid.uuid4().hex}"),
                    "type": "function",
                    "function": {
                        "name": name,
                        "arguments": json.dumps(
                            arguments,
                            ensure_ascii=False,
                            separators=(",", ":"),
                        ),
                    },
                }
            )

    result: dict[str, Any] = {
        "content": str(payload.get("content") or ""),
        "tool_calls": tool_calls,
        "extra_content": {
            "codex_cli": {
                "model_id": model_id,
                "command": _redacted_command(command),
                "raw_response": payload,
            }
        },
    }
    thought_summary = payload.get("thought_summary")
    if isinstance(thought_summary, str) and thought_summary.strip():
        result["thought_summary"] = thought_summary.strip()
    if stdout.strip() or stderr.strip():
        result["extra_content"]["codex_cli"]["stdio"] = {
            "stdout": stdout[-4000:],
            "stderr": stderr[-4000:],
        }
    usage = _extract_usage_from_stdio(stdout=stdout, stderr=stderr)
    if usage:
        result["usage"] = usage
    return result


def _extract_usage_from_stdio(*, stdout: str, stderr: str) -> dict[str, int] | None:
    combined = "\n".join(part for part in (stdout, stderr) if part)
    matches = re.findall(r"tokens used\s*(?:\n|\r\n|\s)+([0-9][0-9,]*)", combined)
    if not matches:
        return None
    total = int(matches[-1].replace(",", ""))
    return {"total_tokens": total}


def _coerce_tool_arguments(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}
    return {}


def _redacted_command(command: list[str]) -> list[str]:
    redacted: list[str] = []
    skip_next = False
    for arg in command:
        if skip_next:
            redacted.append("<path>")
            skip_next = False
            continue
        redacted.append(arg)
        if arg in {"--output-schema", "--output-last-message"}:
            skip_next = True
    return redacted


def _format_codex_error(result: CodexCliExecResult) -> str:
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    details = stderr or stdout or "(no output)"
    return f"Codex CLI exited with status {result.returncode}: {details[-4000:]}"


def _should_pass_model_id(model_id: str) -> bool:
    return bool(model_id and model_id != DEFAULT_CODEX_CLI_MODEL)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        value = float(raw)
    except ValueError:
        return default
    return value if value > 0 else default


_OUTPUT_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "content": {"type": "string"},
        "thought_summary": {"type": "string"},
        "tool_calls": {
            "type": "array",
            "items": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "name": {"type": "string"},
                    "arguments": {
                        "type": "string",
                        "description": "JSON object string with the selected tool arguments.",
                    },
                },
                "required": ["name", "arguments"],
            },
        },
    },
    "required": ["content", "thought_summary", "tool_calls"],
}
