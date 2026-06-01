from __future__ import annotations

import asyncio
import hashlib
import json
import os
import shlex
import tempfile
import uuid
from collections.abc import Awaitable, Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from agent.providers.base import (
    ConversationMessage,
    PrepareRequestResult,
    ProviderResponse,
    ProviderTraceEvent,
    ToolSchema,
)
from articraft.values import reasoning_level_alias

DEFAULT_CODEX_CLI_MODEL = "codex-cli-default"
DEFAULT_CODEX_CLI_TIMEOUT_SECONDS = 900.0
DEFAULT_CODEX_CLI_COMPACTION_THRESHOLD_CHARS = 160_000
DEFAULT_CODEX_CLI_COMPACTION_TAIL_MESSAGES = 8
_CODEX_CLI_COMPACTION_MESSAGE_NAME = "codex_cli_compacted_history"


@dataclass(slots=True, frozen=True)
class CodexCliExecResult:
    returncode: int
    stdout: str
    stderr: str
    last_message: str


@dataclass(slots=True, frozen=True)
class CodexCliRequest:
    prompt: str
    image_paths: list[str]


CodexCliRunner = Callable[[list[str], str, float, Path], Awaitable[CodexCliExecResult]]

_ASSISTANT_TURN_KEYS = frozenset({"content", "thought_summary", "tool_calls"})


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
        self._persistent_image_paths: list[str] = []
        self._compaction_summary: str | None = None
        self._compacted_message_count = 0
        self.timeout_seconds = _env_float(
            "ARTICRAFT_CODEX_CLI_TIMEOUT_SECONDS",
            DEFAULT_CODEX_CLI_TIMEOUT_SECONDS,
        )
        self.compaction_threshold_chars = _env_int(
            "ARTICRAFT_CODEX_CLI_COMPACTION_CHARS",
            DEFAULT_CODEX_CLI_COMPACTION_THRESHOLD_CHARS,
        )
        self.compaction_tail_messages = _env_int(
            "ARTICRAFT_CODEX_CLI_COMPACTION_TAIL_MESSAGES",
            DEFAULT_CODEX_CLI_COMPACTION_TAIL_MESSAGES,
        )

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
    ) -> dict[str, Any]:
        request = _build_codex_request(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
            thinking_level=self.thinking_level,
        )
        return {
            "transport": "codex-cli",
            "model": self.model_id,
            "thinking_level": self.thinking_level,
            "command": self._base_command(
                schema_path=Path("<schema.json>"),
                output_path=Path("<last-message.json>"),
                image_paths=request.image_paths,
            ),
            "prompt": request.prompt,
        }

    async def prepare_next_request(
        self,
        *,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
        completed_turns: int,
        consecutive_compile_failure_count: int = 0,
        last_compile_failure_sig: str | None = None,
    ) -> PrepareRequestResult:
        result = PrepareRequestResult()
        compaction_event = await self._maybe_compact_messages(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
            completed_turns=completed_turns,
            consecutive_compile_failure_count=consecutive_compile_failure_count,
            last_compile_failure_sig=last_compile_failure_sig,
        )
        if compaction_event is not None:
            result.trace_events.append(
                ProviderTraceEvent(
                    event_type=compaction_event["event_type"], payload=compaction_event
                )
            )
            if compaction_event.get("kind") == "codex_cli_compaction":
                result.maintenance_events.append(compaction_event)

        request = self._build_stateful_request(
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
                "image_paths": request.image_paths,
                "prompt_sha256": hashlib.sha256(request.prompt.encode("utf-8")).hexdigest(),
                "prompt_chars": len(request.prompt),
                "consecutive_compile_failure_count": consecutive_compile_failure_count,
                "last_compile_failure_sig": last_compile_failure_sig,
            },
        )
        result.trace_events.append(event)
        return result

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
    ) -> ProviderResponse:
        if self.dry_run:
            raise RuntimeError("Codex CLI transport is unavailable in dry_run mode")

        request = self._build_stateful_request(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
            thinking_level=self.thinking_level,
        )
        with tempfile.TemporaryDirectory(prefix="articraft_codex_cli_") as tmp:
            tmp_dir = Path(tmp)
            schema_path = tmp_dir / "assistant_turn.schema.json"
            output_path = tmp_dir / "assistant_turn.json"
            schema_path.write_text(
                json.dumps(_output_schema(tools), indent=2) + "\n",
                encoding="utf-8",
            )
            command = self._base_command(
                schema_path=schema_path,
                output_path=output_path,
                image_paths=request.image_paths,
            )
            result = await self._runner(command, request.prompt, self.timeout_seconds, output_path)

        if result.returncode != 0:
            raise RuntimeError(_format_codex_error(result))
        payload = _parse_last_message(result.last_message)
        assistant_turn = _validate_assistant_turn(
            payload,
        )
        return _convert_payload_to_provider_response(
            assistant_turn,
            raw_payload=payload,
            model_id=self.model_id,
            command=command,
            stdout=result.stdout,
            stderr=result.stderr,
        )

    async def close(self) -> None:
        return None

    def _build_stateful_request(
        self,
        *,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
        thinking_level: str,
    ) -> CodexCliRequest:
        visible_messages = self._messages_for_request(messages)
        request = _build_codex_request(
            system_prompt=system_prompt,
            messages=visible_messages,
            tools=tools,
            thinking_level=thinking_level,
        )
        self._persistent_image_paths = _merge_image_paths(
            self._persistent_image_paths,
            _image_paths_from_messages(messages),
        )
        return CodexCliRequest(
            prompt=request.prompt,
            image_paths=list(self._persistent_image_paths),
        )

    async def _maybe_compact_messages(
        self,
        *,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
        completed_turns: int,
        consecutive_compile_failure_count: int,
        last_compile_failure_sig: str | None,
    ) -> dict[str, Any] | None:
        if self.dry_run or self.compaction_threshold_chars <= 0 or completed_turns < 2:
            return None

        plan = self._compaction_plan(messages)
        if plan is None:
            return None
        compact_start, compact_end = plan

        visible_messages = self._messages_for_request(messages)
        prompt_chars = len(
            _render_codex_prompt(
                system_prompt=system_prompt,
                messages=visible_messages,
                tools=tools,
                thinking_level=self.thinking_level,
            )
        )
        trigger: str | None = None
        if prompt_chars >= self.compaction_threshold_chars:
            trigger = "prompt_chars"
        elif (
            consecutive_compile_failure_count >= 3
            and last_compile_failure_sig
            and prompt_chars >= max(1, self.compaction_threshold_chars // 2)
        ):
            trigger = "compile_plateau"
        if trigger is None:
            return None

        before_summary = self._compaction_summary
        compacted_messages = messages[compact_start:compact_end]
        summary_prompt = _render_compaction_prompt(
            existing_summary=before_summary,
            messages=compacted_messages,
        )
        with tempfile.TemporaryDirectory(prefix="articraft_codex_cli_compact_") as tmp:
            tmp_dir = Path(tmp)
            schema_path = tmp_dir / "summary.schema.json"
            output_path = tmp_dir / "summary.json"
            schema_path.write_text(json.dumps(_SUMMARY_SCHEMA, indent=2) + "\n", encoding="utf-8")
            command = self._base_command(
                schema_path=schema_path,
                output_path=output_path,
                image_paths=[],
            )
            try:
                exec_result = await self._runner(
                    command,
                    summary_prompt,
                    self.timeout_seconds,
                    output_path,
                )
            except Exception as exc:
                return {
                    "kind": "codex_cli_compaction_skipped",
                    "event_type": "codex_cli_compaction_skipped",
                    "reason": "runner_error",
                    "error": str(exc),
                    "turn_before_request": completed_turns,
                    "trigger": trigger,
                    "prompt_chars": prompt_chars,
                    "compact_start": compact_start,
                    "compact_end": compact_end,
                }

        if exec_result.returncode != 0:
            return {
                "kind": "codex_cli_compaction_skipped",
                "event_type": "codex_cli_compaction_skipped",
                "reason": "nonzero_exit",
                "error": _format_codex_error(exec_result),
                "turn_before_request": completed_turns,
                "trigger": trigger,
                "prompt_chars": prompt_chars,
                "compact_start": compact_start,
                "compact_end": compact_end,
            }

        try:
            payload = _parse_last_message(exec_result.last_message)
            summary = _validate_summary_payload(payload)
        except Exception as exc:
            return {
                "kind": "codex_cli_compaction_skipped",
                "event_type": "codex_cli_compaction_skipped",
                "reason": "invalid_summary",
                "error": str(exc),
                "turn_before_request": completed_turns,
                "trigger": trigger,
                "prompt_chars": prompt_chars,
                "compact_start": compact_start,
                "compact_end": compact_end,
            }

        before_chars = len(before_summary or "")
        self._compaction_summary = summary
        self._compacted_message_count = compact_end
        after_prompt_chars = len(
            _render_codex_prompt(
                system_prompt=system_prompt,
                messages=self._messages_for_request(messages),
                tools=tools,
                thinking_level=self.thinking_level,
            )
        )
        usage = _extract_usage_from_stdio(stdout=exec_result.stdout, stderr=exec_result.stderr)
        return {
            "kind": "codex_cli_compaction",
            "event_type": "codex_cli_compaction",
            "turn_before_request": completed_turns,
            "trigger": trigger,
            "model_id": self.model_id,
            "usage": usage,
            "before_prompt_chars": prompt_chars,
            "after_prompt_chars": after_prompt_chars,
            "estimated_saved_prompt_chars": max(0, prompt_chars - after_prompt_chars),
            "before_summary_chars": before_chars,
            "after_summary_chars": len(summary),
            "before_message_count": len(messages),
            "compacted_message_count": compact_end,
            "compact_start": compact_start,
            "compact_end": compact_end,
            "raw_tail_messages": len(messages) - compact_end,
        }

    def _messages_for_request(
        self, messages: list[ConversationMessage]
    ) -> list[ConversationMessage]:
        if not self._compaction_summary or self._compacted_message_count <= 0:
            return messages
        if len(messages) < self._compacted_message_count:
            return messages
        prefix_count = _prefix_message_count(messages)
        summary_message: ConversationMessage = {
            "role": "user",
            "name": _CODEX_CLI_COMPACTION_MESSAGE_NAME,
            "content": (
                "<codex_cli_compacted_history>\n"
                + self._compaction_summary.strip()
                + "\n</codex_cli_compacted_history>"
            ),
        }
        return [
            *messages[:prefix_count],
            summary_message,
            *messages[self._compacted_message_count :],
        ]

    def _compaction_plan(
        self,
        messages: list[ConversationMessage],
    ) -> tuple[int, int] | None:
        prefix_count = _prefix_message_count(messages)
        tail_count = max(2, self.compaction_tail_messages)
        compact_end = len(messages) - tail_count
        compact_start = max(prefix_count, self._compacted_message_count)
        if compact_end <= compact_start:
            return None
        return compact_start, compact_end

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
        with suppress(ProcessLookupError):
            process.kill()
        with suppress(Exception):
            await process.communicate()
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
    messages: list[ConversationMessage],
    tools: list[ToolSchema],
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


def _build_codex_request(
    *,
    system_prompt: str,
    messages: list[ConversationMessage],
    tools: list[ToolSchema],
    thinking_level: str,
) -> CodexCliRequest:
    return CodexCliRequest(
        prompt=_render_codex_prompt(
            system_prompt=system_prompt,
            messages=messages,
            tools=tools,
            thinking_level=thinking_level,
        ),
        image_paths=_image_paths_from_messages(messages),
    )


def _prefix_message_count(messages: list[ConversationMessage]) -> int:
    if len(messages) <= 1:
        return len(messages)
    return 2


def _render_compaction_prompt(
    *,
    existing_summary: str | None,
    messages: list[ConversationMessage],
) -> str:
    sections = [
        "Summarize Articraft Codex CLI harness history for a future model turn.",
        (
            "Return JSON matching the schema. Keep concrete facts needed to continue: "
            "the user's task, design decisions, exact code/tool changes, compile/test failures, "
            "current blockers, successful compile state if any, and named unresolved defects. "
            "Drop repetitive tool chatter, duplicate docs, and stale failed attempts that no longer matter."
        ),
    ]
    if existing_summary and existing_summary.strip():
        sections.append("<existing_summary>\n" + existing_summary.strip() + "\n</existing_summary>")
    sections.append(
        "<new_history>\n"
        + json.dumps(_conversation_reference(messages), indent=2, ensure_ascii=False)
        + "\n</new_history>"
    )
    return "\n\n".join(sections)


def _conversation_reference(messages: list[ConversationMessage]) -> list[dict[str, Any]]:
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


def _image_paths_from_messages(messages: list[ConversationMessage]) -> list[str]:
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


def _merge_image_paths(existing: list[str], new_paths: list[str]) -> list[str]:
    merged: list[str] = []
    seen: set[str] = set()
    for path in [*existing, *new_paths]:
        normalized = str(path).strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        merged.append(normalized)
    return merged


def _tool_reference(tools: list[ToolSchema]) -> list[dict[str, Any]]:
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


def _tool_names(tools: list[ToolSchema]) -> list[str]:
    return [item["name"] for item in _tool_reference(tools) if item["name"]]


def _parse_last_message(raw: str) -> dict[str, Any]:
    text = raw.strip()
    if not text:
        raise RuntimeError("Codex CLI did not write an assistant response")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RuntimeError("Codex CLI assistant response was not valid JSON") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("Codex CLI assistant response must be a JSON object")
    return payload


def _validate_assistant_turn(
    payload: dict[str, Any],
) -> dict[str, Any]:
    keys = set(payload)
    extra_keys = sorted(keys - _ASSISTANT_TURN_KEYS)
    if extra_keys:
        raise RuntimeError(
            "Codex CLI assistant response had unexpected field(s): " + ", ".join(extra_keys)
        )
    missing_keys = sorted(_ASSISTANT_TURN_KEYS - keys)
    if missing_keys:
        raise RuntimeError(
            "Codex CLI assistant response missing required field(s): " + ", ".join(missing_keys)
        )

    content = payload["content"]
    if not isinstance(content, str):
        raise RuntimeError("Codex CLI assistant response field 'content' must be a string")
    thought_summary = payload["thought_summary"]
    if not isinstance(thought_summary, str):
        raise RuntimeError("Codex CLI assistant response field 'thought_summary' must be a string")
    raw_tool_calls = payload["tool_calls"]
    if not isinstance(raw_tool_calls, list):
        raise RuntimeError("Codex CLI assistant response field 'tool_calls' must be a list")

    normalized_tool_calls = _normalize_tool_calls(raw_tool_calls)

    payload = dict(payload)
    payload["tool_calls"] = normalized_tool_calls

    return payload


def _normalize_tool_calls(tool_calls: list[Any]) -> list[dict[str, str]]:
    normalized: list[dict[str, str]] = []

    for index, tool_call in enumerate(tool_calls, start=1):
        if not isinstance(tool_call, dict):
            raise RuntimeError(
                f"Codex CLI assistant response tool_call[{index}] must be a JSON object"
            )
        name = tool_call.get("name")
        if not isinstance(name, str) or not name:
            raise RuntimeError(
                f"Codex CLI assistant response tool_call[{index}] field 'name' must be a non-empty string"
            )
        if "arguments" not in tool_call:
            raise RuntimeError(
                f"Codex CLI assistant response tool_call[{index}] missing required field 'arguments'"
            )

        normalized_arguments = _normalize_tool_call_arguments(
            tool_call.get("arguments"),
            tool_call_index=index,
        )

        normalized.append(
            {
                "name": str(name),
                "arguments": normalized_arguments,
            }
        )

    return normalized


def _normalize_tool_call_arguments(arguments: Any, *, tool_call_index: int) -> str:
    if isinstance(arguments, dict):
        return json.dumps(arguments, ensure_ascii=False)
    if not isinstance(arguments, str):
        raise RuntimeError(
            f"Codex CLI assistant response tool_call[{tool_call_index}]"
            " field 'arguments' must be a JSON object string or object"
        )

    if not arguments.strip():
        return "{}"

    try:
        parsed = json.loads(arguments)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"Codex CLI assistant response tool_call[{tool_call_index}] field 'arguments'"
            f" must be valid JSON: {exc}"
        ) from exc
    if not isinstance(parsed, dict):
        raise RuntimeError(
            f"Codex CLI assistant response tool_call[{tool_call_index}] field 'arguments'"
            " must decode to a JSON object"
        )
    return arguments


def _validate_summary_payload(payload: dict[str, Any]) -> str:
    extra_keys = sorted(set(payload) - {"summary"})
    if extra_keys:
        raise RuntimeError(
            "Codex CLI compaction response had unexpected field(s): " + ", ".join(extra_keys)
        )
    summary = payload.get("summary")
    if not isinstance(summary, str):
        raise RuntimeError("Codex CLI compaction response field 'summary' must be a string")
    text = summary.strip()
    if not text:
        raise RuntimeError("Codex CLI compaction response field 'summary' must not be empty")
    return text


def _convert_payload_to_provider_response(
    assistant_turn: dict[str, Any],
    *,
    raw_payload: dict[str, Any],
    model_id: str,
    command: list[str],
    stdout: str,
    stderr: str,
) -> ProviderResponse:
    tool_calls: list[dict[str, Any]] = []
    for item in assistant_turn["tool_calls"]:
        name = item["name"]
        serialized_arguments = item["arguments"]
        tool_calls.append(
            {
                "id": f"call_codex_{uuid.uuid4().hex}",
                "type": "function",
                "function": {
                    "name": str(name or ""),
                    "arguments": serialized_arguments,
                },
            }
        )

    result: dict[str, Any] = {
        "content": assistant_turn["content"],
        "tool_calls": tool_calls,
        "extra_content": {
            "codex_cli": {
                "model_id": model_id,
                "command": _redacted_command(command),
                "raw_response": raw_payload,
            }
        },
    }
    thought_summary = assistant_turn["thought_summary"]
    if thought_summary.strip():
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
    totals: list[int] = []
    lines = combined.splitlines()
    for index, line in enumerate(lines):
        lowered = line.lower()
        if "tokens used" not in lowered:
            continue
        marker_index = lowered.find("tokens used")
        candidates = [line[marker_index + len("tokens used") :]]
        if index + 1 < len(lines):
            candidates.append(lines[index + 1])
        for candidate in candidates:
            total = _parse_token_count(candidate)
            if total is not None:
                totals.append(total)
                break
    if not totals:
        return None
    return {"total_tokens": totals[-1]}


def _parse_token_count(text: str) -> int | None:
    normalized = text.strip()
    if not normalized:
        return None
    digits = []
    for char in normalized:
        if char.isdigit() or char == ",":
            digits.append(char)
            continue
        if digits:
            break
    if not digits:
        return None
    token_text = "".join(digits)
    if not token_text[0].isdigit():
        return None
    return int(token_text.replace(",", ""))


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


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        value = int(raw.strip().replace("_", ""))
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


def _output_schema(tools: list[ToolSchema]) -> dict[str, Any]:
    schema = json.loads(json.dumps(_OUTPUT_SCHEMA))
    tool_names = _tool_names(tools)
    if not tool_names:
        return schema
    tool_call_properties = schema["properties"]["tool_calls"]["items"]["properties"]
    tool_call_properties["name"] = {
        "type": "string",
        "enum": tool_names,
        "description": "Name of one available Articraft harness tool.",
    }
    return schema


_SUMMARY_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "summary": {
            "type": "string",
            "description": "Compact continuation state for older Articraft harness history.",
        }
    },
    "required": ["summary"],
}
