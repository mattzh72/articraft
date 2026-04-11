from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import agent.harness as harness
from agent.feedback import build_compile_signal_bundle
from agent.harness import ArticraftAgent
from agent.models import CompileReport, TerminateReason
from agent.tools.compile_model import CompileModelTool
from agent.tools.registry import ToolRegistry
from agent.tools.write_code import WriteCodeTool


class _CountingDisplay:
    def __init__(self) -> None:
        self.current_turn = 0
        self.end_turn_calls = 0

    def start(self) -> None:
        return None

    def start_turn(self, turn_number: int) -> None:
        self.current_turn = turn_number

    def start_llm_wait(self) -> None:
        return None

    def stop_llm_wait(self) -> None:
        return None

    def add_llm_call(self, tokens: dict, cost: float, duration: float) -> None:
        return None

    def add_thinking_summary(self, summary: str) -> None:
        return None

    def add_tool_call(
        self,
        *,
        tool_name: str,
        args: dict,
        success: bool,
        duration: float,
        result: str | None = None,
        compilation: dict | None = None,
        error: str | None = None,
    ) -> None:
        return None

    def end_turn(self, success: bool, error: str | None = None) -> None:
        self.end_turn_calls += 1


def test_compile_async_uses_timeout_wrapper(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(tmp_path / "model.py")
    agent.sdk_package = "sdk"
    agent.runtime_limits = None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )
    captured: dict[str, object] = {}

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str,
        rewrite_visual_glb: bool,
    ) -> CompileReport:
        captured["script_path"] = script_path
        captured["sdk_package"] = sdk_package
        captured["rewrite_visual_glb"] = rewrite_visual_glb
        return report

    monkeypatch.setattr(harness, "compile_urdf_report_maybe_timeout", fake_compile)

    result = asyncio.run(agent._compile_urdf_report_async())

    assert result is report
    assert captured == {
        "script_path": tmp_path / "model.py",
        "sdk_package": "sdk",
        "rewrite_visual_glb": False,
    }


def test_execute_compile_model_reuses_cached_success_for_current_revision() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._current_edit_revision = 2
    agent._last_successful_compile_revision = None
    agent._last_successful_compile_report = None
    agent._compile_attempt_count = 0
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent._last_checkpoint_urdf_sig = None
    agent.checkpoint_urdf_path = None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )
    compile_calls = 0

    async def fake_compile() -> CompileReport:
        nonlocal compile_calls
        compile_calls += 1
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    first = asyncio.run(agent._execute_compile_model(tool_call_id="call_1"))
    second = asyncio.run(agent._execute_compile_model(tool_call_id="call_2"))

    assert compile_calls == 1
    assert agent._compile_attempt_count == 1
    assert agent._last_successful_compile_revision == 2
    assert first.compilation == {"status": "success", "error": None}
    assert second.compilation == {"status": "success", "error": None}
    assert "Compile passed cleanly." in str(first.output)
    assert "Fresh compile already exists for the current code revision" in str(second.output)
    assert "Treat that compile result as authoritative" in str(second.output)


def test_flash_model_disables_post_success_design_audit_by_default(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class _FlashLLM:
        model_id = "gemini-3-flash-preview"

        def __init__(self, *args: object, **kwargs: object) -> None:
            return None

        async def close(self) -> None:
            return None

    monkeypatch.setattr(harness, "GeminiLLM", _FlashLLM)

    agent = ArticraftAgent(
        file_path=str(tmp_path / "model.py"),
        provider="gemini",
        display_enabled=False,
    )

    assert agent._post_success_design_audit_enabled is False


def test_execute_compile_model_failure_leaves_latest_revision_stale() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._current_edit_revision = 1
    agent._last_successful_compile_revision = 0
    agent._last_successful_compile_report = CompileReport(
        urdf_xml="<robot old='1' />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )
    agent._compile_attempt_count = 0
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent._last_checkpoint_urdf_sig = None
    agent.checkpoint_urdf_path = None

    async def fake_compile() -> CompileReport:
        raise RuntimeError("ValueError: bad loft")

    async def fake_persist_failure(_: BaseException) -> bool:
        return False

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_failure_checkpoint_async = fake_persist_failure

    result = asyncio.run(agent._execute_compile_model(tool_call_id="call_1"))

    assert agent._compile_attempt_count == 1
    assert agent._last_successful_compile_revision == 0
    assert agent._latest_code_is_fresh() is False
    assert result.compilation == {
        "status": "error",
        "error": "status=failure failures=1 warnings=0 notes=0\nPrimary issue: RuntimeError: ValueError: bad loft",
    }
    assert "<compile_signals>" in str(result.output)
    assert "bad loft" in str(result.output)


def test_execute_tool_calls_batch_runs_parallel_safe_gemini_calls_concurrently() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.provider = "gemini"

    active_calls = 0
    peak_calls = 0

    async def fake_execute_tool(tool_call: dict) -> tuple[object, dict]:
        nonlocal active_calls, peak_calls
        active_calls += 1
        peak_calls = max(peak_calls, active_calls)
        await asyncio.sleep(0.01)
        active_calls -= 1
        return (
            SimpleNamespace(
                is_success=lambda: True,
                output=tool_call["id"],
                compilation=None,
                error=None,
            ),
            {
                "role": "tool",
                "tool_call_id": tool_call["id"],
                "name": agent._tool_call_name(tool_call),
            },
        )

    agent._execute_tool = fake_execute_tool  # type: ignore[method-assign]

    results = asyncio.run(
        agent._execute_tool_calls_batch(
            [
                {
                    "id": "call_read_code",
                    "type": "function",
                    "function": {"name": "read_code", "arguments": "{}"},
                },
                {
                    "id": "call_find_examples",
                    "type": "function",
                    "function": {"name": "find_examples", "arguments": '{"query":"hinge"}'},
                },
            ]
        )
    )

    assert [tool_call["id"] for tool_call, *_ in results] == [
        "call_read_code",
        "call_find_examples",
    ]
    assert peak_calls == 2


def test_execute_tool_calls_batch_keeps_mutating_gemini_calls_serialized() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.provider = "gemini"

    active_calls = 0
    peak_calls = 0

    async def fake_execute_tool(tool_call: dict) -> tuple[object, dict]:
        nonlocal active_calls, peak_calls
        active_calls += 1
        peak_calls = max(peak_calls, active_calls)
        await asyncio.sleep(0.01)
        active_calls -= 1
        return (
            SimpleNamespace(
                is_success=lambda: True,
                output=tool_call["id"],
                compilation=None,
                error=None,
            ),
            {
                "role": "tool",
                "tool_call_id": tool_call["id"],
                "name": agent._tool_call_name(tool_call),
            },
        )

    agent._execute_tool = fake_execute_tool  # type: ignore[method-assign]

    results = asyncio.run(
        agent._execute_tool_calls_batch(
            [
                {
                    "id": "call_read_code",
                    "type": "function",
                    "function": {"name": "read_code", "arguments": "{}"},
                },
                {
                    "id": "call_edit_code",
                    "type": "function",
                    "function": {
                        "name": "edit_code",
                        "arguments": '{"old_string":"a","new_string":"b"}',
                    },
                },
            ]
        )
    )

    assert [tool_call["id"] for tool_call, *_ in results] == ["call_read_code", "call_edit_code"]
    assert peak_calls == 1


def test_run_discards_pasted_code_and_replaces_it_with_recovery_messages(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class _SequenceLLM:
        model_id = "gemini-2.5-pro"

        def __init__(self, *args: object, **kwargs: object) -> None:
            self._responses = [
                {"content": "```python\nprint('bad')\n```", "tool_calls": []},
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
            ]

        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            assert tools
            return self._responses.pop(0)

        async def close(self) -> None:
            return None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )

    monkeypatch.setattr(harness, "GeminiLLM", _SequenceLLM)
    agent = ArticraftAgent(
        file_path=str(tmp_path / "model.py"),
        provider="gemini",
        max_turns=6,
        display_enabled=False,
        post_success_design_audit=False,
    )
    agent.tool_registry = ToolRegistry([CompileModelTool()])

    async def fake_compile() -> CompileReport:
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is True
    assert "print('bad')" not in json.dumps(result.conversation)
    assert any(
        message.get("role") == "assistant"
        and message.get("content") == "[Previous response pasted code and was discarded.]"
        for message in result.conversation
    )
    assert any(
        message.get("role") == "user"
        and "Source of truth is the file on disk" in str(message.get("content", ""))
        for message in result.conversation
    )


def test_mark_code_mutated_invalidates_fresh_compile_without_rearming_audit() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._current_edit_revision = 3
    agent._last_successful_compile_revision = 3
    agent._last_successful_compile_report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )
    agent._post_success_design_audit_sent = True

    agent._mark_code_mutated("edit_code")

    assert agent._current_edit_revision == 4
    assert agent._latest_code_is_fresh() is False
    assert agent._post_success_design_audit_sent is True


def test_run_requires_compile_model_before_concluding_and_delays_design_audit(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class _SequenceLLM:
        model_id = "gemini-2.5-pro"

        def __init__(self, *args: object, **kwargs: object) -> None:
            self._responses = [
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_edit",
                            "type": "function",
                            "function": {
                                "name": "edit_code",
                                "arguments": json.dumps(
                                    {
                                        "old_string": '"draft_model"',
                                        "new_string": '"draft_model_v2"',
                                        "replace_all": False,
                                    }
                                ),
                            },
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {"content": "Done.", "tool_calls": []},
            ]

        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            assert tools
            return self._responses.pop(0)

        async def close(self) -> None:
            return None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )

    monkeypatch.setattr(harness, "GeminiLLM", _SequenceLLM)
    agent = ArticraftAgent(
        file_path=str(tmp_path / "model.py"),
        provider="gemini",
        max_turns=6,
        post_success_design_audit=True,
        display_enabled=False,
    )

    async def fake_compile() -> CompileReport:
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is True
    assert result.reason == TerminateReason.CODE_VALID
    assert result.compile_attempt_count == 1
    assert result.tool_call_count == 2

    user_messages = [
        str(message.get("content", ""))
        for message in result.conversation
        if message.get("role") == "user"
    ]
    assert any("<compile_required>" in content for content in user_messages)
    assert any("<design_audit>" in content for content in user_messages)

    compile_tool_messages = [
        message
        for message in result.conversation
        if message.get("role") == "tool" and message.get("name") == "compile_model"
    ]
    assert len(compile_tool_messages) == 1
    compile_payload = json.loads(compile_tool_messages[0]["content"])
    assert "Compile passed cleanly." in compile_payload["result"]

    compile_index = result.conversation.index(compile_tool_messages[0])
    audit_index = next(
        index
        for index, message in enumerate(result.conversation)
        if message.get("role") == "user" and "<design_audit>" in str(message.get("content", ""))
    )
    assert audit_index > compile_index + 1


def test_run_accepts_gemini_edit_code_without_replace_all(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    class _SequenceLLM:
        model_id = "gemini-2.5-pro"

        def __init__(self, *args: object, **kwargs: object) -> None:
            self._responses = [
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_edit",
                            "type": "function",
                            "function": {
                                "name": "edit_code",
                                "arguments": json.dumps(
                                    {
                                        "old_string": '"draft_model"',
                                        "new_string": '"draft_model_v2"',
                                    }
                                ),
                            },
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
            ]

        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            assert tools
            return self._responses.pop(0)

        async def close(self) -> None:
            return None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )

    monkeypatch.setattr(harness, "GeminiLLM", _SequenceLLM)
    agent = ArticraftAgent(
        file_path=str(tmp_path / "model.py"),
        provider="gemini",
        max_turns=5,
        post_success_design_audit=False,
        display_enabled=False,
    )

    async def fake_compile() -> CompileReport:
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is True
    assert result.reason == TerminateReason.CODE_VALID
    assert result.compile_attempt_count == 1
    assert result.tool_call_count == 2
    assert result.final_code is not None
    assert '"draft_model_v2"' in result.final_code


@pytest.mark.parametrize(
    ("provider_name", "provider_attr"),
    [
        ("gemini", "GeminiLLM"),
        ("openai", "OpenAILLM"),
    ],
)
def test_design_audit_only_fires_once_even_after_post_audit_edit_cycle(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    provider_name: str,
    provider_attr: str,
) -> None:
    rewritten_code = """
def build_object_model():
    return "v2"


def run_tests():
    return None
""".strip()

    class _SequenceLLM:
        model_id = "test-model"

        def __init__(self, *args: object, **kwargs: object) -> None:
            self._responses = [
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_write_1",
                            "type": "function",
                            "function": {
                                "name": "write_code",
                                "arguments": json.dumps({"code": rewritten_code}),
                            },
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile_1",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_write_2",
                            "type": "function",
                            "function": {
                                "name": "write_code",
                                "arguments": json.dumps({"code": rewritten_code}),
                            },
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile_2",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                {"content": "Done.", "tool_calls": []},
            ]

        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            assert tools
            return self._responses.pop(0)

        async def close(self) -> None:
            return None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )

    monkeypatch.setattr(harness, provider_attr, _SequenceLLM)
    agent = ArticraftAgent(
        file_path=str(tmp_path / f"{provider_name}_model.py"),
        provider=provider_name,
        max_turns=10,
        post_success_design_audit=True,
        display_enabled=False,
    )
    agent.tool_registry = ToolRegistry([WriteCodeTool(), CompileModelTool()])

    async def fake_compile() -> CompileReport:
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is True
    assert result.reason == TerminateReason.CODE_VALID

    user_messages = [
        str(message.get("content", ""))
        for message in result.conversation
        if message.get("role") == "user"
    ]
    assert sum("<design_audit>" in content for content in user_messages) == 1
    assert sum("<compile_required>" in content for content in user_messages) == 2


@pytest.mark.parametrize(
    "finish_response",
    [
        {"content": "Done.", "tool_calls": []},
        {},
    ],
)
def test_finish_attempt_paths_end_turn_and_share_compile_then_audit_gate(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    finish_response: dict[str, object],
) -> None:
    class _SequenceLLM:
        model_id = "gemini-2.5-pro"

        def __init__(self, *args: object, **kwargs: object) -> None:
            self._responses = [
                dict(finish_response),
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "id": "call_compile",
                            "type": "function",
                            "function": {"name": "compile_model", "arguments": "{}"},
                        }
                    ],
                },
                dict(finish_response),
                dict(finish_response),
            ]

        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            assert tools
            return self._responses.pop(0)

        async def close(self) -> None:
            return None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )

    monkeypatch.setattr(harness, "GeminiLLM", _SequenceLLM)
    agent = ArticraftAgent(
        file_path=str(tmp_path / "model.py"),
        provider="gemini",
        max_turns=6,
        post_success_design_audit=True,
        display_enabled=False,
    )
    agent.tool_registry = ToolRegistry([CompileModelTool()])
    agent.display = _CountingDisplay()

    async def fake_compile() -> CompileReport:
        return report

    async def fake_persist(_: str) -> None:
        return None

    agent._compile_urdf_report_async = fake_compile
    agent._persist_compile_success_checkpoint_async = fake_persist

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is True
    assert result.reason == TerminateReason.CODE_VALID
    assert agent.display.end_turn_calls == 4

    user_messages = [
        str(message.get("content", ""))
        for message in result.conversation
        if message.get("role") == "user"
    ]
    assert sum("<compile_required>" in content for content in user_messages) == 1
    assert sum("<design_audit>" in content for content in user_messages) == 1
