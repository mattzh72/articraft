from __future__ import annotations

import asyncio
import copy
import json
from pathlib import Path
from types import SimpleNamespace

from agent.feedback import build_compile_signal_bundle
from agent.harness import ArticraftAgent
from agent.models import CompileReport
from agent.tools.base import ToolResult


class _FakeDisplay:
    current_turn = 0

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def start_turn(self, turn: int) -> None:
        self.current_turn = turn

    def start_llm_wait(self) -> None:
        return None

    def stop_llm_wait(self) -> None:
        return None

    def end_turn(self, success: bool, error: str | None = None) -> None:
        return None

    def add_thinking_summary(self, thinking: str) -> None:
        return None

    def add_llm_call(self, usage: dict[str, int], total_cost: float, duration: float) -> None:
        return None

    def add_tool_call(
        self,
        *,
        tool_name: str,
        args: dict,
        success: bool,
        duration: float,
        result: object = None,
        compilation: dict | None = None,
        error: str | None = None,
    ) -> None:
        return None

    def add_compile_result(
        self,
        *,
        success: bool,
        duration: float,
        warnings: list[str] | None = None,
        error: str | None = None,
    ) -> None:
        return None


class _FakeToolRegistry:
    def get_tool_schemas(self) -> list[dict]:
        return []


class _ResettableLLM:
    def __init__(self, responses: list[dict]) -> None:
        self.responses = list(responses)
        self.calls: list[list[dict]] = []
        self.reset_calls = 0

    async def generate_with_tools(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        self.calls.append(copy.deepcopy(messages))
        return self.responses.pop(0)

    def reset_context(self) -> None:
        self.reset_calls += 1


def _warning_bundle() -> object:
    return build_compile_signal_bundle(
        status="success",
        warnings=[
            "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
            "- link='boom' source='visual' index=0 geometry='Cylinder' dims=(0.06, 0.06, 20)m"
        ],
    )


def test_small_context_compaction_trigger_requires_successful_apply_patch() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)

    assert (
        agent._small_context_compaction_triggered(
            tool_calls=[
                {"function": {"name": "read_file"}},
                {"function": {"name": "apply_patch"}},
            ],
            tool_results=[
                ToolResult(output="ok"),
                ToolResult(output="patched", compilation={"status": "success"}),
            ],
        )
        is True
    )
    assert (
        agent._small_context_compaction_triggered(
            tool_calls=[{"function": {"name": "apply_patch"}}],
            tool_results=[ToolResult(output="patched", compilation={"status": "failed"})],
        )
        is False
    )
    assert (
        agent._small_context_compaction_triggered(
            tool_calls=[{"function": {"name": "apply_patch"}}],
            tool_results=[ToolResult(error="patch failed", compilation={"status": "success"})],
        )
        is False
    )


def test_small_context_compaction_resets_caches_and_dedupe_state() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._base_conversation = [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "make a bracket"},
    ]
    agent._seen_compile_signal_sigs = set()
    agent._seen_find_example_paths = set()
    agent._context_reset_count = 0
    reset_calls: list[str] = []
    agent.llm = SimpleNamespace(reset_context=lambda: reset_calls.append("reset"))
    agent.trace_writer = None

    agent._seed_find_examples_cache_from_conversation(
        [
            {
                "role": "tool",
                "name": "find_examples",
                "content": json.dumps(
                    {
                        "result": [
                            {
                                "path": "sdk/_examples/hybrid/making_lofts.md",
                                "content": "# full example",
                            }
                        ]
                    }
                ),
            }
        ]
    )
    assert agent._seen_find_example_paths == {"sdk/_examples/hybrid/making_lofts.md"}

    conversation: list[dict] = []
    bundle = _warning_bundle()
    assert agent._maybe_inject_compile_signals(conversation, bundle=bundle) is True
    assert agent._maybe_inject_compile_signals(conversation, bundle=bundle) is False

    compacted = agent._apply_small_context_compaction(
        carryover_message={"role": "user", "content": "carryover"},
    )

    assert compacted == [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "make a bracket"},
        {"role": "user", "content": "carryover"},
    ]
    assert agent._seen_find_example_paths == set()
    assert agent._seen_compile_signal_sigs == set()
    assert agent._context_reset_count == 1
    assert reset_calls == ["reset"]

    post_reset_conversation: list[dict] = []
    assert agent._maybe_inject_compile_signals(post_reset_conversation, bundle=bundle) is True
    assert "<compile_signals>" in post_reset_conversation[0]["content"]


def test_small_context_compaction_preserves_design_audit_carryover() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._base_conversation = [
        {"role": "user", "content": "sdk docs"},
        {"role": "user", "content": "make a bracket"},
    ]
    agent._seen_compile_signal_sigs = set()
    agent._seen_find_example_paths = set()
    agent._context_reset_count = 0
    agent._post_success_design_audit_sent = False
    agent._post_success_design_audit_enabled = True
    agent.llm = SimpleNamespace(reset_context=lambda: None)
    agent.trace_writer = None

    conversation: list[dict] = []
    assert agent._maybe_inject_post_success_design_audit(conversation) is True

    compacted = agent._apply_small_context_compaction(carryover_message=conversation[-1])

    assert compacted[:2] == agent._base_conversation
    assert compacted[-1]["role"] == "user"
    assert "<design_audit>" in compacted[-1]["content"]
    assert "Scan each part" in compacted[-1]["content"]


def test_small_context_loop_resumes_from_base_plus_carryover_only(tmp_path: Path) -> None:
    code_path = tmp_path / "model.py"
    code_path.write_text("from __future__ import annotations\n", encoding="utf-8")

    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(code_path)
    agent.max_turns = 3
    agent.sdk_docs_context = "sdk docs"
    agent.display = _FakeDisplay()
    agent.tool_registry = _FakeToolRegistry()
    agent.trace_writer = None
    agent.on_turn_start = None
    agent.system_prompt = ""
    agent.cost_tracker = None
    agent.provider = "openai"
    agent.runtime_limits = None
    agent._small_context_loop_enabled = True
    agent._context_reset_count = 0
    agent._base_conversation = []
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent._post_success_design_audit_sent = False
    agent._post_success_design_audit_enabled = False
    agent._seen_compile_signal_sigs = set()
    agent._seen_tool_error_sigs = set()
    agent._seen_find_example_paths = set()
    agent._last_checkpoint_urdf_sig = None
    agent.checkpoint_urdf_path = None
    agent._ensure_code_file = lambda: None
    agent.llm = _ResettableLLM(
        [
            {
                "content": "",
                "tool_calls": [
                    {
                        "id": "call_apply_patch",
                        "type": "function",
                        "function": {"name": "apply_patch", "arguments": "{}"},
                    }
                ],
            },
            {
                "content": "done",
                "tool_calls": [],
            },
        ]
    )

    async def _fake_execute_tool(tool_call: dict) -> tuple[ToolResult, dict]:
        return (
            ToolResult(output="patched", compilation={"status": "success"}),
            {
                "role": "tool",
                "tool_call_id": tool_call["id"],
                "name": "apply_patch",
                "content": json.dumps({"result": "patched", "compilation": {"status": "success"}}),
            },
        )

    async def _fake_compile_report() -> CompileReport:
        return CompileReport(
            urdf_xml="<robot/>",
            warnings=[],
            signal_bundle=build_compile_signal_bundle(status="success"),
        )

    async def _fake_persist_checkpoint(urdf_xml: str) -> None:
        return None

    agent._execute_tool = _fake_execute_tool
    agent._compile_urdf_report_async = _fake_compile_report
    agent._persist_compile_success_checkpoint_async = _fake_persist_checkpoint

    result = asyncio.run(agent.run("make a bracket"))

    assert result.success is True
    assert result.context_reset_count == 1
    assert agent.llm.reset_calls == 1
    assert len(agent.llm.calls) == 2
    assert [message["role"] for message in agent.llm.calls[1]] == ["user", "user", "user", "user"]
    assert agent.llm.calls[1][-1]["content"].startswith("<small_context_resume>")
    assert "Re-read the current file before editing" in agent.llm.calls[1][-1]["content"]
