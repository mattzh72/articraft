from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

from agent.cost import CostTracker
from agent.harness import ArticraftAgent
from agent.models import TerminateReason
from agent.traces import TraceWriter


class _DummyDisplay:
    def start(self) -> None:
        return None

    def start_turn(self, turn_number: int) -> None:
        return None

    def start_llm_wait(self) -> None:
        return None

    def stop_llm_wait(self) -> None:
        return None

    def add_llm_call(self, tokens: dict, cost: float, duration: float) -> None:
        return None

    def add_thinking_summary(self, summary: str) -> None:
        return None

    def end_turn(self, success: bool, error: str | None = None) -> None:
        return None


class _CapturingCompactionDisplay(_DummyDisplay):
    def __init__(self) -> None:
        self.compaction_events: list[dict[str, object]] = []

    def add_compaction_event(self, **kwargs: object) -> None:
        self.compaction_events.append(dict(kwargs))


class _DummyRegistry:
    def get_tool_schemas(self) -> list[dict]:
        return []


class _SingleResponseLLM:
    model_id = "gpt-5.4"

    async def generate_with_tools(
        self, system_prompt: str, messages: list[dict], tools: list[dict]
    ) -> dict:
        return {
            "usage": {
                "prompt_tokens": 10000,
                "cached_tokens": 0,
                "candidates_tokens": 5000,
                "total_tokens": 15000,
            }
        }

    async def close(self) -> None:
        return None


class _FakeCompactionEvent:
    def __init__(self, payload: dict[str, object]) -> None:
        self._payload = payload
        self.trigger = str(payload.get("trigger") or "compaction")
        self.estimated_saved_next_input_tokens = payload.get("estimated_saved_next_input_tokens")
        self.previous_response_id_cleared = bool(payload.get("previous_response_id_cleared"))
        self.estimate_error = (
            str(payload.get("estimate_error"))
            if isinstance(payload.get("estimate_error"), str)
            else None
        )

    def to_dict(self) -> dict[str, object]:
        return dict(self._payload)


class _MaintenanceOnlyLLM:
    model_id = "gpt-5.4"

    async def prepare_next_request(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
        completed_turns: int,
        consecutive_compile_failure_count: int = 0,
        last_compile_failure_sig: str | None = None,
    ) -> SimpleNamespace:
        payload = {
            "kind": "compaction",
            "turn_before_request": completed_turns,
            "trigger": "hard_pressure",
            "model_id": self.model_id,
            "usage": {
                "prompt_tokens": 10_000,
                "cached_tokens": 0,
                "candidates_tokens": 5_000,
                "total_tokens": 15_000,
            },
            "before_next_input_tokens": 300_000,
            "after_next_input_tokens": 100_000,
            "estimated_saved_next_input_tokens": 200_000,
            "before_item_count": 10,
            "after_item_count": 4,
            "previous_response_id_cleared": True,
            "guardrails": {
                "min_turns_satisfied": False,
                "back_to_back_allowed": True,
                "raw_tail_preserved": True,
            },
        }
        return SimpleNamespace(
            trace_events=[
                SimpleNamespace(
                    event_type="compaction",
                    payload=payload,
                )
            ],
            compaction_event=_FakeCompactionEvent(payload),
        )

    async def generate_with_tools(
        self, system_prompt: str, messages: list[dict], tools: list[dict]
    ) -> dict:
        raise AssertionError("generate_with_tools should not run after maintenance cost limit")

    async def close(self) -> None:
        return None


class _CompactionThenDoneLLM(_MaintenanceOnlyLLM):
    async def generate_with_tools(
        self, system_prompt: str, messages: list[dict], tools: list[dict]
    ) -> dict:
        return {"content": "Done.", "tool_calls": []}


def test_harness_cost_limit_trips_and_persists_cost_json(tmp_path: Path) -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(tmp_path / "model.py")
    agent.max_turns = 3
    agent.sdk_package = "sdk"
    agent.runtime_limits = None
    agent._seen_tool_error_sigs = set()
    agent._seen_find_example_paths = set()
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.checkpoint_urdf_path = None
    agent.trace_writer = None
    agent.provider = "openai"
    agent.llm = _SingleResponseLLM()
    agent.tool_registry = _DummyRegistry()
    agent.on_turn_start = None
    agent.display = _DummyDisplay()
    agent.loaded_system_prompt_path = "designer_system_prompt_openai.txt"
    agent.system_prompt = "system"
    agent.sdk_docs_context = ""
    agent.cost_tracker = CostTracker(
        model_id="gpt-5.4",
        pricing={"input_uncached": 2.5, "input_cached": 0.25, "output": 15.0},
    )
    agent.max_cost_usd = 0.01
    agent._ensure_code_file = lambda: Path(agent.file_path).write_text(
        "# draft\n", encoding="utf-8"
    )

    result = asyncio.run(agent.run("make a hinge"))

    assert result.success is False
    assert result.reason == TerminateReason.COST_LIMIT
    assert "Cost limit exceeded after turn 1" in result.message

    cost_path = tmp_path / "cost.json"
    assert cost_path.exists()
    payload = json.loads(cost_path.read_text(encoding="utf-8"))
    assert payload["total"]["costs_usd"]["total"] > 0.01


def test_harness_persists_compaction_maintenance_cost_and_trace(tmp_path: Path) -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(tmp_path / "model.py")
    agent.max_turns = 3
    agent.sdk_package = "sdk"
    agent.runtime_limits = None
    agent._seen_tool_error_sigs = set()
    agent._seen_find_example_paths = set()
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.checkpoint_urdf_path = None
    agent.trace_writer = TraceWriter(tmp_path / "traces")
    agent.provider = "openai"
    agent.llm = _MaintenanceOnlyLLM()
    agent.tool_registry = _DummyRegistry()
    agent.on_turn_start = None
    agent.display = _DummyDisplay()
    agent.loaded_system_prompt_path = "designer_system_prompt_openai.txt"
    agent.system_prompt = "system"
    agent.sdk_docs_context = "sdk docs"
    agent.cost_tracker = CostTracker(
        model_id="gpt-5.4",
        pricing={"input_uncached": 2.5, "input_cached": 0.25, "output": 15.0},
    )
    agent.max_cost_usd = 0.01
    agent._ensure_code_file = lambda: Path(agent.file_path).write_text(
        "# draft\n", encoding="utf-8"
    )

    try:
        result = asyncio.run(agent.run("make a hinge"))
    finally:
        agent.trace_writer.close()

    assert result.success is False
    assert result.reason == TerminateReason.COST_LIMIT
    assert "Cost limit exceeded before turn 1" in result.message

    cost_path = tmp_path / "cost.json"
    payload = json.loads(cost_path.read_text(encoding="utf-8"))
    assert payload["total"]["costs_usd"]["total"] == 0.0
    assert payload["maintenance_total"]["costs_usd"]["total"] > 0.01
    assert payload["all_in_total"]["costs_usd"]["total"] > 0.01
    assert payload["maintenance_events"][0]["kind"] == "compaction"
    assert payload["maintenance_events"][0]["trigger"] == "hard_pressure"

    trace_path = tmp_path / "traces" / "trajectory.jsonl"
    trace_lines = [
        json.loads(line)
        for line in trace_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert any(line.get("type") == "compaction" for line in trace_lines)


def test_harness_reports_compaction_event_without_cost_tracker(tmp_path: Path) -> None:
    captured_callbacks: list[tuple[dict[str, object], float]] = []
    display = _CapturingCompactionDisplay()

    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(tmp_path / "model.py")
    agent.max_turns = 1
    agent.sdk_package = "sdk"
    agent.runtime_limits = None
    agent._seen_tool_error_sigs = set()
    agent._seen_find_example_paths = set()
    agent._last_compile_failure_sig = None
    agent._consecutive_compile_failure_count = 0
    agent.checkpoint_urdf_path = None
    agent.trace_writer = None
    agent.provider = "openai"
    agent.llm = _CompactionThenDoneLLM()
    agent.tool_registry = _DummyRegistry()
    agent.on_turn_start = None
    agent.on_compaction_event = lambda event, cost: captured_callbacks.append((event, cost))
    agent.on_maintenance_event = None
    agent.display = display
    agent.loaded_system_prompt_path = "designer_system_prompt_openai.txt"
    agent.system_prompt = "system"
    agent.sdk_docs_context = ""
    agent.cost_tracker = None
    agent.max_cost_usd = None
    agent._ensure_code_file = lambda: Path(agent.file_path).write_text(
        "# draft\n", encoding="utf-8"
    )

    result = asyncio.run(agent.run("make a hinge"))

    assert result.reason == TerminateReason.MAX_TURNS
    assert captured_callbacks
    assert captured_callbacks[0][0]["trigger"] == "hard_pressure"
    assert captured_callbacks[0][1] == 0.0
    assert display.compaction_events[0]["trigger"] == "hard_pressure"
    assert display.compaction_events[0]["billed_cost"] == 0.0


def test_cost_tracker_preserves_unbilled_maintenance_events() -> None:
    tracker = CostTracker(
        model_id="gemini-3-flash-preview",
        pricing={"input_uncached": 0.5, "input_cached": 0.05, "output": 3.0},
    )

    event = {
        "kind": "cache_create",
        "model_id": "gemini-3-flash-preview",
        "cache_name": "cachedContents/cache_1",
        "usage": None,
        "accounting_status": "not_billed_v1",
    }
    maintenance_cost = tracker.add_maintenance_event(event)
    payload = tracker.to_dict()

    assert maintenance_cost.total_cost == 0.0
    assert payload["maintenance_events"][0]["tokens"] is None
    assert payload["maintenance_events"][0]["costs_usd"] is None
