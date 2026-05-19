from __future__ import annotations

from typing import Any

AGENT_HARNESS_VALUES = ("articraft", "codex", "claude-code")
EXTERNAL_AGENT_HARNESSES = frozenset({"codex", "claude-code"})


def agent_harness_from_record(record: dict[str, Any]) -> str:
    creator = record.get("creator")
    if not isinstance(creator, dict):
        return "articraft"
    if creator.get("mode") != "external_agent":
        return "articraft"
    agent = creator.get("agent")
    if isinstance(agent, str) and agent in EXTERNAL_AGENT_HARNESSES:
        return agent
    return "articraft"


def within_agent_harness_filters(
    agent_harness: str | None,
    filter_values: list[str] | None,
) -> bool:
    if not filter_values:
        return True
    normalized = {
        value.strip() for value in filter_values if value and value.strip() in AGENT_HARNESS_VALUES
    }
    if not normalized:
        return True
    return (agent_harness or "articraft") in normalized
