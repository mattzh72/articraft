from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from articraft.values import ProviderName

PARALLEL_SAFE_TOOL_NAMES = frozenset({"read_file", "find_examples", "probe_model"})


@dataclass(frozen=True)
class MessageCodec:
    provider: str

    def extract_tool_calls(self, message: dict) -> list[dict]:
        return message.get("tool_calls", []) if isinstance(message, dict) else []

    def extract_text(self, message: dict) -> str:
        if not isinstance(message, dict):
            return ""
        return message.get("content", "") or ""

    def extract_usage(self, message: dict) -> Optional[dict[str, int]]:
        if not isinstance(message, dict):
            return None
        usage = message.get("usage")
        if not isinstance(usage, dict):
            return None
        cleaned: dict[str, int] = {}
        for key, value in usage.items():
            if isinstance(key, str) and isinstance(value, int):
                cleaned[key] = value
        return cleaned or None

    def extract_thinking(self, message: dict) -> Optional[str]:
        if not isinstance(message, dict):
            return None
        return message.get("thought_summary")

    def build_assistant_message(self, message: dict) -> dict:
        text = self.extract_text(message)
        tool_calls = self.extract_tool_calls(message)
        thinking = self.extract_thinking(message)
        usage = self.extract_usage(message)
        extra_content = message.get("extra_content") if isinstance(message, dict) else None

        msg = {"role": "assistant"}
        if thinking:
            msg["thought_summary"] = thinking
        if text:
            msg["content"] = text
        if tool_calls:
            msg["tool_calls"] = tool_calls
        if extra_content:
            msg["extra_content"] = extra_content
        if usage:
            msg["usage"] = usage
        return msg

    def tool_call_name(self, tool_call: dict) -> str:
        if not isinstance(tool_call, dict):
            return ""
        func = tool_call.get("function")
        if isinstance(func, dict):
            name = func.get("name")
            if isinstance(name, str):
                return name
        custom = tool_call.get("custom")
        if isinstance(custom, dict):
            name = custom.get("name")
            if isinstance(name, str):
                return name
        name = tool_call.get("name")
        if isinstance(name, str):
            return name
        return ""

    def tool_call_display_args(self, tool_call: dict) -> dict:
        if not isinstance(tool_call, dict):
            return {}
        func = tool_call.get("function")
        if isinstance(func, dict):
            return func.get("arguments", {})
        custom = tool_call.get("custom")
        if isinstance(custom, dict):
            return {"input": custom.get("input", "")}
        return {}

    def tool_calls_are_parallelizable(self, tool_calls: list[dict]) -> bool:
        if self.provider != ProviderName.GEMINI.value or len(tool_calls) <= 1:
            return False
        return all(
            self.tool_call_name(tool_call) in PARALLEL_SAFE_TOOL_NAMES for tool_call in tool_calls
        )
