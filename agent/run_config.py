from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from articraft.values import ProviderName, normalize_provider_name


@dataclass(slots=True, frozen=True)
class SingleRunSettings:
    provider: str
    model_id: str
    thinking_level: str
    max_turns: int
    system_prompt_path: str
    sdk_package: str
    openai_transport: str = "http"
    openai_reasoning_summary: str | None = "auto"
    max_cost_usd: float | None = None

    def to_summary(self) -> dict[str, Any]:
        summary: dict[str, Any] = {
            "provider": self.provider,
            "model_id": self.model_id,
            "thinking_level": self.thinking_level,
            "max_turns": self.max_turns,
            "max_cost_usd": self.max_cost_usd,
            "system_prompt_path": self.system_prompt_path,
            "sdk_package": self.sdk_package,
        }
        if normalize_provider_name(self.provider) is ProviderName.OPENAI:
            summary["openai_transport"] = self.openai_transport
            summary["openai_reasoning_summary"] = self.openai_reasoning_summary
        return summary
