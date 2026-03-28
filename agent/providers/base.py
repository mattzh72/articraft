from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

ToolSchema = dict[str, Any]
ConversationMessage = dict[str, Any]
ProviderResponse = dict[str, Any]


class ProviderClient(Protocol):
    """Common interface for model providers used by the runtime."""

    model_id: str

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
    ) -> dict[str, Any]: ...

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[ConversationMessage],
        tools: list[ToolSchema],
    ) -> ProviderResponse: ...

    def reset_context(self) -> None: ...

    async def close(self) -> None: ...


@dataclass(slots=True, frozen=True)
class ProviderCapabilities:
    name: str
    supports_tools: bool = True
    supports_images: bool = False
    supports_reasoning_summary: bool = False
    supports_parallel_tool_calls: bool = False
