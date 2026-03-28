from __future__ import annotations

import mimetypes
from pathlib import Path
from typing import Any

from agent.prompts import normalize_sdk_package
from agent.runtime_limits import BatchRuntimeLimits
from agent.tools.apply_patch import ApplyPatchFreeformTool, ApplyPatchTool
from agent.tools.base import (
    BaseDeclarativeTool,
    BaseToolInvocation,
    ToolResult,
    ToolSchema,
    make_tool_schema,
)
from agent.tools.edit_code import EditCodeTool
from agent.tools.find_examples import FindExamplesTool
from agent.tools.probe_model import ProbeModelTool
from agent.tools.read_code import ReadCodeTool
from agent.tools.read_file import ReadFileTool
from agent.tools.registry import ToolRegistry
from agent.tools.write_code import WriteCodeTool

SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER: dict[str, set[str]] = {
    "openai": {
        "image/png",
        "image/jpeg",
        "image/webp",
        "image/gif",
    },
    "gemini": {
        "image/png",
        "image/jpeg",
        "image/webp",
        "image/heic",
        "image/heif",
    },
}


def build_tool_registry(
    provider: str,
    *,
    sdk_package: str = "sdk",
    runtime_limits: BatchRuntimeLimits | None = None,
) -> ToolRegistry:
    provider_norm = (provider or "openai").strip().lower()
    package = normalize_sdk_package(sdk_package)
    if provider_norm == "openai":
        tools: list[BaseDeclarativeTool] = [
            ReadFileTool(),
            ApplyPatchFreeformTool(),
            ProbeModelTool(sdk_package=package, runtime_limits=runtime_limits),
        ]
    else:
        tools = [
            ReadCodeTool(),
            EditCodeTool(),
            ProbeModelTool(sdk_package=package, runtime_limits=runtime_limits),
        ]
    tools.append(FindExamplesTool(sdk_package=package))
    return ToolRegistry(tools)


def provider_system_prompt_suffix(provider: str, *, sdk_package: str = "sdk") -> str:
    normalize_sdk_package(sdk_package)
    return ""


def small_context_loop_mode_hint_text() -> str:
    return (
        "<small_context_loop>\n"
        "- Small-context loop is enabled for this run.\n"
        "- After successful apply_patch turns, prior turn transcript is discarded.\n"
        "- The file on disk is the source of truth.\n"
        "- Re-read the current file before editing; do not rely on prior tool outputs remaining in context.\n"
        "</small_context_loop>"
    )


def build_first_turn_messages(
    user_content: Any,
    *,
    sdk_docs_context: str,
    small_context_loop: bool = False,
) -> list[dict[str, Any]]:
    messages: list[dict[str, Any]] = []
    if sdk_docs_context:
        messages.append({"role": "user", "content": sdk_docs_context})
    messages.append({"role": "user", "content": user_content})
    if small_context_loop:
        messages.append({"role": "user", "content": small_context_loop_mode_hint_text()})
    return messages


def build_initial_user_content(
    text_prompt: str,
    *,
    image_path: Path | None = None,
    image_detail: str = "high",
) -> Any:
    if image_path is None:
        return text_prompt

    return [
        {"type": "input_text", "text": text_prompt},
        {
            "type": "input_image",
            "image_path": str(image_path),
            "detail": image_detail,
        },
    ]


def resolve_image_path(
    image_arg: str | None,
    *,
    provider: str | None = None,
) -> Path | None:
    if not image_arg:
        return None

    path = Path(image_arg).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Image file not found: {path}")
    if not path.is_file():
        raise ValueError(f"Image path is not a file: {path}")

    mime_type, _ = mimetypes.guess_type(path.name)
    provider_norm = (provider or "openai").strip().lower()
    supported_mime_types = SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER.get(provider_norm)
    if supported_mime_types is None:
        raise ValueError(f"Unsupported provider for image validation: {provider}")
    if mime_type not in supported_mime_types:
        raise ValueError(
            f"Unsupported image type for {provider_norm}: {path.name} ({mime_type or 'unknown'})"
        )

    size_bytes = path.stat().st_size
    if provider_norm == "gemini":
        if size_bytes >= 20 * 1024 * 1024:
            raise ValueError(
                f"Image file exceeds Gemini inline request limit: {path} "
                "(must stay under 20 MB including prompt text)"
            )
    elif size_bytes > 50 * 1024 * 1024:
        raise ValueError(f"Image file exceeds 50 MB request limit: {path}")

    return path


__all__ = [
    "BaseDeclarativeTool",
    "BaseToolInvocation",
    "ToolResult",
    "ToolSchema",
    "make_tool_schema",
    "ApplyPatchFreeformTool",
    "ApplyPatchTool",
    "EditCodeTool",
    "FindExamplesTool",
    "ProbeModelTool",
    "ReadCodeTool",
    "ReadFileTool",
    "WriteCodeTool",
    "ToolRegistry",
    "SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER",
    "build_tool_registry",
    "provider_system_prompt_suffix",
    "small_context_loop_mode_hint_text",
    "build_first_turn_messages",
    "build_initial_user_content",
    "resolve_image_path",
]
