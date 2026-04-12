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
from agent.tools.compile_model import CompileModelTool
from agent.tools.edit_code import EditCodeTool, ReplaceTool
from agent.tools.find_examples import FindExamplesTool
from agent.tools.probe_model import ProbeModelTool
from agent.tools.read_code import ReadCodeTool
from agent.tools.read_file import ReadFileTool
from agent.tools.registry import ToolRegistry
from agent.tools.write_code import WriteCodeTool, WriteFileTool

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

_FIRST_TURN_RUNTIME_GUIDANCE_SHARED = (
    "<runtime_task_guidance>\n"
    "- Start with a small coherent backbone or subassembly, then expand incrementally.\n"
    "- For unfamiliar geometry or mechanisms, use `find_examples` before improvising.\n"
    "- Check your work as you go. Do not batch the whole object into one edit.\n"
    "- After each coherent chunk, run `compile_model` before moving on.\n"
    "- Use tools deliberately. Prefer the smallest action that gives decisive evidence.\n"
    "- If the cause is obvious from `model.py` and `compile_model` output, fix it directly.\n"
    "- Use `probe_model` when geometry, pose, support path, or exact-element identity is ambiguous. After a first ambiguous spatial repair does not resolve the issue, `probe_model` is often the best next step to gather evidence before patching again.\n"
    "- Do not do blind self-correction passes without new evidence.\n"
    "</runtime_task_guidance>"
)

_FIRST_TURN_RUNTIME_GUIDANCE_BY_PROVIDER: dict[str, str] = {
    "openai": (
        "<runtime_task_guidance>\n"
        "- Start with a small coherent backbone or subassembly, then expand incrementally.\n"
        '- Read the exact current code with `read_file(path="model.py")` before editing.\n'
        "- Start with a short context pass: decide the next coherent edit, then read only the docs/examples needed for it.\n"
        '- The SDK quickstart/router is preloaded. If a `docs/sdk/references/...` file is relevant, read the full file with `read_file(path="docs/...")`.\n'
        "- Do not front-load unrelated docs, and do not re-read a reference file that is already in context.\n"
        "- Use `find_examples` for unfamiliar geometry, mechanisms, placement patterns, or testing structure, and adapt examples against current SDK docs.\n"
        "- Prefer multiple small `apply_patch` edits over one giant patch.\n"
        "- After each coherent chunk, run `compile_model` before moving on.\n"
        "- Use tools deliberately. Prefer the smallest action that gives decisive evidence.\n"
        "- If the cause is obvious from `model.py` and `compile_model` output, fix it directly.\n"
        "- Use `probe_model` when geometry, pose, support path, or exact-element identity is ambiguous. After a first ambiguous spatial repair does not resolve the issue, `probe_model` is often the best next step to gather evidence before patching again.\n"
        "</runtime_task_guidance>"
    ),
    "gemini": (
        "<runtime_task_guidance>\n"
        "- Start with a small coherent backbone or subassembly, then expand incrementally.\n"
        '- Read the exact current editable code with `read_file(path="model.py")` before editing.\n'
        "- Start with a short context pass: decide the next coherent edit, then read only the docs/examples needed for it.\n"
        '- The SDK quickstart/router is preloaded. If a `docs/sdk/references/...` file is relevant, read the full file with `read_file(path="docs/...")`.\n'
        "- Do not front-load unrelated docs, and do not re-read a reference file that is already in context.\n"
        "- Use `find_examples` for unfamiliar geometry, mechanisms, placement patterns, or testing structure, and adapt examples against current SDK docs.\n"
        "- Prefer small exact `replace` edits over broad rewrites.\n"
        '- If `replace` fails because `old_string` did not match, reread `read_file(path="model.py")` and retry with a smaller exact snippet.\n'
        "- Use `write_file` when you intentionally want to rewrite the full editable section.\n"
        "- After each coherent chunk, run `compile_model` before moving on.\n"
        "- Use tools deliberately. Prefer the smallest action that gives decisive evidence.\n"
        "- If the cause is obvious from `model.py` and `compile_model` output, fix it directly.\n"
        "- Use `probe_model` when geometry, pose, support path, or exact-element identity is ambiguous. After a first ambiguous spatial repair does not resolve the issue, `probe_model` is often the best next step to gather evidence before patching again.\n"
        "</runtime_task_guidance>"
    ),
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
            CompileModelTool(),
            ProbeModelTool(sdk_package=package, runtime_limits=runtime_limits),
        ]
    else:
        tools = [
            ReadFileTool(editable_model_only=True),
            ReplaceTool(),
            WriteFileTool(),
            CompileModelTool(),
            ProbeModelTool(sdk_package=package, runtime_limits=runtime_limits),
        ]
    tools.append(FindExamplesTool(sdk_package=package, include_paths=provider_norm == "openai"))
    return ToolRegistry(tools)


def provider_system_prompt_suffix(provider: str, *, sdk_package: str = "sdk") -> str:
    normalize_sdk_package(sdk_package)
    return ""


def build_first_turn_runtime_guidance(provider: str) -> str:
    provider_norm = (provider or "").strip().lower()
    return _FIRST_TURN_RUNTIME_GUIDANCE_BY_PROVIDER.get(
        provider_norm,
        _FIRST_TURN_RUNTIME_GUIDANCE_SHARED,
    )


def prepend_runtime_guidance(
    user_content: Any,
    *,
    runtime_guidance_text: str | None = None,
) -> Any:
    guidance = (runtime_guidance_text or "").strip()
    if not guidance:
        return user_content

    if isinstance(user_content, str):
        if not user_content.strip():
            return guidance
        return f"{guidance}\n\n{user_content}"

    if not isinstance(user_content, list):
        return user_content

    return [{"type": "input_text", "text": guidance}, *user_content]


def build_first_turn_messages(
    user_content: Any,
    *,
    sdk_docs_context: str,
    provider: str,
) -> list[dict[str, Any]]:
    messages: list[dict[str, Any]] = []
    if sdk_docs_context:
        messages.append({"role": "user", "content": sdk_docs_context})
    messages.append(
        {
            "role": "user",
            "content": prepend_runtime_guidance(
                user_content,
                runtime_guidance_text=build_first_turn_runtime_guidance(provider),
            ),
        }
    )
    return messages


def build_initial_user_content(
    text_prompt: str,
    *,
    image_path: Path | None = None,
    image_detail: str = "high",
    runtime_guidance_text: str | None = None,
) -> Any:
    content: Any
    if image_path is None:
        content = text_prompt
    else:
        content = [
            {"type": "input_text", "text": text_prompt},
            {
                "type": "input_image",
                "image_path": str(image_path),
                "detail": image_detail,
            },
        ]

    return prepend_runtime_guidance(content, runtime_guidance_text=runtime_guidance_text)


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
    "CompileModelTool",
    "EditCodeTool",
    "FindExamplesTool",
    "ProbeModelTool",
    "ReadCodeTool",
    "ReadFileTool",
    "ReplaceTool",
    "WriteCodeTool",
    "WriteFileTool",
    "ToolRegistry",
    "SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER",
    "build_tool_registry",
    "provider_system_prompt_suffix",
    "build_first_turn_runtime_guidance",
    "prepend_runtime_guidance",
    "build_first_turn_messages",
    "build_initial_user_content",
    "resolve_image_path",
]
