from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import logging
import os
import time
from pathlib import Path
from typing import Any, Callable, Optional

from rich.console import Console

from agent.compiler import (
    compile_urdf_report,
    persist_compile_success_artifacts,
)
from agent.cost import CostTracker, pricing_for_provider_model
from agent.defaults import DEFAULT_MAX_TURNS
from agent.feedback import (
    compile_signal_bundle_from_exception,
    contains_code_in_text,
    render_compile_signals,
)
from agent.models import AgentResult, CompileReport, CompileSignalBundle, TerminateReason
from agent.prompts import (
    load_sdk_docs_reference,
    load_system_prompt_text,
)
from agent.prompts import (
    normalize_sdk_docs_mode as _normalize_sdk_docs_mode,
)
from agent.prompts import (
    normalize_sdk_package as _normalize_sdk_package,
)
from agent.providers.gemini import GeminiLLM
from agent.providers.openai import OpenAILLM
from agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
)
from agent.tools import (
    build_tool_registry,
    provider_system_prompt_suffix,
)
from agent.tools.base import ToolResult
from agent.tools.code_region import extract_editable_code
from agent.traces import TraceWriter
from agent.tui.single_run import SingleRunDisplay
from sdk._profiles import get_sdk_profile

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
CONSOLE = Console()


def _minimal_scaffold_text(*, sdk_package: str = "sdk") -> str:
    package = _normalize_sdk_package(sdk_package)
    if package == "sdk_hybrid":
        return """from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk_hybrid import ArticulatedObject, AssetContext, TestContext, TestReport

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model", assets=ASSETS)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
"""
    return """from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import ArticulatedObject, AssetContext, TestContext, TestReport

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model", assets=ASSETS)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
"""


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _short_cache_digest(text: str) -> str:
    digest = hashlib.sha256(text.encode("utf-8")).digest()
    return base64.urlsafe_b64encode(digest).decode("ascii").rstrip("=")


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _prompt_cache_key_strategy_from_env() -> str:
    raw = os.environ.get("OPENAI_PROMPT_CACHE_KEY_STRATEGY")
    if raw is None:
        return "articraft-v1"
    value = raw.strip().lower()
    if not value:
        return "articraft-v1"
    if value in {"articraft-v1", "off"}:
        return value
    logger.warning(
        "Unknown OPENAI_PROMPT_CACHE_KEY_STRATEGY=%r; falling back to articraft-v1",
        raw,
    )
    return "articraft-v1"


def _prompt_cache_retention_from_env(*, model_id: str) -> Optional[str]:
    raw = os.environ.get("OPENAI_PROMPT_CACHE_RETENTION")
    if raw is None:
        if model_id.strip().lower().startswith("gpt-5.4"):
            return "24h"
        return None
    value = raw.strip()
    if not value or value.lower() in {"0", "false", "none", "off"}:
        return None
    return value


def _log_compile_signals(bundle: CompileSignalBundle) -> None:
    failures = sum(1 for signal in bundle.signals if signal.severity == "failure")
    warnings = sum(1 for signal in bundle.signals if signal.severity == "warning")
    notes = sum(1 for signal in bundle.signals if signal.severity == "note")
    summary = " ".join(str(bundle.summary or "").split())
    logger.warning(
        "Compile signals: %s (failures=%s warnings=%s notes=%s)",
        summary,
        failures,
        warnings,
        notes,
    )


def build_openai_prompt_cache_settings(
    *,
    model_id: str,
    sdk_package: str,
    sdk_docs_mode: str,
    system_prompt: str,
    sdk_docs_context: str,
    tools: list[dict[str, Any]],
) -> tuple[Optional[str], Optional[str]]:
    strategy = _prompt_cache_key_strategy_from_env()
    retention = _prompt_cache_retention_from_env(model_id=model_id)

    if strategy == "off":
        logger.info(
            "OpenAI prompt caching disabled (strategy=off, retention=%s)",
            retention or "off",
        )
        return None, retention

    normalized_sdk_package = _normalize_sdk_package(sdk_package)
    normalized_docs_mode = _normalize_sdk_docs_mode(sdk_docs_mode)
    prefix = (os.environ.get("OPENAI_PROMPT_CACHE_KEY_PREFIX") or "").strip()
    key_payload = {
        "provider": "openai",
        "model_id": model_id.strip(),
        "sdk_package": normalized_sdk_package,
        "sdk_docs_mode": normalized_docs_mode,
        "system_prompt_sha256": _sha256_text(system_prompt),
        "sdk_docs_sha256": _sha256_text(sdk_docs_context),
        "tool_schema_sha256": _sha256_text(_canonical_json(tools)),
    }
    digest = _short_cache_digest(_canonical_json(key_payload))
    visible_prefix = "".join(
        ch if ch.isalnum() or ch in {"-", "_"} else "-" for ch in prefix
    ).strip("-_")[:16]
    key = f"ac1:{digest}"
    if visible_prefix:
        key = f"ac1:{visible_prefix}:{digest}"
    if len(key) > 64:
        key = f"ac1:{digest}"

    logger.info(
        "OpenAI prompt caching enabled (strategy=%s, retention=%s, prefix=%s)",
        strategy,
        retention or "off",
        prefix or "<none>",
    )
    return key, retention


class ArticraftAgent:
    def __init__(
        self,
        file_path: str,
        provider: str = "openai",
        model_id: Optional[str] = None,
        openai_transport: str = "http",
        thinking_level: str = "high",
        max_turns: int = DEFAULT_MAX_TURNS,
        system_prompt_path: str = "designer_system_prompt.txt",
        trace_dir: Optional[str] = None,
        display_enabled: Optional[bool] = None,
        on_turn_start: Optional[Callable[[int], None]] = None,
        checkpoint_urdf_path: Optional[Path] = None,
        sdk_package: str = "sdk",
        sdk_docs_mode: str = "full",
        openai_reasoning_summary: Optional[str] = "auto",
    ):
        self.file_path = file_path
        self.max_turns = max_turns
        self.sdk_package = _normalize_sdk_package(sdk_package)
        self.sdk_docs_mode = _normalize_sdk_docs_mode(sdk_docs_mode)
        self._seen_compile_signal_sigs: set[str] = set()
        self._seen_tool_error_sigs: set[str] = set()
        self._last_compile_failure_sig: Optional[str] = None
        self._consecutive_compile_failure_count = 0
        self._post_success_design_audit_sent = False
        self.checkpoint_urdf_path = (
            Path(checkpoint_urdf_path).resolve() if checkpoint_urdf_path else None
        )
        self._last_checkpoint_urdf_sig: Optional[str] = None
        self.trace_writer: Optional[TraceWriter] = (
            TraceWriter(Path(trace_dir)) if trace_dir else None
        )

        provider_norm = (provider or "openai").strip().lower()
        self.provider = provider_norm
        if provider_norm == "gemini":
            if model_id is not None:
                self.llm = GeminiLLM(model_id=model_id, thinking_level=thinking_level)
            else:
                self.llm = GeminiLLM(thinking_level=thinking_level)
        elif provider_norm == "openai":
            if model_id is not None:
                self.llm = OpenAILLM(
                    model_id=model_id,
                    thinking_level=thinking_level,
                    reasoning_summary=openai_reasoning_summary,
                    transport=openai_transport,
                )
            else:
                self.llm = OpenAILLM(
                    thinking_level=thinking_level,
                    reasoning_summary=openai_reasoning_summary,
                    transport=openai_transport,
                )
        else:
            raise ValueError(f"Unsupported provider: {provider}")

        actual_model_id = self.llm.model_id
        self.cost_tracker: Optional[CostTracker] = None
        pricing = pricing_for_provider_model(provider_norm, actual_model_id)
        if pricing:
            self.cost_tracker = CostTracker(model_id=actual_model_id, pricing=pricing)

        self.tool_registry = build_tool_registry(provider_norm, sdk_package=self.sdk_package)
        self.on_turn_start = on_turn_start

        if display_enabled is None:
            display_enabled = os.environ.get("URDF_TUI_ENABLED", "1") != "0"
        self.display = SingleRunDisplay(
            console=CONSOLE,
            model_id=actual_model_id,
            thinking_level=thinking_level,
            max_turns=max_turns,
            enabled=display_enabled,
        )

        (
            self.loaded_system_prompt_path,
            base_system_prompt,
        ) = self._build_system_prompt(system_prompt_path)
        suffix = provider_system_prompt_suffix(provider_norm, sdk_package=self.sdk_package)
        if suffix:
            self.system_prompt = f"{base_system_prompt.rstrip()}\n\n{suffix}\n"
        else:
            self.system_prompt = base_system_prompt
        repo_root = Path(__file__).resolve().parents[1]
        self.sdk_docs_context = load_sdk_docs_reference(
            repo_root,
            sdk_package=self.sdk_package,
            docs_mode=self.sdk_docs_mode,
        )
        if self.provider == "openai":
            prompt_cache_key, prompt_cache_retention = build_openai_prompt_cache_settings(
                model_id=actual_model_id,
                sdk_package=self.sdk_package,
                sdk_docs_mode=self.sdk_docs_mode,
                system_prompt=self.system_prompt,
                sdk_docs_context=self.sdk_docs_context,
                tools=self.tool_registry.get_tool_schemas(),
            )
            self.llm.prompt_cache_key = prompt_cache_key
            self.llm.prompt_cache_retention = prompt_cache_retention

    def _compile_signal_signature(self, bundle: CompileSignalBundle) -> str:
        sig_src = json.dumps(bundle.to_dict(), sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
        return hashlib.sha1(sig_src).hexdigest()

    def _maybe_inject_compile_signals(
        self,
        conversation: list[dict],
        *,
        bundle: CompileSignalBundle,
    ) -> bool:
        if not any(signal.severity == "warning" for signal in bundle.signals):
            return False

        sig = self._compile_signal_signature(bundle)
        if sig in self._seen_compile_signal_sigs:
            return False
        self._seen_compile_signal_sigs.add(sig)

        msg = {"role": "user", "content": render_compile_signals(bundle)}
        conversation.append(msg)
        if self.trace_writer:
            self.trace_writer.write_message(msg)
        return True

    def _maybe_inject_edit_code_guidance(
        self,
        conversation: list[dict],
        *,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> None:
        for tool_call, result in zip(tool_calls, tool_results, strict=False):
            func = tool_call.get("function", {}) if isinstance(tool_call, dict) else {}
            if func.get("name") != "edit_code":
                continue
            if not getattr(result, "error", None):
                continue
            if "Could not find the old_string in the code" not in result.error:
                continue

            sig = "edit_code_old_string_not_found"
            if sig in self._seen_tool_error_sigs:
                return
            self._seen_tool_error_sigs.add(sig)

            msg = {
                "role": "user",
                "content": (
                    "<edit_retry_guidance>\n"
                    "- Your last edit_code failed because `old_string` did not match the file exactly.\n"
                    "- Do NOT guess. Call `read_code` again, then pick a smaller exact snippet from the current file as `old_string` and retry.\n"
                    "- Keep edits surgical.\n"
                    "</edit_retry_guidance>"
                ),
            }
            conversation.append(msg)
            if self.trace_writer:
                self.trace_writer.write_message(msg)
            return

    def _append_compile_failure_signals(
        self,
        conversation: list[dict],
        *,
        bundle: CompileSignalBundle,
    ) -> str:
        sig = self._compile_signal_signature(bundle)
        repeated = sig == self._last_compile_failure_sig
        self._last_compile_failure_sig = sig
        streak = getattr(self, "_consecutive_compile_failure_count", 0) + 1
        self._consecutive_compile_failure_count = streak
        content = render_compile_signals(bundle, repeated=repeated, failure_streak=streak)

        msg = {"role": "user", "content": content}
        conversation.append(msg)
        if self.trace_writer:
            self.trace_writer.write_message(msg)
        return content

    def _maybe_inject_post_success_design_audit(self, conversation: list[dict]) -> bool:
        if self._post_success_design_audit_sent:
            return False

        content = (
            "Compile passed. Do a final design audit before concluding.\n\n"
            "<design_audit>\n"
            "- Check whether the prompt's hero features are prominent and unobscured.\n"
            "- Check whether the dominant silhouette reads correctly.\n"
            "- Check whether materials, proportions, and visible structure support realism.\n"
            "- Check whether your tests prove the important prompt-specific claims, not just generic validity.\n"
            "</design_audit>\n\n"
            "<continue_if_needed>\n"
            "- If any of those are weak, keep editing.\n"
            "</continue_if_needed>"
        )
        msg = {"role": "user", "content": content}
        conversation.append(msg)
        self._post_success_design_audit_sent = True
        if self.trace_writer:
            self.trace_writer.write_message(msg)
        return True

    async def _compile_urdf_report_async(self) -> CompileReport:
        return await asyncio.to_thread(
            compile_urdf_report,
            Path(self.file_path),
            sdk_package=self.sdk_package,
            rewrite_visual_glb=False,
        )

    async def _persist_compile_success_checkpoint_async(self, urdf_xml: str) -> None:
        try:
            self._last_checkpoint_urdf_sig = await asyncio.to_thread(
                persist_compile_success_artifacts,
                urdf_xml=urdf_xml,
                urdf_out=self.checkpoint_urdf_path,
                outputs_root=None,
                previous_sig=self._last_checkpoint_urdf_sig,
            )
        except Exception as exc:
            logger.warning("Failed to persist compile-success checkpoint: %s", exc)

    async def _persist_compile_failure_checkpoint_async(self, exc: BaseException) -> bool:
        compiled_urdf_xml = getattr(exc, "compiled_urdf_xml", None)
        if not isinstance(compiled_urdf_xml, str) or not compiled_urdf_xml.strip():
            return False
        await self._persist_compile_success_checkpoint_async(compiled_urdf_xml)
        return True

    def _build_system_prompt(self, prompt_path: str) -> tuple[Path, str]:
        return load_system_prompt_text(
            prompt_path,
            provider=self.provider,
            sdk_package=self.sdk_package,
        )

    def _extract_tool_calls(self, message: dict) -> list[dict]:
        return message.get("tool_calls", []) if isinstance(message, dict) else []

    def _extract_text(self, message: dict) -> str:
        if not isinstance(message, dict):
            return ""
        return message.get("content", "") or ""

    def _extract_usage(self, message: dict) -> Optional[dict[str, int]]:
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

    def _extract_thinking(self, message: dict) -> Optional[str]:
        if not isinstance(message, dict):
            return None
        return message.get("thought_summary")

    def _build_assistant_message(self, message: dict) -> dict:
        text = self._extract_text(message)
        tool_calls = self._extract_tool_calls(message)
        thinking = self._extract_thinking(message)
        usage = self._extract_usage(message)
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

    def _code_paste_nudge_text(self) -> str:
        if self.provider == "openai":
            return (
                "<tool_use_rules>\n"
                "- Do not paste code in your response.\n"
                "- Use tools to apply your changes.\n"
                "- Use read_file to fetch exact current lines, then apply_patch for edits.\n"
                "- Do not provide file_path.\n"
                "</tool_use_rules>"
            )
        return (
            "<tool_use_rules>\n"
            "- Do not paste code in your response.\n"
            "- Use tools to apply your changes.\n"
            "- Use read_code to fetch exact current text, then edit_code for edits.\n"
            '- If the editable section is empty, initialize it with edit_code using old_string="".\n'
            "</tool_use_rules>"
        )

    def _tool_call_name(self, tool_call: dict) -> str:
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

    def _tool_call_display_args(self, tool_call: dict) -> dict:
        if not isinstance(tool_call, dict):
            return {}
        func = tool_call.get("function")
        if isinstance(func, dict):
            return func.get("arguments", {})
        custom = tool_call.get("custom")
        if isinstance(custom, dict):
            return {"input": custom.get("input", "")}
        return {}

    def _ensure_code_file(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        scaffold_path = repo_root / get_sdk_profile(self.sdk_package).scaffold_path
        path = Path(self.file_path)
        if path.exists():
            existing = path.read_text()
            if existing.strip():
                return
        path.parent.mkdir(parents=True, exist_ok=True)
        if scaffold_path.exists():
            path.write_text(scaffold_path.read_text())
        else:
            path.write_text(_minimal_scaffold_text(sdk_package=self.sdk_package))

    async def _execute_tool(self, tool_call: dict) -> tuple[ToolResult, dict]:
        tool_id = tool_call["id"]
        call_type = str(tool_call.get("type", "function"))
        func_name = self._tool_call_name(tool_call)
        thought_signature = tool_call.get("thought_signature")
        if thought_signature is None:
            thought_signature = (
                tool_call.get("extra_content", {}).get("google", {}).get("thought_signature")
            )

        if call_type == "custom":
            custom = tool_call.get("custom") if isinstance(tool_call, dict) else None
            if not isinstance(custom, dict):
                result = ToolResult(
                    error="Invalid custom tool payload",
                    tool_call_id=tool_id,
                )
                tool_message = {
                    "role": "tool",
                    "tool_call_id": tool_id,
                    "name": func_name,
                    "tool_type": "custom",
                    "content": json.dumps(
                        {k: v for k, v in result.to_dict().items() if k != "tool_call_id"}
                    ),
                }
                if thought_signature:
                    tool_message["thought_signature"] = thought_signature
                return result, tool_message
            func_args = {"input": str(custom.get("input") or "")}
        else:
            function = tool_call.get("function") if isinstance(tool_call, dict) else None
            func_args_str = ""
            if isinstance(function, dict):
                func_args_str = str(function.get("arguments") or "")
            try:
                func_args = json.loads(func_args_str)
            except json.JSONDecodeError as exc:
                result = ToolResult(
                    error=f"Invalid JSON in tool arguments: {str(exc)}",
                    tool_call_id=tool_id,
                )
                tool_message = {
                    "role": "tool",
                    "tool_call_id": tool_id,
                    "name": func_name,
                    "content": json.dumps(result.to_dict()),
                }
                if thought_signature:
                    tool_message["thought_signature"] = thought_signature
                return result, tool_message

        if not isinstance(func_args, dict):
            result = ToolResult(
                error="Tool arguments must be a JSON object",
                tool_call_id=tool_id,
            )
            tool_message = {
                "role": "tool",
                "tool_call_id": tool_id,
                "name": func_name,
                "content": json.dumps(
                    {k: v for k, v in result.to_dict().items() if k != "tool_call_id"}
                ),
            }
            if thought_signature:
                tool_message["thought_signature"] = thought_signature
            return result, tool_message

        func_args.pop("file_path", None)
        func_args["file_path"] = self.file_path

        if func_name == "edit_code":
            try:
                editable = extract_editable_code(Path(self.file_path).read_text(encoding="utf-8"))
            except Exception:
                editable = None
            if (
                func_args.get("old_string") == ""
                and editable is not None
                and editable.strip() != ""
            ):
                result = ToolResult(
                    error=(
                        "old_string cannot be empty unless the editable code section is empty. "
                        "Call read_code to copy exact current text and retry."
                    ),
                    tool_call_id=tool_id,
                )
                tool_message = {
                    "role": "tool",
                    "tool_call_id": tool_id,
                    "name": func_name,
                    "content": json.dumps(
                        {k: v for k, v in result.to_dict().items() if k != "tool_call_id"}
                    ),
                }
                if thought_signature:
                    tool_message["thought_signature"] = thought_signature
                return result, tool_message
            if (
                editable is not None
                and editable.strip() == ""
                and func_args.get("old_string") != ""
            ):
                result = ToolResult(
                    error=(
                        "Editable code section is empty. Initialize it with edit_code using "
                        'old_string="" and new_string containing the initial '
                        "build_object_model() and run_tests() implementation."
                    ),
                    tool_call_id=tool_id,
                )
                tool_message = {
                    "role": "tool",
                    "tool_call_id": tool_id,
                    "name": func_name,
                    "content": json.dumps(
                        {k: v for k, v in result.to_dict().items() if k != "tool_call_id"}
                    ),
                }
                if thought_signature:
                    tool_message["thought_signature"] = thought_signature
                return result, tool_message

        try:
            invocation = await self.tool_registry.build_invocation(
                func_name,
                func_args,
            )

            if not invocation:
                result = ToolResult(
                    error=f"Tool {func_name} not found",
                    tool_call_id=tool_id,
                )
            else:
                result = await invocation.execute()
                result.tool_call_id = tool_id
        except Exception as exc:
            from pydantic import ValidationError

            if isinstance(exc, ValidationError):
                errors = exc.errors()
                missing = [err["loc"][0] for err in errors if err["type"] == "missing"]
                invalid = [
                    f"{err['loc'][0]}: {err['msg']}" for err in errors if err["type"] != "missing"
                ]

                parts = [f"Invalid parameters for {func_name}."]
                if missing:
                    parts.append(f"Missing required: {missing}")
                if invalid:
                    parts.append(f"Invalid values: {invalid}")
                parts.append(f"Provided: {list(func_args.keys())}")
                error_msg = " ".join(parts)
            else:
                error_msg = f"Tool execution error: {str(exc)}"

            result = ToolResult(error=error_msg, tool_call_id=tool_id)

        tool_message = {
            "role": "tool",
            "tool_call_id": tool_id,
            "name": func_name,
            "tool_type": call_type if call_type == "custom" else "function",
            "content": json.dumps(
                {k: v for k, v in result.to_dict().items() if k != "tool_call_id"}
            ),
        }
        if thought_signature:
            tool_message["thought_signature"] = thought_signature
        return result, tool_message

    def _is_code_valid(self, tool_results: list[ToolResult]) -> bool:
        for result in reversed(tool_results):
            if result.compilation:
                return result.compilation.get("status") == "success"
        return False

    def _should_terminate(
        self,
        text_response: str,
        tool_calls: list[dict],
        turn: int,
    ) -> tuple[bool, TerminateReason, str]:
        if contains_code_in_text(text_response):
            return False, TerminateReason.GOAL_COMPLETE, ""
        if text_response.strip() and not tool_calls:
            return True, TerminateReason.GOAL_COMPLETE, "Agent completed with response"
        if turn >= self.max_turns:
            return False, TerminateReason.MAX_TURNS, "Agent hit max turns limit"
        return False, TerminateReason.GOAL_COMPLETE, ""

    async def run(
        self,
        user_message: Any,
        *,
        initial_conversation: Optional[list[dict]] = None,
    ) -> AgentResult:
        self._ensure_code_file()
        self.display.start()
        conversation = initial_conversation or []
        if not conversation:
            conversation.extend(
                _build_first_turn_messages(
                    user_message,
                    sdk_docs_context=self.sdk_docs_context,
                )
            )
            if self.trace_writer:
                for message in conversation:
                    self.trace_writer.write_message(message)
        else:
            user_entry = {"role": "user", "content": user_message}
            conversation.append(user_entry)
            if self.trace_writer:
                self.trace_writer.write_message(user_entry)

        usage_totals: dict[str, int] = {}
        completed_turns = 0
        llm_calls = 0
        tool_call_count = 0
        compile_attempt_count = 0
        had_successful_compile = False
        last_successful_urdf_xml: Optional[str] = None
        last_compile_warnings: list[str] = []

        while completed_turns < self.max_turns:
            turn = completed_turns + 1
            llm_calls += 1
            logger.info(
                "Starting turn %s/%s (llm_call=%s)",
                turn,
                self.max_turns,
                llm_calls,
            )
            self.display.start_turn(turn)
            if self.on_turn_start:
                try:
                    self.on_turn_start(turn)
                except Exception:
                    logger.exception("on_turn_start callback failed for turn %s", turn)

            llm_start = time.monotonic()
            self.display.start_llm_wait()
            logger.info("Calling LLM (%s)...", type(self.llm).__name__)
            try:
                response = await self.llm.generate_with_tools(
                    system_prompt=self.system_prompt,
                    messages=conversation,
                    tools=self.tool_registry.get_tool_schemas(),
                )
            except Exception as exc:
                logger.exception("LLM error on turn %s: %s", turn, exc)
                self.display.end_turn(success=False, error=str(exc))
                return AgentResult(
                    success=False,
                    reason=TerminateReason.ERROR,
                    message=f"LLM error: {str(exc)}",
                    conversation=conversation,
                    compile_warnings=list(last_compile_warnings),
                    turn_count=completed_turns,
                    tool_call_count=tool_call_count,
                    compile_attempt_count=compile_attempt_count,
                )
            finally:
                self.display.stop_llm_wait()
                logger.info("LLM call finished in %.2fs", time.monotonic() - llm_start)

            text = self._extract_text(response)
            tool_calls = self._extract_tool_calls(response)
            thinking = self._extract_thinking(response)
            usage = self._extract_usage(response) or {}
            for key, value in usage.items():
                usage_totals[key] = usage_totals.get(key, 0) + value

            assistant_message = self._build_assistant_message(response)
            has_assistant_payload = any(
                key in assistant_message for key in ("content", "tool_calls", "thought_summary")
            )
            if has_assistant_payload:
                conversation.append(assistant_message)
                if self.trace_writer:
                    self.trace_writer.write_message(assistant_message)
            else:
                logger.warning("Skipping empty assistant message")

            if usage and self.cost_tracker:
                turn_cost = self.cost_tracker.add_turn(usage)
                llm_duration = time.monotonic() - llm_start
                self.display.add_llm_call(usage, turn_cost.total_cost, llm_duration)

            logger.info(
                "Turn %s: %s tool calls, text_length=%s",
                turn,
                len(tool_calls),
                len(text),
            )
            if isinstance(thinking, str) and thinking.strip():
                self.display.add_thinking_summary(thinking)

            is_empty_response = not has_assistant_payload and not tool_calls and not text.strip()
            if is_empty_response:
                if had_successful_compile and isinstance(last_successful_urdf_xml, str):
                    logger.info("Empty response after successful compile; terminating run.")
                    final_code = None
                    try:
                        import aiofiles

                        async with aiofiles.open(self.file_path, "r") as file:
                            final_code = await file.read()
                    except Exception:
                        pass

                    if self.cost_tracker:
                        try:
                            cost_path = Path(self.file_path).parent / "cost.json"
                            self.cost_tracker.save_json(cost_path)
                            logger.info("Saved cost tracking to %s", cost_path)
                        except Exception as exc:
                            logger.warning("Failed to save cost tracking: %s", exc)

                    self.display.current_turn = completed_turns
                    self.display.end_turn(success=True)
                    return AgentResult(
                        success=True,
                        reason=TerminateReason.CODE_VALID,
                        message="Compile succeeded and model returned no further actions",
                        conversation=conversation,
                        final_code=final_code,
                        urdf_xml=last_successful_urdf_xml,
                        compile_warnings=list(last_compile_warnings),
                        turn_count=completed_turns,
                        tool_call_count=tool_call_count,
                        compile_attempt_count=compile_attempt_count,
                        usage=usage_totals or None,
                    )

                logger.warning(
                    "Skipping no-op model response (empty assistant + no tool calls); "
                    "not counting toward max turns."
                )
                self.display.current_turn = completed_turns
                self.display.end_turn(success=True)
                continue

            completed_turns += 1

            if contains_code_in_text(text):
                nudge = {
                    "role": "user",
                    "content": self._code_paste_nudge_text(),
                }
                conversation.append(nudge)
                if self.trace_writer:
                    self.trace_writer.write_message(nudge)
                continue

            should_stop, reason, msg = self._should_terminate(text, tool_calls, turn)
            if should_stop:
                try:
                    compile_attempt_count += 1
                    compile_start = time.monotonic()
                    logger.info("Running URDF compile checks...")
                    report = await self._compile_urdf_report_async()
                    logger.info(
                        "URDF compile passed in %.2fs",
                        time.monotonic() - compile_start,
                    )
                    last_compile_warnings = list(report.warnings)
                    await self._persist_compile_success_checkpoint_async(report.urdf_xml)
                    had_successful_compile = True
                    self._consecutive_compile_failure_count = 0
                    last_successful_urdf_xml = report.urdf_xml
                    compile_duration = time.monotonic() - compile_start
                    self.display.add_compile_result(
                        success=True,
                        duration=compile_duration,
                        warnings=report.warnings,
                    )
                    injected = self._maybe_inject_compile_signals(
                        conversation,
                        bundle=report.signal_bundle,
                    )
                    if injected:
                        continue
                    if self._maybe_inject_post_success_design_audit(conversation):
                        continue
                except TimeoutError as exc:
                    bundle = compile_signal_bundle_from_exception(exc)
                    _log_compile_signals(bundle)
                    compile_duration = time.monotonic() - compile_start
                    retry_message = self._append_compile_failure_signals(
                        conversation,
                        bundle=bundle,
                    )
                    self.display.add_compile_result(
                        success=False,
                        duration=compile_duration,
                        error=retry_message,
                    )
                    continue
                except Exception as exc:
                    await self._persist_compile_failure_checkpoint_async(exc)
                    bundle = compile_signal_bundle_from_exception(exc)
                    _log_compile_signals(bundle)
                    compile_duration = time.monotonic() - compile_start
                    retry_message = self._append_compile_failure_signals(
                        conversation,
                        bundle=bundle,
                    )
                    self.display.add_compile_result(
                        success=False,
                        duration=compile_duration,
                        error=retry_message,
                    )
                    continue

                final_code = None
                try:
                    import aiofiles

                    async with aiofiles.open(self.file_path, "r") as file:
                        final_code = await file.read()
                except Exception:
                    pass

                if self.cost_tracker:
                    try:
                        cost_path = Path(self.file_path).parent / "cost.json"
                        self.cost_tracker.save_json(cost_path)
                        logger.info("Saved cost tracking to %s", cost_path)
                    except Exception as exc:
                        logger.warning("Failed to save cost tracking: %s", exc)

                self.display.end_turn(success=True)
                return AgentResult(
                    success=True,
                    reason=TerminateReason.CODE_VALID,
                    message=msg,
                    conversation=conversation,
                    final_code=final_code,
                    urdf_xml=report.urdf_xml,
                    compile_warnings=list(last_compile_warnings),
                    turn_count=completed_turns,
                    tool_call_count=tool_call_count,
                    compile_attempt_count=compile_attempt_count,
                    usage=usage_totals or None,
                )

            turn_tool_results: list[ToolResult] = []
            tool_call_count += len(tool_calls)
            for tool_call in tool_calls:
                func_name = self._tool_call_name(tool_call)
                func_args = self._tool_call_display_args(tool_call)
                logger.info("Executing tool: %s", func_name)

                tool_start = time.monotonic()
                result, tool_message = await self._execute_tool(tool_call)
                tool_duration = time.monotonic() - tool_start
                logger.info("Tool %s finished in %.2fs", func_name, tool_duration)
                turn_tool_results.append(result)
                conversation.append(tool_message)
                if self.trace_writer:
                    self.trace_writer.write_message(tool_message)

                self.display.add_tool_call(
                    tool_name=func_name,
                    args=func_args,
                    success=result.is_success(),
                    duration=tool_duration,
                    result=result.output if result.is_success() else None,
                    compilation=result.compilation,
                    error=result.error if not result.is_success() else None,
                )

                if result.is_success():
                    logger.info("Tool %s succeeded", func_name)
                else:
                    logger.warning("Tool %s failed: %s", func_name, result.error)

            self._maybe_inject_edit_code_guidance(
                conversation,
                tool_calls=tool_calls,
                tool_results=turn_tool_results,
            )

            if self._is_code_valid(turn_tool_results):
                try:
                    compile_attempt_count += 1
                    compile_start = time.monotonic()
                    logger.info("Running URDF compile checks...")
                    report = await self._compile_urdf_report_async()
                    compile_duration = time.monotonic() - compile_start
                    logger.info("URDF compile passed in %.2fs", compile_duration)
                    last_compile_warnings = list(report.warnings)
                    await self._persist_compile_success_checkpoint_async(report.urdf_xml)
                    had_successful_compile = True
                    self._consecutive_compile_failure_count = 0
                    last_successful_urdf_xml = report.urdf_xml
                    self.display.add_compile_result(
                        success=True,
                        duration=compile_duration,
                        warnings=report.warnings,
                    )
                    injected = self._maybe_inject_compile_signals(
                        conversation,
                        bundle=report.signal_bundle,
                    )
                    if not injected:
                        self._maybe_inject_post_success_design_audit(conversation)
                except TimeoutError as exc:
                    bundle = compile_signal_bundle_from_exception(exc)
                    _log_compile_signals(bundle)
                    compile_duration = time.monotonic() - compile_start
                    retry_message = self._append_compile_failure_signals(
                        conversation,
                        bundle=bundle,
                    )
                    self.display.add_compile_result(
                        success=False,
                        duration=compile_duration,
                        error=retry_message,
                    )
                except Exception as exc:
                    await self._persist_compile_failure_checkpoint_async(exc)
                    bundle = compile_signal_bundle_from_exception(exc)
                    _log_compile_signals(bundle)
                    compile_duration = time.monotonic() - compile_start
                    retry_message = self._append_compile_failure_signals(
                        conversation,
                        bundle=bundle,
                    )
                    self.display.add_compile_result(
                        success=False,
                        duration=compile_duration,
                        error=retry_message,
                    )

            if self._is_code_valid(turn_tool_results):
                logger.info("Code validation passed")

            self.display.end_turn(success=True)

        final_code = None
        try:
            import aiofiles

            async with aiofiles.open(self.file_path, "r") as file:
                final_code = await file.read()
        except Exception:
            pass

        if self.cost_tracker:
            try:
                cost_path = Path(self.file_path).parent / "cost.json"
                self.cost_tracker.save_json(cost_path)
                logger.info("Saved cost tracking to %s", cost_path)
            except Exception as exc:
                logger.warning("Failed to save cost tracking: %s", exc)

        return AgentResult(
            success=False,
            reason=TerminateReason.MAX_TURNS,
            message="Agent hit max turns limit",
            conversation=conversation,
            final_code=final_code,
            compile_warnings=list(last_compile_warnings),
            turn_count=completed_turns,
            tool_call_count=tool_call_count,
            compile_attempt_count=compile_attempt_count,
            usage=usage_totals or None,
        )

    async def close(self) -> None:
        self.display.stop()
        if self.trace_writer:
            self.trace_writer.close()
        await self.llm.close()

    async def __aenter__(self) -> ArticraftAgent:
        return self

    async def __aexit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        await self.close()
