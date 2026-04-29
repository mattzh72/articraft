from __future__ import annotations

import ast
import asyncio
import base64
import hashlib
import json
import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from rich.console import Console

from agent.compiler import (
    compile_urdf_report_maybe_timeout,
    persist_compile_success_artifacts,
)
from agent.cost import CostTracker, pricing_for_provider_model
from agent.defaults import resolve_max_turns
from agent.feedback import (
    compile_signal_bundle_from_exception,
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
from agent.providers.openrouter import OpenRouterLLM
from agent.runtime_limits import BatchRuntimeLimits, local_work_slot
from agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
)
from agent.tools import (
    build_tool_registry,
)
from agent.tools.base import ToolResult
from agent.tools.code_region import extract_editable_code
from agent.traces import TraceWriter
from agent.tui.single_run import SingleRunDisplay
from agent.workspace_docs import build_virtual_workspace
from sdk._profiles import get_sdk_profile

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
CONSOLE = Console()
_FIND_EXAMPLES_SKIPPED_CONTENT = "{Skipped: full content already returned earlier in this run.}"
_MUTATING_TOOL_NAMES = frozenset(
    {"apply_patch", "edit_code", "write_code", "replace", "write_file"}
)
_PARALLEL_SAFE_TOOL_NAMES = frozenset({"read_file", "find_examples", "probe_model"})
_EXACT_ELEMENT_KEYWORDS = frozenset(
    {"elem_a", "elem_b", "positive_elem", "negative_elem", "inner_elem", "outer_elem"}
)
_BASELINE_QC_CALLS = frozenset(
    {
        "check_model_valid",
        "check_mesh_assets_ready",
        "fail_if_isolated_parts",
        "warn_if_part_contains_disconnected_geometry_islands",
        "fail_if_parts_overlap_in_current_pose",
    }
)


@dataclass(frozen=True)
class _CodeContractScan:
    visual_names: tuple[str, ...]
    referenced_exact_names: tuple[str, ...]
    baseline_qc_calls: tuple[str, ...]


def _constant_string(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        value = node.value.strip()
        return value or None
    return None


def _call_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Attribute):
        return node.attr
    if isinstance(node, ast.Name):
        return node.id
    return None


class _CodeContractVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.visual_names: set[str] = set()
        self.referenced_exact_names: set[str] = set()
        self.baseline_qc_calls: set[str] = set()
        self._function_stack: list[str] = []

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_Call(self, node: ast.Call) -> None:
        call_name = _call_name(node.func)
        if call_name == "visual":
            for keyword in node.keywords:
                if keyword.arg != "name":
                    continue
                visual_name = _constant_string(keyword.value)
                if visual_name is not None:
                    self.visual_names.add(visual_name)

        in_run_tests = bool(self._function_stack and self._function_stack[-1] == "run_tests")
        if in_run_tests and call_name in _BASELINE_QC_CALLS and not node.args and not node.keywords:
            self.baseline_qc_calls.add(f"{call_name}()")

        if in_run_tests:
            for keyword in node.keywords:
                if keyword.arg not in _EXACT_ELEMENT_KEYWORDS:
                    continue
                elem_name = _constant_string(keyword.value)
                if elem_name is not None:
                    self.referenced_exact_names.add(elem_name)

        self.generic_visit(node)


def _scan_code_contracts(text: str) -> _CodeContractScan | None:
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = _CodeContractVisitor()
    visitor.visit(tree)
    return _CodeContractScan(
        visual_names=tuple(sorted(visitor.visual_names)),
        referenced_exact_names=tuple(sorted(visitor.referenced_exact_names)),
        baseline_qc_calls=tuple(sorted(visitor.baseline_qc_calls)),
    )


def _minimal_scaffold_text(
    *,
    sdk_package: str = "sdk",
) -> str:
    repo_root = Path(__file__).resolve().parents[1]
    scaffold_path = repo_root / get_sdk_profile(sdk_package).scaffold_path
    if not scaffold_path.exists():
        raise FileNotFoundError(f"Missing scaffold source of truth: {scaffold_path}")
    return scaffold_path.read_text(encoding="utf-8")


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _short_cache_digest(text: str) -> str:
    digest = hashlib.sha256(text.encode("utf-8")).digest()
    return base64.urlsafe_b64encode(digest).decode("ascii").rstrip("=")


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _stable_tool_schema_payload(tools: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def _tool_identity(tool: dict[str, Any]) -> tuple[str, str]:
        tool_type = str(tool.get("type") or "")
        if tool_type == "function":
            function = tool.get("function")
            if isinstance(function, dict):
                return tool_type, str(function.get("name") or "")
        return tool_type, str(tool.get("name") or "")

    normalized = [tool for tool in tools if isinstance(tool, dict)]
    return sorted(normalized, key=_tool_identity)


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
        normalized = model_id.strip().lower()
        if (
            normalized == "gpt-5"
            or normalized.startswith("gpt-5.1")
            or normalized.startswith("gpt-5-codex")
            or normalized.startswith("gpt-5.1-codex")
            or normalized.startswith("gpt-4.1")
        ):
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
    _normalize_sdk_docs_mode(sdk_docs_mode)
    prefix = (os.environ.get("OPENAI_PROMPT_CACHE_KEY_PREFIX") or "").strip()
    key_payload = {
        "provider": "openai",
        "model_id": model_id.strip(),
        "sdk_package": normalized_sdk_package,
        "system_prompt_sha256": _sha256_text(system_prompt),
        "sdk_docs_sha256": _sha256_text(sdk_docs_context),
        "tool_schema_sha256": _sha256_text(_canonical_json(_stable_tool_schema_payload(tools))),
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
        max_turns: int | None = None,
        system_prompt_path: str = "designer_system_prompt.txt",
        trace_dir: Optional[str] = None,
        display_enabled: Optional[bool] = None,
        on_turn_start: Optional[Callable[[int], None]] = None,
        on_compaction_event: Optional[Callable[[dict[str, Any], float], None]] = None,
        on_maintenance_event: Optional[Callable[[dict[str, Any], float], None]] = None,
        checkpoint_urdf_path: Optional[Path] = None,
        sdk_package: str = "sdk",
        sdk_docs_mode: str = "full",
        openai_reasoning_summary: Optional[str] = "auto",
        post_success_design_audit: bool = False,
        max_cost_usd: float | None = None,
        runtime_limits: BatchRuntimeLimits | None = None,
    ):
        self.file_path = file_path
        self.sdk_package = _normalize_sdk_package(sdk_package)
        self.sdk_docs_mode = _normalize_sdk_docs_mode(sdk_docs_mode)
        self.runtime_limits = runtime_limits
        self._seen_compile_signal_sigs: set[str] = set()
        self._seen_tool_error_sigs: set[str] = set()
        self._seen_find_example_paths: set[str] = set()
        self._seen_exact_geometry_contract_sigs: set[str] = set()
        self._seen_baseline_qc_guidance_sigs: set[str] = set()
        self._last_compile_failure_sig: Optional[str] = None
        self._consecutive_compile_failure_count = 0
        self.checkpoint_urdf_path = (
            Path(checkpoint_urdf_path).resolve() if checkpoint_urdf_path else None
        )
        self._last_checkpoint_urdf_sig: Optional[str] = None
        self._current_edit_revision = 0
        self._last_successful_compile_revision: int | None = None
        self._last_successful_compile_report: CompileReport | None = None
        self._compile_attempt_count = 0
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
        elif provider_norm == "openrouter":
            if model_id is not None:
                self.llm = OpenRouterLLM(model_id=model_id, thinking_level=thinking_level)
            else:
                self.llm = OpenRouterLLM(thinking_level=thinking_level)
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
        self.max_turns = resolve_max_turns(model_id=actual_model_id, max_turns=max_turns)
        self.cost_tracker: Optional[CostTracker] = None
        self.max_cost_usd = max_cost_usd
        pricing = pricing_for_provider_model(provider_norm, actual_model_id)
        if pricing:
            self.cost_tracker = CostTracker(model_id=actual_model_id, pricing=pricing)

        self.tool_registry = build_tool_registry(
            provider_norm,
            sdk_package=self.sdk_package,
            runtime_limits=self.runtime_limits,
        )
        self.on_turn_start = on_turn_start
        self.on_compaction_event = on_compaction_event
        self.on_maintenance_event = on_maintenance_event

        if display_enabled is None:
            display_enabled = os.environ.get("URDF_TUI_ENABLED", "1") != "0"
        self.display = SingleRunDisplay(
            console=CONSOLE,
            model_id=actual_model_id,
            thinking_level=thinking_level,
            max_turns=self.max_turns,
            enabled=display_enabled,
        )

        (
            self.loaded_system_prompt_path,
            base_system_prompt,
        ) = self._build_system_prompt(system_prompt_path)
        self.system_prompt = base_system_prompt
        repo_root = Path(__file__).resolve().parents[1]
        self.virtual_workspace = build_virtual_workspace(
            repo_root,
            model_file_path=Path(self.file_path),
            sdk_package=self.sdk_package,
        )
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

    def _reset_run_compile_state(self) -> None:
        self._current_edit_revision = 0
        self._last_successful_compile_revision = None
        self._last_successful_compile_report = None
        self._compile_attempt_count = 0
        self._last_compile_failure_sig = None
        self._consecutive_compile_failure_count = 0
        self._seen_exact_geometry_contract_sigs = set()
        self._seen_baseline_qc_guidance_sigs = set()

    def _latest_code_is_fresh(self) -> bool:
        return (
            self._last_successful_compile_report is not None
            and self._last_successful_compile_revision == self._current_edit_revision
        )

    def _mark_code_mutated(self, tool_name: str) -> None:
        if tool_name not in _MUTATING_TOOL_NAMES:
            return
        self._current_edit_revision += 1

    def _append_compile_required_reminder(self, conversation: list[dict]) -> None:
        msg = {
            "role": "user",
            "content": (
                "<compile_required>\n"
                "The latest code has changed since the last successful compile.\n"
                "Run `compile_model` before concluding.\n"
                "</compile_required>"
            ),
        }
        conversation.append(msg)
        trace_writer = getattr(self, "trace_writer", None)
        if trace_writer:
            trace_writer.write_message(msg)

    def _append_guidance_message(self, conversation: list[dict], content: str) -> None:
        msg = {"role": "user", "content": content}
        conversation.append(msg)
        trace_writer = getattr(self, "trace_writer", None)
        if trace_writer:
            trace_writer.write_message(msg)

    def _guidance_signature_set(self, attr_name: str) -> set[str]:
        current = getattr(self, attr_name, None)
        if isinstance(current, set):
            return current
        created: set[str] = set()
        setattr(self, attr_name, created)
        return created

    def _scan_current_code_contracts(self) -> _CodeContractScan | None:
        try:
            text = Path(self.file_path).read_text(encoding="utf-8")
        except OSError:
            return None
        return _scan_code_contracts(text)

    def _maybe_inject_exact_geometry_contract_guidance(
        self,
        conversation: list[dict],
        *,
        scan: _CodeContractScan,
    ) -> bool:
        missing_names = tuple(
            sorted(set(scan.referenced_exact_names).difference(scan.visual_names))
        )
        if not missing_names:
            return False

        seen = self._guidance_signature_set("_seen_exact_geometry_contract_sigs")
        sig = _sha256_text(json.dumps(missing_names))
        if sig in seen:
            return False
        seen.add(sig)

        joined = ", ".join(repr(name) for name in missing_names)
        self._append_guidance_message(
            conversation,
            "\n".join(
                [
                    "<exact_geometry_contract>",
                    (
                        "- Authored exact checks reference names that are not present in the "
                        f"current file: {joined}."
                    ),
                    "- These names became exact-geometry contracts when `ctx.expect_*` referenced them.",
                    (
                        "- Before more geometry edits, either restore those visual names or "
                        "update/remove the dependent exact checks in the same edit."
                    ),
                    "</exact_geometry_contract>",
                ]
            ),
        )
        return True

    def _maybe_inject_baseline_qc_guidance(
        self,
        conversation: list[dict],
        *,
        scan: _CodeContractScan,
    ) -> bool:
        if not scan.baseline_qc_calls:
            return False

        seen = self._guidance_signature_set("_seen_baseline_qc_guidance_sigs")
        sig = _sha256_text(json.dumps(scan.baseline_qc_calls))
        if sig in seen:
            return False
        seen.add(sig)

        joined = ", ".join(f"`{name}`" for name in scan.baseline_qc_calls)
        self._append_guidance_message(
            conversation,
            "\n".join(
                [
                    "<baseline_qc_guidance>",
                    (
                        "- `run_tests()` currently reintroduces compiler-owned baseline checks: "
                        f"{joined}."
                    ),
                    "- Leave baseline sanity/QC to `compile_model`.",
                    (
                        "- Keep `run_tests()` for prompt-specific exact checks, targeted pose "
                        "checks, and explicit allowances only."
                    ),
                    "</baseline_qc_guidance>",
                ]
            ),
        )
        return True

    def _turn_had_successful_mutation(
        self,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> bool:
        for tool_call, result in zip(tool_calls, tool_results, strict=False):
            if not result.is_success():
                continue
            if self._tool_call_name(tool_call) in _MUTATING_TOOL_NAMES:
                return True
        return False

    def _maybe_inject_code_contract_guidance(
        self,
        conversation: list[dict],
        *,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> None:
        if not self._turn_had_successful_mutation(tool_calls, tool_results):
            return
        scan = self._scan_current_code_contracts()
        if scan is None:
            return
        self._maybe_inject_exact_geometry_contract_guidance(conversation, scan=scan)
        self._maybe_inject_baseline_qc_guidance(conversation, scan=scan)

    def _render_compile_tool_output(self, bundle: CompileSignalBundle) -> str:
        failures = [signal for signal in bundle.signals if signal.severity == "failure"]
        if failures:
            sig = self._compile_signal_signature(bundle)
            repeated = sig == self._last_compile_failure_sig
            self._last_compile_failure_sig = sig
            self._consecutive_compile_failure_count += 1
            return render_compile_signals(
                bundle,
                repeated=repeated,
                failure_streak=self._consecutive_compile_failure_count,
            )

        self._last_compile_failure_sig = None
        self._consecutive_compile_failure_count = 0
        return render_compile_signals(bundle)

    def _render_reused_compile_tool_output(self, bundle: CompileSignalBundle) -> str:
        return (
            "Fresh compile already exists for the current code revision; `compile_model` was not re-run.\n"
            "Treat that compile result as authoritative unless you are about to edit code for one specific unresolved defect.\n\n"
            f"{self._render_compile_tool_output(bundle)}"
        )

    def _maybe_inject_compile_signals(
        self,
        conversation: list[dict],
        *,
        bundle: CompileSignalBundle,
    ) -> bool:
        warning_signals = [signal for signal in bundle.signals if signal.severity == "warning"]
        if not warning_signals:
            return False

        sticky_warning = any(signal.kind == "disconnected_geometry" for signal in warning_signals)
        sig = self._compile_signal_signature(bundle)
        if not sticky_warning and sig in self._seen_compile_signal_sigs:
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
            func_name = func.get("name")
            if func_name not in {"edit_code", "replace"}:
                continue
            if not getattr(result, "error", None):
                continue
            if "Could not find the old_string in the code" not in result.error:
                continue

            sig = f"{func_name}_old_string_not_found"
            if sig in self._seen_tool_error_sigs:
                return
            self._seen_tool_error_sigs.add(sig)

            msg = {
                "role": "user",
                "content": (
                    "<edit_retry_guidance>\n"
                    f"- Your last {func_name} failed because `old_string` did not match the file exactly.\n"
                    '- Do NOT guess. Call `read_file(path="model.py")` again, then pick a smaller exact snippet from the current editable code as `old_string` and retry.\n'
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

    async def _compile_urdf_report_async(self) -> CompileReport:
        async with local_work_slot(self.runtime_limits):
            return await asyncio.to_thread(
                compile_urdf_report_maybe_timeout,
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

    async def _execute_compile_model(self, *, tool_call_id: str) -> ToolResult:
        cached_report = self._last_successful_compile_report
        if self._latest_code_is_fresh() and cached_report is not None:
            return ToolResult(
                output=self._render_reused_compile_tool_output(cached_report.signal_bundle),
                compilation={"status": "success", "error": None},
                tool_call_id=tool_call_id,
            )

        self._compile_attempt_count += 1

        try:
            report = await self._compile_urdf_report_async()
            await self._persist_compile_success_checkpoint_async(report.urdf_xml)
            self._last_successful_compile_report = report
            self._last_successful_compile_revision = self._current_edit_revision
            return ToolResult(
                output=self._render_compile_tool_output(report.signal_bundle),
                compilation={"status": "success", "error": None},
                tool_call_id=tool_call_id,
            )
        except TimeoutError as exc:
            bundle = compile_signal_bundle_from_exception(exc)
            _log_compile_signals(bundle)
            return ToolResult(
                output=self._render_compile_tool_output(bundle),
                compilation={"status": "error", "error": bundle.summary},
                tool_call_id=tool_call_id,
            )
        except Exception as exc:
            await self._persist_compile_failure_checkpoint_async(exc)
            bundle = compile_signal_bundle_from_exception(exc)
            _log_compile_signals(bundle)
            return ToolResult(
                output=self._render_compile_tool_output(bundle),
                compilation={"status": "error", "error": bundle.summary},
                tool_call_id=tool_call_id,
            )

    def _build_system_prompt(self, prompt_path: str) -> tuple[Path, str]:
        return load_system_prompt_text(
            prompt_path,
            provider=self.provider,
            sdk_package=self.sdk_package,
        )

    def _persist_cost_tracking(self) -> None:
        if not self.cost_tracker:
            return
        try:
            cost_path = Path(self.file_path).parent / "cost.json"
            self.cost_tracker.save_json(cost_path)
            logger.info("Saved cost tracking to %s", cost_path)
        except Exception as exc:
            logger.warning("Failed to save cost tracking: %s", exc)

    def _current_total_cost_usd(self) -> float:
        if not self.cost_tracker:
            return 0.0
        return self.cost_tracker.all_in_total_breakdown().total_cost

    def _record_maintenance_event(self, event: dict[str, Any]) -> float:
        billed_cost = 0.0
        if self.cost_tracker:
            billed_cost = self.cost_tracker.add_maintenance_event(event).total_cost

        kind = event.get("kind") or event.get("type")
        if kind == "compaction":
            return billed_cost

        add_maintenance_event = getattr(self.display, "add_maintenance_event", None)
        if callable(add_maintenance_event):
            usage = event.get("usage")
            usage_total_tokens = (
                usage.get("total_tokens")
                if isinstance(usage, dict) and isinstance(usage.get("total_tokens"), int)
                else None
            )
            try:
                add_maintenance_event(
                    event,
                    billed_cost=billed_cost,
                    usage_total_tokens=usage_total_tokens,
                )
            except TypeError:
                add_maintenance_event(event)

        on_maintenance_event = getattr(self, "on_maintenance_event", None)
        if on_maintenance_event:
            try:
                on_maintenance_event(event, billed_cost)
            except Exception:
                logger.exception("on_maintenance_event callback failed")

        return billed_cost

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

    def _context_window_pressure(self, usage: dict[str, int]) -> dict[str, Any] | None:
        context_window_pressure = getattr(self.llm, "context_window_pressure", None)
        if not callable(context_window_pressure):
            return None
        try:
            pressure = context_window_pressure(usage)
        except Exception:
            logger.exception("Failed to compute context-window pressure")
            return None
        to_dict = getattr(pressure, "to_dict", None)
        if callable(to_dict):
            payload = to_dict()
            return payload if isinstance(payload, dict) else None
        return pressure if isinstance(pressure, dict) else None

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

    def _tool_calls_are_parallelizable(self, tool_calls: list[dict]) -> bool:
        if self.provider != "gemini" or len(tool_calls) <= 1:
            return False
        return all(
            self._tool_call_name(tool_call) in _PARALLEL_SAFE_TOOL_NAMES for tool_call in tool_calls
        )

    def _ensure_code_file(self) -> None:
        path = Path(self.file_path)
        if path.exists():
            existing = path.read_text(encoding="utf-8")
            if existing.strip():
                return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            _minimal_scaffold_text(sdk_package=self.sdk_package),
            encoding="utf-8",
        )

    def _seed_find_examples_cache_from_conversation(self, conversation: list[dict]) -> None:
        self._seen_find_example_paths = set()
        for message in conversation:
            if not isinstance(message, dict):
                continue
            if message.get("role") != "tool" or message.get("name") != "find_examples":
                continue
            content = message.get("content")
            if isinstance(content, str):
                try:
                    payload = json.loads(content)
                except json.JSONDecodeError:
                    continue
            elif isinstance(content, dict):
                payload = content
            else:
                continue
            result = payload.get("result") if isinstance(payload, dict) else None
            if not isinstance(result, list):
                continue
            for item in result:
                cache_key = self._find_examples_cache_key(item)
                if cache_key is not None:
                    self._seen_find_example_paths.add(cache_key)

    def _compress_find_examples_output(self, output: Any) -> Any:
        if not isinstance(output, list):
            return output

        compressed: list[Any] = []
        for item in output:
            cache_key = self._find_examples_cache_key(item)
            if cache_key is None:
                compressed.append(item)
                continue

            entry = dict(item)
            if cache_key in self._seen_find_example_paths:
                entry["content"] = _FIND_EXAMPLES_SKIPPED_CONTENT
                entry["content_skipped"] = True
            else:
                self._seen_find_example_paths.add(cache_key)
            compressed.append(entry)
        return compressed

    def _find_examples_cache_key(self, item: Any) -> str | None:
        if not isinstance(item, dict):
            return None
        example_id = item.get("example_id")
        if isinstance(example_id, str) and example_id:
            return example_id
        path = item.get("path")
        if isinstance(path, str) and path:
            return path
        return None

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

        if func_name == "compile_model":
            unexpected = sorted(func_args.keys())
            if unexpected:
                result = ToolResult(
                    error=(
                        f"Invalid parameters for {func_name}. Unexpected parameters: {unexpected}"
                    ),
                    tool_call_id=tool_id,
                )
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
            result = await self._execute_compile_model(tool_call_id=tool_id)
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

        if func_name in {"edit_code", "replace"}:
            old_string_key = "old_string"
            try:
                editable = extract_editable_code(Path(self.file_path).read_text(encoding="utf-8"))
            except Exception:
                editable = None
            if (
                func_args.get(old_string_key) == ""
                and editable is not None
                and editable.strip() != ""
            ):
                result = ToolResult(
                    error=(
                        "old_string cannot be empty unless the editable code section is empty. "
                        'Call `read_file(path="model.py")` to copy exact current editable text and retry.'
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
                and func_args.get(old_string_key) != ""
            ):
                result = ToolResult(
                    error=(
                        "Editable code section is empty. Initialize it with "
                        "`write_file(content=...)` or with "
                        f"{func_name} using "
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
                bind_file_path = getattr(invocation, "bind_file_path", None)
                if callable(bind_file_path):
                    bind_file_path(self.file_path)
                bind_virtual_workspace = getattr(invocation, "bind_virtual_workspace", None)
                if callable(bind_virtual_workspace):
                    bind_virtual_workspace(self.virtual_workspace)
                result = await invocation.execute()
                result.tool_call_id = tool_id
                if result.is_success() and func_name == "find_examples":
                    result.output = self._compress_find_examples_output(result.output)
                if result.is_success():
                    self._mark_code_mutated(func_name)
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

    async def _execute_tool_calls_batch(
        self,
        tool_calls: list[dict],
    ) -> list[tuple[dict, ToolResult, dict, float]]:
        if self._tool_calls_are_parallelizable(tool_calls):
            tool_names = [self._tool_call_name(tool_call) for tool_call in tool_calls]
            logger.info(
                "Executing %s Gemini tool calls in parallel: %s",
                len(tool_calls),
                ", ".join(tool_names),
            )

            async def _run_single(tool_call: dict) -> tuple[ToolResult, dict, float]:
                func_name = self._tool_call_name(tool_call)
                tool_start = time.monotonic()
                result, tool_message = await self._execute_tool(tool_call)
                tool_duration = time.monotonic() - tool_start
                logger.info("Tool %s finished in %.2fs", func_name, tool_duration)
                return result, tool_message, tool_duration

            batched_results = await asyncio.gather(
                *(_run_single(tool_call) for tool_call in tool_calls)
            )
            return [
                (tool_call, result, tool_message, tool_duration)
                for tool_call, (result, tool_message, tool_duration) in zip(
                    tool_calls, batched_results, strict=False
                )
            ]

        executed: list[tuple[dict, ToolResult, dict, float]] = []
        for tool_call in tool_calls:
            func_name = self._tool_call_name(tool_call)
            logger.info("Executing tool: %s", func_name)
            tool_start = time.monotonic()
            result, tool_message = await self._execute_tool(tool_call)
            tool_duration = time.monotonic() - tool_start
            logger.info("Tool %s finished in %.2fs", func_name, tool_duration)
            executed.append((tool_call, result, tool_message, tool_duration))
        return executed

    def _is_code_valid(self, tool_results: list[ToolResult]) -> bool:
        for result in reversed(tool_results):
            if result.compilation:
                return result.compilation.get("status") == "success"
        return False

    def _compile_warnings_snapshot(self) -> list[str]:
        report = self._last_successful_compile_report
        return list(report.warnings) if report else []

    async def _read_final_code(self) -> str | None:
        try:
            import aiofiles

            async with aiofiles.open(self.file_path, "r") as file:
                return await file.read()
        except Exception:
            return None

    def _termination_message(self, text_response: str, tool_calls: list[dict]) -> str | None:
        if text_response.strip() and not tool_calls:
            return "Agent completed with response"
        return None

    async def _build_code_valid_result(
        self,
        *,
        message: str,
        conversation: list[dict],
        turn_count: int,
        tool_call_count: int,
        usage: dict[str, int],
    ) -> AgentResult:
        final_code = await self._read_final_code()
        self._persist_cost_tracking()
        report = self._last_successful_compile_report
        return AgentResult(
            success=True,
            reason=TerminateReason.CODE_VALID,
            message=message,
            conversation=conversation,
            final_code=final_code,
            urdf_xml=report.urdf_xml if report else None,
            compile_warnings=self._compile_warnings_snapshot(),
            turn_count=turn_count,
            tool_call_count=tool_call_count,
            compile_attempt_count=self._compile_attempt_count,
            usage=usage or None,
        )

    async def _handle_finish_attempt(
        self,
        conversation: list[dict],
        *,
        message: str,
        turn_count: int,
        tool_call_count: int,
        usage: dict[str, int],
        display_turn_override: int | None = None,
    ) -> AgentResult | None:
        if not self._latest_code_is_fresh():
            self._append_compile_required_reminder(conversation)
        else:
            if display_turn_override is not None:
                self.display.current_turn = display_turn_override
            self.display.end_turn(success=True)
            return await self._build_code_valid_result(
                message=message,
                conversation=conversation,
                turn_count=turn_count,
                tool_call_count=tool_call_count,
                usage=usage,
            )

        if display_turn_override is not None:
            self.display.current_turn = display_turn_override
        self.display.end_turn(success=True)
        return None

    async def run(
        self,
        user_message: Any,
        *,
        initial_conversation: Optional[list[dict]] = None,
    ) -> AgentResult:
        self._ensure_code_file()
        self._reset_run_compile_state()
        self.display.start()
        conversation = initial_conversation or []
        self._seed_find_examples_cache_from_conversation(conversation)
        if not conversation:
            conversation.extend(
                _build_first_turn_messages(
                    user_message,
                    sdk_docs_context=self.sdk_docs_context,
                    provider=self.provider,
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
                prepare_next_request = getattr(self.llm, "prepare_next_request", None)
                if callable(prepare_next_request):
                    prepare_result = await prepare_next_request(
                        system_prompt=self.system_prompt,
                        messages=conversation,
                        tools=self.tool_registry.get_tool_schemas(),
                        completed_turns=completed_turns,
                        consecutive_compile_failure_count=self._consecutive_compile_failure_count,
                        last_compile_failure_sig=self._last_compile_failure_sig,
                    )
                    for trace_event in getattr(prepare_result, "trace_events", []):
                        if self.trace_writer:
                            event_type = getattr(trace_event, "event_type", None)
                            payload = getattr(trace_event, "payload", None)
                            if isinstance(event_type, str) and isinstance(payload, dict):
                                self.trace_writer.write_event(event_type, payload)
                    for maintenance_event in getattr(prepare_result, "maintenance_events", []):
                        if isinstance(maintenance_event, dict):
                            self._record_maintenance_event(maintenance_event)
                    compaction_event = getattr(prepare_result, "compaction_event", None)
                    if compaction_event is not None:
                        compaction_payload = compaction_event.to_dict()
                        maintenance_cost_usd = self._record_maintenance_event(compaction_payload)
                        usage = compaction_payload.get("usage")
                        usage_total_tokens = (
                            usage.get("total_tokens")
                            if isinstance(usage, dict)
                            and isinstance(usage.get("total_tokens"), int)
                            else None
                        )
                        add_compaction_event = getattr(self.display, "add_compaction_event", None)
                        if callable(add_compaction_event):
                            add_compaction_event(
                                trigger=compaction_event.trigger,
                                estimated_saved_next_input_tokens=(
                                    compaction_event.estimated_saved_next_input_tokens
                                ),
                                billed_cost=maintenance_cost_usd,
                                previous_response_id_cleared=(
                                    compaction_event.previous_response_id_cleared
                                ),
                                usage_total_tokens=usage_total_tokens,
                                estimate_error=compaction_event.estimate_error,
                            )
                        on_compaction_event = getattr(self, "on_compaction_event", None)
                        if on_compaction_event:
                            try:
                                on_compaction_event(
                                    compaction_payload,
                                    maintenance_cost_usd,
                                )
                            except Exception:
                                logger.exception("on_compaction_event callback failed")
                    if (
                        self.max_cost_usd is not None
                        and self._current_total_cost_usd() > self.max_cost_usd
                    ):
                        total_cost = self._current_total_cost_usd()
                        self._persist_cost_tracking()
                        self.display.end_turn(success=False)
                        return AgentResult(
                            success=False,
                            reason=TerminateReason.COST_LIMIT,
                            message=(
                                f"Cost limit exceeded before turn {turn}: "
                                f"cumulative ${total_cost:.6f} exceeded limit ${self.max_cost_usd:.6f}"
                            ),
                            conversation=conversation,
                            compile_warnings=self._compile_warnings_snapshot(),
                            turn_count=completed_turns,
                            tool_call_count=tool_call_count,
                            compile_attempt_count=self._compile_attempt_count,
                            usage=usage_totals or None,
                        )
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
                    compile_warnings=self._compile_warnings_snapshot(),
                    turn_count=completed_turns,
                    tool_call_count=tool_call_count,
                    compile_attempt_count=self._compile_attempt_count,
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

            if usage:
                llm_duration = time.monotonic() - llm_start
                turn_cost_usd = 0.0
                if self.cost_tracker:
                    turn_cost = self.cost_tracker.add_turn(usage)
                    turn_cost_usd = turn_cost.total_cost
                context_pressure = self._context_window_pressure(usage)
                try:
                    self.display.add_llm_call(
                        usage,
                        turn_cost_usd,
                        llm_duration,
                        context_pressure=context_pressure,
                    )
                except TypeError:
                    self.display.add_llm_call(usage, turn_cost_usd, llm_duration)
                if (
                    self.cost_tracker is not None
                    and self.max_cost_usd is not None
                    and self._current_total_cost_usd() > self.max_cost_usd
                ):
                    total_cost = self._current_total_cost_usd()
                    self._persist_cost_tracking()
                    self.display.end_turn(success=False)
                    return AgentResult(
                        success=False,
                        reason=TerminateReason.COST_LIMIT,
                        message=(
                            f"Cost limit exceeded after turn {completed_turns + 1}: "
                            f"cumulative ${total_cost:.6f} exceeded limit ${self.max_cost_usd:.6f}"
                        ),
                        conversation=conversation,
                        compile_warnings=self._compile_warnings_snapshot(),
                        turn_count=completed_turns + 1,
                        tool_call_count=tool_call_count,
                        compile_attempt_count=self._compile_attempt_count,
                        usage=usage_totals or None,
                    )

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
                finish_result = await self._handle_finish_attempt(
                    conversation,
                    message="Compile succeeded and model returned no further actions",
                    turn_count=completed_turns,
                    tool_call_count=tool_call_count,
                    usage=usage_totals,
                    display_turn_override=completed_turns,
                )
                if finish_result is not None:
                    logger.info("Empty response after fresh compile; terminating run.")
                    return finish_result
                continue

            completed_turns += 1

            termination_message = self._termination_message(text, tool_calls)
            if termination_message is not None:
                finish_result = await self._handle_finish_attempt(
                    conversation,
                    message=termination_message,
                    turn_count=completed_turns,
                    tool_call_count=tool_call_count,
                    usage=usage_totals,
                )
                if finish_result is not None:
                    return finish_result
                continue

            turn_tool_results: list[ToolResult] = []
            tool_call_count += len(tool_calls)
            for (
                tool_call,
                result,
                tool_message,
                tool_duration,
            ) in await self._execute_tool_calls_batch(tool_calls):
                func_name = self._tool_call_name(tool_call)
                func_args = self._tool_call_display_args(tool_call)
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
            self._maybe_inject_code_contract_guidance(
                conversation,
                tool_calls=tool_calls,
                tool_results=turn_tool_results,
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

        self._persist_cost_tracking()

        max_turn_message = "Agent hit max turns limit"
        if not self._latest_code_is_fresh():
            max_turn_message = (
                "Agent hit max turns limit before `compile_model` succeeded on the latest revision"
            )

        return AgentResult(
            success=False,
            reason=TerminateReason.MAX_TURNS,
            message=max_turn_message,
            conversation=conversation,
            final_code=final_code,
            compile_warnings=self._compile_warnings_snapshot(),
            turn_count=completed_turns,
            tool_call_count=tool_call_count,
            compile_attempt_count=self._compile_attempt_count,
            usage=usage_totals or None,
        )

    async def close(self) -> None:
        close_events = await self.llm.close()
        if isinstance(close_events, list):
            for event in close_events:
                if not isinstance(event, dict):
                    continue
                if self.trace_writer:
                    event_type = str(event.get("type") or event.get("kind") or "maintenance")
                    self.trace_writer.write_event(event_type, event)
                self._record_maintenance_event(event)
        self.display.stop()
        if self.trace_writer:
            self.trace_writer.close()

    async def __aenter__(self) -> ArticraftAgent:
        return self

    async def __aexit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        await self.close()
