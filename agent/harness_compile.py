from __future__ import annotations

import asyncio
import hashlib
import json
import logging
from pathlib import Path
from typing import Optional

from agent.compiler import (
    compile_urdf_report_maybe_timeout,
    persist_compile_success_artifacts,
)
from agent.feedback import (
    compile_signal_bundle_from_exception,
    render_compile_signals,
)
from agent.harness_guidance import MUTATING_TOOL_NAMES
from agent.models import CompileReport, CompileSignalBundle
from agent.runtime_limits import BatchRuntimeLimits, local_work_slot
from agent.tools.base import ToolResult

logger = logging.getLogger(__name__)


def log_compile_signals(bundle: CompileSignalBundle) -> None:
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


class CompileFeedbackLoop:
    def __init__(
        self,
        *,
        file_path: str,
        sdk_package: str,
        runtime_limits: BatchRuntimeLimits | None,
        checkpoint_urdf_path: Path | None,
    ) -> None:
        self.file_path = file_path
        self.sdk_package = sdk_package
        self.runtime_limits = runtime_limits
        self.checkpoint_urdf_path = checkpoint_urdf_path
        self._last_checkpoint_urdf_sig: Optional[str] = None
        self._current_edit_revision = 0
        self._last_successful_compile_revision: int | None = None
        self._last_successful_compile_report: CompileReport | None = None
        self._compile_attempt_count = 0
        self._last_compile_failure_sig: Optional[str] = None
        self._consecutive_compile_failure_count = 0

    @property
    def compile_attempt_count(self) -> int:
        return self._compile_attempt_count

    @property
    def consecutive_compile_failure_count(self) -> int:
        return self._consecutive_compile_failure_count

    @property
    def last_compile_failure_sig(self) -> str | None:
        return self._last_compile_failure_sig

    @property
    def last_successful_report(self) -> CompileReport | None:
        return self._last_successful_compile_report

    def reset(self) -> None:
        self._current_edit_revision = 0
        self._last_successful_compile_revision = None
        self._last_successful_compile_report = None
        self._compile_attempt_count = 0
        self._last_compile_failure_sig = None
        self._consecutive_compile_failure_count = 0

    def latest_code_is_fresh(self) -> bool:
        return (
            self._last_successful_compile_report is not None
            and self._last_successful_compile_revision == self._current_edit_revision
        )

    def mark_code_mutated(self, tool_name: str) -> None:
        if tool_name not in MUTATING_TOOL_NAMES:
            return
        self._current_edit_revision += 1

    def append_compile_required_reminder(
        self,
        conversation: list[dict],
        *,
        trace_writer: object | None,
    ) -> None:
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
        if trace_writer:
            trace_writer.write_message(msg)

    def compile_warnings_snapshot(self) -> list[str]:
        report = self._last_successful_compile_report
        return list(report.warnings) if report else []

    def _compile_signal_signature(self, bundle: CompileSignalBundle) -> str:
        sig_src = json.dumps(bundle.to_dict(), sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
        return hashlib.sha1(sig_src).hexdigest()

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

    async def execute_compile_model(self, *, tool_call_id: str) -> ToolResult:
        cached_report = self._last_successful_compile_report
        if self.latest_code_is_fresh() and cached_report is not None:
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
            log_compile_signals(bundle)
            return ToolResult(
                output=self._render_compile_tool_output(bundle),
                compilation={"status": "error", "error": bundle.summary},
                tool_call_id=tool_call_id,
            )
        except Exception as exc:
            await self._persist_compile_failure_checkpoint_async(exc)
            bundle = compile_signal_bundle_from_exception(exc)
            log_compile_signals(bundle)
            return ToolResult(
                output=self._render_compile_tool_output(bundle),
                compilation={"status": "error", "error": bundle.summary},
                tool_call_id=tool_call_id,
            )
