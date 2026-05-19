from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any, Literal


class TerminateReason(StrEnum):
    """Reasons the agent runtime can terminate."""

    GOAL_COMPLETE = "GOAL_COMPLETE"
    CODE_VALID = "CODE_VALID"
    MAX_TURNS = "MAX_TURNS"
    COST_LIMIT = "COST_LIMIT"
    ERROR = "ERROR"


@dataclass(slots=True)
class AgentResult:
    """Result from a single agent run."""

    success: bool
    reason: TerminateReason
    message: str
    conversation: list[dict[str, Any]]
    final_code: str | None = None
    urdf_xml: str | None = None
    compile_warnings: list[str] = field(default_factory=list)
    turn_count: int = 0
    tool_call_count: int = 0
    compile_attempt_count: int = 0
    usage: dict[str, int] | None = None


@dataclass(slots=True, frozen=True)
class CompileSignal:
    severity: Literal["failure", "warning", "note"]
    kind: str
    code: str
    summary: str
    details: str = ""
    blocking: bool = False
    source: Literal["compiler", "tests", "harness"] = "compiler"
    group: Literal["build", "qc", "design", "hygiene"] = "qc"
    check_name: str | None = None
    dedupe_key: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "severity": self.severity,
            "kind": self.kind,
            "code": self.code,
            "summary": self.summary,
            "details": self.details,
            "blocking": self.blocking,
            "source": self.source,
            "group": self.group,
            "check_name": self.check_name,
            "dedupe_key": self.dedupe_key,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CompileSignal":
        return cls(
            severity=str(payload.get("severity", "warning")),  # type: ignore[arg-type]
            kind=str(payload.get("kind", "unknown")),
            code=str(payload.get("code", "UNKNOWN")),
            summary=str(payload.get("summary", "")),
            details=str(payload.get("details", "")),
            blocking=bool(payload.get("blocking", False)),
            source=str(payload.get("source", "compiler")),  # type: ignore[arg-type]
            group=str(payload.get("group", "qc")),  # type: ignore[arg-type]
            check_name=(
                None if payload.get("check_name") is None else str(payload.get("check_name"))
            ),
            dedupe_key=(
                None if payload.get("dedupe_key") is None else str(payload.get("dedupe_key"))
            ),
        )


@dataclass(slots=True, frozen=True)
class CompileSignalBundle:
    status: Literal["success", "failure"]
    summary: str
    signals: tuple[CompileSignal, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "summary": self.summary,
            "signals": [signal.to_dict() for signal in self.signals],
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CompileSignalBundle":
        raw_signals = payload.get("signals")
        signals: list[CompileSignal] = []
        if isinstance(raw_signals, list):
            for item in raw_signals:
                if isinstance(item, dict):
                    signals.append(CompileSignal.from_dict(item))
        return cls(
            status=str(payload.get("status", "success")),  # type: ignore[arg-type]
            summary=str(payload.get("summary", "")),
            signals=tuple(signals),
        )


@dataclass(slots=True, frozen=True)
class CompileReport:
    """Compile result plus non-blocking warnings."""

    urdf_xml: str
    warnings: list[str]
    signal_bundle: CompileSignalBundle


@dataclass(slots=True, frozen=True)
class PromptPreviewRequest:
    """Request used to build a provider payload preview."""

    user_content: Any
    provider: str
    model_id: str
    thinking_level: str
    system_prompt_path: str
    sdk_package: str = "sdk"
    openai_transport: str = "http"
    openai_reasoning_summary: str | None = "auto"


@dataclass(slots=True, frozen=True)
class SessionPaths:
    """Filesystem paths bound to a single run."""

    script_path: Path
    trace_dir: Path
    checkpoint_urdf_path: Path | None = None
