from __future__ import annotations

import hashlib
import linecache
import re
import traceback
from dataclasses import dataclass
from typing import Iterable, Literal, Protocol, TypeAlias, runtime_checkable

from agent.models import CompileSignal, CompileSignalBundle

CODE_PATTERN = re.compile(
    r"```(?:python)?\s*\n"
    r"|^(?:from\s+\w+\s+import\s|import\s+\w+|def\s+\w+\s*\(|class\s+\w+)",
    re.MULTILINE,
)
LOFT_PROFILE_AREA_ERROR = "Loft profile area must be non-zero"
LOFT_PROFILE_AREA_HINT = (
    "Hint: LoftGeometry checks profile area in the XY projection. "
    "Use closed loops like `(x_i, y_i, z_const)`. "
    "If you need an XZ/YZ section, author it in XY first and rotate the mesh afterward."
)
MAX_RISK_PATTERN = re.compile(r"max_risk=(low|medium|high)", re.IGNORECASE)

SignalSeverity: TypeAlias = Literal["failure", "warning", "note"]
SignalSource: TypeAlias = Literal["compiler", "tests", "harness"]
SignalGroup: TypeAlias = Literal["build", "qc", "design", "hygiene"]
CompileStatus: TypeAlias = Literal["success", "failure"]


class TestFailureLike(Protocol):
    name: str
    details: str


@runtime_checkable
class TestReportLike(Protocol):
    failures: tuple[TestFailureLike, ...]
    warnings: tuple[str, ...]
    allowances: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ParsedTextBlock:
    text: str
    headline: str
    details: str
    lower: str


@dataclass(frozen=True, slots=True)
class ParsedTestWarning:
    text: str
    headline: str
    details: str
    lower: str
    check_name: str
    detail_text: str
    max_risk: str | None


@dataclass(frozen=True, slots=True)
class SignalSpec:
    severity: SignalSeverity
    kind: str
    code: str
    source: SignalSource = "compiler"
    group: SignalGroup = "qc"
    blocking: bool = False


@dataclass(frozen=True, slots=True)
class CompilerWarningRule:
    needle: str
    spec: SignalSpec
    summary: str


COMPILER_WARNING_RULES: tuple[CompilerWarningRule, ...] = (
    CompilerWarningRule(
        needle="non-finite or absurd geometry dimensions detected",
        spec=SignalSpec(
            severity="warning",
            kind="geometry_scale",
            code="WARN_GEOMETRY_ABSURD",
            group="design",
        ),
        summary="Non-finite or absurd geometry dimensions detected.",
    ),
    CompilerWarningRule(
        needle="geometry outlier dimensions detected",
        spec=SignalSpec(
            severity="warning",
            kind="geometry_scale",
            code="WARN_GEOMETRY_OUTLIER",
            group="design",
        ),
        summary="Geometry outlier dimensions detected.",
    ),
    CompilerWarningRule(
        needle="visual connectivity check failed",
        spec=SignalSpec(
            severity="warning",
            kind="visual_connectivity",
            code="WARN_VISUAL_CONNECTIVITY",
            group="design",
        ),
        summary="Visual connectivity check failed.",
    ),
    CompilerWarningRule(
        needle="geometry overlap check reported overlaps",
        spec=SignalSpec(
            severity="warning",
            kind="geometry_overlap",
            code="WARN_GEOMETRY_OVERLAP",
        ),
        summary="Geometry overlap check reported overlaps.",
    ),
    CompilerWarningRule(
        needle="cwd-relative asset paths detected",
        spec=SignalSpec(
            severity="warning",
            kind="path_hygiene",
            code="WARN_CWD_RELATIVE_ASSET_PATH",
            group="hygiene",
        ),
        summary="cwd-relative asset paths detected.",
    ),
)

COMPILER_WARNING_FALLBACK_SPEC = SignalSpec(
    severity="warning",
    kind="compiler_warning",
    code="WARN_COMPILER",
)
ISOLATED_PART_WARNING_SPEC = SignalSpec(
    severity="warning",
    kind="isolated_part",
    code="WARN_ISOLATED_PART",
    group="design",
)
ALLOWED_ISOLATED_PART_SPEC = SignalSpec(
    severity="note",
    kind="allowed_isolated_part",
    code="NOTE_ALLOWED_ISOLATED_PART",
    group="design",
)
TEST_FAILURE_SPEC = SignalSpec(
    severity="failure",
    kind="test_failure",
    code="TEST_FAILURE",
    source="tests",
    group="qc",
    blocking=True,
)
COPLANAR_HINT_SPEC = SignalSpec(
    severity="note",
    kind="coplanar_surface_hint",
    code="NOTE_COPLANAR_SURFACE",
    source="tests",
    group="qc",
)
COPLANAR_WARNING_SPEC = SignalSpec(
    severity="warning",
    kind="coplanar_surface",
    code="WARN_COPLANAR_SURFACE",
    source="tests",
    group="qc",
)
ALLOWED_OVERLAP_SPEC = SignalSpec(
    severity="note",
    kind="allowed_overlap",
    code="NOTE_ALLOWED_OVERLAP",
    source="tests",
    group="qc",
)
OVERLAP_WARNING_SPEC = SignalSpec(
    severity="warning",
    kind="overlap_warning",
    code="WARN_OVERLAP_SENSOR",
    source="tests",
    group="qc",
)
DISCONNECTED_GEOMETRY_SPEC = SignalSpec(
    severity="failure",
    kind="disconnected_geometry",
    code="TEST_DISCONNECTED_GEOMETRY",
    source="tests",
    group="qc",
    blocking=True,
)
ARTICULATION_ORIGIN_SPEC = SignalSpec(
    severity="warning",
    kind="articulation_origin",
    code="WARN_ARTICULATION_ORIGIN",
    source="tests",
    group="qc",
)
TEST_WARNING_FALLBACK_SPEC = SignalSpec(
    severity="warning",
    kind="test_warning",
    code="TEST_WARNING",
    source="tests",
    group="qc",
)
DEPRECATED_TEST_API_SPEC = SignalSpec(
    severity="warning",
    kind="deprecated_test_api",
    code="WARN_DEPRECATED_TEST_API",
    source="tests",
    group="hygiene",
)
ALLOWANCE_SPEC = SignalSpec(
    severity="note",
    kind="allowance",
    code="ALLOWANCE",
    source="tests",
    group="qc",
)
COMPILE_RUNTIME_SPEC = SignalSpec(
    severity="failure",
    kind="compile_runtime",
    code="COMPILE_RUNTIME_FAILURE",
    source="compiler",
    group="build",
    blocking=True,
)


def contains_code_in_text(text: str) -> bool:
    """Check if a model text response pasted code instead of using tools."""
    if not text:
        return False
    return bool(CODE_PATTERN.search(text))


def _compile_hint_lines(detail_lines: list[str]) -> list[str]:
    if any(LOFT_PROFILE_AREA_ERROR in line for line in detail_lines):
        return [LOFT_PROFILE_AREA_HINT]
    return []


def _exception_detail_lines(exc: BaseException, *, max_detail_lines: int = 40) -> list[str]:
    raw_message = str(exc).strip()
    if raw_message:
        tb_marker = "Traceback (most recent call last):"
        marker_idx = raw_message.find(tb_marker)
        if marker_idx >= 0:
            raw_message = raw_message[:marker_idx].rstrip()

    detail_lines: list[str] = []
    seen: set[str] = set()
    for line in raw_message.splitlines():
        cleaned = line.rstrip()
        if not cleaned:
            continue
        if cleaned in seen:
            continue
        seen.add(cleaned)
        detail_lines.append(cleaned)

    warnings = getattr(exc, "warnings", None)
    if isinstance(warnings, list):
        for warning in warnings:
            cleaned = str(warning).strip()
            if not cleaned or cleaned in seen:
                continue
            seen.add(cleaned)
            detail_lines.append(cleaned)

    if len(detail_lines) > max_detail_lines:
        omitted = len(detail_lines) - max_detail_lines
        detail_lines = detail_lines[:max_detail_lines]
        detail_lines.append(f"... ({omitted} more lines)")

    for hint in _compile_hint_lines(detail_lines):
        if hint not in seen:
            detail_lines.append(hint)
            seen.add(hint)
    return detail_lines


def _exception_location_lines(exc: BaseException) -> list[str]:
    extracted = traceback.extract_tb(exc.__traceback__) if exc.__traceback__ else []
    skip_wrapper_location = isinstance(exc, RuntimeError)
    if not extracted or skip_wrapper_location:
        return []

    last = extracted[-1]
    location = f"{last.filename}:{last.lineno}"
    lines = [f"Location: {location}"]
    raw_line = linecache.getline(last.filename, last.lineno).rstrip("\n")
    if raw_line.strip():
        lines.append(f"Code: {raw_line}")
    return lines


def _parse_text_block(text: str) -> ParsedTextBlock:
    lines = text.splitlines()
    headline = lines[0] if lines else ""
    details = "\n".join(lines[1:]).strip()
    return ParsedTextBlock(text=text, headline=headline, details=details, lower=text.lower())


def _parse_test_warning(text: str) -> ParsedTestWarning:
    parsed = _parse_text_block(text)
    check_name, has_separator, detail_text = parsed.text.partition(":")
    max_risk_match = MAX_RISK_PATTERN.search(parsed.text)
    return ParsedTestWarning(
        text=parsed.text,
        headline=parsed.headline,
        details=parsed.details,
        lower=parsed.lower,
        check_name=check_name.strip(),
        detail_text=detail_text.lstrip() if has_separator else "",
        max_risk=(max_risk_match.group(1).lower() if max_risk_match is not None else None),
    )


def _make_signal(
    *,
    severity: SignalSeverity,
    kind: str,
    code: str,
    summary: str,
    details: str = "",
    blocking: bool = False,
    source: SignalSource = "compiler",
    group: SignalGroup = "qc",
    check_name: str | None = None,
) -> CompileSignal:
    sig_src = "\n".join(
        [
            severity,
            kind,
            code,
            summary,
            details,
            str(blocking),
            source,
            group,
            check_name or "",
        ]
    ).encode("utf-8")
    return CompileSignal(
        severity=severity,
        kind=kind,
        code=code,
        summary=summary,
        details=details,
        blocking=blocking,
        source=source,
        group=group,
        check_name=check_name,
        dedupe_key=hashlib.sha1(sig_src).hexdigest(),
    )


def _signal_from_spec(
    spec: SignalSpec,
    *,
    summary: str,
    details: str = "",
    check_name: str | None = None,
) -> CompileSignal:
    return _make_signal(
        severity=spec.severity,
        kind=spec.kind,
        code=spec.code,
        summary=summary,
        details=details,
        blocking=spec.blocking,
        source=spec.source,
        group=spec.group,
        check_name=check_name,
    )


def _isolated_part_summary(headline: str) -> str:
    summary = "Isolated parts detected."
    if "(visual" in headline or "(visual," in headline:
        return "Isolated visual parts detected."
    if "(collision" in headline or "(collision," in headline:
        return "Isolated collision parts detected."
    return summary


def _warning_signal_from_text(warning: str) -> CompileSignal:
    text = str(warning).strip()
    if not text:
        return _signal_from_spec(
            COMPILER_WARNING_FALLBACK_SPEC,
            summary="Compiler emitted a warning.",
        )

    parsed = _parse_text_block(text)
    if "isolated parts allowed by justification" in parsed.text:
        return _signal_from_spec(
            ALLOWED_ISOLATED_PART_SPEC,
            summary="Isolated-part findings were allowed by justification.",
            details=parsed.details,
        )
    if "isolated parts detected" in parsed.text:
        return _signal_from_spec(
            ISOLATED_PART_WARNING_SPEC,
            summary=_isolated_part_summary(parsed.headline),
            details=parsed.details,
        )

    for rule in COMPILER_WARNING_RULES:
        if rule.needle in parsed.text:
            return _signal_from_spec(
                rule.spec,
                summary=rule.summary,
                details=parsed.details,
            )

    return _signal_from_spec(
        COMPILER_WARNING_FALLBACK_SPEC,
        summary=parsed.headline.replace("IMPORTANT: ", "").strip(),
        details=parsed.details,
    )


def _as_test_report(test_report: object | None) -> TestReportLike | None:
    if test_report is None or not isinstance(test_report, TestReportLike):
        return None
    if not isinstance(test_report.failures, tuple):
        return None
    if not isinstance(test_report.warnings, tuple):
        return None
    if not isinstance(test_report.allowances, tuple):
        return None
    return test_report


def _iter_test_failures(test_report: TestReportLike | None) -> Iterable[CompileSignal]:
    if test_report is None:
        return ()

    signals: list[CompileSignal] = []
    for failure in test_report.failures:
        signals.append(
            _signal_from_spec(
                TEST_FAILURE_SPEC,
                summary=str(failure.name),
                details=str(failure.details).strip(),
                check_name=str(failure.name),
            )
        )
    return signals


def _coplanar_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if "coplanar" not in parsed.lower:
        return None
    max_risk = parsed.max_risk or "medium"
    if max_risk == "low":
        return _signal_from_spec(
            COPLANAR_HINT_SPEC,
            summary="Low-confidence coplanar-surface hint.",
            details=parsed.text,
        )
    return _signal_from_spec(
        COPLANAR_WARNING_SPEC,
        summary=f"Coplanar-surface heuristic reported {max_risk}-risk pair(s).",
        details=parsed.text,
    )


def _allowed_overlap_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.lower.startswith("overlaps detected but allowed by justification"):
        return None
    return _signal_from_spec(
        ALLOWED_OVERLAP_SPEC,
        summary="Overlap findings were allowed by justification.",
        details=parsed.text,
    )


def _overlap_warning_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.check_name.startswith(("warn_if_overlaps(", "warn_if_articulation_overlaps(")):
        return None
    if parsed.check_name.startswith("warn_if_articulation_overlaps("):
        summary = "Articulation overlap sensor reported overlap pair(s)."
    else:
        summary = "Broad overlap sensor reported overlap pair(s)."
    if parsed.detail_text and "overlaps detected" not in parsed.detail_text.lower():
        if parsed.check_name.startswith("warn_if_articulation_overlaps("):
            summary = "Articulation overlap sensor reported a warning."
        else:
            summary = "Broad overlap sensor reported a warning."
    return _signal_from_spec(
        OVERLAP_WARNING_SPEC,
        summary=summary,
        details=parsed.text,
        check_name=parsed.check_name,
    )


def _disconnected_geometry_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.check_name.startswith(
        (
            "warn_if_part_geometry_connected(",
            "warn_if_part_geometry_disconnected(",
        )
    ):
        return None
    summary = "Exact visual connectivity check found disconnected geometry within a part."
    if (
        parsed.detail_text
        and "disconnected geometry islands detected" not in parsed.detail_text.lower()
    ):
        summary = "Part-geometry connectivity check reported a warning."
    return _signal_from_spec(
        DISCONNECTED_GEOMETRY_SPEC,
        summary=summary,
        details=parsed.text,
        check_name=parsed.check_name,
    )


def _articulation_origin_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.check_name.startswith(
        (
            "warn_if_articulation_origin_near_geometry(",
            "warn_if_joint_origin_near_geometry(",
        )
    ):
        return None
    summary = "Exact articulation-origin distance check reported distant joint origins."
    if parsed.detail_text and "far from geometry" not in parsed.detail_text.lower():
        summary = "Articulation-origin distance check reported a warning."
    return _signal_from_spec(
        ARTICULATION_ORIGIN_SPEC,
        summary=summary,
        details=parsed.text,
        check_name=parsed.check_name,
    )


def _deprecated_test_api_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if parsed.lower.startswith("deprecated as default:"):
        return _signal_from_spec(
            DEPRECATED_TEST_API_SPEC,
            summary="Deprecated default scaffold heuristic used; switch to exact checks or targeted probe-backed sensors.",
            details=parsed.text,
            check_name=parsed.check_name,
        )
    if not parsed.lower.startswith("deprecated:"):
        return None
    if "aabb-envelope semantics" not in parsed.lower and "legacy aabb" not in parsed.lower:
        return None
    return _signal_from_spec(
        DEPRECATED_TEST_API_SPEC,
        summary="Deprecated AABB-based test helper used; switch to exact visual checks.",
        details=parsed.text,
        check_name=parsed.check_name,
    )


def _generic_test_warning_signal(parsed: ParsedTestWarning) -> CompileSignal:
    return _signal_from_spec(
        TEST_WARNING_FALLBACK_SPEC,
        summary=parsed.headline,
        details=parsed.details,
    )


def _warning_signal_from_test_text(text: str) -> CompileSignal:
    parsed = _parse_test_warning(text)
    for classifier in (
        _coplanar_signal,
        _allowed_overlap_signal,
        _overlap_warning_signal,
        _disconnected_geometry_signal,
        _articulation_origin_signal,
        _deprecated_test_api_signal,
    ):
        signal = classifier(parsed)
        if signal is not None:
            return signal
    return _generic_test_warning_signal(parsed)


def _iter_test_warnings(test_report: TestReportLike | None) -> Iterable[CompileSignal]:
    if test_report is None:
        return ()

    signals: list[CompileSignal] = []
    for warning in test_report.warnings:
        text = str(warning).strip()
        if not text:
            continue
        signals.append(_warning_signal_from_test_text(text))
    return signals


def _iter_allowance_notes(test_report: TestReportLike | None) -> Iterable[CompileSignal]:
    if test_report is None:
        return ()

    signals: list[CompileSignal] = []
    for allowance in test_report.allowances:
        text = str(allowance).strip()
        if not text:
            continue
        if text.startswith("allow_isolated_part("):
            signals.append(
                _signal_from_spec(
                    ALLOWED_ISOLATED_PART_SPEC,
                    summary="Isolated-part allowance declared.",
                    details=text,
                )
            )
            continue
        signals.append(
            _signal_from_spec(
                ALLOWANCE_SPEC,
                summary=text,
            )
        )
    return signals


def _runtime_failure_signal(exc: BaseException) -> CompileSignal:
    detail_lines = _exception_detail_lines(exc)
    detail_lines.extend(_exception_location_lines(exc))
    summary = f"{type(exc).__name__}: compile execution failed."
    if detail_lines:
        first = detail_lines[0]
        if first:
            summary = f"{type(exc).__name__}: {first}"
    return _signal_from_spec(
        COMPILE_RUNTIME_SPEC,
        summary=summary,
        details="\n".join(detail_lines),
    )


def _signal_key(signal: CompileSignal) -> str:
    return signal.dedupe_key or signal.summary


def _build_bundle_summary(status: CompileStatus, signals: tuple[CompileSignal, ...]) -> str:
    failure_count = sum(1 for signal in signals if signal.severity == "failure")
    warning_count = sum(1 for signal in signals if signal.severity == "warning")
    note_count = sum(1 for signal in signals if signal.severity == "note")

    if failure_count:
        if any(signal.kind == "test_failure" for signal in signals):
            return (
                f"status={status} failures={failure_count} warnings={warning_count} notes={note_count}\n"
                "Primary issue: required QC tests failed."
            )
        return (
            f"status={status} failures={failure_count} warnings={warning_count} notes={note_count}\n"
            "Primary issue: compile execution failed."
        )
    if warning_count:
        return (
            f"status={status} failures=0 warnings={warning_count} notes={note_count}\n"
            "Primary issue: compile passed with warnings."
        )
    return f"status={status} failures=0 warnings=0 notes={note_count}\nCompile passed cleanly."


def build_compile_signal_bundle(
    *,
    status: CompileStatus,
    warnings: Iterable[str] = (),
    test_report: TestReportLike | None = None,
    exc: BaseException | None = None,
) -> CompileSignalBundle:
    deduped: dict[str, CompileSignal] = {}
    normalized_test_report = _as_test_report(test_report)
    test_warning_texts: set[str] = set()
    if normalized_test_report is not None:
        test_warning_texts = {
            text for warning in normalized_test_report.warnings if (text := str(warning).strip())
        }

    for warning in warnings:
        warning_text = str(warning).strip()
        if not warning_text or warning_text in test_warning_texts:
            continue
        signal = _warning_signal_from_text(warning_text)
        deduped[_signal_key(signal)] = signal

    for signal in _iter_test_warnings(normalized_test_report):
        deduped[_signal_key(signal)] = signal

    for signal in _iter_allowance_notes(normalized_test_report):
        deduped[_signal_key(signal)] = signal

    failure_signals = list(_iter_test_failures(normalized_test_report))
    if failure_signals:
        for signal in failure_signals:
            deduped[_signal_key(signal)] = signal
    elif exc is not None:
        runtime_signal = _runtime_failure_signal(exc)
        deduped[_signal_key(runtime_signal)] = runtime_signal

    signals = tuple(deduped.values())
    summary = _build_bundle_summary(status, signals)

    return CompileSignalBundle(status=status, summary=summary, signals=signals)


def compile_signal_bundle_from_exception(exc: BaseException) -> CompileSignalBundle:
    existing = getattr(exc, "compile_signal_bundle", None)
    if isinstance(existing, CompileSignalBundle):
        return existing
    warnings = getattr(exc, "warnings", None)
    test_report = getattr(exc, "test_report", None)
    warning_items = warnings if isinstance(warnings, list) else []
    return build_compile_signal_bundle(
        status="failure",
        warnings=warning_items,
        test_report=_as_test_report(test_report),
        exc=exc,
    )


def _render_signal_lines(signals: Iterable[CompileSignal]) -> str:
    rendered: list[str] = []
    for signal in signals:
        line = f"- [{signal.kind}] {signal.summary}"
        if signal.details:
            line += f"\n{signal.details}"
        rendered.append(line)
    return "\n".join(rendered)


def render_compile_signals(
    bundle: CompileSignalBundle,
    *,
    repeated: bool = False,
    failure_streak: int = 1,
) -> str:
    failures = [signal for signal in bundle.signals if signal.severity == "failure"]
    warnings = [signal for signal in bundle.signals if signal.severity == "warning"]
    notes = [signal for signal in bundle.signals if signal.severity == "note"]

    summary = bundle.summary
    if repeated and failures:
        summary += "\nThis failure matches the previous compile attempt."
    if failure_streak >= 3 and failures:
        summary += f"\nThis is compile failure {failure_streak} in a row."

    parts = ["<compile_signals>", "<summary>", summary, "</summary>"]
    if failures:
        parts.extend(["", "<failures>", _render_signal_lines(failures), "</failures>"])
    if warnings:
        parts.extend(["", "<warnings>", _render_signal_lines(warnings), "</warnings>"])
    if notes and (failures or warnings):
        parts.extend(["", "<notes>", _render_signal_lines(notes), "</notes>"])

    response_rules = [
        "- Resolve failures either by correcting the current code when the representation is sound, or by rethinking the model when the representation itself is wrong.",
        "- Treat warnings as design and QC sensors, not noise.",
        "- If a signal reveals a wrong representation or composition, replace that representation instead of tuning around it.",
    ]
    if repeated and failures:
        response_rules.append(
            "- This failure class repeated. Stop patching symptoms and rewrite the affected region from the root cause."
        )
    if failure_streak >= 3 and failures:
        response_rules.append(
            "- You are in a repair loop. Stop making small placement, tolerance, or box/cylinder tweaks and rewrite the failing subassembly with a more realistic representation."
        )
    parts.extend(
        [
            "",
            "<response_rules>",
            "\n".join(response_rules),
            "</response_rules>",
            "</compile_signals>",
        ]
    )
    return "\n".join(parts)
