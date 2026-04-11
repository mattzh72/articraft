from __future__ import annotations

import hashlib
import linecache
import math
import re
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal, Protocol, TypeAlias, runtime_checkable

from agent.models import CompileSignal, CompileSignalBundle

CODE_PATTERN = re.compile(
    r"```(?:python)?\s*\n"
    r"|^(?:from\s+\w+\s+import\s|import\s+\w+|def\s+\w+\s*\(|class\s+\w+)",
    re.MULTILINE,
)
_EXCEPTION_PREFIX_RE = re.compile(r"^(?:[A-Za-z_][A-Za-z0-9_]*(?:Error|Exception|Failure)):\s*")
LOFT_PROFILE_AREA_ERROR = "Loft profile area must be non-zero"
LOFT_PROFILE_AREA_HINT = (
    "Hint: LoftGeometry checks profile area in the XY projection. "
    "Use closed loops like `(x_i, y_i, z_const)`. "
    "If you need an XZ/YZ section, author it in XY first and rotate the mesh afterward."
)
SDK_SUBMODULE_IMPORT_HINT = (
    "Hint: Public authoring helpers import from top-level `sdk`. "
    "Use `from sdk import place_on_face`, `TestContext`, or `MotionLimits`, "
    "not guessed submodules like `sdk.placement`, `sdk.testing`, or `sdk.core_types`."
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
class ParsedTestFailure:
    name: str
    details: str
    lower_name: str
    lower_details: str


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
ISOLATED_PART_FAILURE_SPEC = SignalSpec(
    severity="failure",
    kind="isolated_part",
    code="TEST_ISOLATED_PART",
    source="tests",
    group="qc",
    blocking=True,
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
    severity="warning",
    kind="disconnected_geometry",
    code="WARN_DISCONNECTED_GEOMETRY",
    source="tests",
    group="qc",
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
MISSING_EXACT_GEOMETRY_SPEC = SignalSpec(
    severity="failure",
    kind="missing_exact_geometry",
    code="TEST_MISSING_EXACT_GEOMETRY",
    source="tests",
    group="qc",
    blocking=True,
)
EXACT_CONTACT_GAP_SPEC = SignalSpec(
    severity="failure",
    kind="exact_contact_gap",
    code="TEST_EXACT_CONTACT_GAP",
    source="tests",
    group="qc",
    blocking=True,
)
REAL_OVERLAP_TEST_SPEC = SignalSpec(
    severity="failure",
    kind="real_overlap",
    code="TEST_REAL_OVERLAP",
    source="tests",
    group="qc",
    blocking=True,
)
REAL_OVERLAP_COMPILER_SPEC = SignalSpec(
    severity="failure",
    kind="real_overlap",
    code="QC_REAL_OVERLAP",
    source="compiler",
    group="qc",
    blocking=True,
)
SINGLE_ROOT_POLICY_SPEC = SignalSpec(
    severity="failure",
    kind="single_root_policy",
    code="QC_SINGLE_ROOT_POLICY",
    source="compiler",
    group="build",
    blocking=True,
)
MODEL_VALIDITY_SPEC = SignalSpec(
    severity="failure",
    kind="model_validity",
    code="QC_MODEL_VALIDITY",
    source="compiler",
    group="build",
    blocking=True,
)
MESH_ASSETS_SPEC = SignalSpec(
    severity="failure",
    kind="mesh_assets",
    code="QC_MESH_ASSETS",
    source="compiler",
    group="build",
    blocking=True,
)
ISOLATED_PART_COMPILER_SPEC = SignalSpec(
    severity="failure",
    kind="isolated_part",
    code="QC_ISOLATED_PART",
    source="compiler",
    group="qc",
    blocking=True,
)

_MIN_DISTANCE_RE = re.compile(
    r"min_distance=(?P<distance>[-+0-9.eE]+)\s+contact_tol=(?P<tol>[-+0-9.eE]+)"
)
_TRACEBACK_FRAME_RE = re.compile(r'^\s*File "([^"]+)", line (\d+), in .+$')
_WINDOWS_ABSOLUTE_PATH_RE = re.compile(r"^[A-Za-z]:[\\/]")
_REPO_ROOT = Path(__file__).resolve().parents[1]


def contains_code_in_text(text: str) -> bool:
    """Check if a model text response pasted code instead of using tools."""
    if not text:
        return False
    return bool(CODE_PATTERN.search(text))


def _compile_hint_lines(detail_lines: list[str]) -> list[str]:
    if any(LOFT_PROFILE_AREA_ERROR in line for line in detail_lines):
        return [LOFT_PROFILE_AREA_HINT]
    lower_lines = [line.lower() for line in detail_lines]
    if any(
        "no module named 'sdk." in line or 'no module named "sdk.' in line for line in lower_lines
    ):
        return [SDK_SUBMODULE_IMPORT_HINT]
    return []


def _sanitize_display_path(path_text: str) -> str:
    raw = path_text.strip()
    if not raw:
        return raw

    normalized = raw.replace("\\", "/")
    if normalized.startswith("<") and normalized.endswith(">"):
        return raw
    if not (
        normalized.startswith("/")
        or normalized.startswith("\\")
        or _WINDOWS_ABSOLUTE_PATH_RE.match(raw)
    ):
        return normalized

    repo_root = _REPO_ROOT.as_posix().rstrip("/")
    if normalized == repo_root:
        return "."
    if repo_root and normalized.startswith(f"{repo_root}/"):
        return normalized[len(repo_root) + 1 :]

    parts = [part for part in normalized.split("/") if part]
    for marker in ("site-packages", "dist-packages"):
        if marker in parts:
            return "/".join(parts[parts.index(marker) :])

    if len(parts) <= 2:
        return "/".join(parts)
    return f".../{'/'.join(parts[-3:])}"


def _display_exception_type(exc: BaseException) -> str:
    remote_error_type = getattr(exc, "remote_error_type", None)
    if isinstance(remote_error_type, str) and remote_error_type.strip():
        return remote_error_type.strip()
    return type(exc).__name__


def _remote_traceback_text(exc: BaseException) -> str:
    remote_traceback = getattr(exc, "remote_traceback", None)
    if not isinstance(remote_traceback, str):
        return ""
    return remote_traceback.strip()


def _remote_traceback_exception_line(exc: BaseException) -> str | None:
    remote_traceback = _remote_traceback_text(exc)
    if not remote_traceback:
        return None
    for line in reversed(remote_traceback.splitlines()):
        cleaned = line.strip()
        if cleaned:
            return cleaned
    return None


def _is_low_information_exception_text(text: str) -> bool:
    stripped = text.strip()
    if not stripped:
        return True
    normalized = stripped
    while True:
        match = _EXCEPTION_PREFIX_RE.match(normalized)
        if match is None:
            break
        normalized = normalized[match.end() :].lstrip()
    if not normalized:
        return True
    return normalized.endswith(":") and " " not in normalized[:-1]


def _strip_redundant_exception_prefix(text: str, exc_type: str) -> str:
    stripped = text.strip()
    prefix = f"{exc_type}:"
    while stripped.startswith(prefix):
        stripped = stripped[len(prefix) :].lstrip()
    return stripped or text.strip()


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

    remote_exception_line = _remote_traceback_exception_line(exc)
    if remote_exception_line and remote_exception_line not in seen:
        if not detail_lines or _is_low_information_exception_text(detail_lines[0]):
            detail_lines.insert(0, remote_exception_line)
            seen.add(remote_exception_line)

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


def _location_lines_from_traceback_text(traceback_text: str) -> list[str]:
    if not traceback_text:
        return []

    lines = traceback_text.splitlines()
    last_frame_idx = -1
    last_filename = ""
    last_lineno = ""
    for idx, line in enumerate(lines):
        match = _TRACEBACK_FRAME_RE.match(line)
        if match is None:
            continue
        last_frame_idx = idx
        last_filename = match.group(1).strip()
        last_lineno = match.group(2).strip()

    if last_frame_idx < 0 or not last_filename or not last_lineno:
        return []

    rendered = [f"Location: {_sanitize_display_path(last_filename)}:{last_lineno}"]
    if last_frame_idx + 1 < len(lines):
        code_line = lines[last_frame_idx + 1].strip()
        if code_line and _TRACEBACK_FRAME_RE.match(lines[last_frame_idx + 1]) is None:
            rendered.append(f"Code: {code_line}")
    return rendered


def _exception_location_lines(exc: BaseException) -> list[str]:
    remote_lines = _location_lines_from_traceback_text(_remote_traceback_text(exc))
    if remote_lines:
        return remote_lines

    extracted = traceback.extract_tb(exc.__traceback__) if exc.__traceback__ else []
    skip_wrapper_location = isinstance(exc, RuntimeError)
    if not extracted or skip_wrapper_location:
        return []

    last = extracted[-1]
    location = f"{_sanitize_display_path(last.filename)}:{last.lineno}"
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


def _parse_test_failure(failure: TestFailureLike) -> ParsedTestFailure:
    name = str(failure.name)
    details = str(failure.details).strip()
    return ParsedTestFailure(
        name=name,
        details=details,
        lower_name=name.lower(),
        lower_details=details.lower(),
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
    summary = "Floating disconnected component(s) detected."
    if "(visual" in headline or "(visual," in headline:
        return "Floating disconnected visual component(s) detected."
    if "(collision" in headline or "(collision," in headline:
        return "Floating disconnected collision component(s) detected."
    return summary


def _format_distance_summary(value: float) -> str:
    magnitude = abs(float(value))
    if magnitude >= 0.1:
        return f"{value:.3g} m"
    mm_value = value * 1000.0
    if abs(mm_value - round(mm_value)) < 1e-9:
        return f"{int(round(mm_value))} mm"
    return f"{mm_value:.3g} mm"


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
        parsed = _parse_test_failure(failure)
        if parsed.name.startswith("fail_if_isolated_parts(") or (
            "isolated parts detected" in parsed.lower_details
        ):
            spec = (
                ISOLATED_PART_COMPILER_SPEC
                if parsed.name == "fail_if_isolated_parts()"
                else ISOLATED_PART_FAILURE_SPEC
            )
            signals.append(
                _signal_from_spec(
                    spec,
                    summary=_isolated_part_summary(
                        parsed.details.splitlines()[0] if parsed.details else parsed.name
                    ),
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        if parsed.name == "check_single_root_part":
            signals.append(
                _signal_from_spec(
                    SINGLE_ROOT_POLICY_SPEC,
                    summary=(
                        "Compiler-owned global QC requires exactly one root part "
                        "(`check_single_root_part`)."
                    ),
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        if parsed.name == "check_model_valid":
            signals.append(
                _signal_from_spec(
                    MODEL_VALIDITY_SPEC,
                    summary=(
                        "Compiler-owned structural validation rejected the current model "
                        "(`check_model_valid`)."
                    ),
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        if parsed.name == "check_mesh_assets_ready":
            signals.append(
                _signal_from_spec(
                    MESH_ASSETS_SPEC,
                    summary=(
                        "Compiler-owned mesh asset readiness checks found missing or unresolved "
                        "assets (`check_mesh_assets_ready`)."
                    ),
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        if "missing exact geometry" in parsed.lower_details:
            signals.append(
                _signal_from_spec(
                    MISSING_EXACT_GEOMETRY_SPEC,
                    summary="Authored exact check references named geometry that is not present.",
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        gap_match = _MIN_DISTANCE_RE.search(parsed.details)
        if gap_match is not None:
            try:
                min_distance = float(gap_match.group("distance"))
            except ValueError:
                min_distance = math.nan
            gap_summary = "nonzero separation"
            if math.isfinite(min_distance):
                gap_summary = _format_distance_summary(min_distance)
            signals.append(
                _signal_from_spec(
                    EXACT_CONTACT_GAP_SPEC,
                    summary=(
                        "Authored exact-contact check found "
                        f"{gap_summary} where contact was expected."
                    ),
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        if "part overlaps detected" in parsed.lower_details:
            spec = (
                REAL_OVERLAP_COMPILER_SPEC
                if parsed.name == "fail_if_parts_overlap_in_current_pose()"
                else REAL_OVERLAP_TEST_SPEC
            )
            summary = (
                "Compiler-owned global QC reported real 3D overlap in the current pose "
                "(`fail_if_parts_overlap_in_current_pose()`)."
                if spec is REAL_OVERLAP_COMPILER_SPEC
                else "Authored QC check found real 3D overlap in the tested pose."
            )
            signals.append(
                _signal_from_spec(
                    spec,
                    summary=summary,
                    details=parsed.details,
                    check_name=parsed.name,
                )
            )
            continue
        signals.append(
            _signal_from_spec(
                TEST_FAILURE_SPEC,
                summary=parsed.name,
                details=parsed.details,
                check_name=parsed.name,
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


def _allowed_isolated_part_warning_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.lower.startswith("isolated parts detected but allowed by justification"):
        return None
    return _signal_from_spec(
        ALLOWED_ISOLATED_PART_SPEC,
        summary="Isolated-part findings were allowed by justification.",
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
    if not parsed.check_name.startswith("warn_if_part_contains_disconnected_geometry_islands("):
        return None
    summary = (
        "Exact visual connectivity check found disconnected geometry within a part; "
        "this may be a real issue and should be investigated."
    )
    if (
        parsed.detail_text
        and "disconnected geometry islands detected" not in parsed.detail_text.lower()
    ):
        summary = "Part-geometry connectivity check reported a warning; investigate it."
    return _signal_from_spec(
        DISCONNECTED_GEOMETRY_SPEC,
        summary=summary,
        details=parsed.text,
        check_name=parsed.check_name,
    )


def _articulation_origin_signal(parsed: ParsedTestWarning) -> CompileSignal | None:
    if not parsed.check_name.startswith("warn_if_articulation_origin_far_from_geometry("):
        return None
    summary = "Exact articulation-origin distance check reported distant articulation origins."
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
        _allowed_isolated_part_warning_signal,
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
    structured_allowed_overlaps = tuple(getattr(test_report, "allowed_overlaps", ()))
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
        if text.startswith("allow_overlap(") and structured_allowed_overlaps:
            continue
        signals.append(
            _signal_from_spec(
                ALLOWANCE_SPEC,
                summary=text,
            )
        )

    for overlap in structured_allowed_overlaps:
        link_a = str(getattr(overlap, "link_a", "")).strip()
        link_b = str(getattr(overlap, "link_b", "")).strip()
        reason = str(getattr(overlap, "reason", "")).strip()
        elem_a = getattr(overlap, "elem_a", None)
        elem_b = getattr(overlap, "elem_b", None)
        details = f"allow_overlap({link_a!r}, {link_b!r})"
        if elem_a is not None or elem_b is not None:
            details += (
                f", elem_a={None if elem_a is None else str(elem_a)!r},"
                f" elem_b={None if elem_b is None else str(elem_b)!r}"
            )
        if reason:
            details += f": {reason}"
        signals.append(
            _signal_from_spec(
                ALLOWED_OVERLAP_SPEC,
                summary="Overlap allowance declared.",
                details=details,
            )
        )
    return signals


def _runtime_failure_signal(exc: BaseException) -> CompileSignal:
    detail_lines = _exception_detail_lines(exc)
    detail_lines.extend(_exception_location_lines(exc))
    exc_type = _display_exception_type(exc)
    summary = f"{exc_type}: compile execution failed."
    if detail_lines:
        first = detail_lines[0]
        if first:
            summary = f"{exc_type}: {_strip_redundant_exception_prefix(first, exc_type)}"
    return _signal_from_spec(
        COMPILE_RUNTIME_SPEC,
        summary=summary,
        details="\n".join(detail_lines),
    )


def _signal_key(signal: CompileSignal) -> str:
    return signal.dedupe_key or signal.summary


def _failure_sort_key(signal: CompileSignal) -> tuple[int, str, str]:
    priority = 90
    if signal.kind == "compile_runtime":
        priority = 0
    elif signal.kind in {"single_root_policy", "model_validity", "mesh_assets"}:
        priority = 1
    elif signal.source == "compiler" and signal.kind in {"isolated_part", "real_overlap"}:
        priority = 2
    elif signal.kind == "missing_exact_geometry":
        priority = 3
    elif signal.kind == "exact_contact_gap":
        priority = 4
    elif signal.kind == "real_overlap":
        priority = 5
    elif signal.kind == "isolated_part":
        priority = 6
    elif signal.kind == "test_failure":
        priority = 7
    return (priority, signal.kind, signal.summary)


def _ordered_failure_signals(signals: Iterable[CompileSignal]) -> list[CompileSignal]:
    return sorted(
        (signal for signal in signals if signal.severity == "failure"),
        key=_failure_sort_key,
    )


def _primary_failure_summary(signals: tuple[CompileSignal, ...]) -> str:
    failures = _ordered_failure_signals(signals)
    if not failures:
        return "Primary issue: compile execution failed."

    primary = failures[0]
    if primary.kind == "compile_runtime":
        primary_summary = primary.summary.strip()
        if not primary_summary:
            return "Primary issue: compile execution failed."
        return f"Primary issue: {primary_summary}"
    if primary.kind == "single_root_policy":
        return "Primary issue: compiler-owned structural policy checks failed."
    if primary.kind == "model_validity":
        return "Primary issue: compiler-owned model validation failed."
    if primary.kind == "mesh_assets":
        return "Primary issue: compiler-owned mesh asset readiness checks failed."
    if primary.source == "compiler" and primary.kind == "isolated_part":
        return "Primary issue: compiler-owned global QC found floating disconnected parts."
    if primary.source == "compiler" and primary.kind == "real_overlap":
        return "Primary issue: compiler-owned global QC reported part overlap that needs classification."
    if primary.kind == "missing_exact_geometry":
        return "Primary issue: authored exact checks reference missing named geometry."
    if primary.kind == "exact_contact_gap":
        return "Primary issue: authored exact-contact checks found separation where contact was expected."
    if primary.kind == "real_overlap":
        return "Primary issue: required QC tests reported part overlap."
    if primary.kind == "isolated_part":
        return "Primary issue: required QC tests found floating disconnected parts."
    return "Primary issue: required QC tests failed."


def _build_bundle_summary(status: CompileStatus, signals: tuple[CompileSignal, ...]) -> str:
    failure_count = sum(1 for signal in signals if signal.severity == "failure")
    warning_count = sum(1 for signal in signals if signal.severity == "warning")
    note_count = sum(1 for signal in signals if signal.severity == "note")

    if failure_count:
        return (
            f"status={status} failures={failure_count} warnings={warning_count} notes={note_count}\n"
            f"{_primary_failure_summary(signals)}"
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
    failures = _ordered_failure_signals(bundle.signals)
    warnings = [signal for signal in bundle.signals if signal.severity == "warning"]
    notes = [signal for signal in bundle.signals if signal.severity == "note"]

    if not failures and not warnings and not notes:
        return "\n".join(
            [
                "<compile_signals>",
                "<summary>",
                bundle.summary,
                "</summary>",
                "</compile_signals>",
            ]
        )

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

    response_rules = _response_rules_for_failures(
        failures,
        repeated=repeated,
        failure_streak=failure_streak,
        include_warning_note=bool(warnings),
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


def _response_rules_for_failures(
    failures: list[CompileSignal],
    *,
    repeated: bool,
    failure_streak: int,
    include_warning_note: bool,
) -> list[str]:
    if not failures:
        rules: list[str] = []
        if include_warning_note:
            rules.append(
                "- Warnings are not blocking, but they are design evidence and should not be ignored."
            )
        return rules

    primary = failures[0]
    intentional_qc_allowance_rule = (
        "- If any disconnected or overlapping finding is intentional, declare or "
        "correct explicit allowances for every intentional case. Scope each "
        "allowance to the exact part or element pair and include a concrete reason. "
        "Otherwise fix the underlying support path, mount, geometry, or pose."
    )
    rules: list[str]
    if primary.kind == "compile_runtime":
        rules = [
            "- Fix the compile/runtime error first. Geometry repair is blocked until the script executes cleanly.",
            "- Read the exception details and location lines before changing unrelated geometry or tests.",
        ]
    elif primary.kind == "single_root_policy":
        rules = [
            "- Fix the part tree first. This is a structural assembly error, not a local geometry issue.",
            "- Add or correct the missing articulation or fixed mount so the model has exactly one rooted assembly.",
            "- Do not spend turns tuning geometry until the root structure is valid.",
        ]
    elif primary.kind == "model_validity":
        rules = [
            "- Fix the model-definition error first. This is a structural validation failure, not a local placement issue.",
            "- Read the validation details carefully before changing unrelated geometry or exact checks.",
        ]
    elif primary.kind == "mesh_assets":
        rules = [
            "- Fix mesh asset readiness first. Missing or unresolved assets block exact geometry checks and downstream QC.",
            "- Resolve the referenced mesh names or asset roots before tuning geometry or tests.",
        ]
    elif primary.source == "compiler" and primary.kind == "isolated_part":
        rules = [
            "- Fix the compiler-owned floating/disconnected part failure first. Do not keep patching authored exact checks until the global QC failure is gone.",
            intentional_qc_allowance_rule,
        ]
    elif primary.source == "compiler" and primary.kind == "real_overlap":
        rules = [
            "- Compiler-owned overlap QC reported a real 3D overlap in the current pose. First decide whether it is an intentional embedding or an unintended collision.",
            "- If the overlap could be intentional, inspect any existing `allow_overlap(...)` coverage and compare its part and element scope to the reported pair before changing geometry.",
            intentional_qc_allowance_rule,
        ]
    elif primary.kind == "missing_exact_geometry":
        rules = [
            "- This is usually a deleted or renamed visual-name contract, not a placement failure.",
            "- Restore the named visual or update/remove the dependent exact check in the same edit.",
            "- Do not rewrite the subassembly just because this repeated.",
        ]
    elif primary.kind == "exact_contact_gap":
        rules = [
            "- This is a gap, not an overlap.",
            "- First verify that the tested pair is the right pair. The support path may be through a different exact element than the one named here.",
            "- If the pair is correct, change geometry or mount placement. Do not relax `contact_tol` unless near-zero contact is actually the intended invariant.",
        ]
    elif primary.kind == "real_overlap":
        rules = [
            "- Fix or explicitly justify the real 3D overlap in the tested pose before adding more exact checks.",
            intentional_qc_allowance_rule,
        ]
    elif primary.kind == "isolated_part":
        rules = [
            "- Fix the floating/disconnected part failure before tuning secondary geometry.",
            intentional_qc_allowance_rule,
        ]
    else:
        rules = [
            "- Failures are blocking and should be investigated before adding more geometry or tests.",
            "- Classify whether this is a local bug, a wrong representation, or a stale exact-check contract before patching.",
        ]

    if include_warning_note:
        rules.append(
            "- Warnings are not blocking, but they are design evidence and should not be ignored."
        )

    if repeated and primary.kind in {"real_overlap", "isolated_part"}:
        rules.append(
            "- This failure class repeated. Stop patching symptoms; either correct the underlying representation/support path or add a precise allowance when the finding is intentional."
        )
    if failure_streak >= 3:
        if primary.kind in {"missing_exact_geometry", "exact_contact_gap"}:
            rules.append(
                "- You are in a repair loop, but this failure family does not call for a geometry rewrite. Audit authored exact-name and exact-pair contracts first."
            )
        elif primary.kind in {"real_overlap", "isolated_part"}:
            rules.append(
                "- You are in a repair loop. Stop making small placement, tolerance, or primitive tweaks without a clear hypothesis. Reassess the subassembly representation, support path, and whether any repeated finding should instead be captured as an explicit allowance."
            )
    return rules
