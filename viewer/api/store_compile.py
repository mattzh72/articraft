from __future__ import annotations

from pathlib import Path
from typing import Any

from storage.materialize import (
    build_compile_fingerprint_from_inputs,
    build_compile_fingerprint_inputs,
    compile_report_matches_model_source_snapshot,
)


def _compile_level_from_report(report: dict[str, Any] | None) -> str | None:
    if not isinstance(report, dict):
        return None
    status = report.get("status")
    if status != "success":
        return None
    metrics = report.get("metrics")
    if isinstance(metrics, dict):
        value = metrics.get("compile_level")
        if isinstance(value, str) and value in {"visual", "full"}:
            return value
    return None


def _compile_report_satisfies_target(
    report: dict[str, Any] | None,
    *,
    target: str,
) -> bool:
    level = _compile_level_from_report(report)
    if level is None:
        return False
    if target == "full":
        return level == "full"
    if target == "visual":
        return level in {"visual", "full"}
    raise ValueError(f"Unsupported materialization target: {target!r}")


def _compile_report_matches_fingerprint(
    report: dict[str, Any] | None,
    *,
    current_fingerprint: str,
) -> bool:
    if not isinstance(report, dict):
        return False
    metrics = report.get("metrics")
    if not isinstance(metrics, dict):
        return False
    fingerprint = metrics.get("materialization_fingerprint")
    return isinstance(fingerprint, str) and fingerprint == current_fingerprint


def _current_compile_fingerprint(
    model_path: Path,
    *,
    compile_report: dict[str, Any] | None,
) -> str | None:
    if not model_path.exists():
        return None

    snapshot_match = compile_report_matches_model_source_snapshot(
        compile_report,
        model_path=model_path,
    )
    if snapshot_match:
        metrics = compile_report.get("metrics") if isinstance(compile_report, dict) else None
        fingerprint = (
            metrics.get("materialization_fingerprint") if isinstance(metrics, dict) else None
        )
        if isinstance(fingerprint, str) and fingerprint:
            return fingerprint

    return build_compile_fingerprint_from_inputs(
        build_compile_fingerprint_inputs(model_path=model_path)
    )
