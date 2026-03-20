from __future__ import annotations

import hashlib
import importlib
import inspect
import json
import logging
import math
import os
import re
import runpy
import statistics
import sys
import traceback
from pathlib import Path
from typing import Any

from agent.feedback import build_compile_signal_bundle
from agent.models import CompileReport, CompileSignalBundle
from agent.mp_utils import get_mp_context
from agent.prompts import normalize_sdk_package

logger = logging.getLogger(__name__)

_COMPILE_TARGETS = {"full", "visual", "collision"}

_EXCEPTION_PREFIX_RE = re.compile(r"^(?:[A-Za-z_][A-Za-z0-9_]*(?:Error|Exception)):\s*")
_GEOMETRY_QC_MARKERS = (
    "isolated parts detected",
    "geometry overlap check reported overlaps",
    "mesh connectivity check failed",
    "visual connectivity check failed",
    "visual/collision geometry appear misaligned",
    "check_no_overlaps(",
    "expect_aabb_",
    "expect_xy_distance",
    "expect_above",
    "expect_joint_motion_axis",
    "urdf tests failed:",
)


def _import_sdk_module(sdk_package: str, module_suffix: str = "") -> Any:
    package = normalize_sdk_package(sdk_package)
    return importlib.import_module(f"{package}{module_suffix}")


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def compile_urdf(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
) -> str:
    """Execute a generated script and return the exported XML payload."""
    report = compile_urdf_report_maybe_timeout(
        script_path,
        sdk_package=sdk_package,
        run_checks=run_checks,
        ignore_geom_qc=ignore_geom_qc,
        target=target,
    )
    for warning in report.warnings:
        logger.warning("%s", warning)
    return report.urdf_xml


def _normalize_compile_target(target: str) -> str:
    target_key = str(target).strip().lower()
    if target_key not in _COMPILE_TARGETS:
        supported = ", ".join(sorted(_COMPILE_TARGETS))
        raise ValueError(f"Unsupported compile target {target!r}. Expected one of: {supported}")
    return target_key


def _extract_urdf_xml(
    globals_dict: dict,
    *,
    sdk_package: str = "sdk",
    target: str = "full",
) -> str | None:
    target_key = _normalize_compile_target(target)
    object_model = globals_dict.get("object_model")
    urdf_xml: str | None = None
    try:
        if object_model is not None:
            compile_object_to_urdf_xml = getattr(
                _import_sdk_module(sdk_package, ".v0._urdf_export"),
                "compile_object_to_urdf_xml",
            )

            script_dir = globals_dict.get("__file__")
            asset_root = Path(script_dir).resolve().parent if isinstance(script_dir, str) else None
            try:
                params = inspect.signature(compile_object_to_urdf_xml).parameters
            except Exception:
                params = {}
            if "asset_root" in params:
                urdf_xml = compile_object_to_urdf_xml(
                    object_model,
                    asset_root=asset_root,
                    include_generated_collisions=target_key != "visual",
                )
            else:
                urdf_xml = compile_object_to_urdf_xml(object_model)
            globals_dict["urdf_xml"] = urdf_xml
    except Exception:
        urdf_xml = None

    if not isinstance(urdf_xml, str):
        maybe_urdf_xml = globals_dict.get("urdf_xml")
        if isinstance(maybe_urdf_xml, str):
            urdf_xml = maybe_urdf_xml
    return urdf_xml


def _attach_compiled_urdf_on_failure(
    exc: BaseException,
    *,
    urdf_xml: str | None,
    warnings: list[str],
    signal_bundle: CompileSignalBundle,
) -> BaseException:
    wrapped = RuntimeError(f"{type(exc).__name__}: {exc}")
    if isinstance(urdf_xml, str) and urdf_xml.strip():
        setattr(wrapped, "compiled_urdf_xml", urdf_xml)
    setattr(wrapped, "warnings", list(warnings))
    test_report = getattr(exc, "test_report", None)
    if test_report is not None:
        setattr(wrapped, "test_report", test_report)
    setattr(wrapped, "compile_signal_bundle", signal_bundle)
    return wrapped


def _strip_exception_prefixes(message: str) -> str:
    stripped = message.strip()
    while True:
        match = _EXCEPTION_PREFIX_RE.match(stripped)
        if match is None:
            return stripped
        stripped = stripped[match.end() :].lstrip()


def _nonblocking_geometry_qc_warning_from_exception(exc: BaseException) -> str | None:
    compiled_urdf_xml = getattr(exc, "compiled_urdf_xml", None)
    if not isinstance(compiled_urdf_xml, str) or not compiled_urdf_xml.strip():
        return None

    message = _strip_exception_prefixes(str(exc))
    lowered = message.lower()
    if not any(marker in lowered for marker in _GEOMETRY_QC_MARKERS):
        return None

    if message.startswith("URDF compile failure ("):
        header_end = message.find("):")
        if header_end != -1:
            context = message[len("URDF compile failure (") : header_end]
            detail = message[header_end + 2 :].lstrip(": ").lstrip()
            geometry_source = context.split(",", 1)[0].strip() or "geometry"
            return f"URDF compile warning ({geometry_source}, non-blocking): {detail}"

    return f"URDF compile warning (non-blocking): {message}"


def compile_urdf_report(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
) -> CompileReport:
    """Execute a generated script and return export XML plus non-blocking warnings."""
    target_key = _normalize_compile_target(target)
    repo_root = Path(__file__).resolve().parents[1]
    script_path = script_path.resolve()
    prev_cwd = Path.cwd()
    os.chdir(script_path.parent)
    sys.path.insert(0, str(repo_root))
    try:
        globals_dict = runpy.run_path(script_path.name)
    finally:
        os.chdir(prev_cwd)
        if sys.path and sys.path[0] == str(repo_root):
            sys.path.pop(0)

    warnings: list[str] = []
    test_report = None
    if run_checks:
        try:
            if target_key == "full":
                test_report = _run_required_tests(
                    globals_dict,
                    sdk_package=sdk_package,
                )
        except Exception as exc:
            urdf_xml = _extract_urdf_xml(
                globals_dict,
                sdk_package=sdk_package,
                target=target_key,
            )
            signal_bundle = build_compile_signal_bundle(
                status="failure",
                warnings=warnings,
                test_report=getattr(exc, "test_report", None),
                exc=exc,
            )
            wrapped = _attach_compiled_urdf_on_failure(
                exc,
                urdf_xml=urdf_xml,
                warnings=warnings,
                signal_bundle=signal_bundle,
            )
            if ignore_geom_qc:
                nonblocking_warning = _nonblocking_geometry_qc_warning_from_exception(wrapped)
                compiled_urdf_xml = getattr(wrapped, "compiled_urdf_xml", None)
                if (
                    nonblocking_warning is not None
                    and isinstance(compiled_urdf_xml, str)
                    and compiled_urdf_xml.strip()
                ):
                    warning_lines = list(warnings)
                    warning_lines.append(nonblocking_warning)
                    return CompileReport(
                        urdf_xml=compiled_urdf_xml,
                        warnings=warning_lines,
                        signal_bundle=build_compile_signal_bundle(
                            status="success",
                            warnings=warning_lines,
                            test_report=getattr(wrapped, "test_report", None),
                        ),
                    )
            raise wrapped from exc

    urdf_xml = _extract_urdf_xml(
        globals_dict,
        sdk_package=sdk_package,
        target=target_key,
    )
    if not isinstance(urdf_xml, str):
        raise ValueError("object_model must compile into an exportable XML payload")
    signal_bundle = build_compile_signal_bundle(
        status="success",
        warnings=warnings,
        test_report=test_report,
    )
    return CompileReport(
        urdf_xml=urdf_xml,
        warnings=warnings,
        signal_bundle=signal_bundle,
    )


def _warn_cwd_relative_asset_paths(*, script_path: Path, warnings: list[str]) -> None:
    """
    Emit a non-blocking warning for path anti-patterns that make outputs depend on current cwd.

    These scripts are expected to write assets relative to script location, not process cwd.
    """
    try:
        source = script_path.read_text(encoding="utf-8")
    except Exception:
        return

    findings: list[str] = []

    if re.search(r'^\s*HERE\s*=\s*Path\(\s*["\']\.\s*["\']\s*\)', source, re.MULTILINE):
        findings.append("Detected `HERE = Path('.')` assignment.")
    if re.search(r'asset_root\s*=\s*["\']\.\s*["\']', source):
        findings.append("Detected `asset_root='.'` usage.")
    if re.search(r'asset_root\s*=\s*Path\(\s*["\']\.\s*["\']\s*\)', source):
        findings.append("Detected `asset_root=Path('.')` usage.")

    if not findings:
        return

    warnings.append(
        "URDF compile warning (non-blocking): cwd-relative asset paths detected.\n"
        + "\n".join(f"- {item}" for item in findings)
        + "\nUse script-local paths instead: `ASSETS = AssetContext.from_script(__file__)`, "
        "`MESH_DIR = ASSETS.mesh_dir`, and set `ArticulatedObject(..., assets=ASSETS)` "
        "or `TestContext(..., asset_root=ASSETS.asset_root)`."
    )


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw.strip())
    except Exception:
        return default


def _iter_model_links(object_model: object) -> list[object]:
    links = getattr(object_model, "links", None)
    if isinstance(links, list):
        return links
    parts = getattr(object_model, "parts", None)
    if isinstance(parts, list):
        return parts
    return []


def _format_dims(dims: tuple[float, float, float]) -> str:
    return "(" + ", ".join(f"{float(v):.4g}" for v in dims) + ")m"


def _warn_geometry_scale_anomalies(
    globals_dict: dict,
    *,
    script_dir: Path,
    warnings: list[str],
    sdk_package: str = "sdk",
) -> None:
    if os.environ.get("URDF_DISABLE_IMPORTANT_GEOMETRY_WARNINGS") in {"1", "true", "TRUE"}:
        return

    object_model = globals_dict.get("object_model")
    if object_model is None:
        return

    links = _iter_model_links(object_model)
    if not links:
        return

    absurd_dim_max = float(os.environ.get("URDF_IMPORTANT_GEOMETRY_MAX_DIM", "1000.0"))
    outlier_ratio = float(os.environ.get("URDF_IMPORTANT_GEOMETRY_OUTLIER_RATIO", "100.0"))
    outlier_abs_min = float(os.environ.get("URDF_IMPORTANT_GEOMETRY_OUTLIER_ABS_MIN", "10.0"))
    max_findings = _env_int("URDF_IMPORTANT_GEOMETRY_MAX_FINDINGS", 10)

    absurd_findings: list[str] = []
    span_records: list[tuple[str, str, int, str, tuple[float, float, float], float]] = []

    for link in links:
        link_name = getattr(link, "name", None)
        if not isinstance(link_name, str) or not link_name:
            continue

        for source_name in ("visuals", "collisions"):
            items = getattr(link, source_name, None)
            if not isinstance(items, list) or not items:
                continue

            for index, item in enumerate(items):
                geometry = getattr(item, "geometry", None)
                if geometry is None:
                    continue
                try:
                    local_min, local_max = _geometry_local_aabb(
                        geometry,
                        script_dir=script_dir,
                        sdk_package=sdk_package,
                    )
                except Exception:
                    continue

                dims = (
                    float(local_max[0] - local_min[0]),
                    float(local_max[1] - local_min[1]),
                    float(local_max[2] - local_min[2]),
                )
                geom_type = type(geometry).__name__
                max_dim = max(abs(d) for d in dims)

                has_non_finite = any(not math.isfinite(v) for v in (*local_min, *local_max, *dims))
                if has_non_finite or max_dim > absurd_dim_max:
                    reason = (
                        "non-finite dimensions" if has_non_finite else f"max_dim={max_dim:.4g}m"
                    )
                    absurd_findings.append(
                        f"- link={link_name!r} source={source_name[:-1]!r} index={index} "
                        f"geometry={geom_type!r} dims={_format_dims(dims)} reason={reason}"
                    )

                if all(math.isfinite(v) for v in dims) and max_dim > 0.0:
                    span_records.append(
                        (link_name, source_name[:-1], index, geom_type, dims, max_dim)
                    )

    if absurd_findings:
        preview = "\n".join(absurd_findings[:max_findings])
        more = (
            ""
            if len(absurd_findings) <= max_findings
            else f"\n... ({len(absurd_findings) - max_findings} more)"
        )
        warnings.append(
            "IMPORTANT: URDF compile warning (non-blocking): non-finite or absurd geometry dimensions detected.\n"
            f"{preview}{more}\n"
            "These dimensions are likely numerically broken and will likely look wrong in the viewer. "
            "Check for sign errors, bad normalization denominators, unit mistakes, or runaway procedural geometry."
        )

    if not span_records:
        return

    spans = [record[5] for record in span_records]
    median_span = float(statistics.median(spans))
    if not math.isfinite(median_span) or median_span <= 0.0:
        return

    outlier_findings: list[str] = []
    for link_name, source_name, index, geom_type, dims, max_dim in span_records:
        if max_dim < outlier_abs_min:
            continue
        ratio = max_dim / max(median_span, 1e-9)
        if ratio < outlier_ratio:
            continue
        outlier_findings.append(
            f"- link={link_name!r} source={source_name!r} index={index} geometry={geom_type!r} "
            f"dims={_format_dims(dims)} max_dim={max_dim:.4g}m median_dim={median_span:.4g}m ratio={ratio:.4g}x"
        )

    if outlier_findings:
        preview = "\n".join(outlier_findings[:max_findings])
        more = (
            ""
            if len(outlier_findings) <= max_findings
            else f"\n... ({len(outlier_findings) - max_findings} more)"
        )
        warnings.append(
            "IMPORTANT: URDF compile warning (non-blocking): geometry outlier dimensions detected.\n"
            f"{preview}{more}\n"
            "One or more members are dramatically larger than the rest of the object and are likely malformed. "
            "Check for bad interpolation, reversed spans, or accidental scale explosions in authored geometry."
        )


def _compile_worker(
    script_path_str: str,
    sdk_package: str,
    run_checks: bool,
    ignore_geom_qc: bool,
    target: str,
    conn: object,
) -> None:
    try:
        report = compile_urdf_report(
            Path(script_path_str),
            sdk_package=sdk_package,
            run_checks=run_checks,
            ignore_geom_qc=ignore_geom_qc,
            target=target,
        )
        payload = {
            "ok": True,
            "urdf_xml": report.urdf_xml,
            "warnings": report.warnings,
            "signal_bundle": report.signal_bundle.to_dict(),
        }
        conn.send(payload)  # type: ignore[attr-defined]
    except BaseException as exc:
        payload = {
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
        compiled_urdf_xml = getattr(exc, "compiled_urdf_xml", None)
        if isinstance(compiled_urdf_xml, str) and compiled_urdf_xml.strip():
            payload["compiled_urdf_xml"] = compiled_urdf_xml
        warnings = getattr(exc, "warnings", None)
        if isinstance(warnings, list):
            payload["warnings"] = [str(w) for w in warnings]
        signal_bundle = getattr(exc, "compile_signal_bundle", None)
        if isinstance(signal_bundle, CompileSignalBundle):
            payload["signal_bundle"] = signal_bundle.to_dict()
        conn.send(payload)  # type: ignore[attr-defined]
    finally:
        try:
            conn.close()  # type: ignore[attr-defined]
        except Exception:
            pass


def compile_urdf_report_maybe_timeout(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
) -> CompileReport:
    """
    Run `compile_urdf_report` with a hard timeout to prevent indefinite hangs.

    Controlled by `URDF_COMPILE_TIMEOUT_SECONDS` (default: 300). Set to 0 to disable.
    """
    timeout_seconds = float(_env_float("URDF_COMPILE_TIMEOUT_SECONDS", 300.0))
    if timeout_seconds <= 0:
        return compile_urdf_report(
            script_path,
            sdk_package=sdk_package,
            run_checks=run_checks,
            ignore_geom_qc=ignore_geom_qc,
            target=target,
        )

    ctx = get_mp_context()
    parent_conn, child_conn = ctx.Pipe(duplex=False)
    proc = ctx.Process(
        target=_compile_worker,
        args=(str(script_path), sdk_package, run_checks, ignore_geom_qc, target, child_conn),
        daemon=True,
    )
    proc.start()
    try:
        try:
            child_conn.close()
        except Exception:
            pass

        if parent_conn.poll(timeout_seconds):
            msg = parent_conn.recv()
        else:
            try:
                proc.terminate()
            except Exception:
                pass
            proc.join(timeout=2.0)
            raise TimeoutError(
                f"URDF compile timed out after {timeout_seconds:.0f}s. "
                "This can happen if the generated script contains a long-running loop, "
                "expensive mesh processing, or very slow overlap checks. "
                "To adjust: set URDF_COMPILE_TIMEOUT_SECONDS, or reduce/disable overlap checks "
                "(URDF_GEOMETRY_OVERLAP_MAX_SAMPLES / URDF_DISABLE_GEOMETRY_OVERLAP_CHECK=1)."
            )
    finally:
        try:
            parent_conn.close()
        except Exception:
            pass

    proc.join(timeout=2.0)
    if proc.is_alive():
        try:
            proc.terminate()
        except Exception:
            pass
        proc.join(timeout=2.0)

    if not isinstance(msg, dict):
        raise RuntimeError(
            f"URDF compile failed: worker returned non-dict payload ({type(msg).__name__})"
        )
    if msg.get("ok") is True:
        urdf_xml = msg.get("urdf_xml")
        warnings = msg.get("warnings")
        signal_bundle_payload = msg.get("signal_bundle")
        if not isinstance(urdf_xml, str):
            raise RuntimeError("URDF compile failed: missing urdf_xml from worker")
        if not isinstance(warnings, list):
            warnings = []
        if isinstance(signal_bundle_payload, dict):
            signal_bundle = CompileSignalBundle.from_dict(signal_bundle_payload)
        else:
            signal_bundle = build_compile_signal_bundle(status="success", warnings=warnings)
        return CompileReport(
            urdf_xml=urdf_xml,
            warnings=[str(w) for w in warnings],
            signal_bundle=signal_bundle,
        )

    error_text = str(msg.get("error", "Unknown compile worker error")).strip()
    tb_text = str(msg.get("traceback", "")).strip()
    if tb_text:
        logger.debug("Compile worker traceback:\n%s", tb_text)
    exc = RuntimeError(error_text or "Unknown compile worker error")
    compiled_urdf_xml = msg.get("compiled_urdf_xml")
    if isinstance(compiled_urdf_xml, str) and compiled_urdf_xml.strip():
        setattr(exc, "compiled_urdf_xml", compiled_urdf_xml)
    warnings = msg.get("warnings")
    if isinstance(warnings, list):
        setattr(exc, "warnings", [str(w) for w in warnings])
    signal_bundle_payload = msg.get("signal_bundle")
    if isinstance(signal_bundle_payload, dict):
        setattr(
            exc,
            "compile_signal_bundle",
            CompileSignalBundle.from_dict(signal_bundle_payload),
        )
    raise exc


def _validate_geometry_overlaps(
    globals_dict: dict,
    *,
    script_dir: Path,
    sdk_package: str = "sdk",
) -> str | None:
    if os.environ.get("URDF_DISABLE_GEOMETRY_OVERLAP_CHECK") in {"1", "true", "TRUE"}:
        return None

    max_samples = int(os.environ.get("URDF_GEOMETRY_OVERLAP_MAX_SAMPLES", "128"))

    object_model = globals_dict.get("object_model")
    if object_model is None:
        return None

    try:
        sdk_mod = _import_sdk_module(sdk_package)
        default_overlap_tol_from_env = getattr(sdk_mod, "default_overlap_tol_from_env")
        default_overlap_volume_tol_from_env = getattr(
            sdk_mod,
            "default_overlap_volume_tol_from_env",
        )
        articulation_type = getattr(sdk_mod, "ArticulationType")
        validate_no_geometry_overlaps = getattr(sdk_mod, "validate_no_geometry_overlaps")
        validation_error = getattr(sdk_mod, "ValidationError")
    except Exception:
        return None

    ignore_fixed = os.environ.get("URDF_GEOMETRY_OVERLAP_IGNORE_FIXED", "1") in {
        "1",
        "true",
        "TRUE",
    }
    ignore_adjacent = os.environ.get("URDF_GEOMETRY_OVERLAP_IGNORE_ADJACENT", "1") in {
        "1",
        "true",
        "TRUE",
    }
    strict = os.environ.get("URDF_GEOMETRY_OVERLAP_STRICT", "0") in {"1", "true", "TRUE"}

    allowed_pairs = None
    if ignore_fixed or ignore_adjacent:
        pairs: list[tuple[str, str]] = []
        joints = getattr(object_model, "articulations", None)
        if isinstance(joints, list):
            for joint in joints:
                parent = getattr(joint, "parent", None)
                child = getattr(joint, "child", None)
                if not isinstance(parent, str) or not isinstance(child, str):
                    continue
                joint_type = getattr(joint, "articulation_type", None)
                if ignore_adjacent or (ignore_fixed and joint_type == articulation_type.FIXED):
                    pairs.append((parent, child))
        allowed_pairs = pairs or None

    overlap_kwargs = {
        "asset_root": script_dir,
        "max_pose_samples": max_samples,
        "overlap_tol": default_overlap_tol_from_env(),
        "overlap_volume_tol": default_overlap_volume_tol_from_env(),
        "allowed_pairs": allowed_pairs,
    }
    try:
        params = inspect.signature(validate_no_geometry_overlaps).parameters
    except Exception:
        params = {}
    if "geometry_source" in params:
        overlap_kwargs["geometry_source"] = "collision"
    else:
        overlap_kwargs["prefer_collisions"] = True

    try:
        validate_no_geometry_overlaps(
            object_model,
            **overlap_kwargs,
        )
    except validation_error as exc:
        if strict:
            raise
        return (
            "URDF compile warning (non-blocking): geometry overlap check reported overlaps.\n"
            f"{exc}\n"
            "If this overlap is acceptable (e.g., conservative false-positive), explicitly allow the specific pair "
            "in `run_tests()` via `ctx.allow_overlap(link_a, link_b, reason=...)` and then run "
            "`ctx.warn_if_overlaps(...)` so the allowance is tracked; otherwise adjust geometry/joints."
        )
    return None


def _format_unsupported_parts_message(
    geometry_source: str,
    findings: list[object],
    *,
    blocking: bool,
) -> str | None:
    if not findings:
        return None

    severity = "failure" if blocking else "warning"
    blocking_label = "blocking" if blocking else "non-blocking"
    lines = [
        f"URDF compile {severity} ({geometry_source}, {blocking_label}): isolated parts detected "
        "(not contacting any other part in the checked pose)."
    ]
    for finding in findings[:8]:
        part = getattr(finding, "part", None)
        nearest_part = getattr(finding, "nearest_part", None)
        min_distance = getattr(finding, "min_distance", None)
        pose = getattr(finding, "pose", None)
        pose_index = getattr(finding, "pose_index", None)
        backend = getattr(finding, "backend", None)

        detail = f"- part={part!r}"
        if nearest_part is not None:
            detail += f" nearest_part={nearest_part!r}"
        if isinstance(min_distance, (int, float)):
            detail += f" approx_gap={float(min_distance):.4g}m"
        if isinstance(pose_index, int):
            detail += f" pose_index={pose_index}"
        if isinstance(pose, dict) and pose:
            pose_preview = ", ".join(f"{k}={v:.4g}" for k, v in sorted(pose.items()))
            detail += f" pose=({pose_preview})"
        else:
            detail += " pose=(rest)"
        if isinstance(backend, str) and backend:
            detail += f" backend={backend}"
        lines.append(detail)

    if len(findings) > 8:
        lines.append(f"... ({len(findings) - 8} more)")

    if blocking:
        lines.append(
            "Conservative collision envelopes still do not touch, so this is usually a real floating-part or bad-mount bug."
        )
    lines.append(
        "Adjust geometry origins or articulation placement so each isolated part is attached."
    )
    return "\n".join(lines)


def _validate_unsupported_parts(
    globals_dict: dict,
    *,
    script_dir: Path,
    sdk_package: str = "sdk",
) -> list[str]:
    if os.environ.get("URDF_DISABLE_ISOLATED_PART_CHECK") in {"1", "true", "TRUE"}:
        return []

    object_model = globals_dict.get("object_model")
    if object_model is None:
        return []

    max_samples = int(os.environ.get("URDF_ISOLATED_PART_MAX_SAMPLES", "1"))

    try:
        sdk_mod = _import_sdk_module(sdk_package)
        default_contact_tol_from_env = getattr(sdk_mod, "default_contact_tol_from_env")
        find_unsupported_parts = getattr(sdk_mod, "find_unsupported_parts")
    except Exception:
        return []

    contact_tol = float(default_contact_tol_from_env())
    warnings: list[str] = []

    for geometry_source in ("visual", "collision"):
        findings = find_unsupported_parts(
            object_model,
            asset_root=script_dir,
            geometry_source=geometry_source,
            max_pose_samples=max_samples,
            contact_tol=contact_tol,
            seed=0,
        )

        is_blocking = geometry_source == "collision"
        message = _format_unsupported_parts_message(
            geometry_source,
            findings,
            blocking=is_blocking,
        )
        if not message:
            continue
        if is_blocking:
            raise RuntimeError(message)
        warnings.append(message)

    return warnings


def _validate_mesh_connectivity(
    globals_dict: dict,
    *,
    script_dir: Path,
    sdk_package: str = "sdk",
) -> None:
    if os.environ.get("URDF_DISABLE_MESH_CONNECTIVITY_CHECK") in {"1", "true", "TRUE"}:
        return

    tol = float(os.environ.get("URDF_MESH_CONNECTIVITY_TOL", "0.02"))
    include_qc = os.environ.get("URDF_MESH_CONNECTIVITY_INCLUDE_QC", "1") in {"1", "true", "TRUE"}

    object_model = globals_dict.get("object_model")
    if object_model is None:
        return

    links = getattr(object_model, "links", None)
    joints = getattr(object_model, "joints", None)
    if not isinstance(links, list) or not isinstance(joints, list):
        return

    link_to_aabbs: dict[
        str, list[tuple[tuple[float, float, float], tuple[float, float, float]]]
    ] = {}
    for link in links:
        name = getattr(link, "name", None)
        if not isinstance(name, str):
            continue

        collisions = getattr(link, "collisions", None)
        visuals = getattr(link, "visuals", None)
        geoms = collisions if isinstance(collisions, list) and collisions else visuals
        qc_collisions = getattr(link, "qc_collisions", None)
        if include_qc and isinstance(qc_collisions, list) and qc_collisions:
            geoms = list(geoms) if isinstance(geoms, list) else []
            geoms.extend(qc_collisions)
        if not isinstance(geoms, list) or not geoms:
            continue

        aabbs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
        for item in geoms:
            origin = getattr(item, "origin", None)
            geometry = getattr(item, "geometry", None)
            if geometry is None:
                continue

            local_min, local_max = _geometry_local_aabb(
                geometry,
                script_dir=script_dir,
                sdk_package=sdk_package,
            )
            world_min, world_max = _transform_aabb(
                local_min,
                local_max,
                origin=origin,
            )
            aabbs.append((world_min, world_max))

        if aabbs:
            link_to_aabbs[name] = aabbs

    for joint in joints:
        parent = getattr(joint, "parent", None)
        child = getattr(joint, "child", None)
        if not isinstance(parent, str) or not isinstance(child, str):
            continue

        parent_aabbs = link_to_aabbs.get(parent)
        child_aabbs = link_to_aabbs.get(child)
        if not parent_aabbs or not child_aabbs:
            continue

        origin = getattr(joint, "origin", None)
        parent_point = getattr(origin, "xyz", (0.0, 0.0, 0.0))
        if not (
            isinstance(parent_point, tuple)
            and len(parent_point) == 3
            and all(isinstance(v, (int, float)) for v in parent_point)
        ):
            parent_point = (0.0, 0.0, 0.0)

        parent_dist = min(_point_aabb_distance(parent_point, aabb) for aabb in parent_aabbs)
        child_dist = min(_point_aabb_distance((0.0, 0.0, 0.0), aabb) for aabb in child_aabbs)

        if parent_dist > tol or child_dist > tol:
            joint_name = getattr(joint, "name", "<unnamed>")
            raise ValueError(
                "Mesh connectivity check failed: joint connection point is not near geometry. "
                f"joint={joint_name!r} parent={parent!r} child={child!r} "
                f"dist_parent={parent_dist:.4g} dist_child={child_dist:.4g} tol={tol:.4g}. "
                "Fix by moving the joint origin and/or adjusting link visual/collision origins "
                "so the parts touch at the joint."
            )


def _warn_visual_connectivity_misalignment(
    globals_dict: dict,
    *,
    script_dir: Path,
    warnings: list[str],
    sdk_package: str = "sdk",
) -> None:
    if os.environ.get("URDF_DISABLE_VISUAL_CONNECTIVITY_WARNINGS") in {"1", "true", "TRUE"}:
        return

    tol = float(
        os.environ.get(
            "URDF_VISUAL_CONNECTIVITY_TOL", os.environ.get("URDF_MESH_CONNECTIVITY_TOL", "0.02")
        )
    )
    max_center_dist = float(os.environ.get("URDF_VISUAL_COLLISION_MAX_CENTER_DIST", "0.03"))
    max_size_rel_err = float(os.environ.get("URDF_VISUAL_COLLISION_MAX_SIZE_REL_ERR", "0.35"))

    object_model = globals_dict.get("object_model")
    if object_model is None:
        return

    links = getattr(object_model, "links", None)
    joints = getattr(object_model, "joints", None)
    if not isinstance(links, list) or not isinstance(joints, list):
        return

    def union_aabb(
        aabbs: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        if not aabbs:
            return None
        min_x = min(a[0][0] for a in aabbs)
        min_y = min(a[0][1] for a in aabbs)
        min_z = min(a[0][2] for a in aabbs)
        max_x = max(a[1][0] for a in aabbs)
        max_y = max(a[1][1] for a in aabbs)
        max_z = max(a[1][2] for a in aabbs)
        return (min_x, min_y, min_z), (max_x, max_y, max_z)

    def aabb_center(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]],
    ) -> tuple[float, float, float]:
        (mn, mx) = aabb
        return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)

    def aabb_size(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]],
    ) -> tuple[float, float, float]:
        (mn, mx) = aabb
        return (float(mx[0] - mn[0]), float(mx[1] - mn[1]), float(mx[2] - mn[2]))

    def dist3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        dx = float(a[0] - b[0])
        dy = float(a[1] - b[1])
        dz = float(a[2] - b[2])
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))

    link_to_vis_aabbs: dict[
        str, list[tuple[tuple[float, float, float], tuple[float, float, float]]]
    ] = {}
    link_to_col_aabbs: dict[
        str, list[tuple[tuple[float, float, float], tuple[float, float, float]]]
    ] = {}

    for link in links:
        name = getattr(link, "name", None)
        if not isinstance(name, str) or not name:
            continue

        visuals = getattr(link, "visuals", None)
        collisions = getattr(link, "collisions", None)

        if isinstance(visuals, list) and visuals:
            vis_aabbs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
            for item in visuals:
                origin = getattr(item, "origin", None)
                geometry = getattr(item, "geometry", None)
                if geometry is None:
                    continue
                local_min, local_max = _geometry_local_aabb(
                    geometry,
                    script_dir=script_dir,
                    sdk_package=sdk_package,
                )
                world_min, world_max = _transform_aabb(local_min, local_max, origin=origin)
                vis_aabbs.append((world_min, world_max))
            if vis_aabbs:
                link_to_vis_aabbs[name] = vis_aabbs

        if isinstance(collisions, list) and collisions:
            col_aabbs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
            for item in collisions:
                origin = getattr(item, "origin", None)
                geometry = getattr(item, "geometry", None)
                if geometry is None:
                    continue
                local_min, local_max = _geometry_local_aabb(
                    geometry,
                    script_dir=script_dir,
                    sdk_package=sdk_package,
                )
                world_min, world_max = _transform_aabb(local_min, local_max, origin=origin)
                col_aabbs.append((world_min, world_max))
            if col_aabbs:
                link_to_col_aabbs[name] = col_aabbs

    if sdk_package not in {"sdk", "sdk_hybrid"}:
        bad_links: list[str] = []
        for link_name, vis_aabbs in link_to_vis_aabbs.items():
            col_aabbs = link_to_col_aabbs.get(link_name)
            if not col_aabbs:
                continue
            vis_u = union_aabb(vis_aabbs)
            col_u = union_aabb(col_aabbs)
            if vis_u is None or col_u is None:
                continue

            c_vis = aabb_center(vis_u)
            c_col = aabb_center(col_u)
            d_center = dist3(c_vis, c_col)

            s_vis = aabb_size(vis_u)
            s_col = aabb_size(col_u)
            denom = max(1e-9, float(max(s_col)))
            size_rel_err = float(dist3(s_vis, s_col) / denom)
            perm_match = float(dist3(tuple(sorted(s_vis)), tuple(sorted(s_col))) / denom)

            if d_center > max_center_dist or size_rel_err > max_size_rel_err:
                hint = ""
                if perm_match <= 0.10 and size_rel_err > 0.10:
                    hint = " (sizes look permuted; common cause is swapped axes / wrong rpy on visuals)"
                bad_links.append(
                    f"link={link_name!r} center_dist={d_center:.4g}m max={max_center_dist:.4g} "
                    f"size_visual={tuple(round(v, 4) for v in s_vis)} size_collision={tuple(round(v, 4) for v in s_col)}{hint}"
                )

        if bad_links:
            preview = "\n".join(bad_links[:10])
            more = "" if len(bad_links) <= 10 else f"\n... ({len(bad_links) - 10} more)"
            warnings.append(
                "URDF compile warning (non-blocking): visual/collision geometry appear misaligned on some links.\n"
                f"{preview}{more}\n"
                "This often means a mesh was generated in the wrong axis convention (e.g. depth along Z instead of Y), "
                "or a visual Origin.rpy/xyz differs from the collision origin. Parts may look disconnected in the viewer."
            )

    vis_failures: list[str] = []
    for joint in joints:
        parent = getattr(joint, "parent", None)
        child = getattr(joint, "child", None)
        if not isinstance(parent, str) or not isinstance(child, str):
            continue

        parent_aabbs = link_to_vis_aabbs.get(parent)
        child_aabbs = link_to_vis_aabbs.get(child)
        if not parent_aabbs or not child_aabbs:
            continue

        origin = getattr(joint, "origin", None)
        parent_point = getattr(origin, "xyz", (0.0, 0.0, 0.0))
        if not (
            isinstance(parent_point, tuple)
            and len(parent_point) == 3
            and all(isinstance(v, (int, float)) for v in parent_point)
        ):
            parent_point = (0.0, 0.0, 0.0)
        parent_point = (float(parent_point[0]), float(parent_point[1]), float(parent_point[2]))

        parent_dist = min(_point_aabb_distance(parent_point, aabb) for aabb in parent_aabbs)
        child_dist = min(_point_aabb_distance((0.0, 0.0, 0.0), aabb) for aabb in child_aabbs)
        if parent_dist > tol or child_dist > tol:
            joint_name = getattr(joint, "name", "<unnamed>")
            vis_failures.append(
                f"joint={joint_name!r} parent={parent!r} child={child!r} "
                f"dist_parent={parent_dist:.4g} dist_child={child_dist:.4g} tol={tol:.4g}"
            )

    if vis_failures:
        preview = "\n".join(vis_failures[:10])
        more = "" if len(vis_failures) <= 10 else f"\n... ({len(vis_failures) - 10} more)"
        warnings.append(
            "URDF compile warning (non-blocking): visual connectivity check failed (collisions may still pass).\n"
            f"{preview}{more}\n"
            "Fix by aligning visual geometry with collisions and placing joint origins at the true attachment interface."
        )


def _geometry_local_aabb(
    geometry: object,
    *,
    script_dir: Path,
    sdk_package: str = "sdk",
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    sdk_mod = _import_sdk_module(sdk_package)
    box = getattr(sdk_mod, "Box")
    cylinder = getattr(sdk_mod, "Cylinder")
    mesh = getattr(sdk_mod, "Mesh")
    sphere = getattr(sdk_mod, "Sphere")

    if isinstance(geometry, box):
        sx, sy, sz = geometry.size
        return (-sx / 2.0, -sy / 2.0, -sz / 2.0), (sx / 2.0, sy / 2.0, sz / 2.0)
    if isinstance(geometry, cylinder):
        radius = float(geometry.radius)
        length = float(geometry.length)
        return (-radius, -radius, -length / 2.0), (radius, radius, length / 2.0)
    if isinstance(geometry, sphere):
        radius = float(geometry.radius)
        return (-radius, -radius, -radius), (radius, radius, radius)
    if isinstance(geometry, mesh):
        filename = getattr(geometry, "filename", "")
        if not isinstance(filename, str) or not filename:
            raise ValueError("Mesh geometry filename is missing")
        mesh_path = (script_dir / filename).resolve()
        if not mesh_path.exists():
            raise ValueError(f"Mesh file not found: {mesh_path}")
        local_min, local_max = _obj_aabb(mesh_path)
        scale = getattr(geometry, "scale", None)
        if scale:
            sx, sy, sz = scale
            local_min = (local_min[0] * sx, local_min[1] * sy, local_min[2] * sz)
            local_max = (local_max[0] * sx, local_max[1] * sy, local_max[2] * sz)
        return local_min, local_max

    raise ValueError(f"Unsupported geometry type for connectivity check: {type(geometry).__name__}")


def _obj_aabb(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    found = False
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.startswith("v "):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            continue
        found = True
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)

    if not found:
        raise ValueError(f"OBJ contains no vertices: {path}")
    return (min_x, min_y, min_z), (max_x, max_y, max_z)


def _transform_aabb(
    local_min: tuple[float, float, float],
    local_max: tuple[float, float, float],
    *,
    origin: object,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xyz = getattr(origin, "xyz", (0.0, 0.0, 0.0)) if origin is not None else (0.0, 0.0, 0.0)
    rpy = getattr(origin, "rpy", (0.0, 0.0, 0.0)) if origin is not None else (0.0, 0.0, 0.0)
    ox, oy, oz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    rr, rp, ry = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    rot = _rpy_matrix(rr, rp, ry)

    corners = []
    for x in (local_min[0], local_max[0]):
        for y in (local_min[1], local_max[1]):
            for z in (local_min[2], local_max[2]):
                corners.append((x, y, z))

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    for x, y, z in corners:
        tx, ty, tz = _mat_vec(rot, (x, y, z))
        tx += ox
        ty += oy
        tz += oz
        min_x = min(min_x, tx)
        min_y = min(min_y, ty)
        min_z = min(min_z, tz)
        max_x = max(max_x, tx)
        max_y = max(max_y, ty)
        max_z = max(max_z, tz)

    return (min_x, min_y, min_z), (max_x, max_y, max_z)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, float, float], ...]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_vec(
    mat: tuple[tuple[float, float, float], ...],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = vec
    return (
        mat[0][0] * x + mat[0][1] * y + mat[0][2] * z,
        mat[1][0] * x + mat[1][1] * y + mat[1][2] * z,
        mat[2][0] * x + mat[2][1] * y + mat[2][2] * z,
    )


def _point_aabb_distance(
    point: tuple[float, float, float],
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> float:
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    px, py, pz = point
    dx = 0.0
    if px < min_x:
        dx = min_x - px
    elif px > max_x:
        dx = px - max_x

    dy = 0.0
    if py < min_y:
        dy = min_y - py
    elif py > max_y:
        dy = py - max_y

    dz = 0.0
    if pz < min_z:
        dz = min_z - pz
    elif pz > max_z:
        dz = pz - max_z

    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _run_required_tests(
    globals_dict: dict,
    *,
    sdk_package: str = "sdk",
) -> object:
    run_tests = globals_dict.get("run_tests")
    if not callable(run_tests):
        raise ValueError(
            "Missing required `run_tests()` in generated script. "
            "Add a top-level `def run_tests() -> TestReport:` and return `ctx.report()`."
        )

    try:
        test_report_type = getattr(_import_sdk_module(sdk_package), "TestReport")
    except Exception as exc:  # pragma: no cover
        raise ValueError(f"Failed to import {sdk_package}.TestReport: {exc}") from exc

    report = run_tests()
    if not isinstance(report, test_report_type):
        raise ValueError(
            f"run_tests() must return {sdk_package}.TestReport (got {type(report).__name__})"
        )

    if not bool(getattr(report, "passed", False)):
        failures = getattr(report, "failures", ())
        lines = ["URDF tests failed:"]
        for failure in list(failures)[:10]:
            name = getattr(failure, "name", "unknown")
            details = getattr(failure, "details", "")
            lines.append(f"- {name}: {details}")
        if len(list(failures)) > 10:
            lines.append(f"... ({len(list(failures)) - 10} more)")
        exc = ValueError("\n".join(lines))
        setattr(exc, "test_report", report)
        raise exc

    return report


def update_manifest(outputs_root: Path) -> None:
    entries = []
    for path in outputs_root.rglob("*.urdf"):
        if "viewer" in path.parts:
            continue
        rel = path.relative_to(outputs_root).as_posix()
        name = path.stem
        entries.append({"name": name, "path": rel})

    entries.sort(key=lambda item: item["name"])
    manifest_path = outputs_root / "manifest.json"
    manifest_path.write_text(json.dumps({"generated": entries}, indent=2))


def persist_compile_success_artifacts(
    *,
    urdf_xml: str,
    urdf_out: Path | None,
    outputs_root: Path | None,
    previous_sig: str | None = None,
) -> str | None:
    """
    Persist the latest compile-success URDF and update manifest opportunistically.

    Returns:
        Content signature for deduping repeated writes.
    """
    if not isinstance(urdf_xml, str):
        return previous_sig

    sig = hashlib.sha1(urdf_xml.encode("utf-8")).hexdigest()
    if previous_sig and sig == previous_sig:
        return previous_sig

    if urdf_out is not None:
        out_path = Path(urdf_out).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(urdf_xml, encoding="utf-8")
        logger.info("Wrote checkpoint URDF to %s", out_path)

    if outputs_root is not None:
        update_manifest(Path(outputs_root).resolve())

    return sig
