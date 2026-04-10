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
import threading
import traceback
from pathlib import Path
from typing import Any

from agent.feedback import build_compile_signal_bundle
from agent.models import CompileReport, CompileSignalBundle
from agent.mp_utils import get_mp_context
from agent.prompts import normalize_sdk_package
from sdk._core.v0.assets import activate_asset_session, asset_session_for_script

logger = logging.getLogger(__name__)

_COMPILE_TARGETS = {"full", "visual"}

_EXCEPTION_PREFIX_RE = re.compile(r"^(?:[A-Za-z_][A-Za-z0-9_]*(?:Error|Exception)):\s*")
_VISUAL_OBJ_MESH_RE = re.compile(
    r"<visual\b[\s\S]*?<mesh\b[^>]*filename=['\"][^'\"]+\.obj['\"]",
    re.IGNORECASE,
)
_GEOMETRY_QC_MARKERS = (
    "isolated parts detected",
    "geometry overlap check reported overlaps",
    "mesh connectivity check failed",
    "visual connectivity check failed",
    "fail_if_parts_overlap_in_sampled_poses(",
    "expect_contact(",
    "expect_gap(",
    "expect_overlap(",
    "expect_within(",
    "expect_aabb_",
    "expect_xy_distance",
    "expect_above",
    "expect_joint_motion_axis",
    "urdf tests failed:",
)
_AUTOMATED_BASELINE_WARNING_CHECK_NAME = (
    "warn_if_part_contains_disconnected_geometry_islands(tol=1e-06)"
)
_AUTOMATED_BASELINE_DEFAULT_CHECK_NAMES = frozenset(
    {
        "check_model_valid",
        "check_single_root_part",
        "check_mesh_assets_ready",
        "fail_if_isolated_parts()",
        _AUTOMATED_BASELINE_WARNING_CHECK_NAME,
        "fail_if_parts_overlap_in_current_pose()",
    }
)
_MODEL_EXECUTION_LOCK = threading.Lock()


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
    rewrite_visual_glb: bool | None = None,
) -> str:
    """Execute a generated script and return the exported XML payload."""
    report = compile_urdf_report_maybe_timeout(
        script_path,
        sdk_package=sdk_package,
        run_checks=run_checks,
        ignore_geom_qc=ignore_geom_qc,
        target=target,
        rewrite_visual_glb=rewrite_visual_glb,
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
    validate_export: bool = True,
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
                    include_physical_collisions=target_key != "visual",
                    validate=validate_export,
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
            geometry_label = context.split(",", 1)[0].strip() or "geometry"
            return f"URDF compile warning ({geometry_label}, non-blocking): {detail}"

    return f"URDF compile warning (non-blocking): {message}"


def compile_urdf_report(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
    rewrite_visual_glb: bool | None = None,
) -> CompileReport:
    session = asset_session_for_script(script_path)
    with activate_asset_session(session):
        return _compile_urdf_report_impl(
            script_path,
            sdk_package=sdk_package,
            run_checks=run_checks,
            ignore_geom_qc=ignore_geom_qc,
            target=target,
            rewrite_visual_glb=rewrite_visual_glb,
        )


def _compile_urdf_report_impl(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
    run_checks: bool = True,
    ignore_geom_qc: bool = False,
    target: str = "full",
    rewrite_visual_glb: bool | None = None,
) -> CompileReport:
    """Execute a generated script and return export XML plus non-blocking warnings."""
    globals_dict = load_model_globals(script_path, sdk_package=sdk_package)
    warnings: list[str] = []
    test_report = None
    target_key = _normalize_compile_target(target)
    script_path = script_path.resolve()
    if run_checks:
        try:
            if target_key == "full":
                authored_report = _run_required_tests(
                    globals_dict,
                    sdk_package=sdk_package,
                )
                baseline_report = _run_compiler_owned_baseline_tests(
                    globals_dict,
                    script_path=script_path,
                    sdk_package=sdk_package,
                    authored_report=authored_report,
                )
                test_report = _merge_test_reports(
                    authored_report,
                    baseline_report,
                    sdk_package=sdk_package,
                )
                _raise_for_failed_test_report(test_report)
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

        warnings.extend(str(item) for item in getattr(test_report, "warnings", ()) or ())

    urdf_xml = _extract_urdf_xml(
        globals_dict,
        sdk_package=sdk_package,
        target=target_key,
        validate_export=not (run_checks and target_key == "full"),
    )
    if not isinstance(urdf_xml, str):
        raise ValueError("object_model must compile into an exportable XML payload")
    if _should_rewrite_visual_meshes_to_glb(
        sdk_package=sdk_package,
        rewrite_visual_glb=rewrite_visual_glb,
    ):
        urdf_xml = rewrite_visual_meshes_to_glb(
            urdf_xml,
            sdk_package=sdk_package,
            asset_root=script_path.parent,
            warnings=warnings,
        )
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


def rewrite_visual_meshes_to_glb(
    urdf_xml: str,
    *,
    sdk_package: str,
    asset_root: Path,
    warnings: list[str],
) -> str:
    if _VISUAL_OBJ_MESH_RE.search(urdf_xml) is None:
        return urdf_xml
    try:
        convert_urdf_visual_meshes_to_glb = getattr(
            _import_sdk_module(sdk_package, ".v0.viewer_assets"),
            "convert_urdf_visual_meshes_to_glb",
        )
        converted_xml, conversion_warnings = convert_urdf_visual_meshes_to_glb(
            urdf_xml,
            asset_root=asset_root,
        )
    except Exception as exc:
        warnings.append(f"Viewer mesh conversion warning: {exc}")
        return urdf_xml

    warnings.extend(str(item) for item in conversion_warnings)
    return converted_xml


def _should_rewrite_visual_meshes_to_glb(
    *,
    sdk_package: str,
    rewrite_visual_glb: bool | None,
) -> bool:
    if rewrite_visual_glb is not None:
        return rewrite_visual_glb
    return True


def load_model_globals(
    script_path: Path,
    *,
    sdk_package: str = "sdk",
) -> dict[str, Any]:
    """Execute a model script and return its globals."""
    normalize_sdk_package(sdk_package)
    repo_root = Path(__file__).resolve().parents[1]
    script_path = script_path.resolve()
    with _MODEL_EXECUTION_LOCK:
        prev_cwd = Path.cwd()
        os.chdir(script_path.parent)
        sys.path.insert(0, str(repo_root))
        try:
            globals_dict = runpy.run_path(script_path.name)
        finally:
            os.chdir(prev_cwd)
            if sys.path and sys.path[0] == str(repo_root):
                sys.path.pop(0)
    return globals_dict


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
        + "\nUse managed mesh helpers instead: `mesh_from_geometry(..., name='part_name')`, "
        "`mesh_from_input('existing_mesh')`, `mesh_from_cadquery(..., name='part_name')`, "
        "and `TestContext(object_model)`."
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
    rewrite_visual_glb: bool,
    conn: object,
) -> None:
    try:
        report = compile_urdf_report(
            Path(script_path_str),
            sdk_package=sdk_package,
            run_checks=run_checks,
            ignore_geom_qc=ignore_geom_qc,
            target=target,
            rewrite_visual_glb=rewrite_visual_glb,
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
    rewrite_visual_glb: bool | None = None,
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
            rewrite_visual_glb=rewrite_visual_glb,
        )

    ctx = get_mp_context()
    parent_conn, child_conn = ctx.Pipe(duplex=False)
    proc = ctx.Process(
        target=_compile_worker,
        args=(
            str(script_path),
            sdk_package,
            run_checks,
            ignore_geom_qc,
            target,
            rewrite_visual_glb,
            child_conn,
        ),
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


def _validate_mesh_connectivity(
    globals_dict: dict,
    *,
    script_dir: Path,
    sdk_package: str = "sdk",
) -> None:
    if os.environ.get("URDF_DISABLE_MESH_CONNECTIVITY_CHECK") in {"1", "true", "TRUE"}:
        return

    tol = float(os.environ.get("URDF_MESH_CONNECTIVITY_TOL", "0.02"))
    object_model = globals_dict.get("object_model")
    if object_model is None:
        return

    find_joint_origin_distance_findings = getattr(
        _import_sdk_module("sdk", "._core.v0.geometry_qc"),
        "find_joint_origin_distance_findings",
    )
    findings = find_joint_origin_distance_findings(
        object_model,
        asset_root=script_dir,
        tol=tol,
    )
    if not findings:
        return

    first = findings[0]
    raise ValueError(
        "Visual connectivity check failed: articulation origin is far from exact visual geometry. "
        f"joint={first.joint!r} parent={first.parent!r} child={first.child!r} "
        f"dist_parent={first.parent_distance:.4g} dist_child={first.child_distance:.4g} "
        f"tol={first.tol:.4g}. "
        "Fix by moving the joint origin and/or adjusting authored visuals so the parts touch where the joint is mounted."
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

    return report


def _report_warning_check_name(text: object) -> str | None:
    warning_text = str(text).strip()
    if not warning_text:
        return None
    check_name, has_separator, _detail = warning_text.partition(":")
    if not has_separator:
        return None
    return check_name.strip() or None


def _build_test_report(
    report_type: type,
    *,
    checks: tuple[str, ...],
    failures: tuple[object, ...],
    warnings: tuple[str, ...],
    allowances: tuple[str, ...],
    allowed_isolated_parts: tuple[str, ...],
    allowed_overlaps: tuple[object, ...],
) -> object:
    return report_type(
        passed=not failures,
        checks_run=len(checks),
        checks=checks,
        failures=failures,
        warnings=warnings,
        allowances=allowances,
        allowed_isolated_parts=allowed_isolated_parts,
        allowed_overlaps=allowed_overlaps,
    )


def _filter_duplicate_automated_baseline_results(
    authored_report: object, baseline_report: object
) -> object:
    authored_check_names = {
        str(name)
        for name in getattr(authored_report, "checks", ())
        if str(name) in _AUTOMATED_BASELINE_DEFAULT_CHECK_NAMES
    }
    if not authored_check_names:
        return baseline_report

    baseline_checks = tuple(
        str(name)
        for name in getattr(baseline_report, "checks", ())
        if str(name) not in authored_check_names
    )
    baseline_failures = tuple(
        failure
        for failure in getattr(baseline_report, "failures", ())
        if str(getattr(failure, "name", "")) not in authored_check_names
    )
    baseline_warnings = tuple(
        str(warning)
        for warning in getattr(baseline_report, "warnings", ())
        if _report_warning_check_name(warning) not in authored_check_names
    )

    return _build_test_report(
        type(baseline_report),
        checks=baseline_checks,
        failures=baseline_failures,
        warnings=baseline_warnings,
        allowances=tuple(str(item) for item in getattr(baseline_report, "allowances", ())),
        allowed_isolated_parts=tuple(
            str(item) for item in getattr(baseline_report, "allowed_isolated_parts", ())
        ),
        allowed_overlaps=tuple(getattr(baseline_report, "allowed_overlaps", ())),
    )


def _merge_test_reports(
    authored_report: object, baseline_report: object, *, sdk_package: str
) -> object:
    filtered_baseline_report = _filter_duplicate_automated_baseline_results(
        authored_report,
        baseline_report,
    )

    try:
        test_report_type = getattr(_import_sdk_module(sdk_package), "TestReport")
    except Exception as exc:  # pragma: no cover
        raise ValueError(f"Failed to import {sdk_package}.TestReport: {exc}") from exc

    check_names: list[str] = []
    seen_checks: set[str] = set()
    for report in (authored_report, filtered_baseline_report):
        for check_name in getattr(report, "checks", ()):
            normalized = str(check_name)
            if normalized in seen_checks:
                continue
            seen_checks.add(normalized)
            check_names.append(normalized)

    failures: list[object] = []
    seen_failures: set[tuple[str, str]] = set()
    for report in (authored_report, filtered_baseline_report):
        for failure in getattr(report, "failures", ()):
            key = (str(getattr(failure, "name", "")), str(getattr(failure, "details", "")))
            if key in seen_failures:
                continue
            seen_failures.add(key)
            failures.append(failure)

    warnings: list[str] = []
    seen_warnings: set[str] = set()
    for report in (authored_report, filtered_baseline_report):
        for warning in getattr(report, "warnings", ()):
            text = str(warning)
            if text in seen_warnings:
                continue
            seen_warnings.add(text)
            warnings.append(text)

    allowances: list[str] = []
    seen_allowances: set[str] = set()
    for report in (authored_report, filtered_baseline_report):
        for allowance in getattr(report, "allowances", ()):
            text = str(allowance)
            if text in seen_allowances:
                continue
            seen_allowances.add(text)
            allowances.append(text)

    allowed_isolated_parts: list[str] = []
    seen_isolated_parts: set[str] = set()
    for report in (authored_report, filtered_baseline_report):
        for part_name in getattr(report, "allowed_isolated_parts", ()):
            normalized = str(part_name)
            if normalized in seen_isolated_parts:
                continue
            seen_isolated_parts.add(normalized)
            allowed_isolated_parts.append(normalized)

    allowed_overlaps: list[object] = []
    seen_overlaps: set[tuple[str, str, str | None, str | None, str]] = set()
    for report in (authored_report, filtered_baseline_report):
        for overlap in getattr(report, "allowed_overlaps", ()):
            key = (
                str(getattr(overlap, "link_a", "")),
                str(getattr(overlap, "link_b", "")),
                (
                    None
                    if getattr(overlap, "elem_a", None) is None
                    else str(getattr(overlap, "elem_a"))
                ),
                (
                    None
                    if getattr(overlap, "elem_b", None) is None
                    else str(getattr(overlap, "elem_b"))
                ),
                str(getattr(overlap, "reason", "")),
            )
            if key in seen_overlaps:
                continue
            seen_overlaps.add(key)
            allowed_overlaps.append(overlap)

    return _build_test_report(
        test_report_type,
        checks=tuple(check_names),
        failures=tuple(failures),
        warnings=tuple(warnings),
        allowances=tuple(allowances),
        allowed_isolated_parts=tuple(allowed_isolated_parts),
        allowed_overlaps=tuple(allowed_overlaps),
    )


def _check_model_has_single_root_part(ctx: object) -> bool:
    root_parts = getattr(ctx.model, "root_parts", None)
    if not callable(root_parts):
        return bool(ctx.check("check_single_root_part", False, "model has no .root_parts()"))

    try:
        roots = root_parts()
    except Exception as exc:
        return bool(ctx.check("check_single_root_part", False, f"{type(exc).__name__}: {exc}"))

    root_names = tuple(
        str(getattr(part, "name", "")).strip()
        for part in roots
        if str(getattr(part, "name", "")).strip()
    )
    if len(root_names) != 1:
        return bool(
            ctx.check(
                "check_single_root_part",
                False,
                f"Expected exactly one root part, found {len(root_names)}: {list(root_names)!r}",
            )
        )
    return bool(ctx.check("check_single_root_part", True))


def _apply_authored_allowances_to_baseline_context(ctx: object, authored_report: object) -> None:
    for part_name in getattr(authored_report, "allowed_isolated_parts", ()):
        ctx.allow_isolated_part(
            str(part_name),
            reason="carried over from authored run_tests() allowance",
        )

    for overlap in getattr(authored_report, "allowed_overlaps", ()):
        ctx.allow_overlap(
            str(getattr(overlap, "link_a", "")),
            str(getattr(overlap, "link_b", "")),
            reason=str(getattr(overlap, "reason", "")),
            elem_a=(
                None
                if getattr(overlap, "elem_a", None) is None
                else str(getattr(overlap, "elem_a"))
            ),
            elem_b=(
                None
                if getattr(overlap, "elem_b", None) is None
                else str(getattr(overlap, "elem_b"))
            ),
        )


def _run_compiler_owned_baseline_tests(
    globals_dict: dict,
    *,
    script_path: Path,
    sdk_package: str,
    authored_report: object,
) -> object:
    object_model = globals_dict.get("object_model")
    if object_model is None:
        raise ValueError("Generated script must define top-level `object_model`")

    sdk_module = _import_sdk_module(sdk_package)
    test_context_type = getattr(sdk_module, "TestContext")
    ctx = test_context_type(object_model, asset_root=script_path.parent)
    _apply_authored_allowances_to_baseline_context(ctx, authored_report)
    ctx.check_model_valid()
    _check_model_has_single_root_part(ctx)
    preliminary_report = ctx.report()
    if not bool(getattr(preliminary_report, "passed", False)):
        return _build_test_report(
            type(preliminary_report),
            checks=tuple(str(name) for name in getattr(preliminary_report, "checks", ())),
            failures=tuple(getattr(preliminary_report, "failures", ())),
            warnings=tuple(str(item) for item in getattr(preliminary_report, "warnings", ())),
            allowances=(),
            allowed_isolated_parts=(),
            allowed_overlaps=(),
        )
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    baseline_report = ctx.report()
    return _build_test_report(
        type(baseline_report),
        checks=tuple(str(name) for name in getattr(baseline_report, "checks", ())),
        failures=tuple(getattr(baseline_report, "failures", ())),
        warnings=tuple(str(item) for item in getattr(baseline_report, "warnings", ())),
        allowances=(),
        allowed_isolated_parts=(),
        allowed_overlaps=(),
    )


def _raise_for_failed_test_report(report: object) -> None:
    if bool(getattr(report, "passed", False)):
        return

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
