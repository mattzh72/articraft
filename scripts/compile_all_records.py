from __future__ import annotations

import argparse
import errno
import importlib
import multiprocessing as mp
import os
import queue
import re
import resource
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    SpinnerColumn,
    TaskID,
    TextColumn,
    TimeElapsedColumn,
)
from rich.table import Table

from agent.mp_utils import (
    configured_mp_start_method_override,
    get_mp_context,
    mp_start_method_env_var,
    resolve_mp_start_method,
)
from storage.materialize import (
    infer_materialization_status,
    materialization_paths,
    urdf_references_external_meshes,
)
from storage.repo import StorageRepo

_GIB = 1024**3
_DEFAULT_MEM_PER_WORKER_GB = 3.0
_DEFAULT_VISUAL_MEM_PER_WORKER_GB = 1.5
_DEFAULT_RESERVE_MEM_GB = 2.0
_VISUAL_AUTO_WORKER_HARD_CAP = 512
_OPEN_FILE_WORKER_RESERVE = 64
_OPEN_FILE_WORKER_FD_BUDGET = 8
_COMPILE_TARGETS = {"full", "visual"}
_BASE_BULK_PRELOAD_MODULES = ("agent.compiler", "viewer.api.store")
_CADQUERY_BULK_PRELOAD_MODULES = ("cadquery", "OCP", "sdk_hybrid")
_EXCEPTION_PREFIX_RE = re.compile(r"^(?:[A-Za-z_][A-Za-z0-9_]*(?:Error|Exception)):\s*")
_GEOMETRY_QC_MARKERS = (
    "isolated parts detected",
    "mesh connectivity check failed",
    "visual connectivity check failed",
    "visual/collision geometry appear misaligned",
    "overlaps detected",
    "check_no_overlaps(",
    "expect_aabb_",
    "expect_xy_distance",
    "expect_above",
    "expect_joint_motion_axis",
)


@dataclass(slots=True, frozen=True)
class CompileCandidate:
    record_id: str
    reason: str
    force: bool = False
    estimated_mesh_bytes: int = 0
    mesh_file_count: int = 0
    sdk_package: str = "sdk"


@dataclass(slots=True, frozen=True)
class WorkerEvent:
    worker_id: int
    generation: int
    event: str
    record_id: str | None = None
    compiled: bool = False
    error: str | None = None


@dataclass(slots=True)
class WorkerState:
    worker_id: int
    generation: int
    process: Any
    task_queue: Any
    assigned: CompileCandidate | None = None
    started_at: float | None = None


@dataclass(slots=True, frozen=True)
class OpenFileWorkerCap:
    worker_cap: int
    soft_limit: int
    open_files: int
    reserve_files: int
    per_worker_budget: int


def _normalize_compile_target(target: str) -> str:
    target_key = str(target).strip().lower()
    if target_key not in _COMPILE_TARGETS:
        supported = ", ".join(sorted(_COMPILE_TARGETS))
        raise ValueError(f"Unsupported compile target {target!r}. Expected one of: {supported}")
    return target_key


def _compile_status(payload: object) -> str | None:
    if isinstance(payload, dict) and isinstance(payload.get("status"), str):
        return str(payload.get("status"))
    return None


def _compile_level(payload: object) -> str | None:
    if not isinstance(payload, dict) or payload.get("status") != "success":
        return None
    metrics = payload.get("metrics")
    if isinstance(metrics, dict):
        value = metrics.get("compile_level")
        if isinstance(value, str) and value in {"visual", "full"}:
            return value
    return None


def _estimate_visual_mesh_footprint(repo: StorageRepo, record_id: str) -> tuple[int, int]:
    mesh_root = repo.layout.record_materialization_asset_meshes_dir(record_id)
    if not mesh_root.exists():
        return 0, 0

    total_bytes = 0
    mesh_files = 0
    for path in mesh_root.rglob("*.obj"):
        try:
            rel_parts = path.relative_to(mesh_root).parts
        except ValueError:
            rel_parts = ()
        if rel_parts and rel_parts[0] == "collision":
            continue
        try:
            total_bytes += path.stat().st_size
            mesh_files += 1
        except OSError:
            continue
    return total_bytes, mesh_files


def _make_candidate(
    repo: StorageRepo,
    *,
    record_id: str,
    reason: str,
    force: bool,
    sdk_package: str,
) -> CompileCandidate:
    estimated_mesh_bytes, mesh_file_count = _estimate_visual_mesh_footprint(repo, record_id)
    return CompileCandidate(
        record_id=record_id,
        reason=reason,
        force=force,
        estimated_mesh_bytes=estimated_mesh_bytes,
        mesh_file_count=mesh_file_count,
        sdk_package=sdk_package,
    )


def _normalize_sdk_package(value: object) -> str:
    normalized = str(value or "sdk").strip().lower()
    if normalized in {"sdk_hybrid", "hybrid"}:
        return "sdk_hybrid"
    if normalized in {"sdk", "base"}:
        return "sdk"
    return normalized or "sdk"


def _record_model_script_path(
    repo: StorageRepo,
    record_id: str,
    record: object,
) -> Path:
    record_dir = repo.layout.record_dir(record_id)
    artifacts = record.get("artifacts") if isinstance(record, dict) else None
    model_name = (
        str(artifacts.get("model_py"))
        if isinstance(artifacts, dict) and artifacts.get("model_py")
        else "model.py"
    )
    return record_dir / model_name


def _artifact_materialization_status(
    repo: StorageRepo,
    record_id: str,
    *,
    urdf_path: Path,
) -> str:
    status = infer_materialization_status(repo, record_id)
    if status == "available":
        return status
    if urdf_path.exists() and not urdf_references_external_meshes(urdf_path):
        return "available"
    return status


def _collect_candidates(
    repo_root: Path,
    *,
    force: bool,
    target: str = "full",
) -> tuple[list[CompileCandidate], int]:
    target_key = _normalize_compile_target(target)
    repo = StorageRepo(repo_root)
    records_root = repo.layout.records_root
    if not records_root.exists():
        return [], 0

    candidates: list[CompileCandidate] = []
    skipped_missing_script = 0

    for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
        record_id = record_dir.name
        record = repo.read_json(repo.layout.record_metadata_path(record_id))
        sdk_package = _normalize_sdk_package(
            record.get("sdk_package") if isinstance(record, dict) else "sdk"
        )
        script_path = _record_model_script_path(repo, record_id, record)
        urdf_path = materialization_paths(repo, record_id)["model_urdf"]
        compile_report_path = materialization_paths(repo, record_id)["compile_report_json"]
        compile_report = repo.read_json(compile_report_path)
        compile_status = _compile_status(compile_report)
        compile_level = _compile_level(compile_report)

        if not script_path.exists():
            if isinstance(record, dict) or compile_status is not None or urdf_path.exists():
                candidates.append(
                    _make_candidate(
                        repo,
                        record_id=record_id,
                        reason="missing model.py",
                        force=False,
                        sdk_package=sdk_package,
                    )
                )
                continue
            skipped_missing_script += 1
            continue

        if force:
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason="forced",
                    force=True,
                    sdk_package=sdk_package,
                )
            )
            continue

        materialization_status = _artifact_materialization_status(
            repo,
            record_id,
            urdf_path=urdf_path,
        )

        if not urdf_path.exists():
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason="missing model.urdf",
                    force=False,
                    sdk_package=sdk_package,
                )
            )
            continue

        if materialization_status == "missing":
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason="missing generated assets",
                    force=False,
                    sdk_package=sdk_package,
                )
            )
            continue

        if compile_status != "success":
            label = compile_status or "unknown"
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason=f"compile status is {label}",
                    force=True,
                    sdk_package=sdk_package,
                )
            )
            continue

        if target_key == "full" and compile_level != "full":
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason="compile level is visual",
                    force=True,
                    sdk_package=sdk_package,
                )
            )

    return candidates, skipped_missing_script


def _sort_candidates_for_compile(candidates: list[CompileCandidate]) -> list[CompileCandidate]:
    return sorted(
        candidates,
        key=lambda candidate: (
            candidate.estimated_mesh_bytes,
            candidate.mesh_file_count,
            candidate.record_id,
        ),
        reverse=True,
    )


def _limit_candidates(
    candidates: list[CompileCandidate],
    *,
    limit: int | None,
) -> list[CompileCandidate]:
    ordered = _sort_candidates_for_compile(candidates)
    if limit is None:
        return ordered
    return ordered[:limit]


def _build_summary_table(
    *,
    scanned: int,
    skipped_missing_script: int,
    candidates: int,
    compiled: int,
    failed: int,
    failure_label: str = "Failed",
) -> Table:
    table = Table(title="Compile Summary")
    table.add_column("Metric")
    table.add_column("Count", justify="right")
    table.add_row("Scanned records", str(scanned))
    table.add_row("Missing model.py", str(skipped_missing_script))
    table.add_row("Queued for compile", str(candidates))
    table.add_row("Compiled", str(compiled))
    table.add_row(failure_label, str(failed))
    table.add_row("Already up to date", str(max(scanned - skipped_missing_script - candidates, 0)))
    return table


def _strip_compile_error_wrappers(record_id: str, error: str) -> str:
    message = error.strip()
    prefix = f"Failed to compile assets for {record_id}: "
    if message.startswith(prefix):
        message = message[len(prefix) :].lstrip()
    while True:
        match = _EXCEPTION_PREFIX_RE.match(message)
        if match is None:
            break
        message = message[match.end() :].lstrip()
    return message


def _is_geometry_qc_issue(message: str) -> bool:
    lowered = message.lower()
    return any(marker in lowered for marker in _GEOMETRY_QC_MARKERS)


def _rewrite_nonblocking_geometry_issue(message: str) -> str:
    if message.startswith("URDF compile failure ("):
        header_end = message.find("):")
        if header_end != -1:
            context = message[len("URDF compile failure (") : header_end]
            detail = message[header_end + 2 :].lstrip(": ").lstrip()
            geometry_source = context.split(",", 1)[0].strip() or "geometry"
            return f"Geometry QC issue ({geometry_source}, non-blocking in compile-all): {detail}"
    return f"Geometry QC issue (non-blocking in compile-all): {message}"


def _format_bulk_compile_error(record_id: str, error: str, *, strict: bool) -> str:
    message = _strip_compile_error_wrappers(record_id, error)
    if strict or not _is_geometry_qc_issue(message):
        return message
    return _rewrite_nonblocking_geometry_issue(message)


def _should_suppress_bulk_compile_error(record_id: str, error: str, *, strict: bool) -> bool:
    if strict:
        return False
    message = _strip_compile_error_wrappers(record_id, error)
    return _is_geometry_qc_issue(message)


def _logical_cpu_count() -> int:
    return max(int(os.cpu_count() or 1), 1)


def _linux_available_memory_bytes() -> int | None:
    meminfo_path = Path("/proc/meminfo")
    if not meminfo_path.exists():
        return None
    try:
        for line in meminfo_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("MemAvailable:"):
                parts = line.split()
                if len(parts) >= 2:
                    return int(parts[1]) * 1024
    except Exception:
        return None
    return None


def _macos_available_memory_bytes() -> int | None:
    try:
        output = subprocess.check_output(
            ["vm_stat"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None

    page_size_match = re.search(r"page size of (\d+) bytes", output)
    if page_size_match is None:
        return None
    page_size = int(page_size_match.group(1))
    page_counts: dict[str, int] = {}
    for line in output.splitlines():
        match = re.match(r"([^:]+):\s+(\d+)\.", line.strip())
        if match is None:
            continue
        page_counts[match.group(1)] = int(match.group(2))

    available_pages = (
        page_counts.get("Pages free", 0)
        + page_counts.get("Pages inactive", 0)
        + page_counts.get("Pages speculative", 0)
        + page_counts.get("Pages purgeable", 0)
    )
    if available_pages <= 0:
        return None
    return available_pages * page_size


def _total_memory_bytes() -> int | None:
    try:
        page_size = int(os.sysconf("SC_PAGE_SIZE"))
        page_count = int(os.sysconf("SC_PHYS_PAGES"))
    except (AttributeError, OSError, ValueError):
        return None
    total = page_size * page_count
    return total if total > 0 else None


def _available_memory_bytes() -> int | None:
    available = _linux_available_memory_bytes()
    if available is not None:
        return available
    available = _macos_available_memory_bytes()
    if available is not None:
        return available
    return _total_memory_bytes()


def _memory_budget_bytes() -> int | None:
    total = _total_memory_bytes()
    if total is not None:
        return total
    return _available_memory_bytes()


def _open_file_soft_limit() -> int | None:
    if not hasattr(resource, "RLIMIT_NOFILE"):
        return None
    try:
        soft_limit, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
    except Exception:
        return None
    if not isinstance(soft_limit, int) or soft_limit <= 0:
        return None
    return soft_limit


def _current_open_file_count() -> int | None:
    for fd_root in ("/dev/fd", "/proc/self/fd"):
        try:
            return len(os.listdir(fd_root))
        except OSError:
            continue
    return None


def _open_file_worker_cap() -> OpenFileWorkerCap | None:
    soft_limit = _open_file_soft_limit()
    open_files = _current_open_file_count()
    if soft_limit is None or open_files is None:
        return None
    usable = soft_limit - open_files - _OPEN_FILE_WORKER_RESERVE
    if usable <= 0:
        worker_cap = 1
    else:
        worker_cap = max(1, usable // _OPEN_FILE_WORKER_FD_BUDGET)
    return OpenFileWorkerCap(
        worker_cap=worker_cap,
        soft_limit=soft_limit,
        open_files=open_files,
        reserve_files=_OPEN_FILE_WORKER_RESERVE,
        per_worker_budget=_OPEN_FILE_WORKER_FD_BUDGET,
    )


def _is_open_file_limit_error(exc: BaseException) -> bool:
    return isinstance(exc, OSError) and exc.errno in {errno.EMFILE, errno.ENFILE}


def _auto_worker_memory_cap(
    *,
    reserve_mem_gb: float,
    mem_per_worker_gb: float,
) -> int | None:
    if reserve_mem_gb < 0:
        raise ValueError("Reserve memory must be zero or greater.")
    if mem_per_worker_gb <= 0:
        raise ValueError("Memory per worker must be greater than zero.")

    budget_bytes = _available_memory_bytes()
    if budget_bytes is None:
        budget_bytes = _memory_budget_bytes()
    if budget_bytes is None:
        return None

    reserve_bytes = int(reserve_mem_gb * _GIB)
    worker_bytes = max(int(mem_per_worker_gb * _GIB), 1)
    usable_bytes = budget_bytes - reserve_bytes
    if usable_bytes <= 0:
        return 1
    return max(1, usable_bytes // worker_bytes)


def _resolve_worker_count(
    raw_value: str,
    *,
    candidate_count: int,
    reserve_mem_gb: float,
    mem_per_worker_gb: float,
    target: str = "full",
) -> int:
    if candidate_count <= 0:
        return 1

    if reserve_mem_gb < 0:
        raise ValueError("Reserve memory must be zero or greater.")
    if mem_per_worker_gb <= 0:
        raise ValueError("Memory per worker must be greater than zero.")

    normalized = str(raw_value).strip().lower()
    open_file_cap = _open_file_worker_cap()
    if normalized == "max":
        resolved = max(1, candidate_count)
        if open_file_cap is not None:
            resolved = min(resolved, open_file_cap.worker_cap)
        return resolved

    if normalized == "auto":
        # Align auto behavior across compile-all variants for speed-first throughput.
        # This mirrors compile-visual's fan-out policy and is intentionally
        # independent of compile target to avoid unexpectedly conservative defaults.
        resolved = max(1, min(candidate_count, _VISUAL_AUTO_WORKER_HARD_CAP))
        if open_file_cap is not None:
            resolved = min(resolved, open_file_cap.worker_cap)
        return resolved

    try:
        requested = int(normalized)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported concurrency value {raw_value!r}. Expected auto, max, or a positive integer."
        ) from exc

    if requested <= 0:
        raise ValueError("Concurrency must be a positive integer, or one of: auto, max.")
    resolved = min(candidate_count, requested)
    if open_file_cap is not None:
        resolved = min(resolved, open_file_cap.worker_cap)
    return resolved


def _format_gib(value: int) -> str:
    return f"{value / _GIB:.1f} GiB"


def _build_concurrency_message(
    *,
    requested: str,
    resolved_workers: int,
    target: str,
    candidate_count: int,
    reserve_mem_gb: float,
    mem_per_worker_gb: float,
) -> str:
    cpu_count = _logical_cpu_count()
    total_bytes = _total_memory_bytes()
    available_bytes = _available_memory_bytes()
    normalized = str(requested).strip().lower()
    open_file_cap = _open_file_worker_cap()
    message = (
        f"Using {resolved_workers} worker"
        f"{'' if resolved_workers == 1 else 's'} "
        f"(`--concurrency={requested}`, target={target}, queued records={candidate_count}, "
        f"{cpu_count} logical CPUs"
    )
    if normalized == "auto":
        message += f", auto mode=throughput-first, hard cap {_VISUAL_AUTO_WORKER_HARD_CAP}"
        if open_file_cap is not None:
            message += (
                f", open files soft limit {open_file_cap.soft_limit}, "
                f"open now {open_file_cap.open_files}, fd cap {open_file_cap.worker_cap}"
            )
        message += ")."
        return message
    if normalized == "max":
        message += ", explicit max fan-out"
        if open_file_cap is not None:
            message += (
                f", open files soft limit {open_file_cap.soft_limit}, "
                f"open now {open_file_cap.open_files}, fd cap {open_file_cap.worker_cap}"
            )
        message += ")."
        return message
    if total_bytes is not None:
        message += f", host memory {_format_gib(total_bytes)}"
    if available_bytes is not None:
        message += f", approx available now {_format_gib(available_bytes)}"
    message += f", reserve {reserve_mem_gb:.1f} GiB, budget {mem_per_worker_gb:.1f} GiB/worker)."
    if open_file_cap is not None:
        message = message[:-1] + (
            f", open files soft limit {open_file_cap.soft_limit}, "
            f"open now {open_file_cap.open_files}, fd cap {open_file_cap.worker_cap})."
        )
    return message


def _build_multiprocessing_message(
    *,
    start_method: str,
    preload_modules: tuple[str, ...],
) -> str:
    env_name = mp_start_method_env_var()
    raw_override = configured_mp_start_method_override()
    override_label = raw_override if raw_override is not None else "<unset>"
    message = f"Multiprocessing: {env_name}={override_label}, effective start method={start_method}"
    if start_method == "forkserver" and preload_modules:
        message += ", forkserver preload=[" + ", ".join(preload_modules) + "]"
    elif start_method == "fork" and preload_modules:
        message += ", parent preload=[" + ", ".join(preload_modules) + "]"
    return message


def _effective_mem_per_worker_gb(*, target: str, requested_mem_per_worker_gb: float) -> float:
    if (
        target == "visual"
        and abs(float(requested_mem_per_worker_gb) - _DEFAULT_MEM_PER_WORKER_GB) < 1e-9
    ):
        return _DEFAULT_VISUAL_MEM_PER_WORKER_GB
    return float(requested_mem_per_worker_gb)


def _bulk_compile_preload_modules(candidates: list[CompileCandidate]) -> tuple[str, ...]:
    modules = list(_BASE_BULK_PRELOAD_MODULES)
    if any(candidate.sdk_package == "sdk_hybrid" for candidate in candidates):
        modules.extend(_CADQUERY_BULK_PRELOAD_MODULES)

    deduped: list[str] = []
    seen: set[str] = set()
    for module_name in modules:
        normalized = str(module_name).strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        deduped.append(normalized)
    return tuple(deduped)


def _preload_modules_in_parent(module_names: tuple[str, ...]) -> None:
    for module_name in module_names:
        try:
            importlib.import_module(module_name)
        except Exception:
            continue


def _worker_loop(
    worker_id: int,
    generation: int,
    repo_root: str,
    strict: bool,
    target: str,
    task_queue: Any,
    result_queue: Any,
) -> None:
    from viewer.api.store import ViewerStore

    viewer_store = ViewerStore(Path(repo_root), ensure_search_index=False)
    while True:
        candidate = task_queue.get()
        if candidate is None:
            return
        if not isinstance(candidate, CompileCandidate):
            continue

        result_queue.put(
            WorkerEvent(
                worker_id=worker_id,
                generation=generation,
                event="started",
                record_id=candidate.record_id,
            )
        )
        try:
            result = viewer_store.materialize_record_assets(
                candidate.record_id,
                force=candidate.force,
                ignore_geom_qc=not strict,
                validate=strict,
                target=target,
                use_compile_timeout=False,
            )
            result_queue.put(
                WorkerEvent(
                    worker_id=worker_id,
                    generation=generation,
                    event="finished",
                    record_id=candidate.record_id,
                    compiled=result.compiled,
                )
            )
        except Exception as exc:
            result_queue.put(
                WorkerEvent(
                    worker_id=worker_id,
                    generation=generation,
                    event="finished",
                    record_id=candidate.record_id,
                    error=str(exc),
                )
            )


def _spawn_worker(
    ctx: mp.context.BaseContext,
    *,
    worker_id: int,
    generation: int,
    repo_root: Path,
    strict: bool,
    target: str,
    result_queue: Any,
) -> WorkerState:
    task_queue = ctx.Queue(maxsize=1)
    try:
        process = ctx.Process(
            target=_worker_loop,
            args=(
                worker_id,
                generation,
                str(repo_root),
                strict,
                target,
                task_queue,
                result_queue,
            ),
            daemon=True,
        )
        process.start()
    except Exception:
        try:
            task_queue.close()
        except Exception:
            pass
        raise
    return WorkerState(
        worker_id=worker_id,
        generation=generation,
        process=process,
        task_queue=task_queue,
    )


def _spawn_initial_workers(
    ctx: mp.context.BaseContext,
    *,
    requested_worker_count: int,
    repo_root: Path,
    strict: bool,
    target: str,
    result_queue: Any,
) -> tuple[dict[int, WorkerState], str | None]:
    workers: dict[int, WorkerState] = {}
    for worker_id in range(requested_worker_count):
        try:
            workers[worker_id] = _spawn_worker(
                ctx,
                worker_id=worker_id,
                generation=1,
                repo_root=repo_root,
                strict=strict,
                target=target,
                result_queue=result_queue,
            )
        except OSError as exc:
            if not _is_open_file_limit_error(exc):
                raise
            if not workers:
                raise RuntimeError(
                    "Open-file limit reached before any bulk compile worker could start. "
                    "Lower --concurrency or raise the OS open-file limit."
                ) from exc
            warning = (
                "Open-file limit reached while starting bulk compile workers; "
                f"continuing with {len(workers)} active workers instead of the requested "
                f"{requested_worker_count}."
            )
            return workers, warning
    return workers, None


def _shutdown_worker(worker: WorkerState, *, terminate: bool) -> None:
    if terminate and worker.process.is_alive():
        try:
            worker.process.terminate()
        except Exception:
            pass
    else:
        try:
            worker.task_queue.put_nowait(None)
        except Exception:
            pass

    try:
        worker.process.join(timeout=2.0)
    except Exception:
        pass
    if worker.process.is_alive():
        try:
            worker.process.terminate()
        except Exception:
            pass
        try:
            worker.process.join(timeout=2.0)
        except Exception:
            pass

    try:
        worker.task_queue.close()
    except Exception:
        pass


def _dispatch_candidate(worker: WorkerState, pending: deque[CompileCandidate]) -> None:
    if worker.assigned is not None or not pending:
        return
    candidate = pending.popleft()
    worker.task_queue.put(candidate)
    worker.assigned = candidate
    worker.started_at = None


def _update_progress_description(
    progress: Progress,
    task_id: TaskID,
    *,
    workers: dict[int, WorkerState],
    pending_count: int,
) -> None:
    active_count = sum(1 for worker in workers.values() if worker.assigned is not None)
    progress.update(
        task_id,
        description=f"Compiling records ({active_count} active, {pending_count} queued)",
    )


def _run_compile_pool(
    *,
    repo_root: Path,
    candidates: list[CompileCandidate],
    strict: bool,
    target: str,
    worker_count: int,
    worker_timeout_seconds: float,
    console: Console,
) -> tuple[int, list[tuple[str, str]]]:
    preload_modules = _bulk_compile_preload_modules(candidates)
    start_method = resolve_mp_start_method(prefer_fork=True)
    if start_method == "fork":
        _preload_modules_in_parent(preload_modules)
    ctx = get_mp_context(prefer_fork=True, forkserver_preload=preload_modules)
    try:
        result_queue = ctx.Queue()
    except OSError as exc:
        if not _is_open_file_limit_error(exc):
            raise
        raise RuntimeError(
            "Open-file limit reached while creating bulk compile queues. "
            "Lower --concurrency or raise the OS open-file limit."
        ) from exc
    pending: deque[CompileCandidate] = deque(_sort_candidates_for_compile(candidates))
    compiled = 0
    failures: list[tuple[str, str]] = []
    completed = 0
    workers, startup_warning = _spawn_initial_workers(
        ctx,
        requested_worker_count=worker_count,
        repo_root=repo_root,
        strict=strict,
        target=target,
        result_queue=result_queue,
    )
    if startup_warning is not None:
        console.print(f"[yellow]{startup_warning}[/yellow]")

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        console=console,
    ) as progress:
        task_id = progress.add_task("Compiling records", total=len(candidates))
        for worker in workers.values():
            _dispatch_candidate(worker, pending)
        _update_progress_description(progress, task_id, workers=workers, pending_count=len(pending))

        while completed < len(candidates):
            now = time.monotonic()
            for worker_id, worker in list(workers.items()):
                if not worker.process.is_alive():
                    if worker.assigned is not None:
                        failures.append(
                            (
                                worker.assigned.record_id,
                                f"Bulk compile worker exited unexpectedly while compiling {worker.assigned.record_id}.",
                            )
                        )
                        completed += 1
                        progress.advance(task_id)
                    _shutdown_worker(worker, terminate=False)
                    try:
                        workers[worker_id] = _spawn_worker(
                            ctx,
                            worker_id=worker_id,
                            generation=worker.generation + 1,
                            repo_root=repo_root,
                            strict=strict,
                            target=target,
                            result_queue=result_queue,
                        )
                        _dispatch_candidate(workers[worker_id], pending)
                    except OSError as exc:
                        if not _is_open_file_limit_error(exc):
                            raise
                        workers.pop(worker_id, None)
                        console.print(
                            "[yellow]Open-file limit reached while restarting a bulk compile worker; "
                            f"continuing with {len(workers)} active workers.[/yellow]"
                        )
                        if not workers and pending:
                            raise RuntimeError(
                                "Open-file limit reached while restarting bulk compile workers and "
                                "no active workers remain. Lower --concurrency or raise the OS "
                                "open-file limit."
                            ) from exc

            if worker_timeout_seconds > 0:
                for worker_id, worker in list(workers.items()):
                    if (
                        worker.assigned is not None
                        and worker.started_at is not None
                        and (now - worker.started_at) > worker_timeout_seconds
                    ):
                        failures.append(
                            (
                                worker.assigned.record_id,
                                "Bulk compile worker exceeded the configured timeout "
                                f"({worker_timeout_seconds:.0f}s).",
                            )
                        )
                        completed += 1
                        progress.advance(task_id)
                        _shutdown_worker(worker, terminate=True)
                        try:
                            workers[worker_id] = _spawn_worker(
                                ctx,
                                worker_id=worker_id,
                                generation=worker.generation + 1,
                                repo_root=repo_root,
                                strict=strict,
                                target=target,
                                result_queue=result_queue,
                            )
                            _dispatch_candidate(workers[worker_id], pending)
                        except OSError as exc:
                            if not _is_open_file_limit_error(exc):
                                raise
                            workers.pop(worker_id, None)
                            console.print(
                                "[yellow]Open-file limit reached while restarting a timed-out bulk "
                                f"compile worker; continuing with {len(workers)} active workers.[/yellow]"
                            )
                            if not workers and pending:
                                raise RuntimeError(
                                    "Open-file limit reached while restarting timed-out bulk compile "
                                    "workers and no active workers remain. Lower --concurrency or "
                                    "raise the OS open-file limit."
                                ) from exc

            try:
                event = result_queue.get(timeout=0.25)
            except queue.Empty:
                _update_progress_description(
                    progress,
                    task_id,
                    workers=workers,
                    pending_count=len(pending),
                )
                continue

            if not isinstance(event, WorkerEvent):
                continue
            worker = workers.get(event.worker_id)
            if worker is None or event.generation != worker.generation:
                continue
            if worker.assigned is None:
                continue

            if event.event == "started":
                worker.started_at = time.monotonic()
            elif event.event == "finished":
                candidate = worker.assigned
                if event.error:
                    failures.append((candidate.record_id, event.error))
                elif event.compiled:
                    compiled += 1
                completed += 1
                worker.assigned = None
                worker.started_at = None
                progress.advance(task_id)
                _dispatch_candidate(worker, pending)

            _update_progress_description(
                progress,
                task_id,
                workers=workers,
                pending_count=len(pending),
            )

    for worker in workers.values():
        _shutdown_worker(worker, terminate=False)
    try:
        result_queue.close()
    except Exception:
        pass
    return compiled, failures


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="compile_all_records.py")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root containing data/records.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Recompile every record with a model.py, even if it already has a successful model.urdf.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show which records would be compiled without doing any compile work.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero if any record compile fails.",
    )
    parser.add_argument(
        "--target",
        choices=sorted(_COMPILE_TARGETS),
        default="full",
        help="Compile target to build.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Compile at most this many queued records, after heavy-first sorting.",
    )
    parser.add_argument(
        "--concurrency",
        default="auto",
        help="Worker count for bulk compile: auto, max, or a positive integer.",
    )
    parser.add_argument(
        "--mem-per-worker-gb",
        type=float,
        default=_DEFAULT_MEM_PER_WORKER_GB,
        help="Approximate memory budget used by auto worker sizing.",
    )
    parser.add_argument(
        "--reserve-mem-gb",
        type=float,
        default=_DEFAULT_RESERVE_MEM_GB,
        help="Memory to leave unallocated when using auto worker sizing.",
    )
    parser.add_argument(
        "--worker-timeout-seconds",
        type=float,
        default=0.0,
        help="Optional generous watchdog for each bulk worker task. Set to 0 to disable.",
    )
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    repo_root = args.repo_root.resolve()
    target_key = _normalize_compile_target(args.target)
    console = Console()

    repo = StorageRepo(repo_root)
    scanned = (
        len([path for path in repo.layout.records_root.iterdir() if path.is_dir()])
        if repo.layout.records_root.exists()
        else 0
    )
    candidates, skipped_missing_script = _collect_candidates(
        repo_root,
        force=args.force,
        target=target_key,
    )
    if args.limit is not None:
        if args.limit <= 0:
            parser.error("--limit must be a positive integer.")
        candidates = _limit_candidates(candidates, limit=int(args.limit))

    if not candidates:
        console.print(
            _build_summary_table(
                scanned=scanned,
                skipped_missing_script=skipped_missing_script,
                candidates=0,
                compiled=0,
                failed=0,
                failure_label="Failed",
            )
        )
        console.print("[green]No records need compilation.[/green]")
        return 0

    if args.dry_run:
        table = Table(title="Compile Queue")
        table.add_column("Record ID")
        table.add_column("Reason")
        table.add_column("Mesh Footprint", justify="right")
        for candidate in candidates:
            table.add_row(
                candidate.record_id,
                candidate.reason,
                _format_gib(candidate.estimated_mesh_bytes)
                if candidate.estimated_mesh_bytes
                else "0.0 GiB",
            )
        console.print(table)
        console.print(
            _build_summary_table(
                scanned=scanned,
                skipped_missing_script=skipped_missing_script,
                candidates=len(candidates),
                compiled=0,
                failed=0,
                failure_label="Failed",
            )
        )
        return 0

    effective_mem_per_worker_gb = _effective_mem_per_worker_gb(
        target=target_key,
        requested_mem_per_worker_gb=float(args.mem_per_worker_gb),
    )

    try:
        worker_count = _resolve_worker_count(
            args.concurrency,
            candidate_count=len(candidates),
            reserve_mem_gb=float(args.reserve_mem_gb),
            mem_per_worker_gb=effective_mem_per_worker_gb,
            target=target_key,
        )
    except ValueError as exc:
        parser.error(str(exc))

    console.print(
        _build_concurrency_message(
            requested=args.concurrency,
            resolved_workers=worker_count,
            target=target_key,
            candidate_count=len(candidates),
            reserve_mem_gb=float(args.reserve_mem_gb),
            mem_per_worker_gb=effective_mem_per_worker_gb,
        )
    )
    console.print(
        _build_multiprocessing_message(
            start_method=resolve_mp_start_method(prefer_fork=True),
            preload_modules=_bulk_compile_preload_modules(candidates),
        ),
        markup=False,
    )
    if float(args.worker_timeout_seconds) > 0:
        console.print(
            f"Bulk worker timeout enabled: {float(args.worker_timeout_seconds):.0f}s per record."
        )
    else:
        console.print("Bulk worker timeout disabled.")

    compiled, failures = _run_compile_pool(
        repo_root=repo_root,
        candidates=candidates,
        strict=bool(args.strict),
        target=target_key,
        worker_count=worker_count,
        worker_timeout_seconds=max(float(args.worker_timeout_seconds), 0.0),
        console=console,
    )

    console.print(
        _build_summary_table(
            scanned=scanned,
            skipped_missing_script=skipped_missing_script,
            candidates=len(candidates),
            compiled=compiled,
            failed=len(failures),
            failure_label="Failed",
        )
    )

    if failures:
        failure_table = Table(title="Compile Failures")
        failure_table.add_column("Record ID")
        failure_table.add_column("Error")
        for record_id, error in failures:
            failure_table.add_row(
                record_id,
                _format_bulk_compile_error(record_id, error, strict=args.strict),
            )
        console.print(failure_table)
        console.print("[red]Finished compiling queued records with failures.[/red]")
        return 1

    console.print("[green]Finished compiling queued records.[/green]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
