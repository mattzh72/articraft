from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import queue
import re
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

from agent.mp_utils import get_mp_context
from storage.materialize import infer_materialization_status, record_artifact_paths
from storage.repo import StorageRepo

_GIB = 1024**3
_DEFAULT_MEM_PER_WORKER_GB = 3.0
_DEFAULT_RESERVE_MEM_GB = 2.0
_COMPILE_TARGETS = {"full", "visual"}
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
    return "full"


def _estimate_visual_mesh_footprint(repo: StorageRepo, record_id: str) -> tuple[int, int]:
    mesh_root = repo.layout.record_asset_meshes_dir(record_id)
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
) -> CompileCandidate:
    estimated_mesh_bytes, mesh_file_count = _estimate_visual_mesh_footprint(repo, record_id)
    return CompileCandidate(
        record_id=record_id,
        reason=reason,
        force=force,
        estimated_mesh_bytes=estimated_mesh_bytes,
        mesh_file_count=mesh_file_count,
    )


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
        artifact_paths = record_artifact_paths(repo, record_id, record=record)
        script_path = artifact_paths["model_py"]
        urdf_path = artifact_paths["model_urdf"]
        compile_report_path = artifact_paths["compile_report_json"]
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
                )
            )
            continue

        materialization_status = infer_materialization_status(repo, record_id, record=record)

        if not urdf_path.exists():
            candidates.append(
                _make_candidate(
                    repo,
                    record_id=record_id,
                    reason="missing model.urdf",
                    force=False,
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


def _resolve_worker_count(
    raw_value: str,
    *,
    candidate_count: int,
    reserve_mem_gb: float,
    mem_per_worker_gb: float,
) -> int:
    if candidate_count <= 0:
        return 1

    normalized = str(raw_value).strip().lower()
    cpu_count = _logical_cpu_count()
    if normalized == "max":
        return max(1, min(candidate_count, cpu_count))

    if normalized == "auto":
        return max(1, min(candidate_count, cpu_count))

    try:
        requested = int(normalized)
    except ValueError as exc:
        raise ValueError(
            f"Unsupported concurrency value {raw_value!r}. Expected auto, max, or a positive integer."
        ) from exc

    if requested <= 0:
        raise ValueError("Concurrency must be a positive integer, or one of: auto, max.")
    return min(candidate_count, requested)


def _format_gib(value: int) -> str:
    return f"{value / _GIB:.1f} GiB"


def _build_concurrency_message(
    *,
    requested: str,
    resolved_workers: int,
    reserve_mem_gb: float,
    mem_per_worker_gb: float,
) -> str:
    cpu_count = _logical_cpu_count()
    total_bytes = _total_memory_bytes()
    available_bytes = _available_memory_bytes()
    message = (
        f"Using {resolved_workers} worker"
        f"{'' if resolved_workers == 1 else 's'} "
        f"(`--concurrency={requested}`, {cpu_count} logical CPUs"
    )
    if total_bytes is not None:
        message += f", host memory {_format_gib(total_bytes)}"
    if available_bytes is not None:
        message += f", approx available now {_format_gib(available_bytes)}"
    message += f", reserve {reserve_mem_gb:.1f} GiB, budget {mem_per_worker_gb:.1f} GiB/worker)."
    return message


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
    return WorkerState(
        worker_id=worker_id,
        generation=generation,
        process=process,
        task_queue=task_queue,
    )


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
    ctx = get_mp_context()
    result_queue = ctx.Queue()
    workers: dict[int, WorkerState] = {}
    pending: deque[CompileCandidate] = deque(_sort_candidates_for_compile(candidates))
    compiled = 0
    failures: list[tuple[str, str]] = []
    completed = 0

    for worker_id in range(worker_count):
        workers[worker_id] = _spawn_worker(
            ctx,
            worker_id=worker_id,
            generation=1,
            repo_root=repo_root,
            strict=strict,
            target=target,
            result_queue=result_queue,
        )

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

    try:
        worker_count = _resolve_worker_count(
            args.concurrency,
            candidate_count=len(candidates),
            reserve_mem_gb=float(args.reserve_mem_gb),
            mem_per_worker_gb=float(args.mem_per_worker_gb),
        )
    except ValueError as exc:
        parser.error(str(exc))

    console.print(
        _build_concurrency_message(
            requested=args.concurrency,
            resolved_workers=worker_count,
            reserve_mem_gb=float(args.reserve_mem_gb),
            mem_per_worker_gb=float(args.mem_per_worker_gb),
        )
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
