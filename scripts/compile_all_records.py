from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

from rich.console import Console
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    SpinnerColumn,
    TextColumn,
    TimeElapsedColumn,
)
from rich.table import Table

from storage.materialize import infer_materialization_status, record_artifact_paths
from storage.repo import StorageRepo
from viewer.api.store import ViewerStore


@dataclass(slots=True, frozen=True)
class CompileCandidate:
    record_id: str
    reason: str
    force: bool = False


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


def _collect_candidates(repo_root: Path, *, force: bool) -> tuple[list[CompileCandidate], int]:
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
        compile_status = (
            str(compile_report.get("status"))
            if isinstance(compile_report, dict) and isinstance(compile_report.get("status"), str)
            else None
        )

        if not script_path.exists():
            if isinstance(record, dict) or compile_status is not None or urdf_path.exists():
                candidates.append(
                    CompileCandidate(record_id=record_id, reason="missing model.py", force=False)
                )
                continue
            skipped_missing_script += 1
            continue

        if force:
            candidates.append(CompileCandidate(record_id=record_id, reason="forced", force=True))
            continue

        materialization_status = infer_materialization_status(repo, record_id, record=record)

        if not urdf_path.exists():
            candidates.append(
                CompileCandidate(record_id=record_id, reason="missing model.urdf", force=False)
            )
            continue

        if materialization_status == "missing":
            candidates.append(
                CompileCandidate(
                    record_id=record_id,
                    reason="missing generated assets",
                    force=False,
                )
            )
            continue

        if compile_status != "success":
            label = compile_status or "unknown"
            candidates.append(
                CompileCandidate(
                    record_id=record_id,
                    reason=f"compile status is {label}",
                    force=True,
                )
            )

    return candidates, skipped_missing_script


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
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    repo_root = args.repo_root.resolve()
    console = Console()

    repo = StorageRepo(repo_root)
    scanned = (
        len([path for path in repo.layout.records_root.iterdir() if path.is_dir()])
        if repo.layout.records_root.exists()
        else 0
    )
    candidates, skipped_missing_script = _collect_candidates(repo_root, force=args.force)

    if not candidates:
        console.print(
            _build_summary_table(
                scanned=scanned,
                skipped_missing_script=skipped_missing_script,
                candidates=0,
                compiled=0,
                failed=0,
                failure_label="Failed" if args.strict else "Non-blocking issues",
            )
        )
        console.print("[green]No records need compilation.[/green]")
        return 0

    if args.dry_run:
        table = Table(title="Compile Queue")
        table.add_column("Record ID")
        table.add_column("Reason")
        for candidate in candidates:
            table.add_row(candidate.record_id, candidate.reason)
        console.print(table)
        console.print(
            _build_summary_table(
                scanned=scanned,
                skipped_missing_script=skipped_missing_script,
                candidates=len(candidates),
                compiled=0,
                failed=0,
                failure_label="Failed" if args.strict else "Non-blocking issues",
            )
        )
        return 0

    viewer_store = ViewerStore(repo_root)
    compiled = 0
    failures: list[tuple[str, str]] = []

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        console=console,
    ) as progress:
        task_id = progress.add_task("Compiling records", total=len(candidates))
        for candidate in candidates:
            progress.update(
                task_id,
                description=f"Compiling {candidate.record_id} ({candidate.reason})",
            )
            try:
                result = viewer_store.materialize_record_assets(
                    candidate.record_id,
                    force=candidate.force,
                    ignore_geom_qc=not args.strict,
                )
            except Exception as exc:
                failures.append((candidate.record_id, str(exc)))
            else:
                if result.compiled:
                    compiled += 1
            finally:
                progress.advance(task_id)

    console.print(
        _build_summary_table(
            scanned=scanned,
            skipped_missing_script=skipped_missing_script,
            candidates=len(candidates),
            compiled=compiled,
            failed=len(failures),
            failure_label="Failed" if args.strict else "Non-blocking issues",
        )
    )

    if failures:
        reported_failures: list[tuple[str, str]] = []
        suppressed_issue_count = 0
        for record_id, error in failures:
            if _should_suppress_bulk_compile_error(record_id, error, strict=args.strict):
                suppressed_issue_count += 1
                continue
            reported_failures.append((record_id, error))

        if reported_failures:
            failure_table = Table(
                title="Compile Failures" if args.strict else "Compile Issues (non-blocking)"
            )
            failure_table.add_column("Record ID")
            failure_table.add_column("Error")
            for record_id, error in reported_failures:
                failure_table.add_row(
                    record_id,
                    _format_bulk_compile_error(record_id, error, strict=args.strict),
                )
            console.print(failure_table)
        if suppressed_issue_count:
            console.print(
                "[yellow]Suppressed detailed output for "
                f"{suppressed_issue_count} non-blocking geometry QC issue(s).[/yellow]"
            )
        if args.strict:
            return 1
        console.print(
            "[yellow]Finished compiling queued records with non-blocking failures.[/yellow]"
        )
        return 0

    console.print("[green]Finished compiling queued records.[/green]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
