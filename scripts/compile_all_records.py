from __future__ import annotations

import argparse
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

from storage.repo import StorageRepo
from viewer.api.store import ViewerStore


@dataclass(slots=True, frozen=True)
class CompileCandidate:
    record_id: str
    reason: str
    force: bool = False


def _record_artifact_path(record_dir: Path, record: dict | None, key: str, default: str) -> Path:
    artifacts = record.get("artifacts") if isinstance(record, dict) else None
    if isinstance(artifacts, dict):
        value = artifacts.get(key)
        if isinstance(value, str) and value.strip():
            return record_dir / value
    return record_dir / default


def _record_assets_available(repo: StorageRepo, record_id: str) -> bool:
    assets_root = repo.layout.record_assets_dir(record_id)
    for child_name in ("meshes", "glb", "viewer"):
        child = assets_root / child_name
        if child.exists() and any(child.iterdir()):
            return True
    return False


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
        script_path = _record_artifact_path(record_dir, record, "model_py", "model.py")
        urdf_path = _record_artifact_path(record_dir, record, "model_urdf", "model.urdf")
        compile_report_path = _record_artifact_path(
            record_dir,
            record,
            "compile_report_json",
            "compile_report.json",
        )

        if not script_path.exists():
            skipped_missing_script += 1
            continue

        if force:
            candidates.append(CompileCandidate(record_id=record_id, reason="forced", force=True))
            continue

        compile_report = repo.read_json(compile_report_path)
        compile_status = (
            str(compile_report.get("status"))
            if isinstance(compile_report, dict) and isinstance(compile_report.get("status"), str)
            else None
        )
        assets_available = _record_assets_available(repo, record_id)

        if not urdf_path.exists():
            candidates.append(
                CompileCandidate(record_id=record_id, reason="missing model.urdf", force=False)
            )
            continue

        if not assets_available:
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
) -> Table:
    table = Table(title="Compile Summary")
    table.add_column("Metric")
    table.add_column("Count", justify="right")
    table.add_row("Scanned records", str(scanned))
    table.add_row("Missing model.py", str(skipped_missing_script))
    table.add_row("Queued for compile", str(candidates))
    table.add_row("Compiled", str(compiled))
    table.add_row("Failed", str(failed))
    table.add_row("Already up to date", str(max(scanned - skipped_missing_script - candidates, 0)))
    return table


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
                    candidate.record_id, force=candidate.force
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
        )
    )

    if failures:
        failure_table = Table(title="Compile Failures")
        failure_table.add_column("Record ID")
        failure_table.add_column("Error")
        for record_id, error in failures:
            failure_table.add_row(record_id, error)
        console.print(failure_table)
        return 1

    console.print("[green]Finished compiling queued records.[/green]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
