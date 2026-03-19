from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
from pathlib import Path

from agent.runner import rerun_record_in_place
from cli.common import add_data_root_argument
from storage.collections import CollectionStore
from storage.models import WorkbenchCollection
from storage.queries import StorageQueries
from storage.repo import StorageRepo
from storage.search import SearchIndex


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _resolve_record_reference(repo: StorageRepo, record_ref: str) -> str:
    candidate = Path(record_ref).expanduser()
    if candidate.exists():
        resolved = candidate.resolve()
        records_root = repo.layout.records_root.resolve()
        try:
            relative = resolved.relative_to(records_root)
        except ValueError as exc:
            raise ValueError(f"Record path must be inside {records_root}") from exc
        if len(relative.parts) != 1 or not resolved.is_dir():
            raise ValueError(f"Record path must point to a direct child of {records_root}")
        return relative.parts[0]

    record_id = record_ref.strip()
    if not record_id:
        raise ValueError("Record reference is required.")
    if repo.layout.record_metadata_path(record_id).exists():
        return record_id
    raise ValueError(f"Record not found: {record_ref}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft-workbench")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("init-storage", help="Create the canonical data/ directory layout.")
    subparsers.add_parser("rebuild-search-index", help="Rebuild the cached viewer search index.")
    rerun = subparsers.add_parser(
        "rerun-record",
        help="Re-run an existing record in place using its stored prompt and provenance settings.",
    )
    rerun.add_argument(
        "record",
        help="Record ID or canonical record directory path under data/records/.",
    )
    subparsers.add_parser("status", help="Show workbench storage status.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo = StorageRepo(args.repo_root)
    collections = CollectionStore(repo)
    queries = StorageQueries(repo)
    search = SearchIndex(repo)

    if args.command == "init-storage":
        repo.ensure_layout()
        if collections.load_workbench() is None:
            collections.save_workbench(
                WorkbenchCollection(
                    schema_version=1,
                    collection="workbench",
                    updated_at=_utc_now(),
                )
            )
        print(f"Initialized workbench storage at {repo.layout.local_workbench_path()}")
        return 0

    if args.command == "status":
        record_count = len(queries.list_record_ids())
        workbench_entries = (collections.load_workbench() or {}).get("entries", [])
        print(f"records={record_count} workbench_entries={len(workbench_entries)}")
        return 0

    if args.command == "rebuild-search-index":
        repo.ensure_layout()
        stats = search.rebuild()
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    if args.command == "rerun-record":
        repo.ensure_layout()
        try:
            record_id = _resolve_record_reference(repo, args.record)
        except ValueError as exc:
            print(str(exc))
            return 1

        exit_code = asyncio.run(
            rerun_record_in_place(
                repo_root=args.repo_root,
                record_id=record_id,
            )
        )
        if exit_code != 0:
            return exit_code

        record = repo.read_json(repo.layout.record_metadata_path(record_id), default={}) or {}
        source = record.get("source") if isinstance(record.get("source"), dict) else {}
        stats = search.rebuild()
        dataset_entry = repo.read_json(repo.layout.record_dataset_entry_path(record_id))
        if isinstance(dataset_entry, dict):
            print(
                f"reran record_id={record_id} run_id={source.get('run_id') or '(unknown)'} "
                f"dataset_id={dataset_entry.get('dataset_id') or '(none)'}"
            )
        else:
            print(f"reran record_id={record_id} run_id={source.get('run_id') or '(unknown)'}")
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    parser.error(f"Unhandled command: {args.command}")
    return 2
