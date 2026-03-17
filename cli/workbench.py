from __future__ import annotations

import argparse
from datetime import datetime, timezone

from storage.collections import CollectionStore
from storage.models import WorkbenchCollection
from storage.queries import StorageQueries
from storage.repo import StorageRepo

from cli.common import add_data_root_argument


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft-workbench")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("init-storage", help="Create the canonical data/ directory layout.")
    subparsers.add_parser("status", help="Show workbench storage status.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo = StorageRepo(args.repo_root)
    collections = CollectionStore(repo)
    queries = StorageQueries(repo)

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
        print(f"Initialized workbench storage at {repo.layout.data_root}")
        return 0

    if args.command == "status":
        record_count = len(queries.list_record_ids())
        workbench_entries = (collections.load_workbench() or {}).get("entries", [])
        print(f"records={record_count} workbench_entries={len(workbench_entries)}")
        return 0

    parser.error(f"Unhandled command: {args.command}")
    return 2
