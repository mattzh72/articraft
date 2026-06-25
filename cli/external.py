from __future__ import annotations

import argparse
from pathlib import Path

from agent.record_persistence import create_draft_record
from cli.common import add_data_root_argument, storage_repo_from_args
from storage.identifiers import validate_category_slug, validate_record_id
from storage.library_manifest import upsert_record


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft external")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="Create an external-agent draft record.")
    init.add_argument("prompt")
    init.add_argument("--agent", choices=("codex", "claude-code", "cursor"), default="codex")
    init.add_argument("--record-id", default=None)
    init.add_argument("--label", default=None)
    init.add_argument("--tag", dest="tags", action="append", default=None)

    finalize = subparsers.add_parser("finalize", help="Finalize a local external-agent record.")
    finalize.add_argument("record")
    finalize.add_argument("--category-slug", default=None)

    subparsers.add_parser("categories", help="List local categories.")
    return parser


def _resolve_record_id(repo, record_ref: str) -> str:
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
        return validate_record_id(relative.parts[0])
    return validate_record_id(record_ref)


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    repo = storage_repo_from_args(args)
    repo.ensure_layout()

    if args.command == "init":
        try:
            record_dir = create_draft_record(
                repo_root=args.repo_root,
                data_root=repo.layout.data_root,
                prompt_text=args.prompt,
                record_id=args.record_id,
                label=args.label,
                tags=list(args.tags or []),
                external_agent=args.agent,
            )
        except Exception as exc:
            print(str(exc))
            return 1
        print(f"record_id={record_dir.name}")
        print(f"record_dir={record_dir}")
        return 0

    if args.command == "finalize":
        try:
            record_id = _resolve_record_id(repo, args.record)
            record_path = repo.layout.record_metadata_path(record_id)
            record = repo.read_json(record_path)
            if not isinstance(record, dict):
                raise ValueError(f"Record not found: {record_id}")
            if args.category_slug:
                record["category_slug"] = validate_category_slug(args.category_slug)
                repo.write_json(record_path, record)
            upsert_record(repo, record_id)
        except Exception as exc:
            print(str(exc))
            return 1
        print(f"finalized record_id={record_id}")
        return 0

    if args.command == "categories":
        root = repo.layout.categories_root
        if root.exists():
            for path in sorted(item for item in root.iterdir() if item.is_dir()):
                print(path.name)
        return 0

    parser.error(f"Unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
