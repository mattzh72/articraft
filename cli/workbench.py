from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
from pathlib import Path

from agent.cost import max_cost_usd_from_env, parse_max_cost_usd
from agent.prompts import normalize_sdk_package
from agent.runner import create_workbench_draft_record, edit_record, rerun_record_in_place
from agent.tools import resolve_image_path
from articraft.values import PROVIDER_VALUES
from cli.common import (
    add_data_root_argument,
    provider_for_record_image,
    refresh_dataset_manifest_if_member,
    warn_if_post_commit_hook_missing,
)
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


def _add_edit_record_options(
    parser: argparse.ArgumentParser,
    *,
    allow_record_id: bool,
    allow_label: bool,
) -> None:
    parser.add_argument("record", help="Record ID or canonical record directory path.")
    parser.add_argument("prompt", help="Edit request to apply to the existing asset.")
    parser.add_argument("--image", default=None, help="Optional new edit reference image.")
    parser.add_argument("--provider", choices=PROVIDER_VALUES, default=None)
    parser.add_argument("--model-id", default=None)
    parser.add_argument("--thinking-level", default=None)
    parser.add_argument("--max-turns", type=int, default=None)
    parser.add_argument("--max-cost-usd", type=float, default=None)
    parser.add_argument("--sdk-package", default=None, help=argparse.SUPPRESS)
    if allow_record_id:
        parser.add_argument("--record-id", default=None, help="Optional child record ID.")
    if allow_label:
        parser.add_argument("--label", default=None)
        parser.add_argument("--tag", dest="tags", action="append", default=None)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft workbench")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("init-storage", help="Create the canonical data/ directory layout.")
    init_record = subparsers.add_parser(
        "init-record",
        help="Create an empty draft workbench record from a prompt without running generation.",
    )
    init_record.add_argument("prompt", help="Prompt text to store with the draft record.")
    init_record.add_argument(
        "--image",
        default=None,
        help="Optional reference image to store with the draft record.",
    )
    init_record.add_argument(
        "--provider",
        default="openai",
        choices=PROVIDER_VALUES,
        help="LLM provider metadata to attach to the draft record.",
    )
    init_record.add_argument(
        "--model-id",
        default=None,
        help="Model ID metadata to attach. Defaults to the provider's current default for the chosen thinking level.",
    )
    init_record.add_argument(
        "--thinking-level",
        default="high",
        help="Thinking level metadata to attach to the draft provenance.",
    )
    init_record.add_argument(
        "--openai-transport",
        default="http",
        help="OpenAI transport metadata to attach when provider=openai.",
    )
    init_record.add_argument(
        "--openai-reasoning-summary",
        default="auto",
        help="OpenAI reasoning summary metadata to attach when provider=openai.",
    )
    init_record.add_argument(
        "--max-turns",
        type=int,
        default=None,
        help="Max-turns metadata to attach to the draft provenance. Defaults vary by model.",
    )
    init_record.add_argument(
        "--max-cost-usd",
        type=float,
        default=None,
        help="Optional per-run USD budget metadata to attach to the draft provenance.",
    )
    init_record.add_argument(
        "--system-prompt-path",
        default="designer_system_prompt.txt",
        help="System prompt file metadata to attach to the draft provenance.",
    )
    init_record.add_argument(
        "--sdk-package",
        default="sdk",
        help=argparse.SUPPRESS,
    )
    init_record.add_argument(
        "--label",
        default=None,
        help="Optional workbench label and display title override.",
    )
    init_record.add_argument(
        "--tag",
        dest="tags",
        action="append",
        default=None,
        help="Optional workbench tag. Repeat to attach multiple tags.",
    )
    init_record.add_argument(
        "--record-id",
        default=None,
        help="Optional explicit record ID. Defaults to a generated rec_* identifier.",
    )
    subparsers.add_parser("rebuild-search-index", help="Rebuild the cached viewer search index.")
    rerun = subparsers.add_parser(
        "rerun-record",
        help="Re-run an existing record in place using its stored prompt and provenance settings.",
    )
    rerun.add_argument(
        "record",
        help="Record ID or canonical record directory path under data/records/.",
    )
    rerun.add_argument(
        "--model-id",
        default=None,
        help="Optional model override for this rerun. A provider switch is inferred for known model families.",
    )
    rerun.add_argument(
        "--thinking-level",
        default=None,
        help="Optional thinking level override for this rerun.",
    )
    rerun.add_argument(
        "--max-cost-usd",
        type=float,
        default=None,
        help="Optional per-run USD budget override for this rerun.",
    )
    rerun.add_argument(
        "--sdk-package",
        default=None,
        help=argparse.SUPPRESS,
    )
    fork = subparsers.add_parser("fork-record", help="Edit an existing record as a copy.")
    _add_edit_record_options(fork, allow_record_id=True, allow_label=True)

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
        print(f"Initialized workbench storage under {repo.layout.records_root}")
        return 0

    if args.command == "status":
        record_count = len(queries.list_record_ids())
        workbench_entries = (collections.load_workbench() or {}).get("entries", [])
        print(f"records={record_count} workbench_entries={len(workbench_entries)}")
        return 0

    if args.command == "init-record":
        warn_if_post_commit_hook_missing(args.repo_root)
        try:
            image_path = resolve_image_path(args.image, provider=args.provider)
        except Exception as exc:
            print(f"Failed to load image: {exc}")
            return 1
        try:
            max_cost_usd = (
                parse_max_cost_usd(args.max_cost_usd, label="--max-cost-usd")
                if args.max_cost_usd is not None
                else max_cost_usd_from_env()
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        try:
            record_dir = create_workbench_draft_record(
                repo_root=args.repo_root,
                prompt_text=args.prompt,
                image_path=image_path,
                provider=args.provider,
                model_id=args.model_id,
                openai_transport=args.openai_transport,
                thinking_level=args.thinking_level,
                max_turns=args.max_turns,
                max_cost_usd=max_cost_usd,
                system_prompt_path=args.system_prompt_path,
                sdk_package=args.sdk_package,
                openai_reasoning_summary=args.openai_reasoning_summary,
                label=args.label,
                tags=args.tags,
                record_id=args.record_id,
            )
        except ValueError as exc:
            print(str(exc))
            return 1

        stats = search.rebuild()
        print(f"initialized record_id={record_dir.name} record_dir={record_dir}")
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
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
        warn_if_post_commit_hook_missing(args.repo_root)
        try:
            record_id = _resolve_record_reference(repo, args.record)
            sdk_package = (
                normalize_sdk_package(args.sdk_package) if args.sdk_package is not None else None
            )
            max_cost_usd = (
                parse_max_cost_usd(args.max_cost_usd, label="--max-cost-usd")
                if args.max_cost_usd is not None
                else None
            )
        except ValueError as exc:
            print(str(exc))
            return 1

        exit_code = asyncio.run(
            rerun_record_in_place(
                repo_root=args.repo_root,
                record_id=record_id,
                model_id=args.model_id,
                thinking_level=args.thinking_level,
                max_cost_usd=max_cost_usd,
                sdk_package=sdk_package,
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

    if args.command == "fork-record":
        repo.ensure_layout()
        warn_if_post_commit_hook_missing(args.repo_root)
        try:
            record_id = _resolve_record_reference(repo, args.record)
            image_provider = provider_for_record_image(
                repo,
                record_id,
                provider_override=args.provider,
            )
            image_path = resolve_image_path(args.image, provider=image_provider)
            sdk_package = (
                normalize_sdk_package(args.sdk_package) if args.sdk_package is not None else None
            )
            max_cost_usd = (
                parse_max_cost_usd(args.max_cost_usd, label="--max-cost-usd")
                if args.max_cost_usd is not None
                else None
            )
        except Exception as exc:
            print(str(exc))
            return 1

        outcome = asyncio.run(
            edit_record(
                repo_root=args.repo_root,
                parent_record_id=record_id,
                edit_prompt=args.prompt,
                image_path=image_path,
                provider=args.provider,
                model_id=args.model_id,
                thinking_level=args.thinking_level,
                max_turns=args.max_turns,
                max_cost_usd=max_cost_usd,
                sdk_package=sdk_package,
                record_id=getattr(args, "record_id", None),
                label=getattr(args, "label", None),
                tags=list(getattr(args, "tags", None) or []),
            )
        )
        if outcome.exit_code != 0:
            print(outcome.message or "Edit failed.")
            return outcome.exit_code

        record = (
            repo.read_json(repo.layout.record_metadata_path(outcome.record_id), default={}) or {}
        )
        revision_id = record.get("active_revision_id") if isinstance(record, dict) else None
        refresh_dataset_manifest_if_member(repo, outcome.record_id)
        stats = search.rebuild()
        print(
            f"edited mode=fork parent_record_id={record_id} "
            f"record_id={outcome.record_id} revision_id={revision_id or '(unknown)'} "
            f"run_id={outcome.run_id}"
        )
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    parser.error(f"Unhandled command: {args.command}")
    return 2
