from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent.runner import create_workbench_draft_record
from cli import compile_record as compile_record_cli
from cli.common import add_data_root_argument, warn_if_post_commit_hook_missing
from storage.categories import CategoryStore
from storage.collections import CollectionStore
from storage.dataset_workflow import promote_record_workflow
from storage.datasets import DatasetStore
from storage.queries import StorageQueries
from storage.repo import StorageRepo
from storage.search import SearchIndex

ALLOWED_EXTERNAL_AGENTS = ("codex", "claude-code")
DEFAULT_PROVIDER_BY_AGENT = {
    "codex": "openai",
    "claude-code": "anthropic",
}


def _add_repo_root_override(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=argparse.SUPPRESS,
        help="Repository root containing the data/ directory.",
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def _external_creator(record: dict[str, Any]) -> dict[str, Any] | None:
    creator = record.get("creator")
    return creator if isinstance(creator, dict) else None


def _validate_external_record(repo: StorageRepo, record_id: str) -> list[str]:
    errors: list[str] = []
    record_path = repo.layout.record_metadata_path(record_id)
    record = repo.read_json(record_path)
    if not isinstance(record, dict):
        return [f"Missing record metadata: {record_path}"]

    creator = _external_creator(record)
    if creator is None:
        errors.append("record.json missing creator metadata for external workflow")
    else:
        if creator.get("mode") != "external_agent":
            errors.append("creator.mode must be 'external_agent'")
        if creator.get("agent") not in ALLOWED_EXTERNAL_AGENTS:
            errors.append("creator.agent must be 'codex' or 'claude-code'")
        if creator.get("trace_available") is not False:
            errors.append("creator.trace_available must be false for external records")

    artifacts = record.get("artifacts")
    if not isinstance(artifacts, dict):
        errors.append("record.json artifacts must be an object")
    else:
        for key in ("prompt_txt", "model_py", "provenance_json"):
            value = artifacts.get(key)
            if not isinstance(value, str) or not value.strip():
                errors.append(f"artifacts.{key} must be set")
                continue
            if not (repo.layout.record_dir(record_id) / value).exists():
                errors.append(f"artifacts.{key} references missing path {value!r}")

    compile_report = repo.layout.record_materialization_compile_report_path(record_id)
    if not compile_report.exists():
        errors.append(f"missing compile report: {compile_report}")

    traces_dir = repo.layout.record_traces_dir(record_id)
    if traces_dir.exists() and any(traces_dir.iterdir()):
        errors.append("external records must not contain Articraft agent traces")

    return errors


def _refresh_external_record(repo: StorageRepo, record_id: str, *, final_status: str) -> None:
    record_path = repo.layout.record_metadata_path(record_id)
    provenance_path = repo.layout.record_dir(record_id) / "provenance.json"
    prompt_path = repo.layout.record_dir(record_id) / "prompt.txt"
    model_path = repo.layout.record_dir(record_id) / "model.py"
    now = _utc_now()

    record = repo.read_json(record_path)
    if not isinstance(record, dict):
        raise ValueError(f"Missing record metadata: {record_path}")
    provenance = repo.read_json(provenance_path)
    if not isinstance(provenance, dict):
        raise ValueError(f"Missing provenance: {provenance_path}")

    hashes = record.get("hashes")
    if not isinstance(hashes, dict):
        hashes = {}
    hashes["prompt_sha256"] = _sha256_file(prompt_path) if prompt_path.exists() else None
    hashes["model_py_sha256"] = _sha256_file(model_path) if model_path.exists() else None
    record["hashes"] = hashes
    record["updated_at"] = now
    if record.get("kind") == "draft_model":
        record["kind"] = "external_model"

    run_summary = provenance.get("run_summary")
    if not isinstance(run_summary, dict):
        run_summary = {}
    run_summary["final_status"] = final_status
    provenance["run_summary"] = run_summary

    repo.write_json(record_path, record)
    repo.write_json(provenance_path, provenance)


def _compile_record(repo_root: Path, record_dir: Path, *, target: str, validate: bool) -> int:
    argv = [
        "--repo-root",
        str(repo_root),
        "--target",
        target,
        str(record_dir),
    ]
    if validate:
        argv.append("--validate")
        argv.append("--strict-geom-qc")
    return compile_record_cli.main(argv)


def _coerce_rating(value: Any) -> int | None:
    if value is None:
        return None
    if isinstance(value, int) and 1 <= value <= 5:
        return value
    return None


def _text_matches_query(*, query: str | None, values: list[str]) -> bool:
    if not query:
        return True
    tokens = [token for token in query.lower().split() if token]
    if not tokens:
        return True
    haystack = " ".join(values).lower()
    return all(token in haystack for token in tokens)


def _record_prompt_text(repo: StorageRepo, record_id: str, record: dict[str, Any]) -> str:
    artifacts = record.get("artifacts") if isinstance(record.get("artifacts"), dict) else {}
    prompt_name = artifacts.get("prompt_txt") if isinstance(artifacts, dict) else None
    prompt_path = repo.layout.record_dir(record_id) / str(prompt_name or "prompt.txt")
    if not prompt_path.exists():
        return ""
    try:
        return prompt_path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _print_examples(
    repo: StorageRepo,
    *,
    query: str | None,
    category_slug: str | None,
    rating_min: int,
    limit: int,
) -> None:
    matches: list[tuple[int, str, dict[str, Any], str]] = []
    for record_id in StorageQueries(repo).list_record_ids():
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            continue
        rating = _coerce_rating(record.get("rating"))
        if rating is None or rating < rating_min:
            continue
        if category_slug and record.get("category_slug") != category_slug:
            continue
        display = record.get("display") if isinstance(record.get("display"), dict) else {}
        prompt_text = _record_prompt_text(repo, record_id, record)
        if not _text_matches_query(
            query=query,
            values=[
                record_id,
                str(display.get("title") or ""),
                str(display.get("prompt_preview") or ""),
                str(record.get("category_slug") or ""),
                prompt_text,
            ],
        ):
            continue
        matches.append((rating, record_id, record, prompt_text))

    matches.sort(key=lambda item: (-item[0], item[1]))
    limited = matches[:limit]
    print(f"example_count={len(limited)} total_matches={len(matches)}")
    if not limited:
        print("examples=(none)")
        return
    for rating, record_id, record, prompt_text in limited:
        display = record.get("display") if isinstance(record.get("display"), dict) else {}
        model_name = (
            record.get("artifacts", {}).get("model_py")
            if isinstance(record.get("artifacts"), dict)
            else None
        ) or "model.py"
        print(f"record_id={record_id}")
        print(f"rating={rating}")
        print(f"title={display.get('title') or record_id}")
        print(f"category_slug={record.get('category_slug') or ''}")
        print(f"prompt={prompt_text or display.get('prompt_preview') or ''}")
        print(f"model={repo.layout.record_dir(record_id) / str(model_name)}")
        print("")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft external")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="Create an external-agent workbench record.")
    _add_repo_root_override(init)
    init.add_argument("prompt", help="Prompt text to store with the record.")
    init.add_argument("--agent", required=True, choices=ALLOWED_EXTERNAL_AGENTS)
    init.add_argument(
        "--model-id",
        default=None,
        help="Optional model ID metadata for the external harness run.",
    )
    init.add_argument(
        "--thinking-level",
        default=None,
        help="Optional thinking/reasoning level metadata for the external harness run.",
    )
    init.add_argument("--label", default=None)
    init.add_argument("--tag", dest="tags", action="append", default=None)
    init.add_argument("--record-id", default=None)

    compile_one = subparsers.add_parser("compile", help="Compile an external-agent record.")
    _add_repo_root_override(compile_one)
    compile_one.add_argument("record")
    compile_one.add_argument("--target", choices=("full", "visual"), default="full")
    compile_one.add_argument("--validate", action="store_true")

    check = subparsers.add_parser("check", help="Compile and validate an external-agent record.")
    _add_repo_root_override(check)
    check.add_argument("record")

    categories = subparsers.add_parser(
        "categories", help="List dataset categories for external agents."
    )
    _add_repo_root_override(categories)

    examples = subparsers.add_parser(
        "examples", help="Find high-rated existing records to use as authoring references."
    )
    _add_repo_root_override(examples)
    examples.add_argument("--query", default=None)
    examples.add_argument("--category-slug", default=None)
    examples.add_argument("--rating-min", type=int, default=5)
    examples.add_argument("--limit", type=int, default=8)

    finalize = subparsers.add_parser(
        "finalize",
        help="Finalize an external-agent record, optionally promoting it to a dataset category.",
    )
    _add_repo_root_override(finalize)
    finalize.add_argument("record")
    finalize.add_argument("--category-slug", default=None)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    repo = StorageRepo(args.repo_root)
    repo.ensure_layout()

    if args.command == "init":
        warn_if_post_commit_hook_missing(args.repo_root)
        provider = DEFAULT_PROVIDER_BY_AGENT[args.agent]
        try:
            record_dir = create_workbench_draft_record(
                repo_root=args.repo_root,
                prompt_text=args.prompt,
                provider=provider,
                model_id=args.model_id,
                thinking_level=args.thinking_level,
                sdk_package="sdk",
                label=args.label,
                tags=args.tags,
                record_id=args.record_id,
                external_agent=args.agent,
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        stats = SearchIndex(repo).rebuild()
        print(f"initialized external record_id={record_dir.name} record_dir={record_dir}")
        print(
            f"agent={args.agent} provider={provider} "
            f"model_id={args.model_id or ''} "
            f"thinking_level={args.thinking_level or ''} "
            "trace_available=false"
        )
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    if args.command == "categories":
        queries = StorageQueries(repo)
        categories = CategoryStore(repo)
        slugs = queries.list_category_slugs()
        print(f"category_count={len(slugs)}")
        if not slugs:
            print("categories=(none)")
            return 0
        for slug in slugs:
            category = categories.load(slug) or {}
            title = str(category.get("title") or slug.replace("_", " ").title())
            count = len(queries.list_record_ids_for_category(slug))
            print(f"{slug}\ttitle={title}\trecords={count}")
        return 0

    if args.command == "examples":
        if args.rating_min < 1 or args.rating_min > 5:
            print("--rating-min must be between 1 and 5")
            return 1
        if args.limit <= 0:
            print("--limit must be a positive integer")
            return 1
        _print_examples(
            repo,
            query=args.query,
            category_slug=str(args.category_slug or "").strip() or None,
            rating_min=args.rating_min,
            limit=args.limit,
        )
        return 0

    try:
        record_id = _resolve_record_reference(repo, args.record)
    except ValueError as exc:
        print(str(exc))
        return 1
    record_dir = repo.layout.record_dir(record_id)

    if args.command == "compile":
        return _compile_record(
            args.repo_root,
            record_dir,
            target=args.target,
            validate=bool(args.validate),
        )

    if args.command == "check":
        status = _compile_record(args.repo_root, record_dir, target="full", validate=True)
        if status != 0:
            return status
        errors = _validate_external_record(repo, record_id)
        if errors:
            for error in errors:
                print(error)
            return 1
        print(f"external record check passed: {record_id}")
        return 0

    if args.command == "finalize":
        status = _compile_record(args.repo_root, record_dir, target="full", validate=True)
        if status != 0:
            return status
        errors = _validate_external_record(repo, record_id)
        if errors:
            for error in errors:
                print(error)
            return 1
        category_slug = str(args.category_slug or "").strip() or None
        _refresh_external_record(
            repo,
            record_id,
            final_status="external_finalized" if category_slug else "external_ready",
        )
        if category_slug is None:
            stats = SearchIndex(repo).rebuild()
            print(f"finalized external workbench record_id={record_id}")
            print(
                f"search_index={stats.path} "
                f"records={stats.record_count} "
                f"categories={stats.category_count} "
                f"workbench_entries={stats.workbench_entry_count}"
            )
            return 0

        try:
            entry, category, _, stats = promote_record_workflow(
                repo,
                DatasetStore(repo),
                StorageQueries(repo),
                record_id=record_id,
                category_title=None,
                category_slug=category_slug,
                dataset_id=None,
                promoted_at=_utc_now(),
            )
        except ValueError as exc:
            print(str(exc))
            return 1
        CollectionStore(repo).ensure_workbench()
        print(
            f"finalized external dataset record_id={record_id} "
            f"dataset_id={entry.get('dataset_id')} "
            f"category_slug={category.get('slug') or category_slug}"
        )
        print(
            f"search_index={stats.path} "
            f"records={stats.record_count} "
            f"categories={stats.category_count} "
            f"workbench_entries={stats.workbench_entry_count}"
        )
        return 0

    parser.error(f"Unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
