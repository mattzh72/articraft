from __future__ import annotations

import argparse
import asyncio
import os
import subprocess
import sys
from pathlib import Path
from shutil import which

from agent import runner as agent_runner
from agent.cost import max_cost_usd_from_env, parse_max_cost_usd
from agent.edit import edit_record
from agent.record_persistence import create_draft_record
from agent.rerun import rerun_record_in_place
from agent.tools import resolve_image_path
from articraft.config import default_model_from_env, default_thinking_level_from_env, load_repo_env
from articraft.values import PROVIDER_VALUES, THINKING_LEVEL_VALUE_SET, THINKING_LEVEL_VALUES
from cli import compile_record as compile_record_cli
from cli import env as env_cli
from cli import external as external_cli
from cli.common import add_data_dir_argument, resolve_data_dir, storage_repo_from_args
from storage.data_validation import validate_data_format
from storage.identifiers import validate_category_slug, validate_record_id
from storage.library_manifest import rebuild_manifest, remove_record, upsert_record
from storage.records import RecordStore
from storage.repo import StorageRepo


def _add_repo_root(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Articraft code repository root.",
    )
    add_data_dir_argument(parser)


def _add_generation_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--provider", choices=PROVIDER_VALUES)
    parser.add_argument("--model", default=None, help="Model ID to use.")
    parser.add_argument(
        "--thinking-level",
        "--thinking",
        default=None,
        choices=THINKING_LEVEL_VALUES,
        help="Thinking budget level.",
    )
    parser.add_argument("--image", default=None, help="Optional reference image.")
    parser.add_argument("--max-cost-usd", type=float, default=None)
    parser.add_argument("--label", default=None)
    parser.add_argument("--tag", dest="tags", action="append", default=None)
    parser.add_argument("--category", default=None, help="Optional category slug.")


def _storage_forward_args(args: argparse.Namespace) -> list[str]:
    argv = ["--repo-root", str(args.repo_root)]
    if getattr(args, "data_dir", None) is not None:
        argv.extend(["--data-dir", str(args.data_dir)])
    return argv


def _model_and_provider(args: argparse.Namespace) -> tuple[str | None, str | None]:
    model = args.model or str(default_model_from_env())
    provider = args.provider
    return model, provider


def _thinking_level(args: argparse.Namespace) -> str:
    thinking_level = str(args.thinking_level or default_thinking_level_from_env())
    if thinking_level not in THINKING_LEVEL_VALUE_SET:
        raise ValueError(
            "ARTICRAFT_THINKING_LEVEL must be one of: " + ", ".join(THINKING_LEVEL_VALUES)
        )
    return thinking_level


def _resolve_record_id(repo: StorageRepo, record_ref: str) -> str:
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
    record_id = validate_record_id(record_ref.strip())
    if not repo.layout.record_metadata_path(record_id).exists():
        raise ValueError(f"Record not found: {record_ref}")
    return record_id


def _run_init(args: argparse.Namespace) -> int:
    env_cli.main([str(args.repo_root)])
    repo = storage_repo_from_args(args)
    repo.ensure_layout()
    rows = rebuild_manifest(repo)
    print(f"Initialized Articraft data at {repo.layout.data_root}")
    print(f"records={len(rows)} manifest={repo.layout.records_manifest_path}")
    return 0


def _run_status(args: argparse.Namespace) -> int:
    repo = storage_repo_from_args(args)
    rows = rebuild_manifest(repo, migrate_metadata=False)
    category_count = (
        len([path for path in repo.layout.categories_root.iterdir() if path.is_dir()])
        if repo.layout.categories_root.exists()
        else 0
    )
    print(f"data_root={repo.layout.data_root}")
    print(f"records={len(rows)} categories={category_count}")
    return 0


def _run_generate(args: argparse.Namespace) -> int:
    try:
        model_id, provider = _model_and_provider(args)
        thinking_level = _thinking_level(args)
    except ValueError as exc:
        print(str(exc))
        return 1
    argv = [
        "--repo-root",
        str(args.repo_root),
        "--prompt",
        args.prompt,
        "--thinking",
        thinking_level,
    ]
    if args.data_dir is not None:
        argv.extend(["--data-dir", str(args.data_dir)])
    if provider:
        argv.extend(["--provider", provider])
    if model_id:
        argv.extend(["--model", model_id])
    if args.image:
        argv.extend(["--image", args.image])
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    if args.label:
        argv.extend(["--label", args.label])
    for tag in args.tags or []:
        argv.extend(["--tag", tag])
    if args.category:
        argv.extend(["--category", args.category])
    return agent_runner.main(argv)


def _run_draft(args: argparse.Namespace) -> int:
    try:
        model_id, provider = _model_and_provider(args)
        thinking_level = _thinking_level(args)
        max_cost_usd = (
            parse_max_cost_usd(args.max_cost_usd, label="--max-cost-usd")
            if args.max_cost_usd is not None
            else max_cost_usd_from_env()
        )
        image_path = (
            resolve_image_path(args.image, provider=provider or "openai") if args.image else None
        )
        record_dir = create_draft_record(
            repo_root=args.repo_root,
            data_root=resolve_data_dir(args.repo_root, args.data_dir),
            prompt_text=args.prompt,
            image_path=image_path,
            provider=provider or "openai",
            model_id=model_id,
            thinking_level=thinking_level,
            max_cost_usd=max_cost_usd,
            label=args.label,
            tags=list(args.tags or []),
            record_id=args.record_id,
        )
    except Exception as exc:
        print(str(exc))
        return 1
    print(f"drafted record_id={record_dir.name} path={record_dir}")
    return 0


def _run_rerun(args: argparse.Namespace) -> int:
    repo = storage_repo_from_args(args)
    try:
        record_id = _resolve_record_id(repo, args.record)
    except ValueError as exc:
        print(str(exc))
        return 1
    return asyncio.run(
        rerun_record_in_place(
            repo_root=args.repo_root,
            data_root=repo.layout.data_root,
            record_id=record_id,
            model_id=args.model,
            thinking_level=args.thinking_level,
            max_cost_usd=args.max_cost_usd,
        )
    )


def _run_fork(args: argparse.Namespace) -> int:
    repo = storage_repo_from_args(args)
    try:
        record_id = _resolve_record_id(repo, args.record)
        image_path = (
            resolve_image_path(args.image, provider=args.provider or "openai")
            if args.image
            else None
        )
    except Exception as exc:
        print(str(exc))
        return 1
    outcome = asyncio.run(
        edit_record(
            repo_root=args.repo_root,
            data_root=repo.layout.data_root,
            parent_record_id=record_id,
            edit_prompt=args.prompt,
            image_path=image_path,
            provider=args.provider,
            model_id=args.model,
            thinking_level=args.thinking_level,
            max_turns=args.max_turns,
            max_cost_usd=args.max_cost_usd,
            record_id=args.record_id,
            label=args.label,
            tags=list(args.tags or []),
        )
    )
    if outcome.message:
        print(outcome.message)
    if outcome.exit_code == 0:
        print(f"forked parent_record_id={record_id} record_id={outcome.record_id}")
    return outcome.exit_code


def _run_compile(args: argparse.Namespace) -> int:
    argv = ["--repo-root", str(args.repo_root), "--target", args.target, args.record]
    if args.data_dir is not None:
        argv[2:2] = ["--data-dir", str(args.data_dir)]
    if args.validate:
        argv.append("--validate")
    if args.strict_geom_qc:
        argv.append("--strict-geom-qc")
    status = compile_record_cli.main(argv)
    if status == 0:
        repo = storage_repo_from_args(args)
        try:
            upsert_record(repo, _resolve_record_id(repo, args.record))
        except Exception:
            pass
    return status


def _viewer_env(args: argparse.Namespace) -> dict[str, str]:
    env = dict(os.environ)
    env["ARTICRAFT_REPO_ROOT"] = str(args.repo_root.resolve())
    if args.data_dir is not None:
        env["ARTICRAFT_DATA_DIR"] = str(resolve_data_dir(args.repo_root, args.data_dir))
    return env


def _viewer_url(args: argparse.Namespace, *, dev_frontend: bool = False) -> str:
    port = "5173" if dev_frontend else str(args.port)
    target = args.target if str(args.target).startswith("/") else f"/{args.target}"
    return f"http://{args.host}:{port}{target}"


def _run_viewer(args: argparse.Namespace) -> int:
    if not which("npm"):
        print("npm is required for viewer/web. Install Node.js and npm first.")
        return 1
    repo = storage_repo_from_args(args)
    rebuild_manifest(repo)
    node_modules = args.repo_root / "viewer" / "web" / "node_modules"
    if not node_modules.is_dir():
        status = subprocess.call(["npm", "--prefix", "viewer/web", "install"], cwd=args.repo_root)
        if status != 0:
            return status
    if args.dev:
        env = _viewer_env(args)
        api = subprocess.Popen(
            [
                "uv",
                "run",
                "uvicorn",
                "viewer.api.app:app",
                "--reload",
                "--host",
                args.host,
                "--port",
                str(args.port),
            ],
            cwd=args.repo_root,
            env=env,
        )
        env["ARTICRAFT_VIEWER_API_HOST"] = args.host
        env["ARTICRAFT_VIEWER_API_PORT"] = str(args.port)
        try:
            print(f"Viewer URL: {_viewer_url(args, dev_frontend=True)}")
            return subprocess.call(
                ["npm", "--prefix", "viewer/web", "run", "dev"],
                cwd=args.repo_root,
                env=env,
            )
        finally:
            api.terminate()
    status = subprocess.call(["npm", "--prefix", "viewer/web", "run", "build"], cwd=args.repo_root)
    if status != 0:
        return status
    print(f"Viewer URL: {_viewer_url(args)}")
    return subprocess.call(
        [
            "uv",
            "run",
            "uvicorn",
            "viewer.api.app:app",
            "--reload",
            "--host",
            args.host,
            "--port",
            str(args.port),
        ],
        cwd=args.repo_root,
        env=_viewer_env(args),
    )


def _run_view(args: argparse.Namespace) -> int:
    repo = storage_repo_from_args(args)
    try:
        record_id = _resolve_record_id(repo, args.record)
    except ValueError as exc:
        print(str(exc))
        return 1
    args.target = f"/viewer?record={record_id}"
    return _run_viewer(args)


def _run_library(args: argparse.Namespace) -> int:
    repo = storage_repo_from_args(args)
    repo.ensure_layout()
    if args.library_command == "status":
        rows = rebuild_manifest(repo, migrate_metadata=False)
        print(f"data_root={repo.layout.data_root}")
        print(f"records={len(rows)} manifest={repo.layout.records_manifest_path}")
        return 0
    if args.library_command == "check":
        result = validate_data_format(
            repo,
            require_records=args.require_records,
            record_ids=list(args.records or []),
        )
        if result.ok:
            print(
                "Library data valid: "
                f"categories={result.category_count} "
                f"records={result.record_count} "
                f"manifest={result.manifest_count}"
            )
            return 0
        for error in result.errors:
            print(error)
        return 1
    if args.library_command == "rebuild-manifest":
        rows = rebuild_manifest(repo)
        print(f"Wrote {repo.layout.records_manifest_path} records={len(rows)}")
        return 0
    if args.library_command == "list":
        rows = rebuild_manifest(repo, migrate_metadata=False)
        for row in rows[: args.limit]:
            print(f"{row['record_id']}\t{row.get('title') or ''}")
        return 0
    if args.library_command == "delete":
        record_id = _resolve_record_id(repo, args.record)
        if not args.execute:
            print(f"Preview only. Re-run with --execute to delete {record_id}.")
            return 0
        if not RecordStore(repo).delete_record(record_id):
            print(f"Record not found: {record_id}")
            return 1
        remove_record(repo, record_id)
        print(f"Deleted {record_id}")
        return 0
    if args.library_command == "set-category":
        record_id = _resolve_record_id(repo, args.record)
        category_slug = validate_category_slug(args.category)
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            print(f"Record not found: {record_id}")
            return 1
        record["category_slug"] = category_slug
        repo.write_json(record_path, record)
        upsert_record(repo, record_id)
        print(f"Updated {record_id} category={category_slug}")
        return 0
    print(f"Unsupported library command: {args.library_command}")
    return 1


def _external(args: argparse.Namespace) -> int:
    external_args = list(args.external_args) or ["--help"]
    return external_cli.main([*_storage_forward_args(args), *external_args])


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft")
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="Initialize local Articraft storage and env.")
    _add_repo_root(init)
    init.set_defaults(func=_run_init)

    status = subparsers.add_parser("status", help="Show local library status.")
    _add_repo_root(status)
    status.set_defaults(func=_run_status)

    generate = subparsers.add_parser("generate", help="Generate a local library record.")
    _add_repo_root(generate)
    generate.add_argument("prompt")
    _add_generation_options(generate)
    generate.set_defaults(func=_run_generate)

    draft = subparsers.add_parser("draft", help="Create a draft local library record.")
    _add_repo_root(draft)
    draft.add_argument("prompt")
    _add_generation_options(draft)
    draft.add_argument("--record-id", default=None)
    draft.set_defaults(func=_run_draft)

    rerun = subparsers.add_parser("rerun", help="Re-run an existing record.")
    _add_repo_root(rerun)
    rerun.add_argument("record")
    rerun.add_argument("--model", default=None)
    rerun.add_argument("--thinking-level", "--thinking", default=None)
    rerun.add_argument("--max-cost-usd", type=float, default=None)
    rerun.set_defaults(func=_run_rerun)

    fork = subparsers.add_parser("fork", help="Edit an existing record as a copy.")
    _add_repo_root(fork)
    fork.add_argument("record")
    fork.add_argument("prompt")
    fork.add_argument("--provider", choices=PROVIDER_VALUES, default=None)
    fork.add_argument("--model", default=None)
    fork.add_argument("--thinking-level", "--thinking", default=None)
    fork.add_argument("--max-turns", type=int, default=None)
    fork.add_argument("--image", default=None)
    fork.add_argument("--max-cost-usd", type=float, default=None)
    fork.add_argument("--record-id", default=None)
    fork.add_argument("--label", default=None)
    fork.add_argument("--tag", dest="tags", action="append", default=None)
    fork.set_defaults(func=_run_fork)

    compile_one = subparsers.add_parser("compile", help="Compile one record.")
    _add_repo_root(compile_one)
    compile_one.add_argument("record")
    compile_one.add_argument("--target", choices=("full", "visual"), default="full")
    compile_one.add_argument("--validate", action="store_true")
    compile_one.add_argument("--strict-geom-qc", action="store_true")
    compile_one.set_defaults(func=_run_compile)

    viewer = subparsers.add_parser("viewer", help="Start the local viewer.")
    _add_repo_root(viewer)
    viewer.add_argument("--dev", action="store_true")
    viewer.add_argument("--host", default="127.0.0.1")
    viewer.add_argument("--port", default="8765")
    viewer.add_argument("--target", default="/")
    viewer.set_defaults(func=_run_viewer)

    view = subparsers.add_parser("view", help="Open the viewer focused on one record.")
    _add_repo_root(view)
    view.add_argument("record")
    view.add_argument("--dev", action="store_true")
    view.add_argument("--host", default="127.0.0.1")
    view.add_argument("--port", default="8765")
    view.set_defaults(func=_run_view)

    library = subparsers.add_parser("library", help="Local library commands.")
    library_sub = library.add_subparsers(dest="library_command", required=True)
    for name, help_text in (
        ("status", "Show manifest status."),
        ("rebuild-manifest", "Rebuild records_manifest.jsonl from records."),
        ("list", "List local records."),
    ):
        child = library_sub.add_parser(name, help=help_text)
        _add_repo_root(child)
        if name == "list":
            child.add_argument("--limit", type=int, default=50)
        child.set_defaults(func=_run_library)
    check = library_sub.add_parser("check", help="Validate local library data.")
    _add_repo_root(check)
    check.add_argument("--require-records", action="store_true")
    check.add_argument("--record", dest="records", action="append", default=[])
    check.set_defaults(func=_run_library)
    delete = library_sub.add_parser("delete", help="Delete a local record.")
    _add_repo_root(delete)
    delete.add_argument("record")
    delete.add_argument("--execute", action="store_true")
    delete.set_defaults(func=_run_library)
    set_category = library_sub.add_parser("set-category", help="Set a record category.")
    _add_repo_root(set_category)
    set_category.add_argument("record")
    set_category.add_argument("category")
    set_category.set_defaults(func=_run_library)

    external = subparsers.add_parser(
        "external", help="External agent authoring commands.", add_help=False
    )
    _add_repo_root(external)
    external.add_argument("external_args", nargs=argparse.REMAINDER)
    external.set_defaults(func=_external)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    load_repo_env(args.repo_root)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
