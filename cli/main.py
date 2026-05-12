from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from agent import runner as agent_runner
from cli import compile_all as compile_all_cli
from cli import compile_record as compile_record_cli
from cli import dataset as dataset_cli
from cli import env as env_cli
from cli import hooks as hooks_cli
from cli import pre_commit as pre_commit_cli
from cli import workbench as workbench_cli
from storage.repo import StorageRepo

DEFAULT_MODEL = "gpt-5.5-2026-04-23"
DEFAULT_THINKING = "high"


def _infer_provider(model_id: str) -> str:
    if model_id.startswith(("gpt-", "o1", "o3", "o4")):
        return "openai"
    if model_id.startswith("gemini-"):
        return "gemini"
    if "/" in model_id or model_id.startswith("openrouter/"):
        return "openrouter"
    raise ValueError(
        f"Unable to infer provider for model '{model_id}'. "
        "Pass --provider explicitly or use a known OpenAI, Gemini, or OpenRouter model ID."
    )


def _model_and_provider(args: argparse.Namespace) -> tuple[str, str]:
    model_id = str(args.model or DEFAULT_MODEL)
    provider = str(args.provider or _infer_provider(model_id))
    return model_id, provider


def _dataset(args: argparse.Namespace, argv: list[str]) -> int:
    return dataset_cli.main(["--repo-root", str(args.repo_root), *argv])


def _workbench(args: argparse.Namespace, argv: list[str]) -> int:
    return workbench_cli.main(["--repo-root", str(args.repo_root), *argv])


def _run_init(args: argparse.Namespace) -> int:
    env_cli.main([str(args.repo_root)])
    dataset_status = _dataset(args, ["init-storage"])
    if dataset_status != 0:
        return dataset_status
    workbench_status = _workbench(args, ["init-storage"])
    if workbench_status != 0:
        return workbench_status
    check_status = _dataset(args, ["validate-format"])
    if check_status != 0:
        return check_status
    return _dataset(args, ["build-manifest"])


def _run_status(args: argparse.Namespace) -> int:
    dataset_status = _dataset(args, ["status"])
    if dataset_status != 0:
        return dataset_status
    return _workbench(args, ["status"])


def _run_generate(args: argparse.Namespace) -> int:
    try:
        model_id, provider = _model_and_provider(args)
    except ValueError as exc:
        print(str(exc))
        return 1
    argv = [
        "--repo-root",
        str(args.repo_root),
        "--prompt",
        args.prompt,
        "--provider",
        provider,
        "--model",
        model_id,
        "--thinking",
        args.thinking_level,
    ]
    if args.image:
        argv.extend(["--image", args.image])
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    return agent_runner.main(argv)


def _run_draft(args: argparse.Namespace) -> int:
    try:
        model_id, provider = _model_and_provider(args)
    except ValueError as exc:
        print(str(exc))
        return 1
    argv = [
        "init-record",
        args.prompt,
        "--provider",
        provider,
        "--model-id",
        model_id,
        "--thinking-level",
        args.thinking_level,
    ]
    if args.image:
        argv.extend(["--image", args.image])
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    if args.label:
        argv.extend(["--label", args.label])
    for tag in args.tags or []:
        argv.extend(["--tag", tag])
    if args.record_id:
        argv.extend(["--record-id", args.record_id])
    return _workbench(args, argv)


def _run_rerun(args: argparse.Namespace) -> int:
    argv = ["rerun-record", args.record]
    if args.model:
        argv.extend(["--model-id", args.model])
    if args.thinking_level:
        argv.extend(["--thinking-level", args.thinking_level])
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    return _workbench(args, argv)


def _run_compile(args: argparse.Namespace) -> int:
    argv = [
        "--repo-root",
        str(args.repo_root),
        "--target",
        args.target,
        args.record,
    ]
    if args.validate:
        argv.append("--validate")
    if args.strict_geom_qc:
        argv.append("--strict-geom-qc")
    return compile_record_cli.main(argv)


def _run_compile_all(args: argparse.Namespace) -> int:
    argv = [
        "--repo-root",
        str(args.repo_root),
        "--target",
        args.target,
        "--concurrency",
        args.concurrency,
    ]
    if args.force:
        argv.append("--force")
    if args.strict:
        argv.append("--strict")
    if args.limit is not None:
        argv.extend(["--limit", str(args.limit)])
    if args.dry_run:
        argv.append("--dry-run")
    if args.verify_assets:
        argv.append("--verify-assets")
    return compile_all_cli.main(argv)


def _resolve_record_id(repo_root: Path, record_ref: str) -> str:
    repo = StorageRepo(repo_root)
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


def _ensure_viewer_inputs(args: argparse.Namespace) -> int:
    status = _dataset(args, ["validate-format"])
    if status != 0:
        return status
    status = _dataset(args, ["build-manifest"])
    if status != 0:
        return status
    return _workbench(args, ["rebuild-search-index"])


def _normalize_viewer_target(target: str) -> str:
    normalized = str(target or "/").strip() or "/"
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    return normalized


def _viewer_url(args: argparse.Namespace, *, dev_frontend: bool = False) -> str:
    port = "5173" if dev_frontend else str(args.port)
    return f"http://{args.host}:{port}{_normalize_viewer_target(args.target)}"


def _run_viewer(args: argparse.Namespace) -> int:
    if not shutil_which("npm"):
        print("npm is required for viewer/web. Install Node.js and npm first.")
        return 1
    node_modules = args.repo_root / "viewer" / "web" / "node_modules"
    if not node_modules.is_dir():
        status = subprocess.call(["npm", "--prefix", "viewer/web", "install"], cwd=args.repo_root)
        if status != 0:
            return status
    status = _ensure_viewer_inputs(args)
    if status != 0:
        return status
    if args.dev:
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
        )
        env = dict(os.environ)
        env["ARTICRAFT_VIEWER_API_HOST"] = args.host
        env["ARTICRAFT_VIEWER_API_PORT"] = str(args.port)
        try:
            print(f"Viewer URL: {_viewer_url(args, dev_frontend=True)}")
            return subprocess.call(
                ["npm", "--prefix", "viewer/web", "run", "dev"], cwd=args.repo_root, env=env
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
    )


def shutil_which(command: str) -> str | None:
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        path = Path(directory) / command
        if path.exists() and os.access(path, os.X_OK):
            return str(path)
    return None


def _run_view(args: argparse.Namespace) -> int:
    try:
        record_id = _resolve_record_id(args.repo_root, args.record)
    except ValueError as exc:
        print(str(exc))
        return 1
    args.target = f"/viewer?record={record_id}"
    return _run_viewer(args)


def _run_dataset_batch_new(args: argparse.Namespace) -> int:
    batch_name = args.name.strip()
    if not batch_name:
        print("Batch name is required.")
        return 1
    spec_path = args.repo_root / "data" / "batch_specs" / f"{batch_name}.csv"
    if spec_path.exists():
        print(f"Batch spec already exists: {spec_path}")
        return 1
    spec_path.parent.mkdir(parents=True, exist_ok=True)
    spec_path.write_text(
        "row_id,category_slug,category_title,prompt,provider,model_id,"
        "thinking_level,max_turns,max_cost_usd,label\n",
        encoding="utf-8",
    )
    print(f"Created {spec_path}")
    return 0


def _run_dataset_run(args: argparse.Namespace) -> int:
    try:
        model_id, provider = _model_and_provider(args)
    except ValueError as exc:
        print(str(exc))
        return 1
    argv = [
        "run-single",
        args.prompt,
        "--category-slug",
        args.category_slug,
        "--provider",
        provider,
        "--model-id",
        model_id,
        "--thinking-level",
        args.thinking_level,
    ]
    if args.image:
        argv.extend(["--image", args.image])
    if args.dataset_id:
        argv.extend(["--dataset-id", args.dataset_id])
    if args.record_id:
        argv.extend(["--record-id", args.record_id])
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    return _dataset(args, argv)


def _run_dataset_batch(args: argparse.Namespace) -> int:
    argv = [
        "run-batch",
        args.spec,
        "--row-concurrency",
        args.row_concurrency,
        "--subprocess-concurrency",
        args.subprocess_concurrency,
    ]
    if args.max_cost_usd is not None:
        argv.extend(["--max-cost-usd", str(args.max_cost_usd)])
    if args.resume:
        argv.append("--resume")
        argv.extend(["--resume-policy", args.resume_policy])
    if args.allow_resume_spec_mismatch:
        argv.append("--allow-resume-spec-mismatch")
    if args.qc_blurb:
        argv.extend(["--qc-blurb", args.qc_blurb])
    if args.keep_awake:
        argv.append("--keep-awake")
    return _dataset(args, argv)


def _run_dataset_record_delete(args: argparse.Namespace) -> int:
    argv = ["delete-record"]
    candidate = Path(args.record).expanduser()
    if candidate.exists() or "/" in args.record:
        argv.extend(["--record-path", args.record])
    else:
        argv.extend(["--record-id", args.record])
    if args.execute:
        argv.append("--execute")
    if args.confirm_record_id:
        argv.extend(["--confirm-record-id", args.confirm_record_id])
    return _dataset(args, argv)


def _run_env_bootstrap(args: argparse.Namespace) -> int:
    return env_cli.main([str(args.repo_root)])


def _run_hooks(args: argparse.Namespace) -> int:
    return hooks_cli.main([args.hooks_command])


def _run_internal_pre_commit(args: argparse.Namespace) -> int:
    return pre_commit_cli.main([args.pre_commit_cli_command, *args.paths])


def _add_repo_root(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repository root containing the data/ directory.",
    )


def _add_generation_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--provider", choices=("openai", "gemini", "openrouter", "anthropic"))
    parser.add_argument("--model", default=DEFAULT_MODEL, help="Model ID to use.")
    parser.add_argument(
        "--thinking-level",
        "--thinking",
        default=DEFAULT_THINKING,
        choices=("low", "med", "high"),
        help="Thinking budget level.",
    )
    parser.add_argument("--image", default=None, help="Optional reference image.")
    parser.add_argument("--max-cost-usd", type=float, default=None)


def _add_internal_pre_commit_commands(subparsers: argparse._SubParsersAction) -> None:
    pre_commit = subparsers.add_parser("pre-commit")
    pre_commit_sub = pre_commit.add_subparsers(dest="pre_commit_command", required=True)
    for command in ("forbidden-paths", "secrets", "smoke-tests", "data-format", "data-check"):
        pre_commit_cmd = pre_commit_sub.add_parser(command)
        pre_commit_cmd.add_argument("paths", nargs="*")
        mapped_command = "data-format" if command == "data-check" else command
        pre_commit_cmd.set_defaults(
            func=_run_internal_pre_commit,
            pre_commit_cli_command=mapped_command,
        )


def _build_internal_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft internal")
    subparsers = parser.add_subparsers(dest="internal_command", required=True)
    _add_internal_pre_commit_commands(subparsers)
    return parser


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft")
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="Initialize local Articraft storage and env.")
    _add_repo_root(init)
    init.set_defaults(func=_run_init)

    status = subparsers.add_parser("status", help="Show dataset and workbench status.")
    _add_repo_root(status)
    status.set_defaults(func=_run_status)

    generate = subparsers.add_parser("generate", help="Generate a workbench record from a prompt.")
    _add_repo_root(generate)
    generate.add_argument("prompt")
    _add_generation_options(generate)
    generate.set_defaults(func=_run_generate)

    draft = subparsers.add_parser("draft", help="Create a draft workbench record.")
    _add_repo_root(draft)
    draft.add_argument("prompt")
    _add_generation_options(draft)
    draft.add_argument("--label", default=None)
    draft.add_argument("--tag", dest="tags", action="append", default=None)
    draft.add_argument("--record-id", default=None)
    draft.set_defaults(func=_run_draft)

    rerun = subparsers.add_parser("rerun", help="Re-run an existing record.")
    _add_repo_root(rerun)
    rerun.add_argument("record")
    rerun.add_argument("--model", default=None, help="Optional model override.")
    rerun.add_argument("--thinking-level", "--thinking", default=None)
    rerun.add_argument("--max-cost-usd", type=float, default=None)
    rerun.set_defaults(func=_run_rerun)

    compile_one = subparsers.add_parser("compile", help="Compile one record.")
    _add_repo_root(compile_one)
    compile_one.add_argument("record")
    compile_one.add_argument("--target", choices=("full", "visual"), default="full")
    compile_one.add_argument("--validate", action="store_true")
    compile_one.add_argument("--strict-geom-qc", action="store_true")
    compile_one.set_defaults(func=_run_compile)

    compile_all = subparsers.add_parser("compile-all", help="Compile queued records.")
    _add_repo_root(compile_all)
    compile_all.add_argument("--target", choices=("full", "visual"), default="visual")
    compile_all.add_argument("--force", action="store_true")
    compile_all.add_argument("--strict", action="store_true")
    compile_all.add_argument("--limit", type=int, default=None)
    compile_all.add_argument("--concurrency", default="auto")
    compile_all.add_argument("--dry-run", action="store_true")
    compile_all.add_argument("--verify-assets", action="store_true")
    compile_all.set_defaults(func=_run_compile_all)

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

    data = subparsers.add_parser("data", help="Data validation commands.")
    data_sub = data.add_subparsers(dest="data_command", required=True)
    data_check = data_sub.add_parser("check", help="Validate checked-in data format.")
    _add_repo_root(data_check)
    data_check.set_defaults(func=lambda args: _dataset(args, ["validate-format"]))

    dataset = subparsers.add_parser("dataset", help="Dataset commands.")
    dataset_sub = dataset.add_subparsers(dest="dataset_command", required=True)
    for name, old_name, help_text in (
        ("status", "status", "Show dataset status."),
        ("validate", "validate", "Validate dataset entries."),
        ("manifest", "build-manifest", "Build the dataset manifest."),
    ):
        child = dataset_sub.add_parser(name, help=help_text)
        _add_repo_root(child)
        child.set_defaults(func=lambda args, old_name=old_name: _dataset(args, [old_name]))

    dataset_run = dataset_sub.add_parser("run", help="Generate one dataset record.")
    _add_repo_root(dataset_run)
    dataset_run.add_argument("prompt")
    dataset_run.add_argument("--category-slug", required=True)
    dataset_run.add_argument("--dataset-id", default=None)
    dataset_run.add_argument("--record-id", default=None)
    _add_generation_options(dataset_run)
    dataset_run.set_defaults(func=_run_dataset_run)

    dataset_batch = dataset_sub.add_parser("batch", help="Run a tracked batch CSV.")
    _add_repo_root(dataset_batch)
    dataset_batch.add_argument("spec")
    dataset_batch.add_argument("--row-concurrency", default="auto")
    dataset_batch.add_argument("--subprocess-concurrency", default="auto")
    dataset_batch.add_argument("--max-cost-usd", type=float, default=None)
    dataset_batch.add_argument("--resume", action="store_true")
    dataset_batch.add_argument(
        "--resume-policy",
        default="failed_or_pending",
        choices=("failed_or_pending", "failed_only", "all"),
    )
    dataset_batch.add_argument("--allow-resume-spec-mismatch", action="store_true")
    dataset_batch.add_argument("--qc-blurb", default=None)
    dataset_batch.add_argument("--keep-awake", action="store_true")
    dataset_batch.set_defaults(func=_run_dataset_batch)

    dataset_batch_new = dataset_sub.add_parser("batch-new", help="Create a new batch CSV.")
    _add_repo_root(dataset_batch_new)
    dataset_batch_new.add_argument("name")
    dataset_batch_new.set_defaults(func=_run_dataset_batch_new)

    dataset_promote = dataset_sub.add_parser("promote", help="Promote a record into a category.")
    _add_repo_root(dataset_promote)
    dataset_promote.add_argument("record")
    dataset_promote.add_argument("category_title")
    dataset_promote.add_argument("--dataset-id", default=None)
    dataset_promote.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "promote-record",
                args.record,
                args.category_title,
                *(["--dataset-id", args.dataset_id] if args.dataset_id else []),
            ],
        )
    )

    category = dataset_sub.add_parser("category", help="Dataset category admin.")
    category_sub = category.add_subparsers(dest="category_command", required=True)
    category_upsert = category_sub.add_parser("upsert")
    _add_repo_root(category_upsert)
    category_upsert.add_argument("category_slug")
    category_upsert.add_argument("--title", default=None)
    category_upsert.add_argument("--description", default=None)
    category_upsert.add_argument("--target-sdk-version", default=None)
    category_upsert.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "upsert-category",
                "--category-slug",
                args.category_slug,
                *(["--title", args.title] if args.title else []),
                *(["--description", args.description] if args.description else []),
                *(
                    ["--target-sdk-version", args.target_sdk_version]
                    if args.target_sdk_version
                    else []
                ),
            ],
        )
    )
    category_delete = category_sub.add_parser("delete")
    _add_repo_root(category_delete)
    category_delete.add_argument("category_slug")
    category_delete.add_argument("--execute", action="store_true")
    category_delete.add_argument("--confirm-slug", default=None)
    category_delete.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "delete-category",
                "--category-slug",
                args.category_slug,
                *(["--execute"] if args.execute else []),
                *(["--confirm-slug", args.confirm_slug] if args.confirm_slug else []),
            ],
        )
    )

    record = dataset_sub.add_parser("record", help="Dataset record admin.")
    record_sub = record.add_subparsers(dest="record_command", required=True)
    record_delete = record_sub.add_parser("delete")
    _add_repo_root(record_delete)
    record_delete.add_argument("record")
    record_delete.add_argument("--execute", action="store_true")
    record_delete.add_argument("--confirm-record-id", default=None)
    record_delete.set_defaults(func=_run_dataset_record_delete)

    supercategory = dataset_sub.add_parser("supercategory", help="Dataset supercategory admin.")
    super_sub = supercategory.add_subparsers(dest="supercategory_command", required=True)
    super_list = super_sub.add_parser("list")
    _add_repo_root(super_list)
    super_list.set_defaults(func=lambda args: _dataset(args, ["list-supercategories"]))
    super_upsert = super_sub.add_parser("upsert")
    _add_repo_root(super_upsert)
    super_upsert.add_argument("supercategory_slug")
    super_upsert.add_argument("--title", default=None)
    super_upsert.add_argument("--description", default=None)
    super_upsert.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "upsert-supercategory",
                "--supercategory-slug",
                args.supercategory_slug,
                *(["--title", args.title] if args.title else []),
                *(["--description", args.description] if args.description else []),
            ],
        )
    )
    super_set = super_sub.add_parser("set")
    _add_repo_root(super_set)
    super_set.add_argument("category_slug")
    super_set.add_argument("supercategory_slug")
    super_set.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "set-supercategory",
                "--category-slug",
                args.category_slug,
                "--supercategory-slug",
                args.supercategory_slug,
            ],
        )
    )
    super_clear = super_sub.add_parser("clear")
    _add_repo_root(super_clear)
    super_clear.add_argument("category_slug")
    super_clear.set_defaults(
        func=lambda args: _dataset(
            args, ["clear-supercategory", "--category-slug", args.category_slug]
        )
    )
    super_delete = super_sub.add_parser("delete")
    _add_repo_root(super_delete)
    super_delete.add_argument("supercategory_slug")
    super_delete.add_argument("--execute", action="store_true")
    super_delete.add_argument("--confirm-slug", default=None)
    super_delete.set_defaults(
        func=lambda args: _dataset(
            args,
            [
                "delete-supercategory",
                "--supercategory-slug",
                args.supercategory_slug,
                *(["--execute"] if args.execute else []),
                *(["--confirm-slug", args.confirm_slug] if args.confirm_slug else []),
            ],
        )
    )

    workbench = subparsers.add_parser("workbench", help="Workbench commands.")
    workbench_sub = workbench.add_subparsers(dest="workbench_command", required=True)
    wb_status = workbench_sub.add_parser("status")
    _add_repo_root(wb_status)
    wb_status.set_defaults(func=lambda args: _workbench(args, ["status"]))
    wb_search = workbench_sub.add_parser("search-index")
    _add_repo_root(wb_search)
    wb_search.set_defaults(func=lambda args: _workbench(args, ["rebuild-search-index"]))

    env = subparsers.add_parser("env", help="Environment helpers.")
    env_sub = env.add_subparsers(dest="env_command", required=True)
    bootstrap = env_sub.add_parser("bootstrap")
    _add_repo_root(bootstrap)
    bootstrap.set_defaults(func=_run_env_bootstrap)

    hooks = subparsers.add_parser("hooks", help="Git hook helpers.")
    hooks_sub = hooks.add_subparsers(dest="hooks_command", required=True)
    for command in ("install", "check", "post-commit-record-authors"):
        hook_cmd = hooks_sub.add_parser(command)
        hook_cmd.set_defaults(func=_run_hooks, hooks_command=command)

    return parser


def main(argv: list[str] | None = None) -> int:
    args_list = sys.argv[1:] if argv is None else argv
    if args_list and args_list[0] == "internal":
        parser = _build_internal_parser()
        args = parser.parse_args(args_list[1:])
        return args.func(args)
    parser = _build_parser()
    args = parser.parse_args(args_list)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
