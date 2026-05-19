from __future__ import annotations

import argparse
import os
import stat
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

from storage.record_authors import (
    sync_record_authors,
    sync_record_rated_by,
    sync_record_secondary_rated_by,
)
from storage.repo import StorageRepo

REPO_ROOT = Path(__file__).resolve().parent.parent
POST_COMMIT_GUARD_ENV = "ARTICRAFT_POST_COMMIT_AUTHOR_SYNC_RUNNING"
MANAGED_POST_COMMIT_MARKER = "# articraft-managed-post-commit"
MANAGED_POST_COMMIT_SCRIPT = """#!/usr/bin/env bash
{marker}
set -euo pipefail
repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"
if command -v uv >/dev/null 2>&1; then
  exec uv run articraft hooks post-commit-record-authors
fi
exec python3 -m cli.main hooks post-commit-record-authors
""".format(marker=MANAGED_POST_COMMIT_MARKER)

HookInstallStatus = Literal["installed", "missing", "unmanaged"]


def _git_output(repo_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "git command failed"
        raise RuntimeError(message)
    return result.stdout


def _git_run(repo_root: Path, *args: str, env: dict[str, str] | None = None) -> None:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "git command failed"
        raise RuntimeError(message)


def _git_path(repo_root: Path, relative_git_path: str) -> Path:
    resolved = _git_output(repo_root, "rev-parse", "--git-path", relative_git_path).strip()
    path = Path(resolved)
    if path.is_absolute():
        return path
    return (repo_root / path).resolve()


def touched_record_ids_for_commit(repo_root: Path, commit_rev: str = "HEAD") -> list[str]:
    output = _git_output(
        repo_root,
        "diff-tree",
        "--no-commit-id",
        "--name-only",
        "-r",
        "--root",
        commit_rev,
    )
    record_ids: list[str] = []
    seen: set[str] = set()
    for line in output.splitlines():
        parts = line.strip().split("/")
        if len(parts) < 3 or parts[0] != "data" or parts[1] != "records":
            continue
        record_id = parts[2].strip()
        if record_id and record_id not in seen:
            seen.add(record_id)
            record_ids.append(record_id)
    return sorted(record_ids)


def touched_record_metadata_ids_for_commit(repo_root: Path, commit_rev: str = "HEAD") -> list[str]:
    output = _git_output(
        repo_root,
        "diff-tree",
        "--no-commit-id",
        "--name-only",
        "-r",
        "--root",
        commit_rev,
    )
    record_ids: list[str] = []
    seen: set[str] = set()
    for line in output.splitlines():
        parts = line.strip().split("/")
        if (
            len(parts) != 4
            or parts[0] != "data"
            or parts[1] != "records"
            or parts[3] != "record.json"
        ):
            continue
        record_id = parts[2].strip()
        if record_id and record_id not in seen:
            seen.add(record_id)
            record_ids.append(record_id)
    return sorted(record_ids)


@dataclass(slots=True, frozen=True)
class PostCommitRecordMetadataResult:
    touched_record_ids: list[str] = field(default_factory=list)
    touched_record_metadata_ids: list[str] = field(default_factory=list)
    updated_author_record_ids: list[str] = field(default_factory=list)
    updated_rated_by_record_ids: list[str] = field(default_factory=list)
    updated_secondary_rated_by_record_ids: list[str] = field(default_factory=list)

    @property
    def updated_record_ids(self) -> list[str]:
        return sorted(
            set(self.updated_author_record_ids)
            | set(self.updated_rated_by_record_ids)
            | set(self.updated_secondary_rated_by_record_ids)
        )


@dataclass(slots=True, frozen=True)
class PostCommitHookStatus:
    status: HookInstallStatus
    hook_path: Path

    @property
    def installed(self) -> bool:
        return self.status == "installed"


def run_post_commit_record_metadata_sync(repo_root: Path) -> PostCommitRecordMetadataResult:
    if os.environ.get(POST_COMMIT_GUARD_ENV):
        return PostCommitRecordMetadataResult()

    repo_root = repo_root.resolve()
    touched_record_ids = touched_record_ids_for_commit(repo_root)
    touched_record_metadata_ids = touched_record_metadata_ids_for_commit(repo_root)
    if not touched_record_ids and not touched_record_metadata_ids:
        return PostCommitRecordMetadataResult()

    repo = StorageRepo(repo_root)
    author_summary = sync_record_authors(repo, record_ids=touched_record_ids)
    rated_by_summary = sync_record_rated_by(repo, record_ids=touched_record_metadata_ids)
    secondary_rated_by_summary = sync_record_secondary_rated_by(
        repo,
        record_ids=touched_record_metadata_ids,
    )
    updated_record_ids = sorted(
        set(author_summary.updated_record_ids)
        | set(rated_by_summary.updated_record_ids)
        | set(secondary_rated_by_summary.updated_record_ids)
    )
    if updated_record_ids:
        updated_paths = [
            repo.layout.record_metadata_path(record_id).resolve().relative_to(repo_root).as_posix()
            for record_id in updated_record_ids
        ]
        _git_run(repo_root, "add", "--", *updated_paths)
        env = dict(os.environ)
        env[POST_COMMIT_GUARD_ENV] = "1"
        _git_run(repo_root, "commit", "--amend", "--no-edit", "--no-verify", env=env)
    return PostCommitRecordMetadataResult(
        touched_record_ids=touched_record_ids,
        touched_record_metadata_ids=touched_record_metadata_ids,
        updated_author_record_ids=list(author_summary.updated_record_ids),
        updated_rated_by_record_ids=list(rated_by_summary.updated_record_ids),
        updated_secondary_rated_by_record_ids=list(secondary_rated_by_summary.updated_record_ids),
    )


def run_post_commit_record_author_sync(repo_root: Path) -> PostCommitRecordMetadataResult:
    return run_post_commit_record_metadata_sync(repo_root)


def get_post_commit_hook_status(repo_root: Path) -> PostCommitHookStatus:
    repo_root = repo_root.resolve()
    hook_path = _git_path(repo_root, "hooks/post-commit")
    if not hook_path.exists():
        return PostCommitHookStatus(status="missing", hook_path=hook_path)

    existing = hook_path.read_text(encoding="utf-8")
    if existing == MANAGED_POST_COMMIT_SCRIPT:
        return PostCommitHookStatus(status="installed", hook_path=hook_path)
    return PostCommitHookStatus(status="unmanaged", hook_path=hook_path)


def install_post_commit_hook(repo_root: Path) -> Path:
    repo_root = repo_root.resolve()
    status = get_post_commit_hook_status(repo_root)
    hook_path = status.hook_path
    if hook_path.exists():
        if status.status == "installed":
            current_mode = stat.S_IMODE(hook_path.stat().st_mode)
            hook_path.chmod(current_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
            return hook_path
        existing = hook_path.read_text(encoding="utf-8")
        if MANAGED_POST_COMMIT_MARKER not in existing:
            raise RuntimeError(
                f"Refusing to overwrite unmanaged post-commit hook at {hook_path}. "
                "Please merge it manually."
            )
    hook_path.parent.mkdir(parents=True, exist_ok=True)
    hook_path.write_text(MANAGED_POST_COMMIT_SCRIPT, encoding="utf-8")
    current_mode = stat.S_IMODE(hook_path.stat().st_mode)
    hook_path.chmod(current_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return hook_path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft hooks")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("install", help="Install the managed post-commit hook.")
    subparsers.add_parser(
        "check",
        help="Report whether the managed post-commit hook is installed, missing, or unmanaged.",
    )
    subparsers.add_parser(
        "post-commit-record-authors",
        help="Hook entrypoint that syncs record author metadata and amends the latest commit.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "install":
        hook_path = install_post_commit_hook(REPO_ROOT)
        print(f"Installed post-commit hook at {hook_path}")
        return 0

    if args.command == "check":
        status = get_post_commit_hook_status(REPO_ROOT)
        print(f"status={status.status} hook_path={status.hook_path}")
        return 0

    if args.command == "post-commit-record-authors":
        result = run_post_commit_record_metadata_sync(REPO_ROOT)
        if result.updated_record_ids:
            detail_parts: list[str] = []
            if result.updated_author_record_ids:
                detail_parts.append("author=" + ", ".join(result.updated_author_record_ids[:10]))
            if result.updated_rated_by_record_ids:
                detail_parts.append(
                    "rated_by=" + ", ".join(result.updated_rated_by_record_ids[:10])
                )
            if result.updated_secondary_rated_by_record_ids:
                detail_parts.append(
                    "secondary_rated_by="
                    + ", ".join(result.updated_secondary_rated_by_record_ids[:10])
                )
            print("Synced record metadata for latest commit: " + "; ".join(detail_parts))
        return 0

    print(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
