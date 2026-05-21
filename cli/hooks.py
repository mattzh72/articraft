from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

REPO_ROOT = Path(__file__).resolve().parent.parent
POST_COMMIT_GUARD_ENV = "ARTICRAFT_POST_COMMIT_AUTHOR_SYNC_RUNNING"
MANAGED_POST_COMMIT_MARKER = "# articraft-managed-post-commit"
GIT_LFS_HOOK_NAMES = ("pre-push", "post-checkout", "post-commit", "post-merge")
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


@dataclass(slots=True, frozen=True)
class HookInstallResult:
    removed_stock_lfs_hooks: list[Path] = field(default_factory=list)


def run_post_commit_record_metadata_sync(repo_root: Path) -> PostCommitRecordMetadataResult:
    _ = repo_root
    return PostCommitRecordMetadataResult()


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
    return status.hook_path


def _is_stock_git_lfs_hook(path: Path, hook_name: str) -> bool:
    if not path.exists() or not path.is_file():
        return False
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return False
    return (
        "This repository is configured for Git LFS" in text
        and f'git lfs {hook_name} "$@"' in text.splitlines()
    )


def remove_stock_git_lfs_hooks(repo_root: Path) -> list[Path]:
    removed: list[Path] = []
    for hook_name in GIT_LFS_HOOK_NAMES:
        for suffix in ("", ".legacy"):
            hook_path = _git_path(repo_root, f"hooks/{hook_name}{suffix}")
            if not _is_stock_git_lfs_hook(hook_path, hook_name):
                continue
            hook_path.unlink()
            removed.append(hook_path)
    return removed


def install_hooks(repo_root: Path) -> HookInstallResult:
    repo_root = repo_root.resolve()
    removed = remove_stock_git_lfs_hooks(repo_root)
    _git_run(repo_root, "config", "--local", "lfs.sshtransfer", "never")
    return HookInstallResult(removed_stock_lfs_hooks=removed)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft hooks")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser(
        "install",
        help="Repair Articraft hook defaults without overwriting custom hooks.",
    )
    subparsers.add_parser(
        "check",
        help="Report whether an old managed post-commit hook is installed, missing, or unmanaged.",
    )
    subparsers.add_parser(
        "post-commit-record-authors",
        help="No-op compatibility entrypoint for old managed metadata hooks.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "install":
        result = install_hooks(REPO_ROOT)
        if result.removed_stock_lfs_hooks:
            print("Removed stock Git LFS hooks:")
            for hook_path in result.removed_stock_lfs_hooks:
                print(f"  - {hook_path}")
        print("Configured lfs.sshtransfer=never.")
        return 0

    if args.command == "check":
        status = get_post_commit_hook_status(REPO_ROOT)
        print(f"status={status.status} hook_path={status.hook_path}")
        return 0

    if args.command == "post-commit-record-authors":
        result = run_post_commit_record_metadata_sync(REPO_ROOT)
        if result.updated_record_ids:
            print("Managed post-commit metadata sync is disabled.")
        return 0

    print(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
