from __future__ import annotations

import subprocess
from pathlib import Path

from cli import hooks


def _git(repo_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _stock_lfs_hook(hook_name: str) -> str:
    return (
        "#!/bin/sh\n"
        'command -v git-lfs >/dev/null 2>&1 || { printf >&2 "\\n%s\\n\\n" '
        '"This repository is configured for Git LFS but git-lfs was not found"; exit 2; }\n'
        f'git lfs {hook_name} "$@"\n'
    )


def test_install_hooks_removes_only_stock_lfs_hooks_and_sets_sshtransfer(
    tmp_path: Path,
) -> None:
    _git(tmp_path, "init")
    hooks_dir = tmp_path / ".git" / "hooks"
    stock_pre_push = hooks_dir / "pre-push"
    stock_post_merge = hooks_dir / "post-merge"
    stock_pre_push_legacy = hooks_dir / "pre-push.legacy"
    custom_post_commit = hooks_dir / "post-commit"

    stock_pre_push.write_text(_stock_lfs_hook("pre-push"), encoding="utf-8")
    stock_post_merge.write_text(_stock_lfs_hook("post-merge"), encoding="utf-8")
    stock_pre_push_legacy.write_text(_stock_lfs_hook("pre-push"), encoding="utf-8")
    custom_post_commit.write_text("#!/bin/sh\necho custom\n", encoding="utf-8")

    result = hooks.install_hooks(tmp_path)

    assert sorted(path.name for path in result.removed_stock_lfs_hooks) == [
        "post-merge",
        "pre-push",
        "pre-push.legacy",
    ]
    assert not stock_pre_push.exists()
    assert not stock_post_merge.exists()
    assert not stock_pre_push_legacy.exists()
    assert custom_post_commit.read_text(encoding="utf-8") == "#!/bin/sh\necho custom\n"
    assert _git(tmp_path, "config", "--get", "lfs.sshtransfer") == "never"
