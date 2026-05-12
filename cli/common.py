from __future__ import annotations

from argparse import ArgumentParser
from pathlib import Path

from cli.hooks import get_post_commit_hook_status


def add_data_root_argument(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repository root containing the data/ directory.",
    )


def warn_if_post_commit_hook_missing(repo_root: Path) -> None:
    try:
        status = get_post_commit_hook_status(repo_root)
    except RuntimeError:
        return

    if status.status == "installed":
        return

    print(
        "Warning: managed post-commit hook is "
        f"{status.status} at {status.hook_path}. "
        "Record metadata will not auto-sync after commits, including rated_by and "
        "secondary_rated_by. "
        "Run `just setup`, or `just hooks-install`, or "
        "`uv run articraft hooks install`."
    )
