from __future__ import annotations

import sys
from pathlib import Path

from articraft.config import bootstrap_env


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if args and args[0] in {"-h", "--help"}:
        print("Usage: articraft env bootstrap [repo_root]")
        return 0
    if len(args) > 1:
        print("Usage: articraft env bootstrap [repo_root]", file=sys.stderr)
        return 2
    repo_root = Path(args[0]).resolve() if args else Path.cwd()
    created, imported_keys = bootstrap_env(repo_root)
    if not created:
        return 0

    print("Created .env from .env.example")
    if imported_keys:
        print(f"Imported environment values for: {', '.join(imported_keys)}")
    else:
        print("No matching provider credentials found in the current shell environment.")
    print("Review .env before running the agent harness.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
