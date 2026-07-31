from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

from articraft.config import load_repo_env


def _npm_setup(repo_root: Path) -> int:
    npm = shutil.which("npm")
    if npm is None:
        print("npm not found; skipping viewer/web dependency install.")
        print("Install Node.js and npm to run the viewer and frontend hooks.")
        return 0

    web_root = repo_root / "viewer" / "web"
    for args in (("ci",), ("run", "typecheck")):
        status = subprocess.call([npm, "--prefix", str(web_root), *args])
        if status != 0:
            return status
    return 0


def _dashscope_test(repo_root: Path) -> int:
    load_repo_env(repo_root)
    api_key = os.environ.get("DASHSCOPE_API_KEY")
    if not api_key:
        print("Missing DASHSCOPE_API_KEY. Put it in .env.", file=sys.stderr)
        return 1

    from openai import OpenAI

    client = OpenAI(
        api_key=api_key,
        base_url=os.environ.get(
            "DASHSCOPE_BASE_URL",
            "https://dashscope.aliyuncs.com/compatible-mode/v1",
        ),
    )
    response = client.chat.completions.create(
        model=os.environ.get("DASHSCOPE_MODEL", "qwen3.6-flash"),
        messages=[{"role": "user", "content": "Reply with only: ok"}],
        extra_body={"enable_thinking": True},
    )
    print(response.choices[0].message.content)
    return 0


def _dashscope_generate(repo_root: Path, prompt: str) -> int:
    load_repo_env(repo_root)
    if not os.environ.get("DASHSCOPE_API_KEY"):
        print("Missing DASHSCOPE_API_KEY. Put it in .env.", file=sys.stderr)
        return 1
    model = os.environ.get("DASHSCOPE_MODEL", "qwen3.6-flash")
    return subprocess.call(
        [
            sys.executable,
            "-m",
            "cli.main",
            "generate",
            "--provider",
            "dashscope",
            "--model",
            model,
            prompt,
        ],
        cwd=repo_root,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Cross-platform Articraft development tasks.")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("npm-setup")
    subparsers.add_parser("dashscope-test")
    generate = subparsers.add_parser("dashscope-generate")
    generate.add_argument("prompt")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()

    if args.command == "npm-setup":
        return _npm_setup(repo_root)
    if args.command == "dashscope-test":
        return _dashscope_test(repo_root)
    if args.command == "dashscope-generate":
        return _dashscope_generate(repo_root, args.prompt)
    parser.error(f"Unsupported command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
