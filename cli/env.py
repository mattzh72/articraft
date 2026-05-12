from __future__ import annotations

import os
import sys
from collections.abc import Mapping
from pathlib import Path

SUPPORTED_ENV_KEYS = (
    "OPENAI_API_KEYS",
    "OPENAI_API_KEY",
    "OPENROUTER_API_KEYS",
    "OPENROUTER_API_KEY",
    "ANTHROPIC_API_KEYS",
    "ANTHROPIC_API_KEY",
    "GEMINI_API_KEYS",
    "GOOGLE_CLOUD_PROJECT",
    "GOOGLE_APPLICATION_CREDENTIALS",
    "GOOGLE_CLOUD_LOCATION",
)


def _quote_env_value(value: str) -> str:
    if value == "":
        return value
    safe_chars = set("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789@%_+=:,./-")
    if all(char in safe_chars for char in value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def bootstrap_env(
    repo_root: Path,
    *,
    environ: Mapping[str, str] | None = None,
) -> tuple[bool, list[str]]:
    values = os.environ if environ is None else environ
    env_path = repo_root / ".env"
    if env_path.exists():
        return False, []

    template_path = repo_root / ".env.example"
    template_lines = template_path.read_text(encoding="utf-8").splitlines()

    rendered_lines: list[str] = []
    seen_keys: set[str] = set()
    imported_keys: list[str] = []

    for line in template_lines:
        key, separator, default_value = line.partition("=")
        normalized_key = key.strip()
        if separator and normalized_key in SUPPORTED_ENV_KEYS:
            seen_keys.add(normalized_key)
            env_value = values.get(normalized_key)
            if env_value:
                rendered_lines.append(f"{normalized_key}={_quote_env_value(env_value)}")
                imported_keys.append(normalized_key)
            else:
                rendered_lines.append(f"{normalized_key}={default_value}")
            continue
        rendered_lines.append(line)

    for key in SUPPORTED_ENV_KEYS:
        env_value = values.get(key)
        if key in seen_keys or not env_value:
            continue
        rendered_lines.append(f"{key}={_quote_env_value(env_value)}")
        imported_keys.append(key)

    env_path.write_text("\n".join(rendered_lines) + "\n", encoding="utf-8")
    return True, imported_keys


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
