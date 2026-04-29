from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SMOKE_TEST_TARGETS = (
    "tests/agent",
    "tests/storage",
    "tests/viewer/test_api.py",
    "tests/workbench",
    "tests/dataset/test_imports.py",
    "tests/sdk/test_imports.py",
    "tests/scripts/test_pre_commit_hooks.py",
)
FORBIDDEN_PATHS = (
    re.compile(r"(^|/)\.env(?!\.example)(\..*)?$"),
    re.compile(r"^data/cache/"),
    re.compile(r"^data/local/"),
    re.compile(r"^data/records/[^/]+/model\.urdf$"),
    re.compile(r"^data/records/[^/]+/assets(?:/|$)"),
)
SECRET_PATTERNS = (
    ("OpenAI API key assignment", re.compile(r"OPENAI_API_KEYS?\s*=\s*['\"]?[^'\"\s]+")),
    (
        "OpenRouter API key assignment",
        re.compile(r"OPENROUTER_API_KEYS?\s*=\s*['\"]?[^'\"\s]+"),
    ),
    ("Gemini API keys assignment", re.compile(r"GEMINI_API_KEYS\s*=\s*['\"]?[^'\"\s]+")),
    ("OpenAI secret token", re.compile(r"\bsk-[A-Za-z0-9_-]{16,}\b")),
    (
        "Private key block",
        re.compile(r"-----BEGIN (?:[A-Z0-9 ]+ )?PRIVATE KEY-----"),
    ),
)


def iter_existing_files(paths: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw_path in paths:
        path = Path(raw_path)
        if not path.exists() or path.is_dir():
            continue
        files.append(path)
    return files


def detect_forbidden_paths(paths: list[str]) -> int:
    violations: list[str] = []
    for raw_path in paths:
        normalized = raw_path.replace("\\", "/")
        if any(pattern.search(normalized) for pattern in FORBIDDEN_PATHS):
            violations.append(normalized)
    if not violations:
        return 0
    print("Refusing to commit sensitive or local-only paths:")
    for path in violations:
        print(f"  - {path}")
    return 1


def decode_text(path: Path) -> str | None:
    content = path.read_bytes()
    if b"\x00" in content:
        return None
    return content.decode("utf-8", errors="replace")


def requires_trailing_newline(path: Path) -> bool:
    normalized = path.as_posix()
    # Generated dataset artifacts are often emitted without a final newline.
    # Keep the stricter rule for source/docs while relaxing it for data payloads.
    return not normalized.startswith("data/")


def detect_secrets(paths: list[str]) -> int:
    findings: list[str] = []
    for path in iter_existing_files(paths):
        text = decode_text(path)
        if text is None:
            continue
        for line_number, line in enumerate(text.splitlines(), start=1):
            for label, pattern in SECRET_PATTERNS:
                if pattern.search(line):
                    findings.append(f"{path}:{line_number}: {label}")
    if not findings:
        return 0
    print("Potential secrets detected:")
    for finding in findings:
        print(f"  - {finding}")
    return 1


def run_smoke_tests() -> int:
    result = subprocess.run(
        ["uv", "run", "--group", "dev", "pytest", "-q", *SMOKE_TEST_TARGETS],
        cwd=REPO_ROOT,
        check=False,
    )
    return result.returncode


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("Usage: pre_commit_hooks.py <forbidden-paths|secrets|smoke-tests> [paths...]")
        return 2

    command = argv[1]
    paths = argv[2:]
    if command == "forbidden-paths":
        return detect_forbidden_paths(paths)
    if command == "secrets":
        return detect_secrets(paths)
    if command == "smoke-tests":
        return run_smoke_tests()

    print(f"Unknown command: {command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
