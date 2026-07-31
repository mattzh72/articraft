#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "${1:-}" in
  official-test)
    exec uv --directory "${ROOT_DIR}" run --frozen python scripts/dev_tasks.py dashscope-test
    ;;
  generate)
    shift
    exec uv --directory "${ROOT_DIR}" run --frozen python scripts/dev_tasks.py dashscope-generate "${1:-}"
    ;;
  *)
    echo "Usage: scripts/dashscope_run.sh {official-test|generate \"prompt text\"}" >&2
    exit 2
    ;;
esac
