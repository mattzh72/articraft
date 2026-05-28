#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "${ROOT_DIR}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/.env"
  set +a
fi

if [[ -f "${ROOT_DIR}/.env.dashscope" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/.env.dashscope"
  set +a
fi

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/dashscope_env.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/dashscope_run.sh official-test
  scripts/dashscope_run.sh generate "prompt text"

Commands:
  official-test  Run a minimal official OpenAI-compatible Chat Completions request.
  generate       Run Articraft generation with --provider dashscope.
EOF
}

cmd="${1:-}"
case "${cmd}" in
  official-test)
    if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
      echo "Missing DASHSCOPE_API_KEY. Put it in .env or .env.dashscope." >&2
      exit 1
    fi
    uv --cache-dir "${ROOT_DIR}/.uv-cache" run python - <<'PY'
import os
from openai import OpenAI

client = OpenAI(
    api_key=os.environ["DASHSCOPE_API_KEY"],
    base_url=os.environ["DASHSCOPE_BASE_URL"],
)
resp = client.chat.completions.create(
    model=os.environ.get("DASHSCOPE_MODEL", "qwen3.6-flash"),
    messages=[{"role": "user", "content": "只回复: ok"}],
    extra_body={"enable_thinking": True},
)
print(resp.choices[0].message.content)
PY
    ;;
  generate)
    if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
      echo "Missing DASHSCOPE_API_KEY. Put it in .env or .env.dashscope." >&2
      exit 1
    fi
    shift
    prompt="${1:-}"
    if [[ -z "${prompt}" ]]; then
      echo "Missing prompt. Usage: scripts/dashscope_run.sh generate \"prompt text\"" >&2
      exit 1
    fi
    uv --cache-dir "${ROOT_DIR}/.uv-cache" run articraft generate \
      --provider dashscope \
      --model "${DASHSCOPE_MODEL}" \
      "${prompt}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
