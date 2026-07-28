default:
    @just --list

host := "127.0.0.1"
port := "8765"

setup root='.':
    uv sync --frozen --group dev --directory {{ quote(root) }}
    uv run --frozen --directory {{ quote(root) }} python scripts/dev_tasks.py --repo-root {{ quote(root) }} npm-setup
    uv run --frozen --directory {{ quote(root) }} articraft init

format:
    uv run --frozen ruff format .

lint:
    uv run --frozen ruff check .

compile record:
    uv run --frozen articraft compile --target visual {{ quote(record) }}

compile-full record:
    uv run --frozen articraft compile --target full {{ quote(record) }}

smoke-tests:
    uv run --frozen --group dev pytest -q tests/storage tests/viewer/test_api.py tests/sdk/test_imports.py tests/cli

test-all:
    uv run --frozen --group dev pytest -q

viewer:
    uv run --frozen articraft viewer --host {{ quote(host) }} --port {{ quote(port) }}

viewer-dev:
    uv run --frozen articraft viewer --dev --host {{ quote(host) }} --port {{ quote(port) }}

dashscope-test:
    uv run --frozen python scripts/dev_tasks.py dashscope-test

dashscope-generate prompt:
    uv run --frozen python scripts/dev_tasks.py dashscope-generate {{ quote(prompt) }}
