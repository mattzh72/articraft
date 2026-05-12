default:
    @just --list

host := "127.0.0.1"
port := "8765"

setup:
    # Create a local env template once; never overwrite an existing secrets file.
    uv run articraft env bootstrap
    uv sync --group dev
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run articraft hooks install
    @if command -v npm >/dev/null 2>&1; then \
        npm --prefix viewer/web ci; \
        npm --prefix viewer/web run typecheck; \
    else \
        echo "npm not found; skipping viewer/web dependency install."; \
        echo "Install Node.js and npm to run the viewer and frontend hooks."; \
    fi
    uv run articraft init

hooks-install:
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run articraft hooks install

format:
    uv run ruff format .

lint:
    uv run ruff check .

smoke-tests:
    uv run --group dev pytest -q \
      tests/agent \
      tests/storage \
      tests/viewer/test_api.py \
      tests/workbench \
      tests/dataset/test_imports.py \
      tests/sdk/test_imports.py \
      tests/cli

test-all:
    uv run --group dev pytest -q

viewer:
    uv run articraft viewer --host {{ quote(host) }} --port {{ quote(port) }}

viewer-dev:
    uv run articraft viewer --dev --host {{ quote(host) }} --port {{ quote(port) }}
