default:
    @just --list

host := "127.0.0.1"
port := "8765"

uv-version-check:
    @version="$(uv --version | awk '{print $2}')"; \
    required="0.9.17"; \
    awk -v v="$version" -v r="$required" 'BEGIN { \
      split(v, a, "."); split(r, b, "."); \
      for (i = 1; i <= 3; i++) { \
        if ((a[i] + 0) > (b[i] + 0)) exit 0; \
        if ((a[i] + 0) < (b[i] + 0)) exit 1; \
      } \
      exit 0; \
    }' || { \
      echo "uv $required or newer is required for the Python dependency cooldown."; \
      echo "Current: uv $version"; \
      exit 1; \
    }

uv-version-ensure:
    @version="$(uv --version | awk '{print $2}')"; \
    required="0.9.17"; \
    if ! awk -v v="$version" -v r="$required" 'BEGIN { \
      split(v, a, "."); split(r, b, "."); \
      for (i = 1; i <= 3; i++) { \
        if ((a[i] + 0) > (b[i] + 0)) exit 0; \
        if ((a[i] + 0) < (b[i] + 0)) exit 1; \
      } \
      exit 0; \
    }'; then \
      echo "Updating uv from $version to $required for the Python dependency cooldown."; \
      uv --no-config self update "$required"; \
    fi

setup:
    just uv-version-ensure
    # Create a local env template once; never overwrite an existing secrets file.
    uv run --frozen articraft env bootstrap
    uv sync --frozen --group dev
    uv run --frozen pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run --frozen articraft hooks install
    @if command -v npm >/dev/null 2>&1; then \
        npm --prefix viewer/web ci; \
        npm --prefix viewer/web run typecheck; \
    else \
        echo "npm not found; skipping viewer/web dependency install."; \
        echo "Install Node.js and npm to run the viewer and frontend hooks."; \
    fi
    uv run --frozen articraft init

hooks-install:
    just uv-version-check
    uv run --frozen pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run --frozen articraft hooks install

format:
    just uv-version-check
    uv run --frozen ruff format .

lint:
    just uv-version-check
    uv run --frozen ruff check .

compile record:
    just uv-version-check
    uv run --frozen articraft compile --target visual {{ quote(record) }}

compile-full record:
    just uv-version-check
    uv run --frozen articraft compile --target full {{ quote(record) }}

compile-all:
    just uv-version-check
    uv run --frozen articraft compile-all

compile-all-force:
    just uv-version-check
    uv run --frozen articraft compile-all --target visual --force

compile-all-force-limit limit:
    just uv-version-check
    uv run --frozen articraft compile-all --target visual --force --limit {{ quote(limit) }}

compile-all-full-force:
    just uv-version-check
    uv run --frozen articraft compile-all --target full --force

smoke-tests:
    just uv-version-check
    uv run --frozen --group dev pytest -q \
      tests/agent \
      tests/storage \
      tests/viewer/test_api.py \
      tests/workbench \
      tests/dataset/test_imports.py \
      tests/sdk/test_imports.py \
      tests/cli

test-all:
    just uv-version-check
    uv run --frozen --group dev pytest -q

viewer:
    just uv-version-check
    uv run --frozen articraft viewer --host {{ quote(host) }} --port {{ quote(port) }}

viewer-dev:
    just uv-version-check
    uv run --frozen articraft viewer --dev --host {{ quote(host) }} --port {{ quote(port) }}
