default:
    @just --list

host := "127.0.0.1"
port := "8765"

setup:
    uv sync --group dev
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run articraft-dataset --repo-root . init-storage
    uv run articraft-workbench --repo-root . init-storage
    uv run articraft-dataset --repo-root . validate
    uv run articraft-dataset --repo-root . build-manifest

hooks-install:
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push

format:
    uv run ruff format .

lint:
    uv run ruff check .

smoke-tests:
    find tests -name 'test_*.py' -type f | sort | xargs -I{} uv run python {}

wb prompt model_arg="model=gpt-5.4" thinking_arg="thinking=high":
    #!/usr/bin/env bash
    set -euo pipefail
    raw_model={{ quote(model_arg) }}
    raw_thinking={{ quote(thinking_arg) }}
    case "$raw_model" in
      model=*)
        model="${raw_model#model=}"
        ;;
      *)
        model="$raw_model"
        ;;
    esac
    case "$raw_thinking" in
      thinking=*)
        thinking="${raw_thinking#thinking=}"
        ;;
      *)
        thinking="$raw_thinking"
        ;;
    esac
    case "$model" in
      gpt-*|o1*|o3*|o4*)
        provider="openai"
        ;;
      gemini-*)
        provider="gemini"
        ;;
      *)
        echo "Unable to infer provider for model '$model'. Supported model prefixes: gpt-, o1, o3, o4, gemini-." >&2
        exit 1
        ;;
    esac
    exec uv run python agent/runner.py \
      --repo-root . \
      --prompt {{ quote(prompt) }} \
      --provider "$provider" \
      --model "$model" \
      --thinking "$thinking"

dataset-validate:
    uv run articraft-dataset --repo-root . validate

dataset-manifest:
    uv run articraft-dataset --repo-root . build-manifest

search-index:
    uv run articraft-workbench --repo-root . rebuild-search-index

viewer mode="prod" api_host=host api_port=port:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v npm >/dev/null 2>&1; then
      echo "npm is required for viewer/web. Install Node.js and npm first."
      exit 1
    fi
    if [ ! -d viewer/web/node_modules ]; then
      npm --prefix viewer/web install
    fi
    uv run articraft-dataset --repo-root . validate
    uv run articraft-dataset --repo-root . build-manifest
    uv run articraft-workbench --repo-root . rebuild-search-index
    case "{{mode}}" in
      prod)
        npm --prefix viewer/web run build
        (
          for _ in {1..60}; do
            if curl -fsS http://{{api_host}}:{{api_port}}/health >/dev/null; then
              open http://{{api_host}}:{{api_port}}/ >/dev/null 2>&1 || uv run python -m webbrowser http://{{api_host}}:{{api_port}}/ >/dev/null 2>&1 || true
              exit 0
            fi
            sleep 0.25
          done
        ) >/dev/null 2>&1 &
        exec uv run uvicorn viewer.api.app:app --reload --host {{api_host}} --port {{api_port}}
        ;;
      dev)
        (
          for _ in {1..60}; do
            if curl -fsS http://127.0.0.1:5173 >/dev/null; then
              open http://127.0.0.1:5173/ >/dev/null 2>&1 || uv run python -m webbrowser http://127.0.0.1:5173/ >/dev/null 2>&1 || true
              exit 0
            fi
            sleep 0.25
          done
        ) >/dev/null 2>&1 &
        trap 'if [ -n "${api_pid:-}" ]; then kill "$api_pid" >/dev/null 2>&1 || true; fi' EXIT INT TERM
        uv run uvicorn viewer.api.app:app --reload --host {{api_host}} --port {{api_port}} &
        api_pid=$!
        ARTICRAFT_VIEWER_API_HOST={{api_host}} ARTICRAFT_VIEWER_API_PORT={{api_port}} exec npm --prefix viewer/web run dev
        ;;
      *)
        echo "Unknown viewer mode: {{mode}}. Expected 'prod' or 'dev'." >&2
        exit 1
        ;;
    esac

viewer-dev api_host=host api_port=port:
    exec just viewer dev {{api_host}} {{api_port}}

viewer-api:
    uv run articraft-workbench --repo-root . rebuild-search-index
    exec uv run uvicorn viewer.api.app:app --reload --host {{host}} --port {{port}}

compile record_dir sdk_package="sdk":
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    sdk_package={{ quote(sdk_package) }}
    script_path="$record_dir/model.py"
    urdf_path="$record_dir/model.urdf"
    if [ ! -f "$script_path" ]; then
      echo "Record model not found: $script_path" >&2
      exit 1
    fi
    RECORD_SCRIPT="$script_path" RECORD_URDF="$urdf_path" RECORD_SDK="$sdk_package" uv run python - <<'PY'
    from pathlib import Path
    import os
    from agent.compiler import compile_urdf_report

    script = Path(os.environ["RECORD_SCRIPT"]).resolve()
    urdf_path = Path(os.environ["RECORD_URDF"]).resolve()
    sdk_package = os.environ["RECORD_SDK"]
    report = compile_urdf_report(script, sdk_package=sdk_package)
    urdf_path.write_text(report.urdf_xml, encoding="utf-8")
    print(f"Recompiled {script}")
    print(f"Wrote URDF to {urdf_path}")
    if report.warnings:
        print(f"Warnings: {len(report.warnings)}")
        for warning in report.warnings:
            print(f"- {warning.splitlines()[0]}")
    else:
        print("Warnings: 0")
    PY

viewer-web-install:
    npm --prefix viewer/web install

viewer-web-dev:
    npm --prefix viewer/web run dev

viewer-web-build:
    npm --prefix viewer/web run build

viewer-web-lint:
    npm --prefix viewer/web run lint

viewer-web-typecheck:
    npm --prefix viewer/web run typecheck
