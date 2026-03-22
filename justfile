default:
    @just --list

host := "127.0.0.1"
port := "8765"
model := "gpt-5.4"
thinking := "high"
sdk := "sdk"
image := ""
dataset_id := ""
mode := "prod"
api_host := host
api_port := port
sdk_package := "sdk"
sdk_override := ""
viewer_target := "/"
concurrency := "auto"
limit := ""
name := ""

setup:
    # Create a local env template once; never overwrite an existing secrets file.
    uv run python scripts/bootstrap_env.py .
    uv sync --group dev
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    @if command -v npm >/dev/null 2>&1; then \
        npm --prefix viewer/web ci; \
        npm --prefix viewer/web run typecheck; \
    else \
        echo "npm not found; skipping viewer/web dependency install."; \
        echo "Install Node.js and npm to run the viewer and frontend hooks."; \
    fi
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
    uv run --group dev pytest -q

perf:
    uv run python -m performance

wb prompt:
    #!/usr/bin/env bash
    set -euo pipefail
    model={{ quote(model) }}
    thinking={{ quote(thinking) }}
    sdk={{ quote(sdk) }}
    image={{ quote(image) }}
    case "$sdk" in
      sdk|base)
        sdk_package="sdk"
        ;;
      sdk_hybrid|hybrid)
        sdk_package="sdk_hybrid"
        ;;
      *)
        echo "Unsupported sdk '$sdk'. Supported values: sdk, base, sdk_hybrid, hybrid." >&2
        exit 1
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
    cmd=(
      uv run python agent/runner.py
      --repo-root .
      --prompt {{ quote(prompt) }}
      --provider "$provider"
      --model "$model"
      --thinking "$thinking"
      --sdk-package "$sdk_package"
    )
    if [ -n "$image" ]; then
      cmd+=(--image "$image")
    fi
    exec "${cmd[@]}"

wb-init prompt:
    #!/usr/bin/env bash
    set -euo pipefail
    model={{ quote(model) }}
    thinking={{ quote(thinking) }}
    sdk={{ quote(sdk) }}
    image={{ quote(image) }}
    case "$sdk" in
      sdk|base)
        sdk_package="sdk"
        ;;
      sdk_hybrid|hybrid)
        sdk_package="sdk_hybrid"
        ;;
      *)
        echo "Unsupported sdk '$sdk'. Supported values: sdk, base, sdk_hybrid, hybrid." >&2
        exit 1
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
    cmd=(
      uv run articraft-workbench
      --repo-root .
      init-record
      {{ quote(prompt) }}
      --provider "$provider"
      --model-id "$model"
      --thinking-level "$thinking"
      --sdk-package "$sdk_package"
    )
    if [ -n "$image" ]; then
      cmd+=(--image "$image")
    fi
    exec "${cmd[@]}"

dataset-validate:
    uv run articraft-dataset --repo-root . validate

dataset-manifest:
    uv run articraft-dataset --repo-root . build-manifest

prune-cache-preview:
    uv run articraft-dataset --repo-root . prune-cache

prune-cache:
    #!/usr/bin/env bash
    set -euo pipefail
    uv run articraft-dataset --repo-root . prune-cache
    printf "\nRemove these empty cache directories? [y/N]: "
    read -r confirm
    case "$confirm" in
      y|Y|yes|YES)
        exec uv run articraft-dataset --repo-root . prune-cache --execute
        ;;
      *)
        echo "Aborting."
        exit 1
        ;;
    esac

dataset-delete-category-preview category_slug:
    uv run articraft-dataset --repo-root . delete-category --category-slug {{ quote(category_slug) }}

dataset-delete-category category_slug:
    #!/usr/bin/env bash
    set -euo pipefail
    category_slug={{ quote(category_slug) }}
    uv run articraft-dataset --repo-root . delete-category --category-slug "$category_slug"
    printf "\nType the category slug '%s' to permanently delete it: " "$category_slug"
    read -r confirm_slug
    if [ "$confirm_slug" != "$category_slug" ]; then
      echo "Confirmation mismatch. Aborting."
      exit 1
    fi
    exec uv run articraft-dataset --repo-root . delete-category \
      --category-slug "$category_slug" \
      --execute \
      --confirm-slug "$confirm_slug"

dataset-delete-record-preview record_path:
    uv run articraft-dataset --repo-root . delete-record --record-path {{ quote(record_path) }}

dataset-delete-record record_path:
    #!/usr/bin/env bash
    set -euo pipefail
    record_path={{ quote(record_path) }}
    uv run articraft-dataset --repo-root . delete-record --record-path "$record_path"
    record_id="$(basename "$record_path")"
    printf "\nType the record ID '%s' to permanently delete it: " "$record_id"
    read -r confirm_record_id
    if [ "$confirm_record_id" != "$record_id" ]; then
      echo "Confirmation mismatch. Aborting."
      exit 1
    fi
    exec uv run articraft-dataset --repo-root . delete-record \
      --record-path "$record_path" \
      --execute \
      --confirm-record-id "$confirm_record_id"

dataset-promote record_ref category_title:
    #!/usr/bin/env bash
    set -euo pipefail
    record_ref={{ quote(record_ref) }}
    category_title={{ quote(category_title) }}
    dataset_id={{ quote(dataset_id) }}
    if [ -n "$dataset_id" ]; then
      exec uv run articraft-dataset --repo-root . promote-record \
        "$record_ref" \
        "$category_title" \
        --dataset-id "$dataset_id"
    fi
    exec uv run articraft-dataset --repo-root . promote-record \
      "$record_ref" \
      "$category_title"

dataset-batch spec_path="":
    #!/usr/bin/env bash
    set -euo pipefail
    spec_path={{ quote(spec_path) }}
    concurrency_value={{ quote(concurrency) }}
    if [ -z "$spec_path" ]; then
      echo "Usage: just concurrency=<n> dataset-batch path/to/batch.csv" >&2
      exit 1
    fi
    exec uv run articraft-dataset --repo-root . run-batch "$spec_path" --concurrency "$concurrency_value"

batch-spec-new:
    #!/usr/bin/env bash
    set -euo pipefail
    batch_name={{ quote(name) }}
    if [ -z "$batch_name" ]; then
      echo "Set name=<batch-id>" >&2
      exit 1
    fi
    spec_path="data/batch_specs/${batch_name}.csv"
    if [ -e "$spec_path" ]; then
      echo "Batch spec already exists: $spec_path" >&2
      exit 1
    fi
    mkdir -p "$(dirname "$spec_path")"
    printf '%s\n' 'row_id,category_slug,category_title,prompt,provider,model_id,thinking_level,max_turns,sdk_package,label' >"$spec_path"
    echo "Created $spec_path"

search-index:
    uv run articraft-workbench --repo-root . rebuild-search-index

rerun record:
    #!/usr/bin/env bash
    set -euo pipefail
    record={{ quote(record) }}
    model_override={{ quote(model) }}
    thinking_override={{ quote(thinking) }}
    sdk_override={{ quote(sdk_override) }}
    cmd=(
      uv run articraft-workbench
      --repo-root .
      rerun-record
      "$record"
    )
    if [ -n "$model_override" ]; then
      cmd+=(--model-id "$model_override")
    fi
    if [ -n "$thinking_override" ]; then
      cmd+=(--thinking-level "$thinking_override")
    fi
    if [ -n "$sdk_override" ]; then
      cmd+=(--sdk-package "$sdk_override")
    fi
    exec "${cmd[@]}"

viewer mode_arg='':
    #!/usr/bin/env bash
    set -euo pipefail
    viewer_target={{ quote(viewer_target) }}
    mode_value={{ quote(mode) }}
    if [ -n {{ quote(mode_arg) }} ]; then
      mode_value={{ quote(mode_arg) }}
    fi
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
    case "$mode_value" in
      prod)
        npm --prefix viewer/web run build
        (
          for _ in {1..60}; do
            if curl -fsS http://{{api_host}}:{{api_port}}/health >/dev/null; then
              open "http://{{api_host}}:{{api_port}}${viewer_target}" >/dev/null 2>&1 || uv run python -m webbrowser "http://{{api_host}}:{{api_port}}${viewer_target}" >/dev/null 2>&1 || true
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
              open "http://127.0.0.1:5173${viewer_target}" >/dev/null 2>&1 || uv run python -m webbrowser "http://127.0.0.1:5173${viewer_target}" >/dev/null 2>&1 || true
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
        echo "Unknown viewer mode: $mode_value. Expected 'prod' or 'dev'." >&2
        exit 1
        ;;
    esac

viewer-dev:
    exec just api_host={{ quote(api_host) }} api_port={{ quote(api_port) }} viewer dev

view record:
    #!/usr/bin/env bash
    set -euo pipefail
    record_ref={{ quote(record) }}
    record_id="$(
      RECORD_REF="$record_ref" uv run python - <<'PY'
    import os
    from pathlib import Path

    from storage.repo import StorageRepo

    repo = StorageRepo(Path(".").resolve())
    candidate = Path(os.environ["RECORD_REF"]).expanduser()
    if candidate.exists():
        resolved = candidate.resolve()
        records_root = repo.layout.records_root.resolve()
        try:
            relative = resolved.relative_to(records_root)
        except ValueError as exc:
            raise SystemExit(f"Record path must be inside {records_root}") from exc
        if len(relative.parts) != 1 or not resolved.is_dir():
            raise SystemExit(f"Record path must point to a direct child of {records_root}")
        print(relative.parts[0])
        raise SystemExit(0)

    record_id = os.environ["RECORD_REF"].strip()
    if not record_id:
        raise SystemExit("Record reference is required.")
    if repo.layout.record_metadata_path(record_id).exists():
        print(record_id)
        raise SystemExit(0)
    raise SystemExit(f"Record not found: {os.environ['RECORD_REF']}")
    PY
    )"
    viewer_target="/viewer?record=${record_id}"
    case "{{mode}}" in
      prod)
        viewer_url="http://{{api_host}}:{{api_port}}${viewer_target}"
        health_url="http://{{api_host}}:{{api_port}}/health"
        ;;
      dev)
        viewer_url="http://127.0.0.1:5173${viewer_target}"
        health_url="http://127.0.0.1:5173"
        ;;
      *)
        echo "Unknown viewer mode: {{mode}}. Expected 'prod' or 'dev'." >&2
        exit 1
        ;;
    esac
    if curl -fsS "$health_url" >/dev/null 2>&1; then
      open "$viewer_url" >/dev/null 2>&1 || uv run python -m webbrowser "$viewer_url" >/dev/null 2>&1 || true
      exit 0
    fi
    exec just mode={{ quote(mode) }} api_host={{ quote(api_host) }} api_port={{ quote(api_port) }} viewer_target="$viewer_target" viewer

viewer-api:
    uv run articraft-workbench --repo-root . rebuild-search-index
    exec uv run uvicorn viewer.api.app:app --reload --host {{api_host}} --port {{api_port}}

compile record_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    if [ ! -f "$record_dir/model.py" ]; then
      echo "Record model not found: $record_dir/model.py" >&2
      exit 1
    fi
    uv run python scripts/materialize_record.py --repo-root . "$record_dir"

compile-visual record_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    if [ ! -f "$record_dir/model.py" ]; then
      echo "Record model not found: $record_dir/model.py" >&2
      exit 1
    fi
    uv run python scripts/materialize_record.py --repo-root . --target visual "$record_dir"

profile-compile-visual record_dir sort='cumtime' rows='40' out='' timeout='0':
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    sort_key={{ quote(sort) }}
    rows={{ quote(rows) }}
    out_path={{ quote(out) }}
    timeout_seconds={{ quote(timeout) }}
    if [ ! -f "$record_dir/model.py" ]; then
      echo "Record model not found: $record_dir/model.py" >&2
      exit 1
    fi
    if [ -z "$out_path" ]; then
      out_path="$record_dir/traces/compile_visual.prof"
    fi
    mkdir -p "$(dirname "$out_path")"
    echo "Profiling visual compile for $record_dir"
    echo "Profile output: $out_path"
    echo "Timeout wrapper seconds: $timeout_seconds"
    URDF_COMPILE_TIMEOUT_SECONDS="$timeout_seconds" uv run python -m cProfile -o "$out_path" \
      scripts/materialize_record.py --repo-root . --target visual "$record_dir"
    echo ""
    echo "Top $rows functions sorted by $sort_key"
    PROFILE_PATH="$out_path" PROFILE_SORT="$sort_key" PROFILE_ROWS="$rows" uv run python - <<'PY'
    import os
    import pstats

    profile_path = os.environ["PROFILE_PATH"]
    sort_key = os.environ["PROFILE_SORT"]
    rows = int(os.environ["PROFILE_ROWS"])
    pstats.Stats(profile_path).sort_stats(sort_key).print_stats(rows)
    PY

compile-strict record_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    if [ ! -f "$record_dir/model.py" ]; then
      echo "Record model not found: $record_dir/model.py" >&2
      exit 1
    fi
    uv run python scripts/materialize_record.py --repo-root . --validate --strict-geom-qc "$record_dir"

compile-unsafe record_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    sdk_package={{ quote(sdk_package) }}
    script_path="$record_dir/model.py"
    if [ ! -f "$script_path" ]; then
      echo "Record model not found: $script_path" >&2
      exit 1
    fi
    RECORD_DIR="$record_dir" RECORD_SCRIPT="$script_path" RECORD_SDK="$sdk_package" uv run python - <<'PY'
    import os
    from agent.compiler import compile_urdf_report
    from storage.repo import StorageRepo
    from pathlib import Path

    record_dir = Path(os.environ["RECORD_DIR"]).resolve()
    script = Path(os.environ["RECORD_SCRIPT"]).resolve()
    sdk_package = os.environ["RECORD_SDK"]
    repo_root = Path.cwd().resolve()
    repo = StorageRepo(repo_root)
    record_id = record_dir.name
    urdf_path = repo.layout.record_materialization_urdf_path(record_id)
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    report = compile_urdf_report(script, sdk_package=sdk_package, run_checks=False)
    urdf_path.write_text(report.urdf_xml, encoding="utf-8")
    print(f"Unsafely recompiled {script}")
    print(f"Wrote URDF to {urdf_path}")
    print("Checks: skipped")
    if report.warnings:
        print(f"Warnings: {len(report.warnings)}")
        for warning in report.warnings:
            print(f"- {warning.splitlines()[0]}")
    else:
        print("Warnings: 0")
    PY

unroll-trajectory record_dir:
    #!/usr/bin/env bash
    set -euo pipefail
    record_dir={{ quote(record_dir) }}
    if [ ! -f "$record_dir/record.json" ]; then
      echo "Record metadata not found: $record_dir/record.json" >&2
      exit 1
    fi
    uv run python scripts/unroll_trajectory.py --repo-root . "$record_dir"

unroll-all-trajectories:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/unroll_all_trajectories.py --repo-root . --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

compile-all:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/compile_all_records.py --repo-root . --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

compile-all-visual:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/compile_all_records.py --repo-root . --target visual --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

force-compile-all:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/compile_all_records.py --repo-root . --force --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

force-compile-all-visual:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/compile_all_records.py --repo-root . --target visual --force --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

compile-all-strict:
    #!/usr/bin/env bash
    set -euo pipefail
    cmd=(uv run python scripts/compile_all_records.py --repo-root . --strict --concurrency {{ quote(concurrency) }})
    if [ -n {{ quote(limit) }} ]; then
      cmd+=(--limit {{ quote(limit) }})
    fi
    exec "${cmd[@]}"

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
