# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

Articraft is an agentic system that generates articulated 3D objects (URDF models) from text prompts using LLMs (OpenAI, Gemini). It includes a custom geometry SDK, a generation agent with tool-use, a local storage/dataset layer, and a web-based 3D viewer.

## Common Commands

Use `uv` for direct Python commands and `just` for the agent-facing shortcuts that help with record iteration. Run `just` or check [`justfile`](/Users/matthewzhou/articraft/justfile) for the full recipe list.
When a recipe supports optional settings, pass them as `just` overrides before the recipe name, for example `just model=gemini-3-flash-preview image=reference.png wb "prompt text"`.

```bash
uv sync --group dev            # Install Python dependencies
uv build                       # Build wheel and sdist
uv run --group dev pytest -q   # Run pytest directly
uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765  # Start API directly

just setup                     # First-time setup shortcut
just smoke-tests               # Run pytest suite
just compile-all-visual        # Fast path: materialize visual meshes for viewer browsing
just compile-all               # Full path: materialize full artifacts, including collisions
just compile-all-strict        # Full path with validation-heavy geometry checks
just name=<batch-id> batch-spec-new   # Create an empty tracked batch CSV with the canonical header
just spec=data/batch_specs/<batch-id>.csv concurrency=8 dataset-batch  # Run a tracked dataset batch CSV
just wb-init "prompt text"     # Create a draft workbench record to edit manually
just compile data/records/<id> # Recompile model.py -> model.urdf
just compile-visual data/records/<id> # Recompile only visual viewer artifacts for one record
just rerun data/records/<id>   # Re-run generation for an existing record
just search-index              # Rebuild the workbench search index
just viewer                    # Start viewer
just viewer-dev                # Start viewer in dev mode
just wb "prompt text"          # Generate an object from a prompt
just image=reference.png wb-init "prompt text"   # Draft a record with a stored reference image
just model=gemini-3-flash-preview image=reference.png wb "prompt text"  # Override defaults
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8  # Run a dataset batch
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8 --resume  # Resume latest run for that spec
```

### Running a single test

```bash
uv run --group dev pytest tests/storage/test_repo.py -q
uv run --group dev pytest tests/sdk/test_imports.py::test_name -q
```

### Frontend only

```bash
npm --prefix viewer/web run dev         # Vite dev server
npm --prefix viewer/web run build       # Production build
npm --prefix viewer/web run lint        # ESLint
npm --prefix viewer/web run typecheck   # tsc
```

## Architecture

### Package layout

- **`agent/`** — Generation runtime. `runner.py` is the main entrypoint; `harness.py` contains `ArticraftAgent` which orchestrates the multi-turn LLM loop; `compiler.py` executes generated scripts and produces URDF; `providers/` has OpenAI and Gemini LLM adapters; `tools/` defines the tool-use registry; `prompts/` has system prompt templates; `tui/` has the terminal display.
- **`agent/batch_runner.py`** — Dataset batch orchestration for tracked CSV specs. Validates per-row settings, preallocates dataset IDs, runs rows concurrently with the batch TUI, and resumes prior runs in place.
- **`sdk/`** — The articulated-object SDK that generated code imports. `sdk/v0/` is the main implementation. Exports geometry primitives (Box, Cylinder, Sphere, etc.), boolean ops, spline helpers, loft/sweep/extrude, placement utilities, and validation (overlap/collision checking). Models are `ArticulatedObject` composed of `Part`s with `Articulation`s.
- **`sdk_hybrid/`** — Alternate SDK variant (same interface, different internals).
- **`storage/`** — On-disk data layer. `layout.py` defines the canonical `data/` directory structure; `repo.py` is `StorageRepo`; `records.py`, `collections.py`, `datasets.py`, `runs.py`, `manifests.py` handle CRUD. `batch_specs.py` manages tracked CSV specs and `dataset_workflow.py` holds shared dataset promotion helpers. `search.py` manages a SQLite search index.
- **`viewer/`** — Local inspection tool. `viewer/api/` is a FastAPI app (`app.py`) served by uvicorn; `viewer/web/` is a React + TypeScript + Tailwind + Three.js SPA (Vite, shadcn/ui components).
- **`cli/`** — CLI entrypoints registered as `articraft-dataset` and `articraft-workbench` in pyproject.toml.
- **`tests/`** — pytest tests mirroring package structure. Smoke-level checks (imports, storage layout, API).

### Data flow

1. User prompt → `agent/runner.py` → `ArticraftAgent` (multi-turn LLM with tool-use) → generated `model.py` that uses `sdk`
2. `agent/compiler.py` executes `model.py` → exports URDF XML + mesh assets
3. Results persisted to `data/records/<record_id>/` (record.json, model.py, model.urdf, provenance.json, cost.json, traces/, assets/)
4. `viewer/api` serves records; `viewer/web` renders them with Three.js

Dataset batch flow:

1. Tracked CSV spec in `data/batch_specs/<batch-id>.csv`
2. `articraft-dataset run-batch ... --concurrency N` validates rows, preallocates dataset IDs, and creates one batch run record
3. Rows execute concurrently with per-row `provider`, `model_id`, `thinking_level`, `max_turns`, and `sdk_package`
4. Successful rows are promoted into canonical `data/records/<record_id>/` storage; resumable row state and latest row results stay under `data/cache/runs/<run_id>/`

### Storage layout (`data/`)

- `data/records/<record_id>/` — One directory per generated object
- `data/categories/` — Dataset category metadata and prompt batches
- `data/batch_specs/` — Tracked dataset batch CSV specs
- `data/local/workbench.json` — Local workbench collection
- `data/cache/` — Manifests, run logs, batch state, search index (all regenerable)
- `data/cache/runs/<run_id>/state/` — Per-row batch resume state
- `data/cache/runs/<run_id>/allocations.json` — Reserved dataset and record IDs for a batch run

### Dataset batch CSV schema

Use tracked specs under `data/batch_specs/` with these columns:

- Required: `category_slug`, `prompt`, `provider`, `model_id`, `thinking_level`, `max_turns`, `sdk_package`
- Required for new categories: `category_title`
- Optional: `row_id`, `label`

Validation rules:

- `provider` must be `openai` or `gemini`
- `thinking_level` must be `low`, `med`, or `high`
- `max_turns` must be a positive integer
- `sdk_package` is normalized through the same validation path as single runs
- `image_path` is intentionally unsupported in v1
- If `row_id` is omitted, it defaults to `row_0001`, `row_0002`, and so on

### Viewer architecture

The API (`viewer/api/app.py`) is a FastAPI app that serves both the REST API and the built frontend (`viewer/web/dist/`). In dev mode, Vite runs on :5173 and proxies `/api` to the uvicorn backend on :8765.

### Viewer compile guidance

- For viewer-first inspection, prefer `just compile-all-visual` before `just viewer`. This is the fast path and only materializes visual meshes.
- Use `just compile-all` when you need full artifacts, including collision geometry.
- Use `just compile-all-strict` when you need the validation-heavy full compile path.
- For one-off records, `just compile-visual data/records/<id>` is the fast single-record path and `just compile data/records/<id>` is the full single-record path.

## Code Style

- Python: `ruff` with line-length 100, target py311. Rules: E, F, I (ignoring E501). Use `from __future__ import annotations`, explicit type hints. 4-space indent.
- TypeScript/React: ESLint + TypeScript strict mode. `@` alias maps to `viewer/web/src/`. Tailwind CSS v4 via Vite plugin. shadcn/ui component library.
- Imports use `@` prefix in the viewer frontend (e.g., `@/components/...`).

## Pre-commit Hooks

Pre-commit runs on commit: forbidden path check, secret scanning, text hygiene (trailing whitespace/newlines), ruff format, ruff check, viewer lint, viewer typecheck. Smoke tests run on pre-push.

Files under `data/` are exempt from the trailing-newline requirement. Paths like `.env`, `data/cache/`, `data/local/`, generated URDFs, and asset dirs are blocked from commits.

## Environment

Provider keys go in `.env`: `OPENAI_API_KEYS` (preferred) or `OPENAI_API_KEY` for OpenAI, `GEMINI_API_KEYS` for Gemini. The `.env` file is gitignored.
