# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## What This Project Is

Articraft is a local-first harness for generating and inspecting articulated 3D objects from text prompts and reference images. The code repo owns the agent runtime, SDK, CLI, storage logic, viewer API, and React/Three.js viewer. Records live in a plain local data root, usually the gitignored `<repo-root>/data` or an external folder passed with `--data-dir` or `ARTICRAFT_DATA_DIR`.

If the user asks Claude Code to author Articraft data, follow [`EXTERNAL_AGENT_DATA.md`](EXTERNAL_AGENT_DATA.md). Use `uv run articraft external ...`; do not manually create record folders.

## Common Commands

```bash
uv sync --group dev
uv build
uv run articraft init
uv run articraft status
uv run articraft library check --require-records
uv run --group dev pytest -q

just setup
just format
just lint
just smoke-tests
just test-all
```

Generation and record workflows:

```bash
uv run articraft generate "prompt text"
uv run articraft generate --model gemini-3-flash-preview --image reference.png "prompt text"
uv run articraft draft "prompt text"
uv run articraft draft --image reference.png "prompt text"
uv run articraft rerun <record-id>
uv run articraft fork <record-id> "edit prompt"
uv run articraft compile <record-id>
uv run articraft view <record-id>
uv run articraft library list
uv run articraft library rebuild-manifest
uv run articraft library set-category <record-id> <category_slug>
```

Viewer/frontend workflows:

```bash
just viewer
just viewer-dev
uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765
npm --prefix viewer/web run dev
npm --prefix viewer/web run build
npm --prefix viewer/web run lint
npm --prefix viewer/web run typecheck
```

## Architecture

- `agent/` - Generation runtime, provider adapters, prompt compiler/loader, tools, cost tracking, and single-record orchestration.
- `sdk/` - Generated object SDK and agent-facing docs/examples.
- `storage/` - Local data layout, records, categories, `records_manifest.jsonl`, run caches, materialization metadata, and validation.
- `viewer/api/` - FastAPI surface for browsing local records and serving materialized assets.
- `viewer/web/` - React/TypeScript/Three.js viewer.
- `cli/` - Top-level `articraft` command.
- `tests/` - pytest tests mirroring package structure.

## Data Flow

1. Prompt/reference image enters `articraft generate`, `draft`, `fork`, `rerun`, or `external init`.
2. The generation harness builds provider-specific requests, runs a multi-turn tool loop, and writes generated `model.py`.
3. `agent/compiler.py` executes `model.py` and exports URDF/mesh artifacts.
4. Canonical record data is persisted under `<data-root>/records/<record_id>/` with `record.json` and revision artifacts under `revisions/<revision_id>/`.
5. `records_manifest.jsonl` stores searchable local-library summaries, one row per real record.
6. Regenerable materialization outputs live under `<data-root>/cache/record_materialization/<record_id>/`.
7. The viewer reads the manifest for browse/search and direct record files for inspection.

## Storage Layout

- `<data-root>/records/<record_id>/` - Canonical record directories.
- `<data-root>/records_manifest.jsonl` - Canonical local library manifest.
- `<data-root>/categories/` - Optional category metadata.
- `<data-root>/cache/record_materialization/<record_id>/` - Derived URDF, compile reports, and viewer assets.
- `<data-root>/cache/runs/<run_id>/` - Single-run metadata, staging, and results.
- `<data-root>/system_prompts/` - Shared prompt snapshots.

## Compile and Viewer Guidance

- Use `uv run articraft compile <record-id>` for the one record you need to inspect.
- `just viewer` builds `viewer/web` and serves it through FastAPI on `127.0.0.1:8765`.
- `just viewer-dev` starts uvicorn plus the Vite dev server, with Vite on `:5173` proxying API requests to `:8765`.

## Code Style

- Python: target Python 3.11+; `.python-version` pins 3.12 locally and the project excludes Python 3.13. Use 4-space indentation, `from __future__ import annotations`, explicit type hints, and minimal imports.
- Ruff is configured in `pyproject.toml`: line length 100, target `py311`, lint rules `E`, `F`, `I`, ignoring `E501`. Use `just format` and `just lint` or the underlying `uv run ruff ...` commands.
- TypeScript/React: strict TypeScript, ESLint, Tailwind CSS v4 via Vite, shadcn/ui, and the `@/` alias for `viewer/web/src`.
- Follow local patterns before adding new abstractions; keep changes scoped to the relevant surface.

## Environment

Provider keys go in `.env`:

- `OPENAI_API_KEYS` or `OPENAI_API_KEY`
- `GEMINI_API_KEYS`
- `ANTHROPIC_API_KEYS` or `ANTHROPIC_API_KEY`
- `OPENROUTER_API_KEYS` or `OPENROUTER_API_KEY`

Optional defaults include `ARTICRAFT_MODEL`, `ARTICRAFT_THINKING_LEVEL`, `ARTICRAFT_MAX_COST_USD`, and `ARTICRAFT_DATA_DIR`.
