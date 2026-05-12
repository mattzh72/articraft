# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

Articraft is an agentic system that generates articulated 3D objects from text prompts and reference images. It includes a custom articulated-object SDK, a multi-turn generation harness with tool use, provider adapters for OpenAI, Gemini, Anthropic, and OpenRouter, canonical local storage/dataset tooling, and a web-based Three.js viewer.

The SDK docs under `sdk/_docs/` are part of the agent authoring contract. Keep them aligned with intended agent behavior and baseline compile/tooling policy; do not document workflows there that the harness is supposed to own automatically.

## Common Commands

Use `uv run articraft ...` for product workflows. `just` is intentionally limited to setup, checks, and viewer startup.

```bash
uv sync --group dev
uv build
uv run articraft init
uv run articraft status
uv run --group dev pytest -q
uv run --group dev pytest tests/storage/test_repo.py -q
uv run --group dev pytest tests/sdk/test_imports.py::test_name -q

just setup
just format
just lint
just smoke-tests
just test-all

uv run articraft data check
uv run articraft env bootstrap
uv run articraft hooks check
uv run articraft hooks install
```

Generation and record workflows:

```bash
uv run articraft generate "prompt text"
uv run articraft generate --model gemini-3-flash-preview --image reference.png "prompt text"
uv run articraft draft "prompt text"
uv run articraft draft --image reference.png "prompt text"
uv run articraft rerun data/records/<id>
uv run articraft compile data/records/<id>
uv run articraft compile data/records/<id> --target visual
uv run articraft compile-all
uv run articraft compile-all --target full
uv run articraft compile-all --target full --strict
uv run articraft view data/records/<id>
```

Dataset workflows:

```bash
uv run articraft dataset run "prompt text" --category-slug <slug>
uv run articraft dataset batch-new <batch-id>
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --resume-policy failed_only
uv run articraft dataset validate
uv run articraft dataset manifest
uv run articraft dataset category upsert <slug> --title "Display Title"
uv run articraft dataset record delete <record-id>
uv run articraft workbench status
uv run articraft workbench search-index
```

Viewer/frontend workflows:

```bash
uv run articraft compile-all
just viewer
just viewer-dev
uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765
npm --prefix viewer/web run dev
npm --prefix viewer/web run build
npm --prefix viewer/web run lint
npm --prefix viewer/web run typecheck
```

## Architecture

### Package Layout

- `agent/` - Generation runtime. `runner.py` provides compatibility entry points, `single_run.py` and `harness.py` orchestrate the multi-turn LLM loop, `compiler.py` executes generated `model.py` files and materializes URDF/mesh outputs, `providers/` contains OpenAI/Gemini/Anthropic/OpenRouter adapters, `tools/` defines tool schemas, `prompts/` manages provider/profile prompt docs, and `tui/` handles terminal displays.
- `agent/batch_runner.py` - Dataset batch orchestration for tracked CSV specs. It validates rows, preallocates collision-resistant dataset/record IDs, runs rows concurrently, handles resume state, and writes batch run cache entries.
- `sdk/` - Generated object SDK. `sdk/v0/` is the public import surface; `sdk/_core/` owns shared geometry/export logic; `sdk/_docs/` and `sdk/_examples/` are loaded into agent authoring context.
- `storage/` - Canonical on-disk data layer. It owns layout, record/category/dataset/run stores, batch specs, materialization metadata, data validation, manifests, record author sync, and the SQLite-backed search index.
- `viewer/` - Local inspection tool. `viewer/api/` is FastAPI, and `viewer/web/` is a React + TypeScript + Tailwind + Three.js SPA using shadcn/ui components.
- `cli/` - Top-level `articraft` command and compatibility subcommands.
- `tests/` - pytest tests mirroring package structure.

### Data Flow

1. Prompt/reference image enters `articraft generate`, `dataset run`, or a batch row.
2. The generation harness builds provider-specific requests, runs a multi-turn tool loop, and writes generated `model.py`.
3. `agent/compiler.py` executes `model.py` and exports URDF/mesh artifacts.
4. Canonical record data is persisted under `data/records/<record_id>/` with `record.json`, `model.py`, provenance/cost/traces/inputs, and optional `dataset_entry.json`.
5. Regenerable materialization outputs are stored under `data/cache/record_materialization/<record_id>/`.
6. `viewer/api` serves records and cached artifacts; `viewer/web` renders and manages them.

### Storage Layout

- `data/records/<record_id>/` - Canonical record directories.
- `data/categories/` - Category metadata.
- `data/supercategories.json` - Supercategory grouping metadata.
- `data/batch_specs/` - Tracked dataset batch CSV specs.
- `data/local/workbench.json` - Local workbench collection.
- `data/cache/manifests/` - Derived manifests.
- `data/cache/record_materialization/<record_id>/` - Derived URDF, compile reports, and viewer assets.
- `data/cache/runs/<run_id>/` - Batch run results, allocations, staging, failures, and resume state.
- `data/cache/search/` - Search index cache.

## Dataset Batch CSV Schema

Batch specs live under `data/batch_specs/`; the filename stem is the `batch_spec_id` used by resume.

Required columns:

- `category_slug`
- `prompt`
- `provider`
- `model_id`
- `thinking_level`
- `max_turns`

Recommended/optional columns:

- `row_id` - Stable row identifier for resume. If omitted, rows default to `row_0001`, `row_0002`, etc.
- `category_title` - Required for categories that do not already exist.
- `max_cost_usd` - Optional per-row budget; blank rows inherit the batch CLI flag or `ARTICRAFT_MAX_COST_USD`.
- `label` - Free-form tracking label.
- `sdk_package` - Optional SDK package override; defaults to `sdk`.

Validation rules:

- `provider` must be `openai`, `gemini`, `anthropic`, or `openrouter`.
- `model_id` must agree with the inferred provider when inference is possible.
- `thinking_level` must be `low`, `med`, or `high`.
- `max_turns` must be a positive integer.
- `image_path` is intentionally unsupported in batch CSV v1.
- New batch specs should generally leave per-row cost caps blank unless requested and use `max_turns=100` by default.

Resume uses the same `batch_spec_id`, stable `row_id` values, prior allocations from `data/cache/runs/<run_id>/allocations.json`, and a resume policy of `failed_or_pending`, `failed_only`, or `all`. By default, resume rejects row spec changes unless `--allow-resume-spec-mismatch` is passed.

## Compile and Viewer Guidance

- Use `uv run articraft compile-all` before `just viewer` for fast visual-only viewer assets.
- Use `uv run articraft compile-all --target full` when collision-inclusive URDFs are needed in bulk.
- Add `--strict` only when validation-heavy geometry checks should fail the bulk compile.
- Use `uv run articraft compile data/records/<id>` for one-off full record recompiles; add `--target visual` for viewer-only assets.
- `just viewer` builds `viewer/web` and serves it through FastAPI on `127.0.0.1:8765`.
- `just viewer-dev` starts uvicorn plus the Vite dev server, with Vite on `:5173` proxying API requests to `:8765`.

## Code Style

- Python: target Python 3.11+; `.python-version` pins 3.12 locally and the project excludes Python 3.13. Use 4-space indentation, `from __future__ import annotations`, explicit type hints, and minimal imports.
- Ruff is configured in `pyproject.toml`: line length 100, target `py311`, lint rules `E`, `F`, `I`, ignoring `E501`. Use `just format` and `just lint` or the underlying `uv run ruff ...` commands.
- TypeScript/React: strict TypeScript, ESLint, Tailwind CSS v4 via Vite, shadcn/ui, and the `@/` alias for `viewer/web/src`.
- Follow local patterns before adding new abstractions; keep changes scoped to the relevant surface.

## Hooks and Commit Safety

`just setup` installs pre-commit/pre-push hooks and the managed post-commit hook. Pre-commit blocks sensitive/local paths, scans staged secrets, validates `data/` format, runs Ruff format/check, and runs viewer lint/typecheck for frontend changes. Pre-push runs the smoke test suite.

Blocked paths include `.env`, `data/cache/`, `data/local/`, workbench-only records, generated URDFs, and generated asset directories. Files under `data/` are exempt from the trailing-newline rule because generated dataset artifacts may omit final newlines.

## Environment

Provider keys go in `.env`:

- `OPENAI_API_KEYS` or `OPENAI_API_KEY`
- `GEMINI_API_KEYS`
- `ANTHROPIC_API_KEYS` or `ANTHROPIC_API_KEY`
- `OPENROUTER_API_KEYS` or `OPENROUTER_API_KEY`

Optional defaults include `ARTICRAFT_MAX_COST_USD` for per-run budgets and provider-specific knobs such as OpenAI transport/cache settings, Anthropic cache settings, and OpenRouter token/retry settings.

`articraft generate` defaults to `gpt-5.5-2026-04-23` with `--thinking-level high`. Provider inference handles known OpenAI, Gemini, Claude, and OpenRouter-style model IDs; pass `--provider` explicitly when using an ambiguous model name.

## Paper Dataset Counts

When editing the Articraft paper, distinguish raw generated records from the final curated dataset. The final paper dataset includes only retained 4-5 star objects; lower-rated 1/2/3-star records may exist in `data/records/` as bad examples or audit material but should not be counted as final dataset objects. Use "over 10K" for the final curated object count unless the retained 4-5-star subset is recomputed and the paper is intentionally updated.
