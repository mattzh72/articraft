# Repository Guidelines

## Project Structure & Module Organization
`agent/` contains the generation runtime, provider adapters, prompt compiler/loader, tools, cost tracking, TUI helpers, and batch orchestration. `storage/` owns the canonical `data/` layout, records, categories, dataset metadata, batch specs, run caches, materialization metadata, and search indexes. `sdk/` and `sdk/_core/` define the articulated-object SDK layers; `sdk/_docs/` and `sdk/_examples/` are agent-facing authoring assets. `viewer/api/` exposes the FastAPI surface, and `viewer/web/` is the React/TypeScript/Three.js viewer. `cli/` contains the `articraft` entry points. `tests/` mirrors the main packages with focused smoke/regression coverage.

## Build, Test, and Development Commands
Use `uv run articraft ...` for product workflows, and `just` for local setup/check/viewer shortcuts. Run `just` to list available shortcuts.

- `uv sync --group dev` installs Python and development dependencies.
- `just setup` bootstraps `.env`, syncs dependencies, installs pre-commit/pre-push plus the managed post-commit hook, installs viewer dependencies when `npm` exists, and initializes storage.
- `uv build` creates the wheel and source distribution in `dist/`.
- `uv run articraft init` creates the canonical `data/` tree and local workbench state.
- `uv run articraft status` shows dataset and workbench status.
- `uv run --group dev pytest -q` runs the full Python regression suite.
- `just smoke-tests` runs the fast pre-push suite; `just test-all` runs the full Python suite.
- `just format` runs `ruff format .`; `just lint` runs `ruff check .`.
- `uv run articraft data check` validates checked-in data format.
- `uv run articraft hooks check` verifies the managed git hook setup.
- `uv run articraft env bootstrap` creates `.env` from `.env.example` without overwriting existing secrets.

## Generation, Dataset, and Viewer Commands
- External agent data generation must follow [`EXTERNAL_AGENT_DATA.md`](EXTERNAL_AGENT_DATA.md). If the user asks Codex, Claude Code, or another external harness to generate Articraft data, use `uv run articraft external ...`; do not manually create records or use an alternate workflow.
- `uv run articraft generate "prompt text"` runs generation in workbench mode.
- `uv run articraft generate --model gemini-3-flash-preview --image reference.png "prompt text"` overrides model and adds a reference image.
- `uv run articraft draft "prompt text"` creates a draft workbench record without running generation.
- `uv run articraft draft --image reference.png "prompt text"` stores a reference image with the draft.
- `uv run articraft rerun data/records/<id>` reruns generation for an existing record.
- `uv run articraft compile data/records/<id>` recompiles a record into `data/cache/record_materialization/<id>/`.
- `uv run articraft compile-all` is the recommended fast visual-only materialization path before viewer browsing.
- `uv run articraft compile-all --target full` runs non-strict full bulk compile; add `--strict` for validation-heavy full compile.
- `uv run articraft dataset run "prompt text" --category-slug <slug>` generates one record directly into a dataset category.
- `uv run articraft dataset batch-new <batch-id>` creates `data/batch_specs/<batch-id>.csv` with the canonical v1 header.
- `uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto` runs a tracked batch.
- `uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume` resumes the latest prior run for that spec.
- `uv run articraft workbench search-index` rebuilds the viewer search index.
- `just viewer` starts the built viewer flow; `just viewer-dev` starts uvicorn and Vite together.
- `uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765` starts only the API.
- `npm --prefix viewer/web run dev`, `build`, `lint`, and `typecheck` run frontend-only workflows.

## Coding Style & Naming Conventions
Target Python 3.11+; `.python-version` pins 3.12 for local `uv` use and the project excludes Python 3.13. Keep 4-space indentation, `from __future__ import annotations`, explicit type hints, and small module-level helpers for CLI wiring. Python formatting and import checks are handled by Ruff (`line-length = 100`, target `py311`, rules `E`, `F`, `I`, ignoring `E501`). Use `snake_case` for functions/modules/variables and `PascalCase` for models/classes. For `viewer/web`, use strict TypeScript, ESLint, Tailwind CSS v4, shadcn/ui components, and the `@/` import alias.

## Testing Guidelines
Tests run under `pytest` with importlib mode and xdist auto/worksteal by default. Add coverage under the mirrored package path, name files `test_<feature>.py`, and prefer native pytest functions and fixtures with plain `assert` statements. Keep coverage focused on fast import, storage, CLI, viewer API, workbench, SDK, and integration smoke checks. For prompt regressions, prefer durable behavioral checks over brittle formatting or line-budget assertions; relax or remove stale prompt assertions in the same change that updates the intended prompt contract.

## Dataset Batch Specs
Batch specs live under `data/batch_specs/`. The CSV filename stem is the `batch_spec_id` used for resume. Required columns are `category_slug`, `prompt`, `provider`, `model_id`, `thinking_level`, and `max_turns`; `category_title` is required for new categories. Recommended/optional columns are `row_id`, `max_cost_usd`, `label`, and `sdk_package` (`sdk` is the default when omitted).

Supported providers are `openai`, `gemini`, `anthropic`, and `openrouter`. `thinking_level` must be `low`, `med`, or `high`; `max_turns` must be a positive integer. If `row_id` is omitted, rows default to `row_0001`, `row_0002`, and so on, but stable explicit IDs are better for resume. `image_path` is intentionally unsupported in batch CSV v1. For new batch specs, leave per-row cost caps blank unless explicitly requested and use `max_turns=100` by default.

## Commit & Pull Request Guidelines
Recent commits use short, imperative subjects such as `Move prompt compiler under agent` and `Consolidate SDK around canonical core profiles`. Keep commit titles concise and scoped to one logical change. PRs should describe the affected surface (`agent`, `storage`, `sdk`, `viewer`, or `cli`), include the exact `uv`, `just`, and `npm` commands run, and attach screenshots only when API or viewer behavior changes.

Pre-commit blocks sensitive/local paths, scans staged content for provider secrets, validates data format for `data/` changes, runs Ruff format/check, and runs viewer lint/typecheck for frontend changes. Pre-push runs smoke tests. Files under `data/` are exempt from the trailing-newline requirement; `.env`, `data/cache/`, `data/local/`, workbench-only records, generated URDFs, and generated asset dirs are intentionally blocked from commits.

## Configuration Tips
Provider code loads `.env`. Set `OPENAI_API_KEYS` or `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS` or `ANTHROPIC_API_KEY`, and `OPENROUTER_API_KEYS` or `OPENROUTER_API_KEY` as needed. `ARTICRAFT_MAX_COST_USD` can set a default per-run budget. `generate` defaults to `gpt-5.5-2026-04-23` with `--thinking-level high`; provider inference works for known OpenAI, Gemini, Claude, and OpenRouter-style model IDs, otherwise pass `--provider`.

## Paper Dataset Counts
When editing the Articraft paper, distinguish raw generated records from the final curated dataset. The final paper dataset includes only retained 4-5 star objects; lower-rated 1/2/3-star records may exist in `data/records/` as bad examples or audit material but should not be counted as final dataset objects. Use "over 10K" for the final curated object count unless the retained 4-5-star subset is recomputed and the paper is intentionally updated.

## Agent Docs Contract
The SDK docs under `sdk/_docs/` are part of the agent authoring contract in this repository. Keep them aligned with the intended agent behavior and baseline compile/tooling policy; do not document agent-facing workflows there that the harness is supposed to own automatically.
