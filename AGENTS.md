# Repository Guidelines

## Project Structure & Module Organization
`agent/` contains the generation runtime, provider adapters, prompt compiler/loader, tools, cost tracking, TUI helpers, and single-record orchestration. `storage/` owns the gitignored local `data/` layout, records, categories, `records_manifest.jsonl`, run caches, and materialization metadata. `sdk/` and `sdk/_core/` define the articulated-object SDK layers; `sdk/_docs/` and `sdk/_examples/` are agent-facing authoring assets. `viewer/api/` exposes the FastAPI surface, and `viewer/web/` is the React/TypeScript/Three.js viewer. `cli/` contains the `articraft` entry points. `tests/` mirrors the main packages with focused smoke/regression coverage.

## Build, Test, and Development Commands
Use `uv run articraft ...` for product workflows, and `just` for local setup/check/viewer shortcuts. Run `just` to list available shortcuts.

- `uv sync --group dev` installs Python and development dependencies.
- `just setup` bootstraps `.env`, syncs dependencies, installs viewer dependencies when `npm` exists, and initializes storage.
- `uv build` creates the wheel and source distribution in `dist/`.
- `uv run articraft init` creates the gitignored local `data/` tree and empty library manifest.
- `uv run articraft status` shows local library status.
- `uv run --group dev pytest -q` runs the full Python regression suite.
- `just smoke-tests` runs the fast pre-push suite; `just test-all` runs the full Python suite.
- `just format` runs `ruff format .`; `just lint` runs `ruff check .`.
- `uv run articraft library check` validates local library format.
- `uv run articraft env bootstrap` creates `.env` from `.env.example` without overwriting existing secrets.

## Generation, Dataset, and Viewer Commands
- External agent data generation must follow [`EXTERNAL_AGENT_DATA.md`](EXTERNAL_AGENT_DATA.md). If the user asks Codex, Claude Code, Cursor, or another external harness to generate Articraft data, use `uv run articraft external ...`; do not manually create records or use an alternate workflow.
- `uv run articraft generate "prompt text"` writes a local library record.
- `uv run articraft generate --model gemini-3-flash-preview --image reference.png "prompt text"` overrides model and adds a reference image.
- `uv run articraft draft "prompt text"` creates a draft local record without running generation.
- `uv run articraft draft --image reference.png "prompt text"` stores a reference image with the draft.
- `uv run articraft rerun <record-id>` reruns generation for an existing record.
- `uv run articraft compile <record-id>` recompiles one record into `<data-root>/cache/record_materialization/<id>/`.
- `uv run articraft library list` lists records from `records_manifest.jsonl`.
- `uv run articraft library rebuild-manifest` rebuilds `records_manifest.jsonl` from `records/`.
- `uv run articraft library check --require-records` validates that manifest rows point at real records.
- `just viewer` starts the built viewer flow; `just viewer-dev` starts uvicorn and Vite together.
- `uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765` starts only the API.
- `npm --prefix viewer/web run dev`, `build`, `lint`, and `typecheck` run frontend-only workflows.

## Coding Style & Naming Conventions
Target Python 3.11+; `.python-version` pins 3.12 for local `uv` use and the project excludes Python 3.13. Keep 4-space indentation, `from __future__ import annotations`, explicit type hints, and small module-level helpers for CLI wiring. Python formatting and import checks are handled by Ruff (`line-length = 100`, target `py311`, rules `E`, `F`, `I`, ignoring `E501`). Use `snake_case` for functions/modules/variables and `PascalCase` for models/classes. For `viewer/web`, use strict TypeScript, ESLint, Tailwind CSS v4, shadcn/ui components, and the `@/` import alias.

## Testing Guidelines
Tests run under `pytest` with importlib mode and xdist auto/worksteal by default. Add coverage under the mirrored package path, name files `test_<feature>.py`, and prefer native pytest functions and fixtures with plain `assert` statements. Keep coverage focused on fast import, storage, CLI, viewer API, SDK, and integration smoke checks. For prompt regressions, prefer durable behavioral checks over brittle formatting or line-budget assertions; relax or remove stale prompt assertions in the same change that updates the intended prompt contract.

## Commit & Pull Request Guidelines
Recent commits use short, imperative subjects such as `Move prompt compiler under agent` and `Consolidate SDK around canonical core profiles`. Keep commit titles concise and scoped to one logical change. PRs should describe the affected surface (`agent`, `storage`, `sdk`, `viewer`, or `cli`), include the exact `uv`, `just`, and `npm` commands run, and attach screenshots only when API or viewer behavior changes.

Canonical data lives in the configured data root, not the code repository. Keep `.env`, `.env.*`, local caches, generated URDFs, generated asset dirs, and `data/` out of code-repo commits.

## Configuration Tips
Provider code loads `.env`. Set `OPENAI_API_KEYS` or `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS` or `ANTHROPIC_API_KEY`, `DASHSCOPE_API_KEYS` or `DASHSCOPE_API_KEY`, and `OPENROUTER_API_KEYS` or `OPENROUTER_API_KEY` as needed. `ARTICRAFT_MODEL` and `ARTICRAFT_THINKING_LEVEL` set default generation model and thinking level; without those values, `generate` defaults to `gpt-5.5-2026-04-23` with `--thinking-level high`. `ARTICRAFT_CODEX_MODEL` is required when using `--provider codex-cli` unless `--model` is passed explicitly. `ARTICRAFT_MAX_COST_USD` can set a default per-run budget. Provider inference works for known OpenAI, Gemini, Claude, Qwen/DashScope, DeepSeek, Codex CLI, and OpenRouter-style model IDs, otherwise pass `--provider`.

## Paper Object Counts
When editing the Articraft paper, distinguish raw generated records from the final curated object set. The final paper count includes only retained 4-5 star objects; lower-rated 1/2/3-star records may exist as bad examples or audit material but should not be counted as final objects. Use "over 10K" for the final curated object count unless the retained 4-5-star subset is recomputed and the paper is intentionally updated.

## Agent Docs Contract
The SDK docs under `sdk/_docs/` are part of the agent authoring contract in this repository. Keep them aligned with the intended agent behavior and baseline compile/tooling policy; do not document agent-facing workflows there that the harness is supposed to own automatically.
