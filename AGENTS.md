# Repository Guidelines

## Project Structure & Module Organization
`agent/` contains the generation runtime, providers, prompt compiler, and TUI helpers. `storage/` owns the canonical `data/` layout, batch spec storage, record models, and query helpers used by the dataset and workbench flows. `sdk/` and `sdk/_core/` define the articulated-object SDK layers and shared geometry/export logic. `viewer/api/` exposes the FastAPI surface. `cli/` contains the repository entry points, while `tests/` mirrors the main packages with smoke-style checks such as `tests/storage/test_repo.py`.

## Build, Test, and Development Commands
Use `uv run articraft ...` for the product CLI, and `just` only for developer setup/check/viewer shortcuts. For additional shortcuts, run `just` or check [`justfile`](justfile).

- `uv sync --group dev` installs the project and development dependencies.
- `uv build` creates the wheel and source distribution in `dist/`.
- `uv run articraft init` creates the canonical `data/` tree and local workbench state.
- `uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto` runs a tracked dataset batch CSV into canonical records.
- `uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume` resumes the latest prior run for that batch spec.
- `uv run articraft workbench status` inspects the workbench store state.
- `uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765` starts the local API directly.
- `uv run --group dev pytest -q` runs the full Python regression suite.
- `just setup` handles first-time repo setup and storage initialization.
- `just smoke-tests` runs the fast pre-push smoke suite.
- `just test-all` runs the full Python regression suite.
- `uv run articraft compile-all` is the recommended path before `just viewer`; it uses the faster visual-only bulk materialization path for viewer assets.
- `uv run articraft compile-all --target full` runs the non-strict full bulk compile path when you need collision-inclusive URDFs without validation-heavy strict checks.
- `uv run articraft compile-all --target full --strict` runs the validation-heavy full compile path.
- `uv run articraft dataset batch-new <batch-id>` creates `data/batch_specs/<batch-id>.csv` with the canonical batch CSV header.
- `uv run articraft dataset batch --row-concurrency 8 --subprocess-concurrency auto data/batch_specs/<batch-id>.csv` runs the dataset batch shortcut; batch row provider/model settings come from the CSV.
- `uv run articraft draft "prompt text"` creates a draft workbench record without running generation, which is useful when you want an empty record directory to work in.
- `uv run articraft draft --image reference.png "prompt text"` stores a reference image with the draft so later reruns keep the same conditioning.
- `uv run articraft compile data/records/<id>` recompiles a record's `model.py` into cache-backed materialization outputs under `data/cache/record_materialization/<id>/`.
- `uv run articraft rerun data/records/<id>` reruns generation for an existing workbench record in place.
- `uv run articraft workbench search-index` rebuilds the workbench search index after record changes.
- `just viewer` starts the local viewer flow.
- `just viewer-dev` starts the API and Vite dev server together.
- `uv run articraft generate "prompt text"` runs generation in workbench mode.

## Coding Style & Naming Conventions
Target Python 3.11+ and keep 4-space indentation. Follow the existing style: `from __future__ import annotations`, explicit type hints, and small module-level helpers for CLI wiring. Use `snake_case` for functions, modules, and variables, `PascalCase` for models/classes, and keep package names aligned with their surface area (`agent`, `storage`, `viewer`). No formatter or linter is configured in this repository today, so match the surrounding file style closely and keep imports ordered and minimal.

## Testing Guidelines
Tests run under `pytest`. Add new coverage under the mirrored package path, name files `test_<feature>.py`, and prefer native pytest test functions and fixtures. Keep coverage focused on fast import, storage, and integration smoke checks with plain `assert` statements. For prompt regression tests, prefer durable behavioral checks over brittle formatting or line-budget assertions; if a regression test no longer matches the intended prompt contract, relax or remove it as part of the same change.

## Commit & Pull Request Guidelines
Recent commits use short, imperative subjects such as `Move prompt compiler under agent` and `Consolidate SDK around canonical core profiles`. Keep commit titles concise and scoped to one logical change. PRs should describe the affected surface (`agent`, `storage`, `sdk`, or `viewer`), include the exact `uv` commands run, and attach screenshots only when API or viewer behavior changes.

## Configuration Tips
Provider code loads environment variables from `.env`. Set `OPENAI_API_KEYS` (preferred) or `OPENAI_API_KEY` for OpenAI flows, `ANTHROPIC_API_KEYS` (preferred) or `ANTHROPIC_API_KEY` for Anthropic flows, and `GEMINI_API_KEYS` for Gemini flows before running the agent runtime. Dataset batch specs live under `data/batch_specs/`; each row sets its own `provider`, `model_id`, `thinking_level`, `max_turns`, and `sdk_package`, while row and subprocess concurrency are supplied at invocation time. For new dataset batch specs, leave per-row cost caps blank unless explicitly requested and use `max_turns=100` by default.

## Paper Dataset Counts
When editing the Articraft paper, distinguish raw generated records from the final curated dataset. The final paper dataset includes only retained 4-5 star objects; lower-rated 1/2/3-star records may exist in `data/records/` as bad examples or audit material but should not be counted as final dataset objects. Use "over 10K" for the final curated object count unless the retained 4-5-star subset is recomputed and the paper is intentionally updated.

## Agent Docs Contract
The SDK docs under `sdk/_docs/` are part of the agent authoring contract in this repository. Keep them aligned with the intended agent behavior and baseline compile/tooling policy; do not document agent-facing workflows there that the harness is supposed to own automatically.
