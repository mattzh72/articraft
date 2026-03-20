# Repository Guidelines

## Project Structure & Module Organization
`agent/` contains the generation runtime, providers, prompt compiler, and TUI helpers. `storage/` owns the canonical `data/` layout, batch spec storage, record models, and query helpers used by the dataset and workbench flows. `sdk/`, `sdk_hybrid/`, and `sdk/_core/` define the articulated-object SDK layers and shared geometry/export logic. `viewer/api/` exposes the FastAPI surface. `cli/` contains the repository entry points, while `tests/` mirrors the main packages with smoke-style checks such as `tests/storage/test_repo.py`.

## Build, Test, and Development Commands
Use `uv` for direct Python workflows, and `just` for the agent-facing shortcuts that speed up common iteration loops. For additional shortcuts, run `just` or check [`justfile`](/Users/matthewzhou/articraft/justfile).
When a `just` recipe supports optional settings, pass them as overrides before the recipe name, for example `just model=gemini-3-flash-preview image=reference.png wb "prompt text"`.

- `uv sync --group dev` installs the project and development dependencies.
- `uv build` creates the wheel and source distribution in `dist/`.
- `uv run articraft-dataset --repo-root . init-storage` creates the canonical `data/` tree for dataset work.
- `uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8` runs a tracked dataset batch CSV into canonical records.
- `uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8 --resume` resumes the latest prior run for that batch spec.
- `uv run articraft-workbench --repo-root . status` inspects the workbench store state.
- `uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765` starts the local API directly.
- `uv run --group dev pytest -q` runs the current smoke-test suite.
- `just setup` handles first-time repo setup and storage initialization.
- `just smoke-tests` runs the smoke-test suite.
- `just compile-all-visual` is the recommended fast path before `just viewer`; it materializes visual meshes for saved records.
- `just compile-all` runs the full artifact path, including collision geometry, when that matters for inspection.
- `just compile-all-strict` runs the validation-heavy full compile path.
- `just name=<batch-id> batch-spec-new` creates `data/batch_specs/<batch-id>.csv` with the canonical batch CSV header.
- `just spec=data/batch_specs/<batch-id>.csv concurrency=8 dataset-batch` runs the dataset batch shortcut; batch row provider/model settings come from the CSV.
- `just wb-init "prompt text"` creates a draft workbench record without running generation, which is useful when you want an empty record directory to work in.
- `just image=reference.png wb-init "prompt text"` stores a reference image with the draft so later reruns keep the same conditioning.
- `just compile-visual data/records/<id>` recompiles only the visual viewer artifacts for a single record.
- `just compile data/records/<id>` recompiles a record's `model.py` into `model.urdf`.
- `just rerun data/records/<id>` reruns generation for an existing workbench record in place.
- `just search-index` rebuilds the workbench search index after record changes.
- `just viewer` starts the local viewer flow.
- `just viewer-dev` starts the API and Vite dev server together.
- `just wb "prompt text"` runs generation in workbench mode.

## Coding Style & Naming Conventions
Target Python 3.11+ and keep 4-space indentation. Follow the existing style: `from __future__ import annotations`, explicit type hints, and small module-level helpers for CLI wiring. Use `snake_case` for functions, modules, and variables, `PascalCase` for models/classes, and keep package names aligned with their surface area (`agent`, `storage`, `viewer`). No formatter or linter is configured in this repository today, so match the surrounding file style closely and keep imports ordered and minimal.

## Testing Guidelines
Tests run under `pytest`. Add new coverage under the mirrored package path, name files `test_<feature>.py`, and prefer native pytest test functions and fixtures. Keep coverage focused on fast import, storage, and integration smoke checks with plain `assert` statements.

## Commit & Pull Request Guidelines
Recent commits use short, imperative subjects such as `Move prompt compiler under agent` and `Consolidate SDK around canonical core profiles`. Keep commit titles concise and scoped to one logical change. PRs should describe the affected surface (`agent`, `storage`, `sdk`, or `viewer`), include the exact `uv` commands run, and attach screenshots only when API or viewer behavior changes.

## Configuration Tips
Provider code loads environment variables from `.env`. Set `OPENAI_API_KEYS` (preferred) or `OPENAI_API_KEY` for OpenAI flows and `GEMINI_API_KEYS` for Gemini flows before running the agent runtime. Dataset batch specs live under `data/batch_specs/`; each row sets its own `provider`, `model_id`, `thinking_level`, `max_turns`, and `sdk_package`, while `concurrency` is supplied at invocation time.
