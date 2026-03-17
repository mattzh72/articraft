# Repository Guidelines

## Project Structure & Module Organization
`agent/` contains the generation runtime, providers, prompt compiler, and TUI helpers. `storage/` owns the canonical `data/` layout, record models, and query helpers used by the dataset and workbench flows. `sdk/`, `sdk_hybrid/`, and `sdk/_core/` define the articulated-object SDK layers and shared geometry/export logic. `viewer/api/` exposes the FastAPI surface. `cli/` contains the repository entry points, while `tests/` mirrors the main packages with smoke-style checks such as `tests/storage/test_repo.py`.

## Build, Test, and Development Commands
Use `uv` for local work:

- `uv sync` installs the project and runtime dependencies into the managed environment.
- `uv build` creates the wheel and source distribution in `dist/`.
- `uv run articraft-dataset init-storage --repo-root .` creates the canonical `data/` tree for dataset work.
- `uv run articraft-workbench status --repo-root .` inspects the workbench store state.
- `uv run uvicorn viewer.api.app:app --reload` starts the local API at `http://127.0.0.1:8000`.
- `find tests -name 'test_*.py' -type f | sort | xargs -I{} uv run python {}` runs the current smoke-test suite.

## Coding Style & Naming Conventions
Target Python 3.11+ and keep 4-space indentation. Follow the existing style: `from __future__ import annotations`, explicit type hints, and small module-level helpers for CLI wiring. Use `snake_case` for functions, modules, and variables, `PascalCase` for models/classes, and keep package names aligned with their surface area (`agent`, `storage`, `viewer`). No formatter or linter is configured in this repository today, so match the surrounding file style closely and keep imports ordered and minimal.

## Testing Guidelines
Tests are executable scripts, not a configured `pytest` suite. Add new coverage under the mirrored package path, name files `test_<feature>.py`, and keep them runnable with `uv run python tests/...`. Prefer fast import, storage, and integration smoke checks with plain `assert` statements.

## Commit & Pull Request Guidelines
Recent commits use short, imperative subjects such as `Move prompt compiler under agent` and `Consolidate SDK around canonical core profiles`. Keep commit titles concise and scoped to one logical change. PRs should describe the affected surface (`agent`, `storage`, `sdk`, or `viewer`), include the exact `uv` commands run, and attach screenshots only when API or viewer behavior changes.

## Configuration Tips
Provider code loads environment variables from `.env`. Set `OPENAI_API_KEY` for OpenAI flows and `GEMINI_API_KEYS` for Gemini flows before running the agent runtime.
