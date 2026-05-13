# Contributing to Articraft

Thank you for your interest in improving Articraft! We welcome contributions from everyone. Whether it's a bug report, a new feature, a fix, or better documentation, everything helps.

Please review our [Code of Conduct](CODE_OF_CONDUCT.md) before participating to help us maintain a welcoming and safe community.

## Getting Started

1. **Architecture & Project Layout:** To understand how the repository is structured, please read the [Architecture Guide](docs/architecture.md). It explains `agent/`, `storage/`, `sdk/`, `viewer/`, and more.
2. **Setup:** If you haven't yet, bootstrap your dev environment from the root:
    ```bash
    uv sync --group dev
    npm --prefix viewer/web ci
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run articraft hooks install
    ```

## Development Workflow

### Useful Commands
We use `just` as our primary task runner. Run `just` without arguments to see all available shortcuts.
- `just format`: Format code using Ruff.
- `just lint`: Lint code with Ruff.
- `just viewer-dev`: Start both the uvicorn API and Vite frontend for rapid local UI iteration.

### Python Development
Target Python 3.11+. The repo is managed by `uv` and uses `ruff` for all formatting and checking. Make sure you run `just format` and `just lint` before submitting a PR.
Tests use `pytest`. We prioritize fast import time, robust validation over brittle line-assertions, and functional behavioral checks.

### Frontend Development
The viewer uses React, TypeScript, Tailwind CSS v4, shadcn/ui, and Three.js. Strict TypeScript and ESLint checks are enforced for the web interface.
```bash
npm --prefix viewer/web run dev        # start Vite for frontend dev
npm --prefix viewer/web run typecheck  # run TSC
npm --prefix viewer/web run lint       # run ESLint
```

## Creating a Pull Request

When submitting a PR, keep changes scoped to one logical addition or fix:

### Validating Your Change
1. Try to run the fastest checks prior to pushing:
   ```bash
   just smoke-tests
   npm --prefix viewer/web run typecheck
   npm --prefix viewer/web run lint
   ```
2. For broader Python changes run `just test-all`.

### Commit Guidelines
We care deeply about commit hygiene.
- Use short, imperative subjects (e.g., `Move prompt compiler under agent`, NOT `Moved prompt compiler` or `Moving prompt compiler`).
- Keep commit titles concise (under 50 characters is a good rule of thumb) and scoped to **one logical change**.
- If a commit fixes an issue, reference it in the body.

### PR Requirements
1. Fill out the Pull Request template indicating exactly what area (`agent`, `storage`, `sdk`, `viewer`, `cli`) is affected.
2. Include the exact `uv`, `just`, and `npm` commands you ran to verify the change.
3. Attach screenshots **only** when API or viewer behavior changes.
4. **Data Caveat:** Do not commit `.env`, `data/cache/`, `data/local/`, generated URDFs, or record asset directories. The pre-commit checks will generally block sensitive paths. Files under `data/` are exempt from the trailing-newline requirement.
