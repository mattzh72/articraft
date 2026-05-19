# Contributing to Articraft

Thank you for your interest in improving Articraft! We welcome contributions from everyone. Whether it's a bug report, a new feature, a fix, or better documentation, everything helps.

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

## Data Contribution Workflow

A huge part of Articraft's mission is crowdsourcing a massive, diverse dataset. If you're contributing objects, follow this consistent workflow:

1. **Choose Your Generation Path**:
   - *Targeted Authoring*: Use `uv run articraft dataset run <prompt> --category-slug <slug>`.
   - *Editing Existing Assets*: Use `uv run articraft fork data/records/<record_id> "<edit prompt>"`. Forking creates a child record and leaves the parent unchanged; see [Editing Existing Records](docs/record_editing.md).
   - *AI-Assisted*: Open Claude Code, Cursor, or Codex in the repo and prompt it to "Follow `EXTERNAL_AGENT_DATA.md`". (Do not run the `articraft external` CLI yourself; the agent will do it internally).
   - *Bulk Generation*: Use batch CSVs. See our [Dataset Generation Guide](docs/dataset_generation.md).
2. **Local Validation**: 
   - All assets MUST compile without errors locally. Any physics warnings, overlapping parts, or disconnected links must be fixed before proceeding.
3. **Visual Curation & Rating**:
   - Open the viewer (`just viewer`) and manually inspect your generated asset.
   - **Crucial Step:** Rate the asset! You must use the viewer's rating system (1-5 stars) to submit an asset. We accept all ratings (even 1-star assets are incredibly useful as negative examples), but you must actively record the rating.
4. **Finalize & Categorize**:
   - Only records assigned to a dataset category should be pushed. Workbench records are local drafts.
5. **Commit and PR**:
   - Stage **only** the `data/records/<id>` folders. The pre-commit hooks will block caches and URDF files.
   - Create a PR with standard naming, for instance: `Add 50 washing machines to dataset`.
   - **Massive PRs Are Welcome**: You can submit anywhere from a single object to thousands of records at once.
   - **Screenshots Strongly Encouraged**: Including a screenshot or GIF of the asset in the PR description makes reviewer validation much faster.
