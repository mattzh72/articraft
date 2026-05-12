# Contributing

Thanks for improving Articraft. Keep changes scoped to one logical area and include the commands you ran when opening a pull request.

## Setup

```bash
uv sync --group dev
npm --prefix viewer/web ci
uv run pre-commit install --hook-type pre-commit --hook-type pre-push
uv run articraft hooks install
```

## Before Opening A PR

Run the fastest checks that cover your change:

```bash
just smoke-tests
npm --prefix viewer/web run typecheck
npm --prefix viewer/web run lint
```

For broader Python changes, run:

```bash
just test-all
```

## Data And Generated Artifacts

Do not commit `.env`, `data/cache/`, `data/local/`, generated URDFs, or record asset directories. Generated records under `data/records/` may include prompts, traces, provider/model metadata, ratings, and cost accounting; only commit them when the change explicitly concerns curated dataset artifacts.

## PR Notes

Describe the affected surface area (`agent`, `storage`, `sdk`, `viewer`, or `data`) and list the exact `uv`, `just`, or `npm` commands you ran. Include screenshots only for viewer or API behavior changes.
