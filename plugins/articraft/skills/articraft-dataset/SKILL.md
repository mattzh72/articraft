---
name: articraft-library
description: Use when working with Articraft local library records, categories, manifest rebuilds, validation, or data-folder maintenance.
---

# Articraft Local Library

Use this skill when the user asks about Articraft record authoring, categories, local data validation, manifest updates, or sharing a data folder.

## Commands

From the repo root, prefer `uv run articraft ...` product commands and `just` shortcuts.

Check current state:

```bash
uv run articraft status
uv run articraft library status
```

Validate a local data folder:

```bash
uv run articraft library check --require-records
```

Rebuild the manifest:

```bash
uv run articraft library rebuild-manifest
```

List records:

```bash
uv run articraft library list
```

Assign a category:

```bash
uv run articraft library set-category <record-id> <category_slug>
```

## Data Root

The default data root is the gitignored `<repo-root>/data`. To use another local/exportable folder, pass `--data-dir` or set:

```bash
export ARTICRAFT_DATA_DIR=/path/to/articraft-data
```

The data root contains top-level `records/`, `categories/`, `records_manifest.jsonl`, `system_prompts/`, and `cache/`.

## Contribution Rules

Data authoring work should follow `CONTRIBUTING.md` and `EXTERNAL_AGENT_DATA.md`.

Before sharing a data folder, run the relevant checks and report exact commands:

```bash
uv run articraft library check --require-records
just smoke-tests
```

If viewer behavior or asset quality is part of the change, inspect records with the local viewer and include screenshots or notes when useful.
