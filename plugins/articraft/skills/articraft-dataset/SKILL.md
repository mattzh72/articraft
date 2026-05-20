---
name: articraft-dataset
description: Use when working with Articraft dataset categories, batch specs, batch runs, dataset validation, manifests, or contribution preparation.
---

# Articraft Dataset

Use this skill when the user asks about Articraft dataset generation, batch CSVs, categories, manifest updates, dataset validation, or contribution workflow.

## Commands

From the repo root, prefer `uv run articraft ...` product commands and `just` shortcuts.

Check current state:

```bash
uv run articraft status
uv run articraft dataset status
```

Validate checked-in data:

```bash
uv run articraft data check
uv run articraft dataset validate
```

Build the manifest:

```bash
uv run articraft dataset manifest
```

## Batch Specs

Batch specs live under:

```text
data/batch_specs/
```

Create a new batch CSV with the canonical header:

```bash
uv run articraft dataset batch-new <batch-id>
```

Required columns are:

```text
category_slug,prompt,provider,model_id,thinking_level,max_turns
```

`category_title` is required for new categories. Recommended optional columns are `row_id`, `max_cost_usd`, `label`, and `sdk_package`. Use `max_turns=100` by default unless the user requests another value. Leave per-row cost caps blank unless explicitly requested.

Supported providers:

```text
openai, gemini, anthropic, openrouter, codex-cli
```

Use `codex-cli` when the user wants no-key Codex generation while keeping Articraft's internal harness loop. For `codex-cli`, use `model_id=codex-cli-default` unless the user requests a specific Codex model.

Supported thinking levels:

```text
low, med, high
```

`image_path` is intentionally unsupported in batch CSV v1.

## Running Batches

Run a tracked batch:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto
```

Resume the latest prior run for the same spec:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume
```

Use stable explicit `row_id` values when authoring specs meant to be resumed or reviewed.

## Contribution Rules

Dataset contribution work should follow `CONTRIBUTING.md` and `EXTERNAL_AGENT_DATA.md`. Workbench records are local drafts. Only records assigned to a dataset category should be prepared for commit.

Before a data PR, run the relevant checks and report exact commands:

```bash
uv run articraft data check
just smoke-tests
```

If viewer behavior or asset quality is part of the change, inspect records with the local viewer and include screenshots or notes when useful.
