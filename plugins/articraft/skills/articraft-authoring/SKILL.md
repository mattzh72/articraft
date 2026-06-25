---
name: articraft-authoring
description: Use when creating, editing, checking, or finalizing Articraft articulated-object records in the local library.
---

# Articraft Authoring

Use this skill when the user asks Codex to create, edit, fix, improve, check, or finalize an Articraft asset or record.

## Core Rule

There are three supported Articraft authoring modes. Choose the mode from the user's request before creating a record.

- Use native Articraft generation when the user needs Articraft-managed run metadata, cost accounting, turn counts, or the full agent trajectory. This path requires the relevant provider API key.
- Use no-key Codex generation when the user wants Codex access without provider API keys. This path uses `--provider codex-cli` inside Articraft's internal harness, so Articraft still owns the loop, tools, compile feedback, turn counts, compile-attempt counts, record persistence, and trajectory.
- Use external Codex authoring only when the user explicitly asks Codex to manually edit `model.py` outside the internal harness. This path creates an external-agent record and intentionally has no Articraft internal trace.

Never manually create record directories, invent record metadata, copy record folders, write traces, or bypass the CLI.

Read the repository contract before external authoring:

```bash
sed -n '1,220p' EXTERNAL_AGENT_DATA.md
```

Also read the core quality requirements before writing geometry:

```text
agent/prompts/sections/designer_common.md
agent/prompts/sections/link_naming.md
```

Use SDK docs and examples while authoring:

```text
sdk/_docs/
sdk/_examples/
```

## Setup

From the Articraft repo root:

```bash
uv sync --group dev
uv run articraft init
```

If the user wants a specific data folder, pass `--data-dir` or set `ARTICRAFT_DATA_DIR`.

## Create A New Record

For a full Articraft run with cost, turn count, and trajectory, use native generation:

```bash
uv run articraft generate "<prompt>"
```

For no-key Codex generation with Articraft loop parity, use the Codex CLI provider:

```bash
uv run articraft generate --provider codex-cli --model <codex-model-id> "<prompt>"
```

For image-conditioned no-key Codex generation:

```bash
uv run articraft generate --provider codex-cli --model <codex-model-id> --image <reference-image> "<prompt>"
```

For external Codex drafting, create the record through the external CLI and identify Codex:

```bash
uv run articraft external init --agent codex "<prompt>"
```

The command prints `record_id` and `record_dir`. Edit only that generated record's active revision.

## Edit An Existing Record

Fork an existing record; do not manually copy record folders:

```bash
uv run articraft fork <record-id> "<edit request>"
```

For no-key Codex edits with Articraft loop parity:

```bash
uv run articraft fork --provider codex-cli --model <codex-model-id> <record-id> "<edit request>"
```

## Authoring Standard

Build a realistic articulated asset with:

- connected, non-floating structure
- meaningful user-facing articulation
- semantic link names
- visible mechanisms and realistic materials
- prompt-specific checks in `run_tests()`
- no unintentional intersections or disconnected parts

Prefer relevant SDK helpers, CadQuery geometry, lofts, sweeps, booleans, mesh helpers, colors, and materials over boxy placeholder geometry.

## Validation Loop

For native/API and no-key Codex provider runs, the harness calls `compile_model` during generation. Recompile or inspect after generation when needed:

```bash
uv run articraft compile <record-id>
```

For external drafts, run the same one-record compile command during development, update the active `model.py`, and repeat until it passes.

## Finalize

Finalize external records to upsert `records_manifest.jsonl`; pass a category only when the user asks for one:

```bash
uv run articraft external finalize <record-id>
uv run articraft external finalize <record-id> --category-slug <slug>
```

Preserve `creator.mode=external_agent`, `creator.agent=codex`, and `creator.trace_available=false`.
