---
name: articraft-authoring
description: Use when creating, editing, checking, or finalizing Articraft articulated-object records in the repository. Covers the supported external-agent workflow, SDK docs, model.py authoring loop, validation, and provenance rules.
---

# Articraft Authoring

Use this skill when the user asks Codex to create, edit, fix, improve, check, or finalize an Articraft asset or record.

## Core Rule

External agents must use the `articraft external` workflow. Do not manually create `data/records/<id>` directories, invent metadata, copy record folders, write traces, or bypass the CLI.

Read the repository contract before authoring:

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

If the user is asking for a dataset contribution, inspect categories first:

```bash
uv run articraft external categories
```

## Create A New Record

Create the record through the CLI and identify Codex:

```bash
uv run articraft external init --agent codex --model-id <model-id> --thinking-level <low|med|high> "<prompt>"
```

If the model or thinking level is unknown, omit those flags rather than guessing.

The command prints `record_id`, `record_dir`, and active paths. Edit only the CLI-printed active `model=` path, usually:

```text
data/records/<record_id>/revisions/<revision_id>/model.py
```

## Edit An Existing Record

Fork through the external CLI:

```bash
uv run articraft external fork --agent codex data/records/<record_id> "<edit request>"
```

Do not edit an existing dataset or workbench record in place unless the user explicitly asks for broader repository maintenance unrelated to data authoring.

## Authoring Standard

Build a realistic articulated asset with:

- connected, non-floating structure
- meaningful user-facing articulation
- semantic link names
- visible mechanisms and realistic materials
- prompt-specific checks in `run_tests()`
- no unintentional intersections or disconnected parts

Prefer relevant SDK helpers, CadQuery geometry, lofts, sweeps, booleans, mesh helpers, colors, and materials over boxy placeholder geometry.

Search for high-rated references when useful:

```bash
uv run articraft external examples --query "<object name>"
uv run articraft external examples --category-slug <slug> --rating-min 5
```

## Validation Loop

Run the strict external check during development:

```bash
uv run articraft external check data/records/<record_id>
```

Inspect failures and warnings, update the active `model.py`, and repeat until the check passes. If a viewer inspection is needed, compile/open through the viewer workflow.

## Finalize

For a workbench-only object:

```bash
uv run articraft external finalize data/records/<record_id>
```

For a dataset contribution, only when the user explicitly requested dataset promotion:

```bash
uv run articraft external finalize data/records/<record_id> --category-slug <slug>
```

Leave workbench-only records uncommitted. Preserve `creator.mode=external_agent`, `creator.agent=codex`, and `creator.trace_available=false`.
