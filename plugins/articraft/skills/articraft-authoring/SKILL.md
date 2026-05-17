---
name: articraft-authoring
description: Use when creating, editing, checking, or finalizing Articraft articulated-object records in the repository. Covers native API generation, no-key Codex CLI generation, legacy external drafts, SDK docs, validation, and provenance rules.
---

# Articraft Authoring

Use this skill when the user asks Codex to create, edit, fix, improve, check, or finalize an Articraft asset or record.

## Core Rule

There are three supported Articraft authoring modes. Choose the mode from the user's request before creating a record.

- Use native Articraft generation when the user needs Articraft-managed run metadata, cost accounting, turn counts, or the full agent trajectory. This path requires the relevant provider API key.
- Use no-key Codex generation when the user wants Codex access without provider API keys. This path uses `--provider codex-cli` inside Articraft's internal harness, so Articraft still owns the loop, tools, compile feedback, turn counts, compile-attempt counts, record persistence, and trajectory. Total token count is recorded when Codex CLI emits its `tokens used` line; billable cost remains unavailable.
- Use legacy external Codex authoring only when the user explicitly asks Codex to manually edit `model.py` outside the internal harness. This path creates an external-agent record and intentionally has no Articraft internal trace.

Never manually create `data/records/<id>` directories, invent record metadata, copy record folders, write traces, or bypass the CLI.

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

For a full Articraft run with cost, turn count, and trajectory, use native generation:

```bash
uv run articraft generate "<prompt>"
```

For no-key Codex generation with Articraft loop parity, use the Codex CLI provider:

```bash
uv run articraft generate --provider codex-cli "<prompt>"
```

Use an explicit Codex model only when requested or already configured:

```bash
uv run articraft generate --provider codex-cli --model <codex-model-id> "<prompt>"
```

For image-conditioned no-key Codex generation:

```bash
uv run articraft generate --provider codex-cli --image <reference-image> "<prompt>"
```

For legacy external Codex drafting, create the record through the external CLI and identify Codex:

```bash
uv run articraft external init --agent codex --model-id <model-id> --thinking-level <low|med|high> "<prompt>"
```

If the model or thinking level is unknown, omit those flags rather than guessing.

The command prints `record_id`, `record_dir`, and active paths. Edit only the CLI-printed active `model=` path, usually:

```text
data/records/<record_id>/revisions/<revision_id>/model.py
```

External Codex authoring does not produce Articraft internal turn telemetry. Leave provenance turn/tool/compile counters unset unless the CLI records them from a supported Articraft command.

## Edit An Existing Record

Fork through the external CLI:

```bash
uv run articraft external fork --agent codex data/records/<record_id> "<edit request>"
```

Do not edit an existing dataset or workbench record in place unless the user explicitly asks for broader repository maintenance unrelated to data authoring.

For no-key Codex edits with Articraft loop parity, use the internal fork command instead:

```bash
uv run articraft fork --provider codex-cli data/records/<record_id> "<edit request>"
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

Search for high-rated references when useful:

```bash
uv run articraft external examples --query "<object name>"
uv run articraft external examples --category-slug <slug> --rating-min 5
```

## Validation Loop

For native/API and no-key Codex provider runs, the harness calls `compile_model` during generation. Recompile or inspect after generation when needed:

```bash
uv run articraft compile data/records/<record_id>
```

For legacy external drafts, run the strict external check during development:

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
