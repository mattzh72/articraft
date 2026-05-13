# Data Authoring Guidelines

You are an external agent harness authoring Articraft data in this repository. This is your only supported workflow.

Do not manually create `data/records/<id>` folders, invent record metadata, write Articraft agent traces, or bypass these commands.

## Read This First

Before you create or edit a record, read the core Articraft authoring requirements:

```text
agent/prompts/sections/designer_common.md
agent/prompts/sections/link_naming.md
```

These define the non-negotiable quality bar: realistic geometry, primary user-facing articulation, no floating parts, no unintentional overlaps, prompt-specific tests, and concise semantic link names.

Quality and realism are very important here. Use every appropriate modeling tool available in the SDK and repository docs to make the geometry read as the real object, not a placeholder. For example, things that are hollow should be modeled as hollow; curved, tapered, beveled, perforated, soft, transparent, or compound forms should use suitable primitives, CadQuery geometry, lofts, sweeps, booleans, mesh helpers, colors, and materials instead of boxy approximations when the real object needs more detail.

Then use the SDK docs and examples while you author:

```text
sdk/_docs/
sdk/_examples/
```

You may also search existing examples. Prefer high-quality 5-star records as references, and use lower-rated records only as cautionary examples:

```bash
uv run articraft external examples --query "washing machine"
uv run articraft external examples --category-slug washing_machine --rating-min 5
```

Use this loop for high-quality output:

1. Read the user prompt and identify the real object, scale, visible materials, main mechanisms, and controls.
2. Read the relevant SDK docs/examples before writing geometry.
3. Build the object with realistic connected structure and articulated primary mechanisms.
4. Add prompt-specific checks in `run_tests()` for the visual/mechanical claims you made.
5. Compile, inspect failures/warnings, fix the model, and repeat until `external check` passes.

## 1. Initialize The Repo

Run this before authoring:

```bash
uv sync --group dev
uv run articraft init
```

## 2. Create A Record

Create the record through the external CLI and identify yourself explicitly:

```bash
uv run articraft external init --agent codex "washing machine"
uv run articraft external init --agent claude-code "washing machine"
```

`codex` records default to provider metadata `openai`; `claude-code` records default to `anthropic`.

Strongly prefer registering your configured model and thinking/reasoning level. If you know either value, include it in `external init`:

```bash
uv run articraft external init --agent claude-code --model-id claude-sonnet-4-6 --thinking-level high "washing machine"
uv run articraft external init --agent codex --model-id gpt-5.5-2026-04-23 --thinking-level high "washing machine"
```

The command prints `record_id` and `record_dir`. Edit only that generated record unless the user explicitly asks for broader repository changes.

Allowed external agent ids are:

- `codex`
- `claude-code`

## 3. Author The Object

Edit:

```text
data/records/<record_id>/model.py
```

The user prompt is stored in:

```text
data/records/<record_id>/prompt.txt
```

Use the SDK docs and examples:

```text
sdk/_docs/
sdk/_examples/
```

Your object must be a high-quality articulated asset: meaningful parts, correct joints, visible mechanical structure, stable geometry, realistic materials/colors, semantic link names, and prompt-specific tests in `run_tests()`.

## 4. Iterate

Compile during development:

```bash
uv run articraft external compile data/records/<record_id>
```

Run the stricter external check before finalizing:

```bash
uv run articraft external check data/records/<record_id>
```

Repeat authoring, compile, and check until the record passes.

## 5. Finalize

For a workbench-only object, finalize without a category:

```bash
uv run articraft external finalize data/records/<record_id>
```

For a dataset contribution, first inspect existing categories:

```bash
uv run articraft external categories
```

Then finalize into the best category:

```bash
uv run articraft external finalize data/records/<record_id> --category-slug washing_machine
```

Use `--category-slug` only when the user asked to add the object to the dataset.

## Rules

You must:

- use `articraft external init` before writing a record
- preserve `creator.mode=external_agent`
- preserve `creator.agent=codex` or `creator.agent=claude-code`
- preserve `creator.trace_available=false`
- use `articraft external compile`, `check`, and `finalize`
- leave workbench-only records uncommitted

You must not:

- manually create record directories
- claim internal Articraft harness traces
- write files under `traces/`
- edit unrelated records while authoring one object
- promote to the dataset unless the user requested dataset contribution
