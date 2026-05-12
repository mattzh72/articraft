# Articraft: An Agentic System for Scalable Articulated 3D Asset Generation

Generate articulated 3D assets from prompts and inspect them locally.

## 1. Install Prerequisites

You need:

- Python 3.12 recommended (`.python-version` is committed for `uv`), or Python 3.11
- [`uv`](https://docs.astral.sh/uv/)
- [`just`](https://github.com/casey/just)
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) if you want the viewer and frontend hooks ready locally

Important:
`cadquery` depends on `vtk`, and the currently supported wheels for this repo do not install on CPython 3.13. The repo pins Python 3.12 for `uv` via `.python-version`. If you are not using `uv`'s pinned interpreter behavior, use Python 3.11 or 3.12 and avoid 3.13+.

If you do not have `just` yet:

```bash
# macOS with Homebrew
brew install just

# cross-platform installer
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
export PATH="$PATH:$HOME/bin"
```

## 2. Run Setup

From the repo root:

```bash
just setup
```

This does the initial local setup:

- creates `.env` from `.env.example` if `.env` does not exist yet, seeding supported provider env vars from your current shell when present
- installs Python dev dependencies, including `pre-commit` and `ruff`
- installs `viewer/web` dependencies when `npm` is available
- installs the git `pre-commit` and `pre-push` hooks
- installs the managed git `post-commit` hook used to auto-sync record metadata
- initializes local storage under `data/`
- builds the dataset manifest used by the viewer

You can verify the hook setup with:

```bash
uv run articraft hooks check
```

If you skip `just setup` and install pieces manually, record metadata will not auto-sync after commits until the managed post-commit hook is installed.

## 3. Add Your API Key

Open `.env` and set one provider key:

```text
Set `OPENAI_API_KEY` to your OpenAI key in `.env`
```

or:

```text
Set `GEMINI_API_KEYS` to your Gemini keys in `.env`
```

or:

```text
Set `ANTHROPIC_API_KEYS` to your Anthropic keys in `.env`
```

Optional:

```text
Set `ARTICRAFT_MAX_COST_USD` to a positive per-run USD budget default in `.env`
```

## 4. Open The Viewer Fast

If you just cloned the repo and want to browse saved records in the viewer, precompile the saved records first:

```bash
uv run articraft compile-all
```

This materializes the viewer-ready artifacts for saved records.

Then open the viewer:

```bash
just viewer
```

This starts the local API, builds the web app if needed, and opens the viewer.

## 5. Generate Your First Object

Run:

```bash
uv run articraft generate "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

This saves a generated record into the local workbench.

If you do not pass overrides, `articraft generate` defaults to:

```bash
--model gpt-5.5-2026-04-23 --thinking-level high
```

To change either setting, pass overrides on the same command:

```bash
# Use a different model (same SDK)
uv run articraft generate --model gemini-3-flash-preview "Create a compact desk fan with adjustable tilt."

# Change both model and prompt together
uv run articraft generate --model gpt-5.5-2026-04-23 "Create a compact desk fan with adjustable tilt."
```

To stop a run after it exceeds a USD budget, pass `--max-cost-usd ...`:

```bash
uv run articraft generate --max-cost-usd 1.5 "Create a compact desk fan with adjustable tilt."
```

`generate` and `draft` always use the canonical `sdk` pipeline. Record-based commands like `compile` and `rerun` use the record's saved SDK metadata automatically.

To run a single prompt directly into a dataset category instead of the workbench, use:

```bash
uv run articraft dataset run \
  "Create a backyard gas grill with a wheeled lower cart, a rectangular cookbox, side shelves, a front control panel, and a domed lid hinged along the rear edge of the cookbox." \
  --category-slug grill_with_hinged_lid
```

`dataset run` accepts the same `--model`, `--thinking-level`, `--image`, and `--max-cost-usd` overrides as `generate`. It auto-allocates a collision-resistant `ds_<category>_<token>` dataset ID for the target category unless you also pass `--dataset-id`.

The underlying CLI command is:

```bash
uv run articraft dataset run \
  "Create a backyard gas grill with a wheeled lower cart, a rectangular cookbox, side shelves, a front control panel, and a domed lid hinged along the rear edge of the cookbox." \
  --category-slug grill_with_hinged_lid \
  --provider openai \
  --model gpt-5.5-2026-04-23 \
  --thinking-level high \
  --max-cost-usd 2.0
```

If the target category slug already exists, the new record is appended under that category. If it does not exist yet, the command creates the category metadata automatically using a slug-derived title.

## 6. Run And Resume Dataset Batches

Dataset batches are driven by CSV specs under `data/batch_specs/`. The CSV filename stem becomes the batch's `batch_spec_id`.

That detail matters for resume:

- `--resume` looks up the latest prior run for the same `batch_spec_id`
- if you rename `chairs_v1.csv` to `chairs_v2.csv`, resume will treat it as a different batch
- resume also matches rows by `row_id`, so stable `row_id` values are important if you plan to retry or edit a spec later

### 6.1 Create the spec

Start with the built-in template:

```bash
uv run articraft dataset batch-new <batch-id>
```

This creates:

```text
data/batch_specs/<batch-id>.csv
```

with the current v1 header:

```csv
row_id,category_slug,category_title,prompt,provider,model_id,thinking_level,max_turns,max_cost_usd,label
```

### 6.2 Fill the CSV

Each row is one dataset generation job.

| Column | Required | Details |
| --- | --- | --- |
| `row_id` | Recommended | Stable row identifier used by resume. If omitted, it defaults to `row_0001`, `row_0002`, and so on, based on row order. For resumable batches, set this explicitly and do not change it after the first run. |
| `category_slug` | Yes | Dataset category slug. |
| `category_title` | Sometimes | Required for any row whose `category_slug` does not already exist in repo storage. If a new category appears on multiple rows, include the title on each of those rows. |
| `prompt` | Yes | The generation prompt. |
| `provider` | Yes | Must be `openai` or `gemini`. |
| `model_id` | Yes | Model to use for that row. It must agree with `provider`. |
| `thinking_level` | Yes | Must be `low`, `med`, or `high`. |
| `max_turns` | Yes | Positive integer turn cap for the row. |
| `max_cost_usd` | No | Optional positive per-row USD budget. If blank, the row inherits the batch CLI flag or `ARTICRAFT_MAX_COST_USD`. |
| `label` | No | Optional free-form label for your own tracking. |

Batch CSV v1 notes:

- `image_path` is not supported in batch CSV v1
- duplicate `row_id` values are rejected
- if the batch introduces a new category and `category_title` is missing, validation fails before the run starts

Example:

```csv
row_id,category_slug,category_title,prompt,provider,model_id,thinking_level,max_turns,max_cost_usd,label
hinge_01,hinge,Hinge,"Create a steel door hinge with two rectangular leaves and a center pin.",openai,gpt-5.4,high,12,1.5,baseline
hinge_02,hinge,Hinge,"Create a compact cabinet hinge with offset leaves and a short pin.",gemini,gemini-3-flash-preview,med,10,,compact
```

### 6.3 Run the first pass

Use `uv` directly:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --max-cost-usd 2.0
```

Useful execution controls:

- `--row-concurrency`: maximum number of live batch rows at once; use `auto`, `max`, or a positive integer
- `--subprocess-concurrency`: maximum number of compile/QC/probe subprocess-heavy operations at once; use `auto`, `max`, or a positive integer
- `--max-cost-usd`: default per-row USD budget for rows whose `max_cost_usd` CSV cell is blank

If any row fails, the batch exits non-zero. That is expected. The normal recovery path is to fix the issue and rerun with `--resume`.

### 6.4 Resume a batch safely

The most common recovery command is:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume
```

What `--resume` does:

- reuses the latest prior run for the same `batch_spec_id`
- resumes that run in place under the existing `data/cache/runs/<run_id>/`
- reuses the prior `dataset_id` and `record_id` allocations from `allocations.json`
- preserves rows that already have durable successful outputs instead of rerunning them
- by default, reruns rows whose latest status is `failed`, `pending`, or `running`

Resume policies:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --resume-policy failed_only
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --resume-policy failed_or_pending
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --resume-policy all
```

- `failed_only`: rerun only rows whose latest status is `failed`
- `failed_or_pending`: rerun `failed`, `pending`, and interrupted `running` rows; this is the default
- `all`: rerun every row using the existing allocations

Important resume rules:

- keep the CSV filename stable so the `batch_spec_id` stays the same
- keep `row_id` stable; changing or reordering implicit row ids can break resume matching
- by default, resume rejects spec changes for `category_slug`, `prompt`, `provider`, `model_id`, `thinking_level`, `max_turns`, and `max_cost_usd`
- if a row already produced a durable record but the cached state says `running`, resume reconciles that success instead of rerunning it

You can bypass the spec-compatibility check:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --allow-resume-spec-mismatch
```

Use `--allow-resume-spec-mismatch` only for deliberate recovery work. It forces the current CSV to reuse the prior run's row allocations even though the row definitions no longer match.

This is the escape hatch for "retry the same row, but with different execution settings." A common example is raising `max_turns` or `max_cost_usd` for failed rows, or switching a failed row to a different `provider`, `model_id`, `thinking_level`, or `prompt` before resuming.

Typical override workflow:

1. Edit the existing row in `data/batch_specs/<batch-id>.csv` without changing its `row_id`.
2. Resume with `--allow-resume-spec-mismatch`.
3. Usually pair that with `--resume-policy failed_only` so already successful rows stay preserved.

Example:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume --resume-policy failed_only --allow-resume-spec-mismatch
```

When you do this:

- the rerun uses the current CSV values for row execution settings
- the rerun still keeps the existing `dataset_id`, `record_id`, and prompt allocation from the original run
- only rows selected by the current `resume_policy` will actually rerun
- keep `row_id` and the batch filename stable
- do not treat `category_slug` as a routine override during resume; it is part of dataset identity, and changing it while reusing prior allocations can lead to confusing results

### 6.5 Know where the outputs and state go

After a successful row:

- the canonical record is written under `data/records/<record-id>/`
- dataset storage and category metadata are updated

During the batch:

- resumable run state lives under `data/cache/runs/<run_id>/`
- `run.json` stores batch-level metadata
- `allocations.json` stores the stable `dataset_id` and `record_id` assigned to each `row_id`
- `results.jsonl` stores one result row per batch row
- `state/<row_id>.json` stores per-row attempt status

If you need to inspect or debug a resume decision, `allocations.json`, `results.jsonl`, and `state/<row_id>.json` are the first files to check.

## 7. Reference

List commands:

```bash
just
```

Open the viewer in frontend dev mode:

```bash
just viewer-dev
just host=0.0.0.0 port=9000 viewer-dev
```

Rebuild the viewer search index:

```bash
uv run articraft workbench search-index
```

Recompile one saved record:

```bash
uv run articraft compile data/records/<record-id>
uv run articraft compile data/records/<record-id> --validate --strict-geom-qc
```

Bulk compile variants:

```bash
uv run articraft compile-all
uv run articraft compile-all --target full --strict
uv run articraft compile-all --force
```

Rerun generation for an existing record:

```bash
uv run articraft rerun data/records/<record-id>
```

Generate with overrides:

```bash
uv run articraft generate --model gemini-3-flash-preview "Create a compact tabletop fan with an oscillating head and tilt adjustment."
uv run articraft generate --image reference.png "Create a weighted desk lamp with articulated arms."
uv run articraft generate --model gpt-5.5-2026-04-23 --thinking-level high --image reference.png "Create a tower crane with a rotating top and suspended hook."
```

Run a batch directly with `uv`:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume
```

See Section 6 for the batch CSV schema, resume policies, and run-state details.

## 8. Citation

```bibtex
@misc{articraft2026,
  title={{Articraft}: An Agentic System for Scalable Articulated 3D Asset Generation},
  author={{Articraft contributors}},
  howpublished={\url{https://github.com/mattzh72/articraft}},
  year={2026}
}
```
