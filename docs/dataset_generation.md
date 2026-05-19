# Dataset Generation Guide

Dataset batches in Articraft are driven by CSV specs under `data/batch_specs/`. The CSV filename stem becomes the batch's `batch_spec_id`.

That detail matters for resume:
- `--resume` looks up the latest prior run for the same `batch_spec_id`
- If you rename `chairs_v1.csv` to `chairs_v2.csv`, resume will treat it as a different batch
- Resume matches rows by `row_id`, so stable `row_id` values are important if you plan to retry or edit a spec later

## 1. Create the spec

Start with the built-in template:

```bash
uv run articraft dataset batch-new <batch-id>
```

This creates: `data/batch_specs/<batch-id>.csv` with the current v1 header.

## 2. Fill the CSV

Each row is one dataset generation job.

| Column | Required | Details |
| --- | --- | --- |
| `row_id` | Recommended | Stable row identifier used by resume. Default uses ordering. |
| `category_slug` | Yes | Dataset category slug. |
| `category_title` | Sometimes | Required for any row whose `category_slug` does not already exist. |
| `prompt` | Yes | The generation prompt. |
| `provider` | Yes | `openai`, `gemini`, `anthropic`, or `openrouter`. |
| `model_id` | Yes | Model to use. Must agree with `provider`. |
| `thinking_level` | Yes | `low`, `med`, or `high`. |
| `max_turns` | Yes | Positive integer turn cap. |
| `max_cost_usd` | No | Optional positive per-row USD budget. |
| `label` | No | Optional free-form label. |

**Notes:**
- `image_path` is not supported in batch CSV v1
- Duplicate `row_id` values are rejected

## 3. Run the first pass

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto
```

Useful execution controls:
- `--row-concurrency`: maximum number of live batch rows at once; use `auto`, `max`, or an int.
- `--subprocess-concurrency`: maximum concurrent compile/QC/probe operations.
- `--max-cost-usd`: default per-row USD budget for blank rows.

## 4. Resume a batch safely

If a batch row fails, you can safely recover with:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --row-concurrency 8 --subprocess-concurrency auto --resume
```

What `--resume` does:
- Reuses the latest prior run for the same `batch_spec_id`
- Reuses prior `dataset_id` and `record_id` allocations
- Preserves successful rows
- Reruns `failed`, `pending`, or interrupted `running` rows (default behavior).

### Resume policies

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --resume-policy failed_only
```

- `failed_only`: rerun only rows whose latest status is `failed`
- `failed_or_pending`: rerun failed, pending, and interrupted (default)
- `all`: rerun every row

You can bypass the spec-compatibility check if modifying parameters like `max_turns` or `prompt` for failed rows:

```bash
uv run articraft dataset batch data/batch_specs/<batch-id>.csv --resume --allow-resume-spec-mismatch
```

## 5. Output and State

After a row succeeds, the canonical record is updated under `data/records/<record-id>/`. The resummable run state lives under `data/cache/runs/<run_id>/`.
