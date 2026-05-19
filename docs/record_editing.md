# Editing Existing Records

Articraft supports editing an existing asset by forking it into a new record. Forking keeps the parent record unchanged and gives the edit run its own prompt, model output, provenance, cost, and trajectory.

## Fork an asset

Use `articraft fork` with a record ID or canonical record directory:

```bash
uv run articraft fork data/records/<record_id> "make the handle longer"
```

You can pass the same model controls used by generation:

```bash
uv run articraft fork data/records/<record_id> \
  --model gemini-3-flash-preview \
  --thinking-level low \
  --max-cost-usd 1.5 \
  "make the hinge wider and reinforce the mounting plate"
```

Reference images can be supplied for the edit run:

```bash
uv run articraft fork data/records/<record_id> \
  --image reference.png \
  "match the latch shape in the reference image"
```

If you want a stable child ID, provide one explicitly:

```bash
uv run articraft fork data/records/<record_id> \
  --record-id rec_longer_handle_edit \
  "make the handle longer"
```

## What forking stores

Forking creates a new child record with its own first revision:

```text
data/records/<child_record_id>/
  record.json
  collections/
  revisions/
    rev_000001/
      prompt.txt
      model.py
      provenance.json
      cost.json   # optional
      inputs/     # only new inputs supplied for this edit
      traces/
```

The child stores only the new edit run's artifacts. It does not copy the parent's traces, cost files, provenance history, old model snapshots, or inputs. Instead, the child records lineage references back to the parent record and revision so the UI can show parent history without duplicating files.

## Dataset and workbench behavior

Forks inherit the parent's collection by default:

- Forking a workbench record creates a workbench child.
- Forking a dataset record creates a dataset child in the same category.
- Dataset children get a distinct dataset ID derived from the parent dataset ID and child record ID.

The parent record is not mutated. There is no user-facing in-place edit command because it creates confusing ownership, provenance, and UI state. Use `fork` for edits and `rerun` only when you intentionally want to regenerate an existing record from its stored prompt/provenance settings.

## View the edit history

After a fork finishes, rebuild or open the viewer as usual:

```bash
uv run articraft compile data/records/<child_record_id> --target visual
just viewer
```

The viewer history panel can traverse lineage references, so the child edit and its parent conversation remain inspectable without copying parent trajectories into the child record.
