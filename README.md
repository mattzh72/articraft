# articraft

Generate articulated 3D objects from prompts and inspect them locally.

## 1. Install Prerequisites

You need:

- Python 3.11+
- [`uv`](https://docs.astral.sh/uv/)
- [`just`](https://github.com/casey/just)
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) if you want the viewer and frontend hooks ready locally

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
- initializes local storage under `data/`
- builds the dataset manifest used by the viewer

## 3. Add Your API Key

Open `.env` and set one provider key:

```text
Set `OPENAI_API_KEY` to your OpenAI key in `.env`
```

or:

```text
Set `GEMINI_API_KEYS` to your Gemini keys in `.env`
```

## 4. Open The Viewer Fast

If you just cloned the repo and want to browse saved records with meshes as quickly as possible, precompile the visual artifacts first:

```bash
just compile-all-visual
```

This is the recommended fast path for viewer use. It materializes viewer-ready visual meshes for saved records without doing the full collision-oriented compile.

If you want the full artifact path instead, including collision geometry, use:

```bash
just compile-all
```

If you want the slower validation-heavy variant of the full path, use:

```bash
just compile-all-strict
```

Then open the viewer:

```bash
just viewer
```

This starts the local API, builds the web app if needed, and opens the viewer.

## 5. Generate Your First Object

Run:

```bash
just wb "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

This saves a generated record into the local workbench.

If you do not pass overrides, `just wb` defaults to:

```bash
model=gpt-5.4
thinking=high
sdk=sdk
```

## 6. Run A Dataset Batch From CSV

Tracked dataset batch specs live under `data/batch_specs/`. Each CSV row defines one dataset generation job, including its own model settings.

Create a new empty spec with the correct header:

```bash
just name=<batch-id> batch-spec-new
```

Use this v1 header:

```csv
row_id,category_slug,category_title,prompt,provider,model_id,thinking_level,max_turns,sdk_package,label
```

Notes:

- Required columns: `category_slug`, `prompt`, `provider`, `model_id`, `thinking_level`, `max_turns`, `sdk_package`
- `category_title` is required when a row introduces a new category
- `row_id` and `label` are optional; if `row_id` is omitted it defaults to `row_0001`, `row_0002`, and so on
- `image_path` is not supported in v1
- `provider` must be `openai` or `gemini`
- `thinking_level` must be `low`, `med`, or `high`

Run a batch directly:

```bash
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8
```

Resume the latest run for that spec:

```bash
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8 --resume
```

Or use the `just` shortcut:

```bash
just spec=data/batch_specs/<batch-id>.csv concurrency=8 dataset-batch
```

Batch rows run concurrently up to the requested limit, successful outputs are promoted into canonical dataset storage under `data/records/`, and resumable batch state is stored under `data/cache/runs/<run_id>/`.

## 7. Reference

List commands:

```bash
just
```

Open the viewer in frontend dev mode:

```bash
just viewer-dev
just api_host=0.0.0.0 api_port=9000 viewer-dev
```

Rebuild the viewer search index:

```bash
just search-index
```

Recompile one saved record:

```bash
just compile-visual data/records/<record-id>
just compile data/records/<record-id>
just compile-strict data/records/<record-id>
just sdk_package=sdk_hybrid compile data/records/<record-id>
```

Bulk compile variants:

```bash
just compile-all-visual
just compile-all
just compile-all-strict
just force-compile-all-visual
just force-compile-all
```

Rerun generation for an existing record:

```bash
just rerun data/records/<record-id>
```

Generate with overrides:

```bash
just model=gemini-3-flash-preview wb "Create a compact tabletop fan with an oscillating head and tilt adjustment."
just image=reference.png wb "Create a weighted desk lamp with articulated arms."
just model=gpt-5.4 thinking=high image=reference.png wb "Create a tower crane with a rotating top and suspended hook."
```

Run a batch directly with `uv`:

```bash
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8
uv run articraft-dataset --repo-root . run-batch data/batch_specs/<batch-id>.csv --concurrency 8 --resume
```
