# External Agent Authoring

You are an external agent authoring one Articraft local-library record. This is the supported workflow.

Do not manually create `<data-root>/records/<id>` folders, invent record metadata, write Articraft traces, or bypass the CLI commands below.

If the user wants Articraft-managed turns, tools, compile feedback, and trajectory, use the internal Codex CLI provider instead of this external workflow:

```bash
uv run articraft generate --provider codex-cli --model <codex-model-id> "<prompt>"
```

## Read This First

Before you create or edit a record, read the core authoring requirements:

```text
agent/prompts/sections/designer_common.md
agent/prompts/sections/link_naming.md
```

Then use the SDK docs and examples while you author:

```text
sdk/_docs/
sdk/_examples/
```

Quality and realism are important. Build the object with meaningful parts, primary user-facing articulation, realistic connected structure, concise semantic link names, and prompt-specific checks in `run_tests()`.

## 1. Initialize Storage

```bash
uv sync --group dev
uv run articraft init
```

By default Articraft writes to the gitignored `<repo-root>/data`. To work in another local/exportable data folder:

```bash
export ARTICRAFT_DATA_DIR=/Users/mzhou/articraft-data
uv run articraft status
```

## 2. Create A Record

Create the record through the external CLI and identify yourself explicitly:

```bash
uv run articraft external init --agent codex "washing machine"
uv run articraft external init --agent claude-code "washing machine"
uv run articraft external init --agent cursor "washing machine"
```

The command prints `record_id` and `record_dir`. Edit only that generated record unless the user explicitly asks for broader repository changes.

Allowed external agent ids are:

- `codex`
- `claude-code`
- `cursor`

## 3. Edit Existing Records

When the user asks you to modify an existing Articraft asset, fork it instead of manually copying record folders:

```bash
uv run articraft fork <record_id> "make the handle longer"
```

Forking creates a child record and leaves the parent unchanged.

## 4. Author The Object

Edit the active model file under:

```text
<data-root>/records/<record_id>/revisions/<revision_id>/model.py
```

The prompt is stored next to the active revision. Use `record.json` to confirm the active revision id when needed.

## 5. Iterate

Compile the one record you are editing:

```bash
uv run articraft compile <record_id>
```

Repeat authoring and compiling until the record passes. The viewer will point missing-artifact messages at this same command.

## 6. Finalize

Finalize the record to upsert `records_manifest.jsonl`. Add a category only when the user requested one:

```bash
uv run articraft external finalize <record_id>
uv run articraft external finalize <record_id> --category-slug washing_machine
uv run articraft library check --require-records
```

## Rules

You must:

- use `articraft external init` before writing a new external-agent record from scratch
- use `articraft fork` before modifying an existing record
- preserve `creator.mode=external_agent`
- preserve `creator.agent=codex`, `creator.agent=claude-code`, or `creator.agent=cursor`
- preserve `creator.trace_available=false`
- run `articraft compile <record_id>` before finalizing
- run `articraft external finalize <record_id>` when done
- edit only the active revision for the one record you are authoring

You must not:

- manually create record directories
- manually copy parent record folders for edits
- claim internal Articraft harness traces
- write files under `traces/`
- edit unrelated records while authoring one object
