# Codex Plugin Setup

You can create Articraft objects with Codex even when you do not have local provider API keys configured in `.env`. In this mode, Codex is the AI runtime and Articraft provides the repository, SDK, validation commands, and local library workflow.

Codex-authored data must follow the same external-agent contract as Claude Code and Cursor. The source of truth is [EXTERNAL_AGENT_DATA.md](../EXTERNAL_AGENT_DATA.md), which tells Codex how to initialize, compile, finalize, and categorize records.

## Add The Codex Plugin

This repository includes a repo-local Codex plugin under `plugins/articraft/` for Articraft-specific Codex workflows.

After completing the Quickstart setup from the repository root:

1. Open this repository in Codex.
2. Restart or reload Codex so it can discover `.agents/plugins/marketplace.json`.
3. In the Codex plugin or marketplace UI, install **Articraft** from **Articraft Local**.
4. Let Codex run local repository commands. It will use `uv run articraft external ...` internally when creating or editing records.

You do not need to add Articraft provider keys for this path. The external agent supplies the model access, while the repository records the contribution as `creator.mode=external_agent` and `creator.agent=codex`.

The plugin itself adds Codex guidance and does not introduce any additional credential requirements beyond the Articraft workflows you choose to run.

## Prompt Codex

Once Codex is connected to the repository, start with a direct request:

```text
Create a realistic articulated [object name] in Articraft. Follow EXTERNAL_AGENT_DATA.md.
```

For example:

```text
Create a realistic articulated microscope with a tilting head, rotating objective turret, adjustable stage, and focus knobs. Follow EXTERNAL_AGENT_DATA.md.
```

Codex should initialize the record with:

```bash
uv run articraft external init --agent codex "<object prompt>"
```

It should then edit only the generated record's active `model.py`, run:

```bash
uv run articraft compile <record_id>
```

and finalize the record, using `--category-slug` only when assigning a library category.

## Before Opening A PR

Ask Codex to leave you with:

- the generated `record_id`
- the category slug used, if any
- the validation command it ran
- any warnings or known limitations

Then inspect the object in the viewer and rate it before sharing the data folder.
