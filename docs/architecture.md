# Articraft Architecture & Organization

Articraft is designed for local-first articulated 3D asset generation through iterative modeling feedback loops.

## Project Structure & Module Organization

- **`agent/`**: Contains the generation runtime, provider adapters, prompt compiler/loader, tools, cost tracking, TUI helpers, and single-record orchestration.
- **`storage/`**: Owns the local `data/` layout, records, categories, `records_manifest.jsonl`, run caches, and materialization metadata.
- **`sdk/`** & **`sdk/_core/`**: Define the articulated-object SDK layers used by the generation models.
- **`sdk/_docs/`** & **`sdk/_examples/`**: Agent-facing authoring reference material and assets.
- **`viewer/api/`**: Exposes the FastAPI surface.
- **`viewer/web/`**: The React/TypeScript/Three.js viewer used for inspecting object and geometry structures visually.
- **`cli/`**: Contains the `articraft` entry points and subcommands.
- **`tests/`**: Mirrors the main packages with focused smoke and regression coverage.

## Local Library Concepts

Articraft uses one library model: every record is local, complete, and browsable. The default data root is the gitignored `<repo-root>/data`; set `ARTICRAFT_DATA_DIR` or pass `--data-dir` to point at another local/exportable folder.

The viewer and CLI read `records_manifest.jsonl` for record summaries and then load complete record files directly from `records/<record_id>/record.json`. Rebuild the manifest with:

```bash
uv run articraft library rebuild-manifest
```

## Dependencies

- **Python Runtime**: Pinned to version `3.12` for `uv` (currently avoids `3.13` out of the box because of `cadquery` & `vtk` wheel constraints).
- **Frontend App**: standard npm `package.json` for Vite and Three.js running within `viewer/web/`.
