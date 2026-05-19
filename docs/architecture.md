# Articraft Architecture & Organization

Articraft is designed for scalable articulated 3D asset generation through iterative modeling feedback loops. 

## Project Structure & Module Organization

- **`agent/`**: Contains the generation runtime, provider adapters, prompt compiler/loader, tools, cost tracking, TUI helpers, and batch orchestration.
- **`storage/`**: Owns the canonical `data/` layout, records, categories, dataset metadata, batch specs, run caches, materialization metadata, and search indexes.
- **`sdk/`** & **`sdk/_core/`**: Define the articulated-object SDK layers used by the generation models.
- **`sdk/_docs/`** & **`sdk/_examples/`**: Agent-facing authoring reference material and assets.
- **`viewer/api/`**: Exposes the FastAPI surface.
- **`viewer/web/`**: The React/TypeScript/Three.js viewer used for inspecting object and geometry structures visually.
- **`cli/`**: Contains the `articraft` entry points and subcommands.
- **`tests/`**: Mirrors the main packages with focused smoke and regression coverage.

## Dataset & Workbench Concepts

There are two primary ways to run generation in Articraft:
1. **Workbench**: Isolated runs via `articraft generate` for direct exploration and testing. State is stored locally and isn't inherently categorized until placed into a dataset.
2. **Dataset Batches**: Batch-run workflows driven by CSV specs under `data/batch_specs/`. These are tracked and assigned to a `category_slug`. This scales well with high concurrency and has a comprehensive resume and auto-recovery framework.

See [Dataset Generation](dataset_generation.md) for detailed workflows on creating and managing large-scale batch generation.

## Dependencies

- **Python Runtime**: Pinned to version `3.12` for `uv` (currently avoids `3.13` out of the box because of `cadquery` & `vtk` wheel constraints).
- **Frontend App**: standard npm `package.json` for Vite and Three.js running within `viewer/web/`.
