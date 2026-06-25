---
name: articraft-viewer
description: Use when compiling Articraft records, opening the local viewer, inspecting articulated assets visually, or working on viewer API/web code.
---

# Articraft Viewer

Use this skill when the user asks to compile records, browse generated objects, inspect articulation, start the viewer, or work on `viewer/api` or `viewer/web`.

## Compile Records

For one record:

```bash
uv run articraft compile <record_id>
```

## Start The Viewer

Built viewer flow:

```bash
just viewer
```

Development viewer flow with API and Vite:

```bash
just viewer-dev
```

API only:

```bash
uv run uvicorn viewer.api.app:app --reload --host 127.0.0.1 --port 8765
```

Frontend-only commands:

```bash
npm --prefix viewer/web run dev
npm --prefix viewer/web run typecheck
npm --prefix viewer/web run lint
npm --prefix viewer/web run build
```

## Visual QA

When inspecting an asset, verify:

- all primary parts are visible and connected
- articulations move the intended parts
- no major unintended overlap or floating geometry is visible
- link names and controls are semantic
- materials and colors match the object prompt
- history/lineage is preserved for forked records

If the viewer or frontend code changed, run the relevant frontend checks:

```bash
npm --prefix viewer/web run typecheck
npm --prefix viewer/web run lint
```

If API behavior changed, run focused viewer/API tests:

```bash
uv run --group dev pytest -q tests/viewer
```
