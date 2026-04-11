# Asset Helpers

## Purpose

Use these helpers when you need an explicit asset root for authored meshes or
input meshes outside the managed harness defaults.

In the normal virtual workspace flow, you usually do not need this page.
Reach for `AssetContext` when you are writing a reusable local script, test, or
tooling flow that needs stable on-disk mesh paths.

## Import

```python
from sdk import AssetContext, ArticulatedObject
```

## Recommended Surface

- `AssetContext(...)`
- `AssetContext.from_script(...)`
- `ArticulatedObject(..., assets=...)`

## `AssetContext`

```python
AssetContext(
    root: str | Path,
    mesh_subdir: str = "assets/meshes",
)
```

- `root`: asset root directory for the authored script or test.
- `mesh_subdir`: relative mesh directory under `root`. Defaults to
  `"assets/meshes"`.

### `AssetContext.from_script(...)`

```python
AssetContext.from_script(
    script_path: str | Path,
) -> AssetContext
```

- Resolves the parent directory of the given script path.
- This is the normal entry point for standalone `model.py` style scripts.

### Properties

```python
asset_root: Path
mesh_dir: Path
```

- `asset_root`: resolved root directory for all relative mesh lookups.
- `mesh_dir`: resolved directory where managed mesh outputs should be written.

### Methods

```python
ensure_mesh_dir() -> Path
mesh_path(filename: str | Path, *, ensure_dir: bool = True) -> Path
mesh_ref(filename: str | Path) -> str
```

- `ensure_mesh_dir()`: creates `mesh_dir` if needed and returns it.
- `mesh_path(...)`: returns an on-disk path under `mesh_dir`.
- `mesh_ref(...)`: returns the SDK-facing mesh filename string relative to the
  asset root, for example `"assets/meshes/bracket.obj"`.

## Where `assets=` Is Used

These public surfaces accept an explicit asset owner or root:

- `ArticulatedObject(..., assets=...)`
- `export_cadquery_mesh(..., assets=...)`
- `export_cadquery_components(..., assets=...)`
- `mesh_from_cadquery(..., assets=...)`
- `mesh_components_from_cadquery(..., assets=...)`

If an object model already carries `assets=...`, prefer `TestContext(model)`
instead of repeating `asset_root=...`.

## Recommended Pattern

```python
from sdk import AssetContext, ArticulatedObject, TestContext

ASSETS = AssetContext.from_script(__file__)

model = ArticulatedObject(name="example", assets=ASSETS)
ctx = TestContext(model)
```

## Writing Managed Meshes

```python
from sdk import AssetContext, BoxGeometry, mesh_from_geometry

ASSETS = AssetContext.from_script(__file__)

mesh = mesh_from_geometry(
    BoxGeometry((0.10, 0.04, 0.02)),
    ASSETS.mesh_path("body.obj"),
)
```

- Use `mesh_path(...)` when a helper expects a writable path.
- Use `mesh_ref(...)` when you need the SDK mesh reference string only.

## Notes

- `AssetContext` is public, but it is an advanced script/runtime helper rather
  than the default authoring path inside the harness.
- Relative mesh references are resolved against `asset_root`.
- Use one coherent asset root per authored model when possible.
