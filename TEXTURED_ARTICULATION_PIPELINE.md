# Textured Articulation Pipeline

Build an articulated asset from URDF, merge it into a whole mesh, texture it with Hunyuan3D, and visualize the textured result while still using the original URDF joints.

## 1. Compile A Record Into Materialization Outputs

Start from a saved record and compile it into the canonical materialization directory:

```bash
just compile data/records/<record-id>
```

Example:

```bash
just compile data/records/rec_conventional_oven_with_dropdown_door_0001
```

This creates:

```text
data/cache/record_materialization/<record-id>/
```

and usually includes:

```text
model.urdf
assets/
compile_report.json
```

## 2. Treat The URDF As The Source Of Truth

The URDF is the canonical articulation definition.

It defines:

- link names
- joint hierarchy
- joint axes and limits
- link-local geometry placement

Important:
all later stages should stay in the same coordinate frame as `model.urdf`.

## 3. Merge URDF Parts Into A Whole Mesh

Use the merge script to bake all URDF link and visual transforms into one whole mesh:

```bash
uv run python scripts/merge_urdf_meshes.py \
  data/cache/record_materialization/<record-id>
```

Example:

```bash
uv run python scripts/merge_urdf_meshes.py \
  data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001
```

This writes:

```text
data/cache/record_materialization/<record-id>/model_merged.obj
data/cache/record_materialization/<record-id>/model_merged.parts.json
```

What these files mean:

- `model_merged.obj` is the whole object mesh in URDF world coordinates
- `model_merged.parts.json` preserves per-link and per-component vertex and face ranges so the whole mesh can be mapped back to URDF parts later

## 4. Upload The Whole Mesh To Hunyuan3D

The texturing stage is external to this repo.

Upload:

```text
data/cache/record_materialization/<record-id>/model_merged.obj
```

to Hunyuan3D.

Then download the generated textured result in `.usdz` format.

Important:

- do not rotate, recenter, or rescale the mesh before upload
- download the result in `.usdz` format
- the downloaded `.usdz` must stay aligned to the original URDF coordinate system

## 5. Place The Downloaded USDZ Into The Materialization Directory

The current viewer path expects a textured `.usdz` file at the root of the materialization directory.

Put the downloaded file here:

```text
data/cache/record_materialization/<record-id>/<name>.usdz
```

Example:

```text
data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001/oven_textured.usdz
```

## 6. Check That The Textured USDZ Still Matches The URDF

Before opening the viewer, verify that the textured USDZ is still aligned with the URDF:

```bash
uv run python scripts/check_urdf_usdz_alignment.py \
  data/cache/record_materialization/<record-id>
```

Example:

```bash
uv run python scripts/check_urdf_usdz_alignment.py \
  data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001
```

This checks the textured USDZ against the URDF link bounds after applying the real USD transform stack.

If this passes, the textured asset is in the same coordinate frame as the original URDF.

## 7. Open The Viewer

Start the viewer:

```bash
just viewer
```

or for local web development:

```bash
just viewer-dev
```

## 8. How The Viewer Uses The Asset

The viewer uses two different sources for two different jobs:

- `model.urdf` provides articulation and link hierarchy
- the textured `.usdz` provides appearance

The viewer workflow is:

1. Load `model.urdf`
2. Build the URDF joint tree
3. Load the textured `.usdz`
4. Match USDZ nodes back to URDF link names
5. Attach textured nodes under the corresponding URDF link groups
6. Drive motion using the URDF joints

This means:

- articulation always comes from the URDF
- the textured mesh is only the visual surface
- moving a joint moves the textured geometry through the URDF hierarchy

## 9. Coordinate Convention Summary

Keep these rules throughout the pipeline:

- `model.urdf` is the source of truth
- `model_merged.obj` must stay in URDF coordinates
- the Hunyuan3D textured mesh must stay in the same coordinates
- the final `.usdz` must also stay in the same coordinates

Do not:

- apply an extra global rotation
- recenter the mesh at the origin unless the URDF already assumes that
- rescale the textured mesh independently from the URDF

## 10. Minimal End-To-End Example

```bash
# 1. Compile the record
just compile data/records/rec_conventional_oven_with_dropdown_door_0001

# 2. Merge all URDF parts into one whole mesh
uv run python scripts/merge_urdf_meshes.py \
  data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001

# 3. Upload this mesh to Hunyuan3D:
#    data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001/model_merged.obj
#
# 4. Download the generated result as USDZ and place it at:
#    data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001/oven_textured.usdz

# 5. Verify alignment
uv run python scripts/check_urdf_usdz_alignment.py \
  data/cache/record_materialization/rec_conventional_oven_with_dropdown_door_0001

# 6. Open the viewer
just viewer
```

## 11. Output Summary

At the end of the pipeline, you should have:

```text
data/cache/record_materialization/<record-id>/
  model.urdf
  model_merged.obj
  model_merged.parts.json
  <textured-name>.usdz
  assets/
```

Roles of each file:

- `model.urdf`: articulation source of truth
- `model_merged.obj`: whole mesh for texturing
- `model_merged.parts.json`: part recovery metadata
- `<textured-name>.usdz`: textured viewer asset
