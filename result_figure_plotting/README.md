# result_figure_plotting

Render figures for one or more articraft records: motion poses, joint overlays, and part segmentation. Two layers:

- **Per-record rendering** (`render_object.sh`) — render one record from many camera angles, useful for browsing.
- **Spec-driven figure pipeline** (`make_figures.py`) — given a picker JSON of `(record_id, camera_angle)` cells, produce the final per-record tile (3-panel composite) and a stacked contact sheet.

## Files

```
make_figures.py                end-to-end pipeline (render → crop → compose → contact sheet)
visualize.py                   CLI that drives Blender for one record
render_urdf_viz.py             Blender-side URDF parser + scene/camera/render code
render_object.sh               wrapper: compile-if-needed → 8 angles × N modes for ONE record
render_batch.sh                run render_object.sh over a list of record ids
symmetrical_garden_02_4k.exr   default HDRI (sits next to visualize.py)
```

## Prerequisites

- `just` (homebrew: `brew install just`)
- `uv` and a synced articraft env (`uv sync --group dev` once at the repo root)
- Blender at `/Applications/Blender.app/Contents/MacOS/blender`
- Python deps for the pipeline: `pillow` (already pulled in via the repo env)

## Quick start — figure pipeline

Given a picker JSON like `selecting.json` or `picker_layout_*.json` at the repo root, render and assemble all selected records:

```bash
cd result_figure_plotting

# Default: 128 samples, 2160x2160, joint overlay on, 5% margin crop
uv run python make_figures.py ../selecting.json --out output_selected_hq_cropped

# Drop the overlay (motion-only + segmentation)
uv run python make_figures.py ../selecting.json --out output_selected --no-overlay

# Lower quality, faster iteration
uv run python make_figures.py ../selecting.json --out output_selected_quick \
    --samples 64 --resolution 1080x1080

# Reuse cached renders if you only changed cropping/margin settings
uv run python make_figures.py ../selecting.json --out output_selected_hq_cropped --skip-render
```

The picker JSON must contain a `cells` array of objects with at least `record_id` and `angle_idx`. `slot` and `short_name` are optional and used for tile folder names.

### Output layout

```
<out_dir>/
  _renders/
    no_overlay/<record_id>/angle_<n>/  motion_1.png motion_half.png part_segmentation.png
    overlay/<record_id>/angle_<n>/     motion_1.png motion_half.png      (only if overlay on)
  <NN>_<short_name>/
    motion_1.png                cropped clean rest pose
    motion_half.png             cropped clean halfway pose
    part_segmentation.png       cropped segmentation
    motion_1_overlay.png        cropped rest pose + joint overlay   (overlay on)
    motion_half_overlay.png     cropped halfway pose + joint overlay (overlay on)
    composite.png               3-panel: motion_1, motion_half + overlay, part_segmentation
                                (transparent background, no padding)
  all_records_contact_sheet.png stack of every record's composite.png
```

`composite.png` and `all_records_contact_sheet.png` are RGBA PNGs with a transparent background and no inter-panel padding — ready to drop into a layout document.

## Joint overlay

There is one overlay style: **`I_axis_through`** (the `JOINT_OVERLAY_STYLE` env var). It renders a symmetric double-headed axis arrow through the joint origin — yellow for prismatic, red with a base ring for revolute / continuous — and uses an always-in-front two-pass composite so the marker is never occluded by the model. Revolute / continuous markers anchor at the joint origin (rotation center). Prismatic markers anchor at the child link's bbox center so the arrow rides the moving part. Per-joint sizing scales with the child link's bbox diagonal so a tiny part on a tall tower doesn't shrink to invisibility.

This is also what `make_figures.py` bakes into `motion_1_overlay.png` / `motion_half_overlay.png` automatically.

## Per-record rendering (browse mode)

For browsing a single object across 8 camera angles, use the older wrapper:

```bash
bash render_object.sh rec_traditional_windmill_0002
bash render_object.sh rec_xxx --modes motion_1,motion_half,part_segmentation,collision
bash render_object.sh rec_xxx --high                # 256 samples + 1440x1440
bash render_object.sh rec_xxx --angles 16           # 16 turntable angles
```

Or batch over a list:

```bash
bash render_batch.sh records.txt --high             # records.txt: one id per line
```

These write into `output/<record_id>/angle_*/` with one `composite.png` per angle and a contact sheet at the record root. They use `--no-joint-overlay` by default — the figure pipeline above is the path that produces overlay tiles.

## Camera environment vars

`render_urdf_viz.py` reads three env vars to override the camera setup without command-line plumbing:

| Env var              | Default | Purpose                                                    |
|----------------------|---------|------------------------------------------------------------|
| `CAM_NUM_ANGLES`     | 4       | Azimuth count (4 keeps front/right/back/left labels; otherwise even spacing) |
| `CAM_ELEVATION_DEG`  | 20      | Camera elevation                                           |
| `CAM_MARGIN`         | 1.08    | Bbox padding multiplier; bigger = more breathing room      |
| `JOINT_OVERLAY_ANGLES` | (all) | Comma-separated indices to render only specific cameras   |
| `JOINT_OVERLAY_STYLE`  | `A_amber_arc` | Overlay preset; figure pipeline forces `I_axis_through` |

`make_figures.py` sets `CAM_NUM_ANGLES=8`, `CAM_MARGIN=1.25`, `CAM_ELEVATION_DEG=35`, and `JOINT_OVERLAY_ANGLES=<the cell's angle_idx>` so each record renders only the chosen camera.

## Notes

- The pipeline assumes its parent directory is the articraft repo. Pass `--repo /path/to/articraft` to point elsewhere.
- If a record hasn't been compiled yet, run `just compile data/records/<id>` from the repo root first.
- Outputs at 2160² / 128 samples take roughly 10–20 s per render on Apple Silicon (≈1–2 minutes per record with the overlay pass).
