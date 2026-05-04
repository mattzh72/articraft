# result_figure_plotting

Standalone tool that turns one articraft record id into a multi-angle Blender Cycles figure (motion poses + part segmentation), compiling the record first if it has not been materialized yet.

## What it produces

For each record id you pass, an output directory like:

```
output/<record_id>/
  angle_0/  motion_1.png  motion_half.png  part_segmentation.png  composite.png
  angle_1/  ...
  ...
  angle_7/  ...
  composite.png         ← contact sheet (one row per angle, one column per mode)
  legend.json
```

Defaults: **8 azimuths** (every 45° around the object), elevation **35°**, **1.25×** bbox margin so motion poses never crop, samples **64**, resolution **1080×1080**, modes `motion_1, motion_half, part_segmentation`, no joint overlay.

`motion_half` puts each joint at the **midpoint of its URDF range** (revolute / prismatic → `(lower + upper) / 2`, continuous → π/2). This is the "halfway-extended" pose you'd see in the middle of a teaser_blender oscillation, not a fixed 30°.

## Files

```
render_object.sh               wrapper: compile → render
visualize.py                   CLI that drives Blender (composes contact sheet)
render_urdf_viz.py             Blender-side URDF parser + scene/camera/render code
symmetrical_garden_02_4k.exr   default HDRI (sits next to visualize.py)
output/                        renders land here by default
```

## Prerequisites

- `just` (homebrew: `brew install just`)
- `uv` and a synced articraft env (`uv sync --group dev` once at the repo root)
- Blender at `/Applications/Blender.app/Contents/MacOS/blender`

The tool assumes its parent directory is the articraft repo (it lives at `<articraft>/result_figure_plotting/`). To point at a different repo, pass `--repo /path/to/articraft`.

## Quick start

```bash
cd result_figure_plotting

# Render one record at default 8 angles / 64 samples / 1080²
bash render_object.sh rec_branching_tree_with_three_independent_rotary_branches_5587a586efc940938efbddd243355ec3
```

If the record has not been materialized, the wrapper runs `just compile data/records/<id>` first. The materialization is cached at `data/cache/record_materialization/<id>/`.

## Flags

```
--output <dir>        output root (default: ./output)
--angles N            azimuths around the object (default 8, evenly spaced)
--elevation D         camera elevation in degrees (default 35)
--samples N           Cycles samples (default 64; 256 with --high)
--resolution WxH      pixel size (default 1080x1080; 1440x1440 with --high)
--margin F            bbox padding multiplier so motion never crops (default 1.25)
--modes csv           subset of motion_1, motion_15, motion_30, motion_half,
                      part_segmentation, collision (default the first three)
--high                preset: 256 samples + 1440x1440
--repo <path>         articraft repo root (default: parent dir of this folder)
```

## More examples

```bash
# Higher quality preset
bash render_object.sh rec_xxx --high

# Add the collision view
bash render_object.sh rec_xxx --modes motion_1,motion_half,part_segmentation,collision

# 16 angles for a thorough turntable
bash render_object.sh rec_xxx --angles 16

# Custom output dir
bash render_object.sh rec_xxx --output /tmp/myrender

# Different articraft repo
bash render_object.sh rec_xxx --repo /Users/me/other-articraft
```

## Calling the renderer directly

The wrapper is a thin shell over `visualize.py`. The same command, written out:

```bash
CAM_NUM_ANGLES=8 CAM_MARGIN=1.25 CAM_ELEVATION_DEG=35 \
  python visualize.py \
    --urdf ../data/cache/record_materialization/<id>/model.urdf \
    --output ./output \
    --modes motion_1,motion_half,part_segmentation \
    --samples 64 --resolution 1080x1080 --no-joint-overlay
```

`render_urdf_viz.py` reads three env vars to override the camera setup without command-line plumbing:

| Env var              | Default | Purpose                                                    |
|----------------------|---------|------------------------------------------------------------|
| `CAM_NUM_ANGLES`     | 4       | Azimuth count (4 keeps front/right/back/left labels; otherwise even spacing) |
| `CAM_ELEVATION_DEG`  | 20      | Camera elevation                                           |
| `CAM_MARGIN`         | 1.08    | Bbox padding multiplier; bigger = more breathing room      |
| `JOINT_OVERLAY_ANGLES` | (all) | Comma-separated indices to render only specific cameras   |

## Why `motion_half` exists

`motion_30` clamps to each joint's URDF upper limit, which means a joint with a 0.45 rad upper hits its stop while one with a 1.5 rad upper sits at 30° — visually inconsistent across records. `motion_half` is the same per-joint sweep the teaser_blender oscillation passes through at its midpoint, so every articulated joint reads as "half-extended" regardless of its limit magnitude.

## Notes

- A render-blocking bug where joint origin `rpy` was discarded for revolute / continuous joints (cylinders mounted with a non-Z axis showed up vertical) is **fixed** here in `render_urdf_viz.py`'s `set_joint_pose`. If you copy `render_urdf_viz.py` somewhere else, keep the `rpy_q @ Quaternion(axis, value)` composition.
- Outputs at 8 angles × 3 modes / 64 samples / 1080² take roughly 4–8 minutes per object on Apple Silicon.
- The HDRI sits next to `visualize.py` so the folder is portable.
