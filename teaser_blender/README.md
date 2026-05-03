# teaser_blender/

URDF → animated `.blend`. Real-world scale, HDRI environment, floor, every joint animated (continuous joints spin, revolute/prismatic sweep their full URDF range).

## One-time

Records must be compiled before export. From the repo root:

```bash
just compile data/records/<record_id>      # one record
# or compile a whole list:
xargs -n1 -P8 -I{} just compile data/records/{} < good_case_id/<list>.txt
```

Compiled URDFs land in `data/cache/record_materialization/<record_id>/`.

## One object

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b -P teaser_blender/urdf_to_blender.py -- <record_id>
```

Output: `teaser_blender/blender_out/<record_id>.blend`.

## Many objects in one grid

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b -P teaser_blender/urdf_to_blender.py -- \
    --grid teaser_blender/blender_out/<name>.blend \
    --from teaser_blender/good_case_id/<list>.txt
```

Real-world scale preserved; cell size = largest object's footprint + padding.

## Flags

- `--padding <m>` — meters between grid cells. Default 0.4. Use larger for architectural, smaller for handheld.
- `--hdri <path.exr>` — swap the environment map. Default: sibling `symmetrical_garden_02_4k.exr`.
- `--no-hdri` / `--no-floor` — skip either.

## Per-tier export (object_scales.json)

```bash
B=/Applications/Blender.app/Contents/MacOS/Blender
$B -b -P teaser_blender/urdf_to_blender.py -- --grid teaser_blender/blender_out/tier_handheld.blend      --padding 0.1 --from teaser_blender/good_case_id/tier_handheld.txt
$B -b -P teaser_blender/urdf_to_blender.py -- --grid teaser_blender/blender_out/tier_tabletop.blend      --padding 0.3 --from teaser_blender/good_case_id/tier_tabletop.txt
$B -b -P teaser_blender/urdf_to_blender.py -- --grid teaser_blender/blender_out/tier_human_scale.blend   --padding 1.0 --from teaser_blender/good_case_id/tier_human_scale.txt
$B -b -P teaser_blender/urdf_to_blender.py -- --grid teaser_blender/blender_out/tier_architectural.blend --padding 8.0 --from teaser_blender/good_case_id/tier_architectural.txt
```

## Files

```
urdf_to_blender.py            converter
render_urdf_viz.py            URDF parser/builder (used by the converter)
symmetrical_garden_02_4k.exr  default HDRI
object_scales.json            tier buckets
good_case_id/*.txt            curated record-id lists
blender_out/                  outputs (self-contained: HDRI sits beside the .blends)
```
