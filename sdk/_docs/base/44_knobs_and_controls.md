# Knobs and Controls

## Purpose

Use `KnobGeometry` for rotary control caps and appliance knobs before reaching
for manual revolve-plus-detail construction.

## Import

```python
from sdk import (
    KnobGeometry,
    KnobSkirt,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    KnobBore,
    KnobRelief,
    mesh_from_geometry,
)
```

## Recommended APIs

| Shape intent | Helper |
| --- | --- |
| Plain appliance, range, encoder, clamp, or pointer knob | `KnobGeometry` |
| Skirt ring, flare, and lower-edge chamfer details | `KnobSkirt` |
| Fluted, ribbed, scalloped, or knurled side detail | `KnobGrip` |
| Pointer lines, notches, wedges, dots | `KnobIndicator` |
| Caps, recesses, and top inserts | `KnobTopFeature` |
| Shaft interface details | `KnobBore` |
| Side windows, top recesses, coin slots | `KnobRelief` |

## API Reference

### `KnobGeometry`

```python
KnobGeometry(
    diameter,
    height,
    *,
    body_style: Literal[
        "cylindrical",
        "tapered",
        "domed",
        "mushroom",
        "skirted",
        "hourglass",
        "faceted",
        "lobed",
    ] = "cylindrical",
    top_diameter=None,
    base_diameter=None,
    crown_radius: float = 0.0,
    edge_radius: float = 0.0,
    side_draft_deg: float = 0.0,
    skirt: KnobSkirt | None = None,
    grip: KnobGrip | None = None,
    indicator: KnobIndicator | None = None,
    top_feature: KnobTopFeature | None = None,
    bore: KnobBore | None = None,
    body_reliefs: Sequence[KnobRelief] = (),
    center: bool = True,
)
```

- Builds a rotary knob aligned to local `Z`.
- `center=False` places the mounting face on `z=0`.
- Use the detail dataclasses to add visible grip, pointer, bore, and cap
  features without switching to manual mesh work.
- This helper is intended to cover plain appliance knobs, fluted or knurled
  knobs, mushroom clamp knobs, faceted synth knobs, lobed hand knobs, and
  pointer-style dial caps.

## Advice

- Start with `KnobGeometry` whenever the part is visibly a knob, even if the
  final result has multiple detail features.
- Choose a silhouette before you add detail. `skirted` reads like appliance or
  oven controls, `faceted` reads like a machined or synth control, `lobed`
  reads like a hand-tightened clamp knob, `domed` reads like soft consumer
  electronics, and `hourglass` reads like a finger-pinched dial.
- Model the shaft interface when it would be visible or when the prompt calls
  out a D-shaft, splined, or keyed connection.
- Use indicators and top features for semantic detail instead of engraving them
  manually with ad hoc booleans.
- Do not default every generic "knob" prompt to the same skirted + fluted +
  engraved line stack. Match the product family and vary the body style, grip
  style, top feature, and indicator treatment together.

## Examples

### Range knob

```python
range_knob = KnobGeometry(
    0.042,
    0.024,
    body_style="skirted",
    top_diameter=0.034,
    skirt=KnobSkirt(0.052, 0.006, flare=0.08, chamfer=0.0012),
    grip=KnobGrip(style="fluted", count=18, depth=0.0014),
    indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
    bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
)
mesh = mesh_from_geometry(range_knob, "range_knob")
```

### Knurled encoder knob

```python
encoder_knob = KnobGeometry(
    0.020,
    0.015,
    body_style="cylindrical",
    grip=KnobGrip(style="knurled", count=36, depth=0.0008, helix_angle_deg=20.0),
    indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
)
mesh = mesh_from_geometry(encoder_knob, "encoder_knob")
```

### Faceted synth knob

```python
synth_knob = KnobGeometry(
    0.026,
    0.018,
    body_style="faceted",
    base_diameter=0.028,
    top_diameter=0.020,
    edge_radius=0.0007,
    grip=KnobGrip(style="ribbed", count=12, depth=0.0007, width=0.0016),
    indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
    top_feature=KnobTopFeature(style="top_insert", diameter=0.010, height=0.0012),
)
mesh = mesh_from_geometry(synth_knob, "synth_knob")
```

### Lobed clamp knob

```python
clamp_knob = KnobGeometry(
    0.040,
    0.022,
    body_style="lobed",
    base_diameter=0.028,
    top_diameter=0.036,
    crown_radius=0.0014,
    bore=KnobBore(style="round", diameter=0.008),
    body_reliefs=(KnobRelief(style="top_recess", width=0.014, depth=0.0016),),
)
mesh = mesh_from_geometry(clamp_knob, "clamp_knob")
```

## See Also

- `40_mesh_geometry.md` for lower-level lathe, profile, and export helpers
- For a fuller worked script, see the base SDK fluted range knob example.
