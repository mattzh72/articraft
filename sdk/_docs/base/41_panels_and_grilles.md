# Panels and Grilles (`sdk`)

## Purpose

Use these helpers for panel faces with repeated openings, slots, or slats.
They stay on the base `sdk` surface and export through `mesh_from_geometry(...)`.

## Import

```python
from sdk import (
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleFrame,
    VentGrilleMounts,
    VentGrilleSleeve,
    mesh_from_geometry,
)
```

## Recommended Surface

- `PerforatedPanelGeometry` for round-hole plates
- `SlotPatternPanelGeometry` for repeated slot faces
- `VentGrilleGeometry` for framed vent/register faces with slats and an optional rear sleeve
- `VentGrilleSlats`, `VentGrilleFrame`, `VentGrilleMounts`, and `VentGrilleSleeve`
  for vent-specific customization

### `PerforatedPanelGeometry`

```python
PerforatedPanelGeometry(
    panel_size,
    thickness,
    *,
    hole_diameter,
    pitch,
    frame=0.008,
    corner_radius=0.0,
    stagger=False,
    center=True,
)
```

- Builds a flat plate in local `XY` with round through-holes.
- `pitch` can be a scalar or `(x_pitch, y_pitch)` tuple.
- `stagger=True` offsets alternating rows.
- `center=False` places the rear face on `z=0`.

### `SlotPatternPanelGeometry`

```python
SlotPatternPanelGeometry(
    panel_size,
    thickness,
    *,
    slot_size,
    pitch,
    frame=0.008,
    corner_radius=0.0,
    slot_angle_deg=0.0,
    stagger=False,
    center=True,
)
```

- Builds a slotted face in local `XY`.
- `slot_angle_deg` rotates each slot in plan view and uses degrees.
- `pitch` can be a scalar or `(x_pitch, y_pitch)` tuple.
- `center=False` places the rear face on `z=0`.

### `VentGrilleGeometry`

```python
VentGrilleGeometry(
    panel_size,
    *,
    frame: float = 0.012,
    face_thickness: float = 0.004,
    duct_depth: float = 0.026,
    duct_wall: float = 0.003,
    slat_pitch: float = 0.018,
    slat_width: float = 0.009,
    slat_angle_deg: float = 35.0,
    slat_thickness: float | None = None,
    corner_radius: float = 0.0,
    slats: VentGrilleSlats | None = None,
    frame_profile: VentGrilleFrame | None = None,
    mounts: VentGrilleMounts | None = None,
    sleeve: VentGrilleSleeve | None = None,
    center: bool = True,
)
```

- Builds a framed vent/register face with real slats and an optional rear sleeve.
- The grille lies in local `XY` and extends along `Z`.
- `slat_angle_deg` uses degrees.
- `center=False` places the rear-most face on `z=0`.
- Use `VentGrilleSlats` for slat profile, direction, setback, and divider bars.
- Use `VentGrilleFrame` for flush, beveled, or radiused frame treatments.
- Use `VentGrilleMounts` for corner mounting holes.
- Use `VentGrilleSleeve` to remove the sleeve or switch between short and full-depth sleeves.

### `VentGrilleSlats`

```python
VentGrilleSlats(
    profile: Literal["flat", "airfoil", "boxed"] = "flat",
    direction: Literal["down", "up"] = "down",
    inset: float = 0.0,
    divider_count: int = 0,
    divider_width: float = 0.004,
)
```

- `profile` changes the slat cross-section.
- `direction` flips the slat angle upward or downward.
- `inset` shifts the slats rearward into the sleeve.
- `divider_count` and `divider_width` add vertical divider bars across the opening.

### `VentGrilleFrame`

```python
VentGrilleFrame(
    style: Literal["flush", "beveled", "radiused"] = "flush",
    depth: float = 0.0,
)
```

- `style="beveled"` adds a front chamfered face treatment.
- `style="radiused"` rounds the front face perimeter.
- `depth` controls the visible bevel or round-over amount.

### `VentGrilleMounts`

```python
VentGrilleMounts(
    style: Literal["none", "holes"] = "none",
    inset: float = 0.008,
    hole_diameter: float | None = None,
)
```

- `style="holes"` cuts four corner mounting holes through the face.
- `inset` controls the hole setback from the outer perimeter.

### `VentGrilleSleeve`

```python
VentGrilleSleeve(
    style: Literal["none", "short", "full"] = "full",
    depth: float | None = None,
    wall: float | None = None,
)
```

- `style="none"` removes the rear sleeve entirely.
- `style="short"` uses a shallow sleeve.
- `style="full"` uses the full rear sleeve depth.
- `depth` and `wall` override the default sleeve dimensions.

## Examples

```python
speaker_face = PerforatedPanelGeometry(
    (0.16, 0.10),
    0.004,
    hole_diameter=0.006,
    pitch=(0.012, 0.012),
    frame=0.010,
    corner_radius=0.004,
    stagger=True,
)
mesh = mesh_from_geometry(speaker_face, "speaker_face")
```

```python
filter_face = SlotPatternPanelGeometry(
    (0.18, 0.09),
    0.004,
    slot_size=(0.024, 0.006),
    pitch=(0.032, 0.016),
    frame=0.010,
    corner_radius=0.004,
    slot_angle_deg=18.0,
    stagger=True,
)
mesh = mesh_from_geometry(filter_face, "filter_face")
```

```python
vent_grille = VentGrilleGeometry(
    (0.18, 0.10),
    frame=0.012,
    face_thickness=0.004,
    duct_depth=0.026,
    duct_wall=0.003,
    slat_pitch=0.018,
    slat_width=0.009,
    slat_angle_deg=30.0,
    corner_radius=0.006,
    slats=VentGrilleSlats(
        profile="airfoil",
        direction="down",
        divider_count=2,
        divider_width=0.004,
    ),
    frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
    mounts=VentGrilleMounts(style="holes", inset=0.010, hole_diameter=0.0032),
    sleeve=VentGrilleSleeve(style="full"),
)
mesh = mesh_from_geometry(vent_grille, "vent_grille")
```

## Clarifications for agent usage

- Use `VentGrilleGeometry` for a finished grille/register face. Do not fall back to
  manual slat-by-slat mesh assembly when this helper matches the shape.
- `VentGrilleSleeve(style="none")` is useful for shallow appliance vents and face-only grilles.
- Divider bars are part of `VentGrilleSlats`, not a separate frame helper.
