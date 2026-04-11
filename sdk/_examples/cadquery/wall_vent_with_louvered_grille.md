---
title: 'Wall Vent with Louvered Grille'
description: 'Trimmed from the wall-vent workbench record; shows a CadQuery-authored vent shell with a real slatted grille and recessed duct.'
tags:
  - cadquery
  - examples
  - vent
  - grille
  - louver
---
# Wall Vent with Louvered Grille

This trimmed CadQuery example is useful when a vent or grille needs to read as a clean manufactured shell rather than a stack of procedural mesh fragments.

```python
import cadquery as cq

from sdk import (
    ArticulatedObject,
    Box,
    Cylinder,
    Inertial,
    Origin,
    mesh_from_cadquery,
)


WIDTH = 0.18
HEIGHT = 0.10
FACE_THICKNESS = 0.004
FRAME = 0.012
OPENING_WIDTH = WIDTH - 2.0 * FRAME
OPENING_HEIGHT = HEIGHT - 2.0 * FRAME
DUCT_DEPTH = 0.026
DUCT_WALL = 0.003
SLAT_PITCH = 0.018
SLAT_HEIGHT = 0.009
SLAT_THICKNESS = 0.0014
SLAT_CHORD = OPENING_WIDTH + 0.004
SLAT_ANGLE_DEG = -35.0
FACE_ORIGIN_Z = 0.002


def _build_vent_shell_shape() -> cq.Workplane:
    front_ring = cq.Workplane("XY").box(WIDTH, HEIGHT, FACE_THICKNESS)
    front_ring = front_ring.cut(
        cq.Workplane("XY").box(
            OPENING_WIDTH,
            OPENING_HEIGHT,
            FACE_THICKNESS + 0.002,
        )
    )

    duct_outer = cq.Workplane("XY").box(OPENING_WIDTH, OPENING_HEIGHT, DUCT_DEPTH)
    duct_inner = cq.Workplane("XY").box(
        OPENING_WIDTH - 2.0 * DUCT_WALL,
        OPENING_HEIGHT - 2.0 * DUCT_WALL,
        DUCT_DEPTH + 0.004,
    )
    duct_shell = duct_outer.cut(duct_inner).translate(
        (0.0, 0.0, -FACE_THICKNESS / 2.0 - DUCT_DEPTH / 2.0 + 0.001)
    )

    shape = front_ring.union(duct_shell)

    y = -OPENING_HEIGHT / 2.0 + SLAT_PITCH / 2.0
    limit = OPENING_HEIGHT / 2.0 - SLAT_PITCH / 2.0
    while y <= limit + 1e-9:
        slat = cq.Workplane("XY").box(SLAT_CHORD, SLAT_HEIGHT, SLAT_THICKNESS)
        slat = slat.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), SLAT_ANGLE_DEG)
        slat = slat.translate((0.0, y, -0.001))
        shape = shape.union(slat)
        y += SLAT_PITCH

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vent", assets=ASSETS)
    model.material("plastic_white", rgba=(0.90, 0.90, 0.90, 1.0))
    model.material("metal_silver", rgba=(0.70, 0.70, 0.70, 1.0))

    vent_body = model.part("vent_body")
    vent_mesh = mesh_from_cadquery(_build_vent_shell_shape(), "vent_shell_mesh.obj", assets=ASSETS)
    vent_body.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, 0.0, FACE_ORIGIN_Z)),
        material="plastic_white",
        name="vent_shell",
    )

    screw_radius = 0.002
    screw_length = 0.002
    screw_offset_x = WIDTH / 2.0 - 0.006
    screw_offset_y = HEIGHT / 2.0 - 0.006
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        vent_body.visual(
            Cylinder(radius=screw_radius, length=screw_length),
            origin=Origin(
                xyz=(
                    sx * screw_offset_x,
                    sy * screw_offset_y,
                    FACE_ORIGIN_Z + FACE_THICKNESS + screw_length / 2.0,
                )
            ),
            material="metal_silver",
            name=f"screw_{i}",
        )

    vent_body.inertial = Inertial.from_geometry(
        Box((WIDTH, HEIGHT, FACE_THICKNESS + DUCT_DEPTH + FACE_ORIGIN_Z - 0.001)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, (FACE_ORIGIN_Z + FACE_THICKNESS - (DUCT_DEPTH - 0.001)) / 2.0)),
    )

    return model
```
