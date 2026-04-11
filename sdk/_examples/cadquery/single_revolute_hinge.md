---
title: 'Single Revolute Hinge'
description: 'Trimmed from the 5-star wall-mounted cabinet record; shows the real body, door, and hinge layout.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - hinge
  - cabinet
---
# Single Revolute Hinge

This excerpt keeps the actual part split and hinge placement from the 5-star cabinet example. Helper mesh builders and dimensional constants are omitted here, but the object logic is unchanged.

The important sign choice is that the closed door extends along local `+X`
from the hinge line, so `axis=(0, 0, 1)` makes positive joint values swing the
free edge toward local/front `+Y` instead of into the carcass.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_cabinet", assets=ASSETS)

    carcass_finish = model.material("carcass_finish", rgba=(0.93, 0.94, 0.95, 1.0))
    door_finish = model.material("door_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("cabinet_body")
    body.visual(_body_mesh(), material=carcass_finish)
    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=8.5,
    )

    door = model.part("door")
    door.visual(_door_panel_mesh(), material=door_finish)
    door.visual(_door_handle_mesh(), material=handle_finish)
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent="cabinet_body",
        child="door",
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 - HINGE_SIDE_OFFSET,
                CABINET_DEPTH / 2.0 + FRONT_GAP,
                0.0,
            )
        ),
        # Closed door geometry extends along +X from the hinge line.
        # Positive q around +Z swings the free edge outward toward +Y.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=10.0, velocity=1.5),
    )

    return model
```
