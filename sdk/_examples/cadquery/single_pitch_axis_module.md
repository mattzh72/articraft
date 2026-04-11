---
title: 'Single Pitch Axis Module'
description: 'Trimmed from the 5-star motorized tilt cradle record; shows the stand, cradle, and centered tilt axis.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - pitch
  - cradle
---
# Single Pitch Axis Module

This excerpt keeps the real stand-and-cradle split from the 5-star tilt record. The helper mesh builders are omitted, but the pitch-joint placement is the same.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_tilt_cradle", assets=ASSETS)
    model.material("tower_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("plate_light", rgba=(0.74, 0.77, 0.80, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_shape(), "stand.obj", assets=ASSETS),
        material="tower_dark",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.12)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_make_cradle_shape(), "cradle.obj", assets=ASSETS),
        material="plate_light",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.140, 0.076, 0.040)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=TILT_LOWER,
            upper=TILT_UPPER,
            effort=18.0,
            velocity=1.5,
        ),
    )

    return model
```
