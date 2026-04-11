---
title: 'Single Continuous Rotary Shaft'
description: 'Trimmed from the 5-star pottery-wheel record; shows a continuous spindle with a fixed driven wheelhead.'
tags:
  - cadquery
  - examples
  - articulation
  - continuous
  - rotary
  - spindle
---
# Single Continuous Rotary Shaft

This excerpt keeps the actual three-part split from the 5-star pottery-wheel record: grounded base, continuous shaft, and a fixed wheelhead carried by that shaft.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel", assets=ASSETS)

    model.material("body_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("wheelhead", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(_make_base_visual_mesh(), material="body_gray")
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.32, 0.31)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
        material="steel",
    )
    shaft.visual(
        Box(COUPLING_LUG_SIZE),
        origin=Origin(xyz=COUPLING_LUG_ORIGIN),
        material="steel",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
    )

    wheelhead = model.part("wheelhead")
    wheelhead.visual(_make_wheelhead_visual_mesh(), material="wheelhead")
    wheelhead.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEELHEAD_RADIUS, length=WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="shaft",
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    model.articulation(
        "shaft_to_wheelhead",
        ArticulationType.FIXED,
        parent="shaft",
        child="wheelhead",
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    return model
```
