---
title: 'Vertical Slide with Wrist Hinge'
description: 'Trimmed from the 5-star Z-axis wrist module record; shows a lifting carriage with a hinged nose bracket.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - revolute
  - wrist
---
# Vertical Slide with Wrist Hinge

This excerpt keeps the column, carriage, and wrist split from the 5-star Z-axis wrist record. It is a good pattern for lift-and-tilt toolheads.

```python
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="z_axis_wrist_module", assets=ASSETS)

    model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("ground_rail", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("machine_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("safety_orange", rgba=(0.88, 0.50, 0.12, 1.0))

    column = model.part("column")
    carriage = model.part("carriage")
    wrist = model.part("wrist")

    _add_visual_mesh(column, _make_column_shape(), "column.obj", "machine_gray")
    _add_visual_mesh(carriage, _make_carriage_shape(), "carriage.obj", "painted_steel")
    _add_visual_mesh(wrist, _make_wrist_shape(), "wrist.obj", "safety_orange")

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.004, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.22, effort=900.0, velocity=0.35),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=0.90, effort=45.0, velocity=2.5),
    )

    return model
```
