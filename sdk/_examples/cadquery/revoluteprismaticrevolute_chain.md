---
title: 'Revolute-prismatic-revolute Chain'
description: 'Trimmed from the 5-star service-manipulator record; shows a swivel base, mid-span slide, and wrist pitch.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - prismatic
  - manipulator
---
# Revolute-prismatic-revolute Chain

This excerpt keeps the real three-stage motion stack from the 5-star service-manipulator record. The helper shapes are omitted, but the part layout and joint origins are the same.

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
    model = ArticulatedObject(name="service_manipulator", assets=ASSETS)

    model.material("charcoal", rgba=(0.22, 0.23, 0.26, 1.0))
    model.material("silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("accent_blue", rgba=(0.17, 0.35, 0.58, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS), material="charcoal")
    base.visual(Box((0.07, 0.004, 0.035)), origin=Origin(xyz=(0.0, 0.089, 0.052)), material="accent_blue")

    arm_outer = model.part("arm_outer")
    arm_outer.visual(mesh_from_cadquery(_arm_outer_shape(), "arm_outer.obj", assets=ASSETS), material="charcoal")
    arm_outer.visual(Box((0.085, 0.05, 0.006)), origin=Origin(xyz=(0.165, 0.0, 0.067)), material="accent_blue")

    slide = model.part("slide")
    slide.visual(mesh_from_cadquery(_slide_shape(), "slide.obj", assets=ASSETS), material="silver")
    slide.visual(Box((0.028, 0.012, 0.03)), origin=Origin(xyz=(0.03, 0.033, 0.0)), material="rubber")

    wrist = model.part("wrist")
    wrist.visual(mesh_from_cadquery(_wrist_shape(), "wrist.obj", assets=ASSETS), material="silver")
    wrist.visual(Box((0.01, 0.05, 0.05)), origin=Origin(xyz=(0.13, 0.0, 0.0)), material="rubber")

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-BASE_SWIVEL_LIMIT, upper=BASE_SWIVEL_LIMIT, effort=50.0, velocity=1.2),
    )
    model.articulation(
        "mid_slide",
        ArticulationType.PRISMATIC,
        parent=arm_outer,
        child=slide,
        origin=Origin(xyz=(0.135, 0.0, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_MAX, effort=40.0, velocity=0.25),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=wrist,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=WRIST_LOWER, upper=WRIST_UPPER, effort=18.0, velocity=1.8),
    )

    return model
```
