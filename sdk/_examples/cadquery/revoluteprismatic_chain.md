---
title: 'Revolute-prismatic Chain'
description: 'Trimmed from the 5-star inspection-arm record; shows a rotary base hinge feeding a linear extension stage.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - prismatic
  - inspection
---
# Revolute-prismatic Chain

This is a trimmed excerpt from the 5-star inspection-arm record. It keeps the actual mixed topology and the extra inspection-head details on the sliding stage.

```python
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_arm", assets=ASSETS)

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("zinc", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("aluminum", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("anodized_dark", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("glass", rgba=(0.52, 0.72, 0.84, 0.55))
    model.material("amber", rgba=(0.93, 0.64, 0.17, 0.95))

    base_mount = model.part("base_mount")
    base_mount.visual(mesh_from_cadquery(_base_mount_shape(), "base_mount.obj", assets=ASSETS), material="powder_steel")
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.032, 0.032):
            base_mount.visual(Cylinder(radius=0.008, length=0.004), origin=Origin(xyz=(x_pos, y_pos, 0.014)), material="zinc")

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(mesh_from_cadquery(_pivot_arm_shape(), "pivot_arm.obj", assets=ASSETS), material="aluminum")

    extension_stage = model.part("extension_stage")
    extension_stage.visual(mesh_from_cadquery(_extension_rail_shape(), "extension_rail.obj", assets=ASSETS), material="anodized_dark")
    extension_stage.visual(mesh_from_cadquery(_inspection_head_shape(), "inspection_head.obj", assets=ASSETS), material="matte_black")
    extension_stage.visual(Cylinder(radius=0.013, length=0.018), origin=Origin(xyz=(0.242, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material="glass")
    extension_stage.visual(Cylinder(radius=0.006, length=0.005), origin=Origin(xyz=(0.229, 0.0, -0.018), rpy=(0.0, pi / 2.0, 0.0)), material="amber")

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=18.0, velocity=1.5),
    )
    model.articulation(
        "stage_extension",
        ArticulationType.PRISMATIC,
        parent=pivot_arm,
        child=extension_stage,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.14, effort=12.0, velocity=0.25),
    )

    return model
```
