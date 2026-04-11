---
title: 'Linear Carriage with Independent Rotary End-effector'
description: 'Trimmed from the 5-star rail-mounted spindle record; shows a prismatic carriage carrying its own rotary spindle.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - revolute
  - spindle
---
# Linear Carriage with Independent Rotary End-effector

This excerpt keeps the real split from the 5-star rail-mounted spindle record: the slide motion lives on the carriage, and the spindle gets its own independent rotary articulation.

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
    model = ArticulatedObject(name="linear_carriage_spindle", assets=ASSETS)

    model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("dark_polymer", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("tool_steel", rgba=(0.56, 0.57, 0.60, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(mesh_from_cadquery(_rail_shape(), "rail_base.obj", assets=ASSETS), material="anodized_aluminum")

    carriage = model.part("carriage")
    carriage.visual(mesh_from_cadquery(_carriage_shape(), "carriage.obj", assets=ASSETS), material="dark_polymer")

    spindle = model.part("spindle")
    spindle.visual(Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_FLANGE_LENGTH), origin=Origin(xyz=(0.0, SPINDLE_FLANGE_LENGTH / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material="tool_steel")
    spindle.visual(Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH), origin=Origin(xyz=(0.0, SPINDLE_FLANGE_LENGTH + (SPINDLE_BODY_LENGTH / 2.0), 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material="tool_steel")
    spindle.visual(Box((SPINDLE_SIDE_MODULE_X, SPINDLE_SIDE_MODULE_Y, SPINDLE_SIDE_MODULE_Z)), origin=Origin(xyz=(0.0, SPINDLE_FLANGE_LENGTH + 0.022, SPINDLE_FLANGE_RADIUS + 0.012)), material="dark_polymer")

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.14, upper=0.14, effort=150.0, velocity=0.5),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, SPINDLE_JOINT_Y, SPINDLE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.4, upper=1.4, effort=12.0, velocity=8.0),
    )

    return model
```
