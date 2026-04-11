---
title: 'Coaxial Rotary Stack'
description: 'Trimmed from the 5-star nested-turntable record; shows two revolute stages sharing one centerline.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - coaxial
  - turntable
---
# Coaxial Rotary Stack

This is the core layout from the 5-star nested-turntable record: grounded pedestal, slew carrier, and top platter, all rotating about the same vertical axis.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_industrial_turntable", assets=ASSETS)

    model.material("machine_base", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("bearing_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("tooling_orange", rgba=(0.83, 0.39, 0.10, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(mesh_from_cadquery(_build_pedestal_shape(), "pedestal_base.obj", assets=ASSETS), material="machine_base")

    slew_carrier = model.part("slew_carrier")
    slew_carrier.visual(mesh_from_cadquery(_build_slew_shape(), "slew_carrier.obj", assets=ASSETS), material="bearing_steel")

    top_platter = model.part("top_platter")
    top_platter.visual(mesh_from_cadquery(_build_platter_shape(), "top_platter.obj", assets=ASSETS), material="tooling_orange")

    model.articulation(
        "base_to_slew",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=slew_carrier,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-PRIMARY_AXIS_LIMIT, upper=PRIMARY_AXIS_LIMIT, effort=80.0, velocity=1.2),
    )
    model.articulation(
        "slew_to_platter",
        ArticulationType.REVOLUTE,
        parent=slew_carrier,
        child=top_platter,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-SECONDARY_AXIS_LIMIT, upper=SECONDARY_AXIS_LIMIT, effort=45.0, velocity=2.0),
    )

    return model
```
