---
title: 'Single Yaw Axis Module'
description: 'Trimmed from the 5-star pan-turntable record; shows the base, turntable plate, payload pedestal, and fixed payload mount.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - yaw
  - turntable
---
# Single Yaw Axis Module

The 5-star pan example is slightly richer than a bare rotating puck: it uses a revolute turntable plus a fixed pedestal to stage the payload above the axis.

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
    model = ArticulatedObject(name="pan_turntable_sensor_mount", assets=ASSETS)
    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("pedestal_gray", rgba=(0.58, 0.60, 0.64, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_shape(), "turntable_base.obj", "powder_black")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.030),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    turntable = model.part("turntable_plate")
    _add_mesh_visual(turntable, _build_turntable_shape(), "turntable_plate.obj", "machined_aluminum")
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.011),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    pedestal = model.part("payload_pedestal")
    _add_mesh_visual(pedestal, _build_pedestal_shape(), "payload_pedestal.obj", "pedestal_gray")
    pedestal.inertial = Inertial.from_geometry(
        Box((0.075, 0.055, 0.089)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.967, upper=2.967, effort=12.0, velocity=2.5),
    )
    model.articulation(
        "turntable_to_pedestal",
        ArticulationType.FIXED,
        parent=turntable,
        child=pedestal,
        origin=Origin(xyz=(0.035, 0.0, 0.008)),
    )

    return model
```
