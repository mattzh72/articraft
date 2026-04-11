---
title: 'Prismatic-revolute-revolute Chain'
description: 'Trimmed from the 5-star linear-dual-rotary module record; shows a slide feeding a shoulder and forearm.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - revolute
  - forearm
---
# Prismatic-revolute-revolute Chain

This is the actual mixed topology from the 5-star linear-dual-rotary module record: translate first, then pivot twice off the moving carriage.

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
    model = ArticulatedObject(name="linear_dual_rotary_module", assets=ASSETS)

    model.material("base_steel", rgba=(0.53, 0.56, 0.60, 1.0))
    model.material("carriage_dark", rgba=(0.20, 0.22, 0.26, 1.0))
    model.material("joint_blue", rgba=(0.24, 0.39, 0.65, 1.0))
    model.material("arm_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "module_base.obj", "base_steel")
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.12, 0.04)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    carriage = model.part("carriage")
    _add_mesh_visual(carriage, _make_carriage_shape(), "module_carriage.obj", "carriage_dark")

    shoulder = model.part("shoulder_link")
    _add_mesh_visual(shoulder, _make_shoulder_shape(), "module_shoulder.obj", "joint_blue")

    forearm = model.part("forearm_link")
    _add_mesh_visual(forearm, _make_forearm_shape(), "module_forearm.obj", "arm_silver")

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.08, 0.0, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.14, effort=250.0, velocity=0.25),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.052, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.112, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.1, effort=25.0, velocity=1.5),
    )

    return model
```
