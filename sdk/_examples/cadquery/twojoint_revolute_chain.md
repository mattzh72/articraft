---
title: 'Two-joint Revolute Chain'
description: 'Trimmed from the 5-star reading-lamp record; shows a realistic 2R chain with a base, lower arm, and upper assembly.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - lamp
  - serial
---
# Two-joint Revolute Chain

This excerpt keeps the real reading-lamp decomposition from the 5-star record instead of reducing the chain to anonymous blocks.

Both arm links extend along local `+X` from their pivots, so the pitch joints
use `axis=(0, -1, 0)` to make positive joint values raise the arms upward.

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
    model = ArticulatedObject(name="reading_lamp", assets=ASSETS)

    model.material("powder_black", rgba=(0.12, 0.12, 0.14, 1.0))
    model.material("warm_brass", rgba=(0.66, 0.56, 0.29, 1.0))
    model.material("shade_cream", rgba=(0.93, 0.91, 0.84, 1.0))

    base = model.part("base")
    base.visual(_mesh("lamp_base.obj", _build_base_shape()), material="powder_black")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(_mesh("lower_arm.obj", _build_lower_arm_shape()), material="warm_brass")
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.260, 0.024, 0.024)),
        mass=0.42,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(_mesh("upper_arm_frame.obj", _build_upper_arm_frame_shape()), material="warm_brass")
    upper_assembly.visual(_mesh("lamp_shade.obj", _build_shade_shape()), material="shade_cream")
    upper_assembly.inertial = Inertial.from_geometry(
        Box((0.305, 0.072, 0.060)),
        mass=0.48,
        origin=Origin(xyz=(0.168, 0.0, -0.018)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lower_arm",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        # Closed arm geometry extends along +X from the shoulder.
        # -Y makes positive q pitch the arm upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_assembly",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_assembly",
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        # Same sign convention: positive q raises the upper assembly.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=12.0,
            velocity=1.6,
        ),
    )

    return model
```
