---
title: 'Three-joint Revolute Chain'
description: 'Trimmed from the 5-star microphone-boom record; shows a base yaw, elbow pitch, and wrist pitch chain.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - boom
  - wrist
---
# Three-joint Revolute Chain

The 5-star microphone-boom record is a good reference for a realistic 3R chain because each stage has a clear role: base, lower arm, upper arm, and end tool.

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
    Sphere,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_mounted_microphone_boom", assets=ASSETS)

    model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("satin_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dark_polymer", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("mic_body", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("charcoal_grille", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_clamp.obj", assets=ASSETS), material="matte_black")
    base.inertial = Inertial.from_geometry(
        Box((0.070, 0.080, 0.110)),
        mass=1.6,
        origin=Origin(xyz=(-0.006, 0.0, -0.048)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(mesh_from_cadquery(_make_lower_arm_shape(), "lower_arm.obj", assets=ASSETS), material="satin_silver")

    upper_arm = model.part("upper_arm")
    upper_arm.visual(mesh_from_cadquery(_make_upper_arm_shape(), "upper_arm.obj", assets=ASSETS), material="satin_silver")

    microphone = model.part("microphone")
    microphone.visual(mesh_from_cadquery(_make_microphone_mount_shape(), "microphone_mount.obj", assets=ASSETS), material="dark_polymer")
    microphone.visual(
        Cylinder(radius=0.021, length=0.082),
        origin=Origin(xyz=(0.084, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="mic_body",
    )
    microphone.visual(
        Cylinder(radius=0.0185, length=0.028),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="charcoal_grille",
    )
    microphone.visual(Sphere(radius=0.0185), origin=Origin(xyz=(0.153, 0.0, 0.0)), material="charcoal_grille")

    model.articulation("base_yaw", ArticulationType.REVOLUTE, parent=base, child=lower_arm, origin=Origin(xyz=(0.0, 0.0, 0.0)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=12.0, velocity=2.0))
    model.articulation("elbow_pitch", ArticulationType.REVOLUTE, parent=lower_arm, child=upper_arm, origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)), axis=(0.0, -1.0, 0.0), motion_limits=MotionLimits(lower=-0.70, upper=1.15, effort=8.0, velocity=2.5))
    model.articulation("wrist_pitch", ArticulationType.REVOLUTE, parent=upper_arm, child=microphone, origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)), axis=(0.0, -1.0, 0.0), motion_limits=MotionLimits(lower=-1.00, upper=0.75, effort=4.0, velocity=3.0))

    return model
```
