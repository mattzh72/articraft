---
title: 'Primitive Revolute Arm'
description: 'A rooted base, upright, and rotating arm made only from boxes and cylinders.'
tags:
  - primitives
  - articulation
  - revolute
  - arm
---
# Primitive Revolute Arm

```python
from sdk_primitives import ArticulatedObject, ArticulationType, Box, Cylinder, MotionLimits, Origin

model = ArticulatedObject(name="primitive_arm")
model.material("steel", rgba=(0.25, 0.28, 0.32, 1.0))
base = model.part("base")
base.visual(Box((0.4, 0.3, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material="steel")
arm = model.part("arm")
arm.visual(Box((0.5, 0.08, 0.08)), origin=Origin(xyz=(0.25, 0.0, 0.0)), material="steel")
arm.visual(Cylinder(radius=0.06, length=0.1), origin=Origin(rpy=(1.5708, 0.0, 0.0)), material="steel")
model.articulation(
    "base_to_arm",
    ArticulationType.REVOLUTE,
    parent=base,
    child=arm,
    origin=Origin(xyz=(0.0, 0.0, 0.3)),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=25.0, velocity=1.0),
)
```
