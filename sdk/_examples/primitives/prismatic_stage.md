---
title: 'Primitive Prismatic Stage'
description: 'A box rail and sliding carriage with a bounded prismatic joint.'
tags:
  - primitives
  - articulation
  - prismatic
  - slider
  - carriage
---
# Primitive Prismatic Stage

```python
from sdk_primitives import ArticulatedObject, ArticulationType, Box, MotionLimits, Origin

model = ArticulatedObject(name="primitive_stage")
rail = model.part("rail")
rail.visual(Box((0.8, 0.12, 0.08)), material=model.material("rail_gray", rgba=(0.35, 0.37, 0.4, 1.0)))
carriage = model.part("carriage")
carriage.visual(Box((0.2, 0.22, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.07)), material="rail_gray")
model.articulation(
    "rail_to_carriage",
    ArticulationType.PRISMATIC,
    parent=rail,
    child=carriage,
    axis=(1.0, 0.0, 0.0),
    motion_limits=MotionLimits(lower=-0.25, upper=0.25, effort=200.0, velocity=0.3),
)
```
