---
title: 'Primitive Branching Joint Tree'
description: 'A central body with two independently hinged side panels in a valid URDF tree.'
tags:
  - primitives
  - articulation
  - branching
  - hinge
  - panels
---
# Primitive Branching Joint Tree

```python
from sdk_primitives import ArticulatedObject, ArticulationType, Box, MotionLimits, Origin

model = ArticulatedObject(name="branching_panels")
body = model.part("body")
body.visual(Box((0.3, 0.3, 0.25)), material=model.material("body", rgba=(0.3, 0.3, 0.35, 1.0)))
for side, y, sign in (("left", 0.18, 1.0), ("right", -0.18, -1.0)):
    panel = model.part(f"{side}_panel")
    panel.visual(Box((0.45, 0.02, 0.2)), origin=Origin(xyz=(0.225, sign * 0.01, 0.0)), material="body")
    model.articulation(
        f"body_to_{side}_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.0, y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57, effort=5.0, velocity=1.0),
    )
```
