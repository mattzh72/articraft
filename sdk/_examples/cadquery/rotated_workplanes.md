---
title: 'Rotated Workplanes'
description: 'You can create a rotated work plane by specifying angles of rotation relative to another workplane'
tags:
  - cadquery
  - examples
  - rotated
  - workplanes
---
# Rotated Workplanes

You can create a rotated work plane by specifying angles of rotation relative to another workplane. This variant is authored directly in meters.

```python
import cadquery as cq

from sdk import mesh_from_cadquery

result = (
    cq.Workplane("front")
    .box(0.100, 0.100, 0.006)
    .faces(">Z")
    .workplane()
    .transformed(offset=cq.Vector(0.0, -0.038, 0.025), rotate=cq.Vector(60, 0, 0))
    .rect(0.038, 0.038, forConstruction=True)
    .vertices()
    .hole(0.006)
)

mesh = mesh_from_cadquery(result, "rotated_workplane_plate")
```
