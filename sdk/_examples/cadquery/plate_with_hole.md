---
title: 'Plate with Hole'
description: 'A rectangular box, but with a hole added.'
tags:
  - cadquery
  - examples
  - plate
  - with
  - hole
---
# Plate with Hole

A rectangular box, but with a hole added.

">Z" selects the top most face of the resulting box. The hole is located in the center because the default origin of a working plane is the projected origin of the last Workplane, the last Workplane having origin at (0,0,0) the projection is at the center of the face. The default hole depth is through the entire part.

```python
# The dimensions of the box. These can be modified rather than changing the
# object's cad directly.
length = 80.0
height = 60.0
thickness = 10.0
center_hole_dia = 22.0

# Create a box based on the dimensions above and add a 22mm center hole
result = (
    cq.Workplane("XY")
    .box(length, height, thickness)
    .faces(">Z")
    .workplane()
    .hole(center_hole_dia)
)
```
