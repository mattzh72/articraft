---
title: 'Inside Chamfer on a Shelled Cube'
description: 'Use logical selector operators to chamfer only the interior edges of a shelled cube.'
tags:
  - cadquery
  - examples
  - inside
  - chamfer
  - shelled
  - cube
---
# Inside Chamfer on a Shelled Cube

```python
result = (
    cq.Workplane("XY")
    .box(2, 2, 2)
    .faces(">Z")
    .shell(-0.2)
    .faces(">Z")
    .edges("not(<X or >X or <Y or >Y)")
    .chamfer(0.125)
)
```
