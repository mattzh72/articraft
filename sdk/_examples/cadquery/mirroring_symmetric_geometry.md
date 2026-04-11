---
title: 'Mirroring Symmetric Geometry'
description: 'You can mirror 2D geometry when your shape is symmetric. In this example we also introduce horizontal and vertical lines, which make for slightly easier coding.'
tags:
  - cadquery
  - examples
  - mirroring
  - symmetric
  - geometry
---
# Mirroring Symmetric Geometry

You can mirror 2D geometry when your shape is symmetric. In this example we also introduce horizontal and vertical lines, which make for slightly easier coding.

```python
r = cq.Workplane("front").hLine(1.0)  # 1.0 is the distance, not coordinate
r = (
    r.vLine(0.5).hLine(-0.25).vLine(-0.25).hLineTo(0.0)
)  # hLineTo allows using xCoordinate not distance
result = r.mirrorY().extrude(0.25)  # mirror the geometry and extrude
```
