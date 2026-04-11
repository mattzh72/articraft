---
title: 'A Parametric Bearing Pillow Block'
description: 'Combining a few basic functions, its possible to make a very good parametric bearing pillow block, with just a few lines of code.'
tags:
  - cadquery
  - examples
  - a
  - parametric
  - bearing
  - pillow
  - block
---
# A Parametric Bearing Pillow Block

Combining a few basic functions, its possible to make a very good parametric bearing pillow block, with just a few lines of code.

```python
(length, height, bearing_diam, thickness, padding) = (30.0, 40.0, 22.0, 10.0, 8.0)

result = (
    cq.Workplane("XY")
    .box(length, height, thickness)
    .faces(">Z")
    .workplane()
    .hole(bearing_diam)
    .faces(">Z")
    .workplane()
    .rect(length - padding, height - padding, forConstruction=True)
    .vertices()
    .cboreHole(2.4, 4.4, 2.1)
)
```
