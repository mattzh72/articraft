---
title: 'Making Counter-bored and Counter-sunk Holes'
description: 'Counterbored and countersunk holes are so common that CadQuery creates macros to create them in a single step.'
tags:
  - cadquery
  - examples
  - making
  - counter
  - bored
  - and
  - sunk
  - holes
---
# Making Counter-bored and Counter-sunk Holes

Counterbored and countersunk holes are so common that CadQuery creates macros to create them in a single step.

Similar to `Workplane.hole()`, these functions operate on a list of points as well as a single point.

```python
result = (
    cq.Workplane(cq.Plane.XY())
    .box(4, 2, 0.5)
    .faces(">Z")
    .workplane()
    .rect(3.5, 1.5, forConstruction=True)
    .vertices()
    .cboreHole(0.125, 0.25, 0.125, depth=None)
)
```
