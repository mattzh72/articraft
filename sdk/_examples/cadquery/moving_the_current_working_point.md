---
title: 'Moving The Current Working Point'
description: 'In this example, a closed profile is required, with some interior features as well.'
tags:
  - cadquery
  - examples
  - moving
  - the
  - current
  - working
  - point
---
# Moving The Current Working Point

In this example, a closed profile is required, with some interior features as well.

This example also demonstrates using multiple lines of code instead of longer chained commands, though of course in this case it was possible to do it in one long line as well.

A new work plane center can be established at any point.

```python
result = cq.Workplane("front").circle(
    3.0
)  # current point is the center of the circle, at (0, 0)
result = result.center(1.5, 0.0).rect(0.5, 0.5)  # new work center is (1.5, 0.0)

result = result.center(-1.5, 1.5).circle(0.25)  # new work center is (0.0, 1.5).
# The new center is specified relative to the previous center, not global coordinates!

result = result.extrude(0.25)
```
