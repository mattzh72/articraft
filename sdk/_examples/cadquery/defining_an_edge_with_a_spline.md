---
title: 'Defining an Edge with a Spline'
description: 'This example defines a side using a spline curve through a collection of points. Useful when you have an edge that needs a complex profile'
tags:
  - cadquery
  - examples
  - defining
  - an
  - edge
  - with
  - a
  - spline
---
# Defining an Edge with a Spline

This example defines a side using a spline curve through a collection of points. Useful when you have an edge that needs a complex profile

```python
s = cq.Workplane("XY")
sPnts = [
    (2.75, 1.5),
    (2.5, 1.75),
    (2.0, 1.5),
    (1.5, 1.0),
    (1.0, 1.25),
    (0.5, 1.0),
    (0, 1.0),
]
r = s.lineTo(3.0, 0).lineTo(3.0, 1.0).spline(sPnts, includeCurrent=True).close()
result = r.extrude(0.5)
```
