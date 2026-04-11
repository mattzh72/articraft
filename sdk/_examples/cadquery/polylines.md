---
title: 'Polylines'
description: '`Workplane.polyline()` allows creating a shape from a large number of chained points connected by lines.'
tags:
  - cadquery
  - examples
  - polylines
---
# Polylines

`Workplane.polyline()` allows creating a shape from a large number of chained points connected by lines.

This example uses a polyline to create one half of an i-beam shape, which is mirrored to create the final profile.

```python
(L, H, W, t) = (100.0, 20.0, 20.0, 1.0)
pts = [
    (0, H / 2.0),
    (W / 2.0, H / 2.0),
    (W / 2.0, (H / 2.0 - t)),
    (t / 2.0, (H / 2.0 - t)),
    (t / 2.0, (t - H / 2.0)),
    (W / 2.0, (t - H / 2.0)),
    (W / 2.0, H / -2.0),
    (0, H / -2.0),
]
result = cq.Workplane("front").polyline(pts).mirrorY().extrude(L)
```
