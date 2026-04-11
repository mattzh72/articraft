---
title: 'Copying Workplanes'
description: 'An existing CQ object can copy a workplane from another CQ object.'
tags:
  - cadquery
  - examples
  - copying
  - workplanes
---
# Copying Workplanes

An existing CQ object can copy a workplane from another CQ object.

```python
result = (
    cq.Workplane("front")
    .circle(1)
    .extrude(10)  # make a cylinder
    # We want to make a second cylinder perpendicular to the first,
    # but we have no face to base the workplane off
    .copyWorkplane(
        # create a temporary object with the required workplane
        cq.Workplane("right", origin=(-5, 0, 0))
    )
    .circle(1)
    .extrude(10)
)
```
