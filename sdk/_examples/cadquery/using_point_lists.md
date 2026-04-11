---
title: 'Using Point Lists'
description: 'Sometimes you need to create a number of features at various locations, and using `Workplane.center()` is too cumbersome.'
tags:
  - cadquery
  - examples
  - using
  - point
  - lists
---
# Using Point Lists

Sometimes you need to create a number of features at various locations, and using `Workplane.center()` is too cumbersome.

You can use a list of points to construct multiple objects at once. Most construction methods, like `Workplane.circle()` and `Workplane.rect()`, will operate on multiple points if they are on the stack

```python
r = cq.Workplane("front").circle(2.0)  # make base
r = r.pushPoints(
    [(1.5, 0), (0, 1.5), (-1.5, 0), (0, -1.5)]
)  # now four points are on the stack
r = r.circle(0.25)  # circle will operate on all four points
result = r.extrude(0.125)  # make prism
```
