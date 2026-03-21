---
title: 'BGA Package'
description: 'A generic ball grid array package with an optional spherical index mark cut into the top face.'
tags:
  - cadquery
  - examples
  - bga
  - package
  - smd
---
# BGA Package

This example mirrors the `cq-electronics` generic BGA package model.

```python
import cadquery as cq

length = 20
width = 20
height = 1
simple = False

result = cq.Workplane("XY").box(length, width, height)

if not simple:
    index_mark_radius = 2
    index_mark_loc_x = -((length / 2) - 1)
    index_mark_loc_y = -((width / 2) - 1)
    index_mark_elevation = (height / 2) + (index_mark_radius * 0.93)

    index_mark = cq.Workplane(
        origin=(
            index_mark_loc_x,
            index_mark_loc_y,
            index_mark_elevation,
        )
    ).sphere(index_mark_radius)

    result = result.cut(index_mark)
```
