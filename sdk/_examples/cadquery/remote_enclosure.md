---
title: 'Remote Enclosure'
description: 'A compact enclosure with a shelled top, countersunk button holes, and a mating cover.'
tags:
  - cadquery
  - examples
  - remote
  - enclosure
---
# Remote Enclosure

```python
show_top = True
show_cover = True

width = 2.2
height = 0.5
length = 1.5
trapezoid_fudge = 1.5
x_hole_offset = 0.500
y_hole_offset = 0.500
z_fillet_radius = 0.50
y_fillet_radius = 0.250
lip_height = 0.1
wall_thickness = 0.06
cover_thickness = 0.2
hole_radius = 0.30
countersink_angle = 100

xyplane = cq.Workplane("XY")


def trapezoid(b1, b2, h):
    y = h / 2
    x1 = b1 / 2
    x2 = b2 / 2
    return (
        xyplane.moveTo(-x1, y)
        .polyline([(x1, y), (x2, -y), (-x2, -y)])
        .close()
    )


def base(h):
    return (
        trapezoid(width, width * trapezoid_fudge, length)
        .extrude(h)
        .translate((0, 0, height / 2))
        .edges("Z")
        .fillet(z_fillet_radius)
    )


top = (
    base(height)
    .edges(">Z")
    .fillet(y_fillet_radius)
    .faces("<Z")
    .shell(-wall_thickness)
    .faces(">Z")
    .workplane(centerOption="CenterOfMass")
    .pushPoints(
        [
            (0, 0),
            (-x_hole_offset, 0),
            (0, -y_hole_offset),
            (x_hole_offset, 0),
            (0, y_hole_offset),
        ]
    )
    .cskHole(
        diameter=hole_radius,
        cskDiameter=hole_radius * 1.5,
        cskAngle=countersink_angle,
    )
)

cover = (
    base(cover_thickness)
    .translate((0, 0, -cover_thickness + lip_height))
    .cut(top)
    .edges("#Z")
    .fillet(0.020)
)

if show_top and show_cover:
    result = cq.Compound.makeCompound([top.val(), cover.val()])
elif show_top:
    result = top
else:
    result = cover
```
