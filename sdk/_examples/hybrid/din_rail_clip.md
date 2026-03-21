---
title: 'DIN Rail Clip'
description: 'A plastic DIN rail clip that uses tagged workplanes, countersunk holes, and selector-based chamfers.'
tags:
  - cadquery
  - examples
  - din
  - rail
  - clip
  - cskHole
  - workplaneFromTagged
---
# DIN Rail Clip

This example mirrors the `cq-electronics` DIN rail clip model.

```python
import cadquery as cq

top_hat_width = 35

m_countersink_angle = 90
m4_clearance_normal_diameter = 4.5
m4_countersink_diameter = 9.4

length = 76
width = 20
height = 8
between_mount_holes = 63
rail_aperture_depth = 4
corner_chamfer = 3

half_length = length / 2
rail_aperture_offset = half_length - 30
half_between_mount_holes = between_mount_holes / 2
rail_aperture_center = (-rail_aperture_offset, 0)

outer_mount_hole_centers = [
    (half_between_mount_holes, 0),
    (-half_between_mount_holes, 0),
]

result = (
    cq.Workplane()
    .box(length, width, height)
    .faces("<<Z")
    .workplane()
    .tag("workplane__rail_face")
    .pushPoints([rail_aperture_center])
    .rect(top_hat_width, width)
    .cutBlind(-rail_aperture_depth)
    .workplaneFromTagged("workplane__rail_face")
    .pushPoints(outer_mount_hole_centers)
    .cskHole(
        m4_clearance_normal_diameter,
        m4_countersink_diameter,
        m_countersink_angle,
    )
    .faces(">Z[1]")
    .workplane(centerOption="CenterOfBoundBox")
    .tag("workplane__rail_aperture_face")
    .cskHole(
        m4_clearance_normal_diameter,
        m4_countersink_diameter,
        m_countersink_angle,
    )
    .edges("|Z and (>X or <X)")
    .chamfer(corner_chamfer)
)
```
