---
title: 'PiTray Clip'
description: 'A sheet-metal style bracket assembly that combines a right-angle tray bracket with a DIN rail clip.'
tags:
  - cadquery
  - examples
  - pitray
  - clip
  - bracket
  - assembly
  - din
---
# PiTray Clip

This example mirrors the `cq-electronics` PiTray clip assembly model.

```python
import cadquery as cq

colors = {
    "black_plastic": (5 / 255, 5 / 255, 5 / 255),
    "stainless_steel": (243 / 255, 243 / 255, 243 / 255),
}

din_top_hat_width = 35
m_countersink_angle = 90
m2r5_tap_hole_diameter = 2.15
m4_clearance_normal_diameter = 4.5
m4_countersink_diameter = 9.4
m4_tap_hole_diameter = 3.2

din_clip_length = 76
din_clip_width = 20
din_clip_height = 8
din_clip_between_mount_holes = 63
din_clip_rail_aperture_depth = 4
din_clip_corner_chamfer = 3

rpi_hole_centers_long = 58


def make_din_clip():
    half_length = din_clip_length / 2
    rail_aperture_offset = half_length - 30
    half_between_mount_holes = din_clip_between_mount_holes / 2
    rail_aperture_center = (-rail_aperture_offset, 0)

    outer_mount_hole_centers = [
        (half_between_mount_holes, 0),
        (-half_between_mount_holes, 0),
    ]

    return (
        cq.Workplane()
        .box(din_clip_length, din_clip_width, din_clip_height)
        .faces("<<Z")
        .workplane()
        .tag("workplane__rail_face")
        .pushPoints([rail_aperture_center])
        .rect(din_top_hat_width, din_clip_width)
        .cutBlind(-din_clip_rail_aperture_depth)
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
        .chamfer(din_clip_corner_chamfer)
    )


length = 76
width = 20
height = 15
thickness = 1.4

pcb_screw_cylinder_radius = 4.1 / 2
pcb_screw_cylinder_from_edge = 3.7
pcb_screw_cylinder_length = 5.5

clip_screw_cylinder_radius = 7.95 / 2
clip_screw_cylinder_length = 4

corner_fillet_radius = 3

pcb_screw_center = (
    -rpi_hole_centers_long / 2,
    height / 2 - pcb_screw_cylinder_from_edge,
)

pcb_mount_hole_centers = [
    pcb_screw_center,
    (-pcb_screw_center[0], pcb_screw_center[1]),
]

clip_mount_hole_center = (-din_clip_between_mount_holes / 2, 0)
clip_mount_hole_centers = [
    clip_mount_hole_center,
    (-clip_mount_hole_center[0], clip_mount_hole_center[1]),
]

cutout_origin = (-width / 2, -height / 2 + thickness)

angle = (
    cq.Workplane("ZY")
    .box(height, width, length)
    .faces(">X")
    .workplane()
    .pushPoints([cutout_origin])
    .rect(
        width - thickness,
        height - thickness,
        centered=False,
    )
    .cutThruAll()
)

bracket = (
    angle.faces(">>Y")
    .workplane(centerOption="CenterOfBoundBox")
    .tag("workplane__pcb_face")
    .workplane(offset=-pcb_screw_cylinder_length / 2)
    .pushPoints(pcb_mount_hole_centers)
    .cylinder(pcb_screw_cylinder_length, pcb_screw_cylinder_radius)
    .workplaneFromTagged("workplane__pcb_face")
    .pushPoints(pcb_mount_hole_centers)
    .hole(m2r5_tap_hole_diameter)
    .faces("<<Z")
    .workplane(centerOption="CenterOfBoundBox")
    .tag("workplane__clip_face")
    .workplane(offset=-clip_screw_cylinder_length / 2)
    .pushPoints(clip_mount_hole_centers)
    .cylinder(clip_screw_cylinder_length, clip_screw_cylinder_radius)
    .workplaneFromTagged("workplane__clip_face")
    .pushPoints(clip_mount_hole_centers)
    .hole(m4_tap_hole_diameter)
)

selector = {
    "edge_1": "(>>X and >>Z)",
    "edge_2": "(>>X and <<Y)",
    "edge_3": "(<<X and >>Z)",
    "edge_4": "(<<X and <<Y)",
}
bracket = bracket.edges(" or ".join(selector.values())).fillet(corner_fillet_radius)

din_clip = make_din_clip()
din_clip_elevation = -(height / 2 + din_clip_height / 2)

result = cq.Assembly()
result.add(
    bracket,
    name="bracket",
    color=cq.Color(*colors["stainless_steel"]),
)
result.add(
    din_clip,
    name="din_clip",
    color=cq.Color(*colors["black_plastic"]),
    loc=cq.Location(cq.Vector(0, 0, din_clip_elevation)),
)
```
