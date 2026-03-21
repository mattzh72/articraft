---
title: 'Raspberry Pi 3 Model B Assembly'
description: 'A constrained PCB assembly that combines a board, solder mask layers, a pin header, an RJ45 jack, and BGA packages.'
tags:
  - cadquery
  - examples
  - raspberry
  - pi
  - pcb
  - assembly
  - constrain
---
# Raspberry Pi 3 Model B Assembly

This example mirrors the `cq-electronics` Raspberry Pi 3 Model B assembly model.

```python
import cadquery as cq

colors = {
    "black_plastic": (5 / 255, 5 / 255, 5 / 255),
    "gold_plate": (255 / 255, 173 / 255, 0 / 255),
    "package_black": (13 / 255, 13 / 255, 13 / 255),
    "pcb_substrate_chiffon": (218 / 255, 207 / 255, 132 / 255),
    "solder_mask_green": (0 / 255, 155 / 255, 0 / 255),
    "tin_plate": (135 / 255, 155 / 255, 155 / 255),
}


def make_pin_header(rows=1, columns=1, above=7, below=3, *, simple=True):
    pitch = 2.54
    pin_width = 0.64
    pin_chamfer = 0.2
    base_height = 2.4

    pin_length = above + base_height + below
    base_width = pitch * rows
    base_length = pitch * columns

    pin_points = []
    for row in range(rows):
        loc_y = (pitch / 2) + (row * pitch)
        for column in range(columns):
            loc_x = (pitch / 2) + (column * pitch)
            pin_points.append((loc_x, loc_y))

    base = (
        cq.Workplane()
        .box(base_length, base_width, base_height, centered=False)
        .faces(">Z")
        .workplane()
        .pushPoints(pin_points)
        .rect(pin_width, pin_width)
        .cutThruAll()
    )

    pin = cq.Workplane().box(pin_width, pin_width, pin_length)

    if not simple:
        pin = pin.edges("<Z").chamfer(pin_chamfer).edges(">Z").chamfer(pin_chamfer)

    pin_elevation = (pin_length / 2) - below

    header = cq.Assembly(color=cq.Color(*colors["gold_plate"]))
    header.add(
        base,
        name="base",
        color=cq.Color(*colors["black_plastic"]),
    )

    for row in range(rows):
        loc_y = (pitch / 2) + (row * pitch)
        for column in range(columns):
            loc_x = (pitch / 2) + (column * pitch)
            location = cq.Location(cq.Vector(loc_x, loc_y, pin_elevation))
            header.add(
                pin,
                name=f"pin_{row}-{column}",
                loc=location,
            )

    return header


def make_rj45_surface_mount_jack(length=21, *, simple=True):
    width = 16
    height = 14

    aperture_width = 11.68
    aperture_height = 7.75
    aperture_depth = -15
    keyway_width = 6
    keyway_height = 1.5
    retainer_width = 3.25
    retainer_height = 1.5
    retainer_depth = 2

    keyway_elevation = -(aperture_height / 2) - (keyway_height / 2)
    retainer_elevation = keyway_elevation - (retainer_height / 2)

    jack = cq.Workplane("XY").box(length, width, height)

    if not simple:
        jack = (
            jack.faces(">X")
            .workplane()
            .tag("aperture")
            .rect(aperture_width, aperture_height)
            .cutBlind(aperture_depth)
            .workplaneFromTagged("aperture")
            .move(0, keyway_elevation)
            .rect(keyway_width, keyway_height)
            .cutBlind(aperture_depth)
            .workplaneFromTagged("aperture")
            .move(0, retainer_elevation)
            .rect(retainer_width, retainer_height)
            .cutBlind(aperture_depth)
            .faces(">X")
            .workplane(offset=-retainer_depth)
            .move(0, retainer_elevation)
            .rect(keyway_width, keyway_height)
            .cutBlind(aperture_depth + retainer_depth)
        )

    return jack


def make_bga(length, width, height=1, *, simple=True):
    package = cq.Workplane("XY").box(length, width, height)

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

        package = package.cut(index_mark)

    return package


height = 56
width = 85
thickness = 1.5
solder_mask_thickness = 0.05

corner_radius = 3

hole_diameter = 2.7
hole_solder_mask_diameter = 6
hole_offset_from_edge = 3.5
hole_centers_long = 58

simple = False

offset = -width / 2 + hole_offset_from_edge + hole_centers_long
hole_points = [
    (
        height / 2 - hole_offset_from_edge,
        offset,
    ),
    (
        -(height / 2 - hole_offset_from_edge),
        -(width / 2 - hole_offset_from_edge),
    ),
    (
        -(height / 2 - hole_offset_from_edge),
        offset,
    ),
    (
        height / 2 - hole_offset_from_edge,
        -(width / 2 - hole_offset_from_edge),
    ),
]

substrate = (
    cq.Workplane("XY")
    .box(height, width, thickness)
    .faces(">Z")
    .workplane(centerOption="CenterOfMass")
    .rect(
        height - (hole_offset_from_edge * 2),
        width - (hole_offset_from_edge * 2),
        forConstruction=True,
    )
    .vertices()
    .pushPoints(hole_points)
    .hole(hole_diameter)
    .tag("holes")
    .edges("|Z")
    .fillet(corner_radius)
)

substrate.edges("%CIRCLE", tag="holes").edges("<X and <Y and >Z").tag("hole_upper")
substrate.edges("%CIRCLE", tag="holes").edges("<X and <Y and <Z").tag("hole_lower")

solder_mask = (
    cq.Workplane("XY")
    .box(height, width, solder_mask_thickness)
    .faces(">Z")
    .workplane(centerOption="CenterOfMass")
    .pushPoints(hole_points)
    .hole(hole_solder_mask_diameter)
    .tag("holes")
    .edges("|Z")
    .fillet(corner_radius)
)

solder_mask.edges("%CIRCLE", tag="holes").edges("<X and <Y and >Z").tag("hole_upper")
solder_mask.edges("%CIRCLE", tag="holes").edges("<X and <Y and <Z").tag("hole_lower")

ethernet_port = make_rj45_surface_mount_jack(simple=simple)
ethernet_port.faces("<Z").tag("board_side")
ethernet_port.faces(">X").tag("aperture_side")

gpio = make_pin_header(rows=2, columns=20, simple=simple)

bcm2837 = make_bga(14, 14, simple=simple)
bcm2837.faces("<Z").tag("board_side")

usb_controller = make_bga(9, 9, simple=simple)
usb_controller.faces("<Z").tag("board_side")

ram = make_bga(9, 9, simple=simple)
ram.faces("<Z").tag("board_side")

result = (
    cq.Assembly()
    .add(
        substrate,
        name="pcb_substrate",
        color=cq.Color(*colors["pcb_substrate_chiffon"]),
    )
    .add(
        solder_mask,
        name="pcb_solder_mask_top",
        color=cq.Color(*colors["solder_mask_green"]),
    )
    .add(
        solder_mask,
        name="pcb_solder_mask_bottom",
        color=cq.Color(*colors["solder_mask_green"]),
    )
    .add(
        ethernet_port,
        name="ethernet_port",
        color=cq.Color(*colors["tin_plate"]),
    )
    .add(
        bcm2837,
        name="bcm2837",
        color=cq.Color(*colors["package_black"]),
    )
    .add(
        usb_controller,
        name="usb_controller",
        color=cq.Color(*colors["package_black"]),
    )
    .add(
        ram,
        name="ram",
        color=cq.Color(*colors["package_black"]),
    )
    .add(
        gpio,
        name="gpio",
        loc=cq.Location(
            cq.Vector(-27, 15.5, (thickness + solder_mask_thickness) / 2),
            cq.Vector(0, 0, 1),
            -90,
        ),
    )
)

result = (
    result.constrain("pcb_substrate", "Fixed")
    .constrain("pcb_solder_mask_top", "FixedRotation", (0, 0, 0))
    .constrain("pcb_solder_mask_bottom", "FixedRotation", (0, 0, 0))
    .constrain("pcb_substrate?hole_upper", "pcb_solder_mask_top?hole_lower", "Point")
    .constrain("pcb_substrate?hole_lower", "pcb_solder_mask_bottom?hole_upper", "Point")
    .constrain("ethernet_port", "FixedRotation", (0, 0, 90))
    .constrain(
        "ethernet_port?board_side",
        "pcb_solder_mask_top@faces@>Z",
        "PointInPlane",
    )
    .constrain(
        "ethernet_port@faces@>X",
        "pcb_solder_mask_top@faces@>Y",
        "PointInPlane",
        param=2.5,
    )
    .constrain(
        "ethernet_port@faces@<Y",
        "pcb_solder_mask_top@faces@>X",
        "PointInPlane",
        param=-2,
    )
    .constrain("bcm2837", "FixedRotation", (0, 0, 0))
    .constrain("bcm2837", "pcb_solder_mask_top@faces@>Z", "PointInPlane")
    .constrain(
        "bcm2837@faces@<Y",
        "pcb_solder_mask_top@faces@<Y",
        "PointInPlane",
        param=-20,
    )
    .constrain(
        "bcm2837@faces@>X",
        "pcb_solder_mask_top@faces@>X",
        "PointInPlane",
        param=-20,
    )
    .constrain("usb_controller", "FixedRotation", (0, 0, 0))
    .constrain("usb_controller", "pcb_solder_mask_top@faces@>Z", "PointInPlane")
    .constrain(
        "usb_controller@faces@<Y",
        "pcb_solder_mask_top@faces@<Y",
        "PointInPlane",
        param=-53,
    )
    .constrain(
        "usb_controller@faces@>X",
        "pcb_solder_mask_top@faces@>X",
        "PointInPlane",
        param=-28,
    )
    .constrain("ram", "FixedRotation", (0, 180, 0))
    .constrain("ram", "pcb_solder_mask_bottom@faces@<Z", "PointInPlane")
    .constrain(
        "ram@faces@<Y",
        "pcb_solder_mask_top@faces@<Y",
        "PointInPlane",
        param=-37,
    )
    .constrain(
        "ram@faces@>X",
        "pcb_solder_mask_top@faces@<X",
        "PointInPlane",
        param=-17,
    )
)

result.solve()
```
