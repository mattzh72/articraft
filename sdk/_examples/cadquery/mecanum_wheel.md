---
title: 'Mecanum Wheel'
description: 'Build a mecanum wheel assembly with a flanged hub, repeated mounting lugs, and lofted rollers tilted around the rim.'
tags:
  - cadquery
  - examples
  - mecanum
  - mercanum
  - wheel
  - roller
  - assembly
  - loft
---
# Mecanum Wheel

This example derives the critical CadQuery geometry from a generated mecanum wheel model: the lofted roller tread, the annular hub and flanges, the repeated spoke-and-lug structure, and the tilted roller layout around the rim.

```python
import math

import cadquery as cq

num_rollers = 11

hub_thickness = 0.056
hub_core_radius = 0.028
hub_drum_radius = 0.042
hub_drum_thickness = 0.038
hub_flange_inner_radius = 0.037
hub_flange_outer_radius = 0.062
hub_flange_thickness = 0.007
hub_flange_offset = 0.0185
hub_bore_radius = 0.0135
hub_spoke_length = 0.038
hub_spoke_width = 0.010
hub_spoke_height = 0.036
hub_spoke_center_radius = 0.039
num_bolts = 6
bolt_circle_radius = 0.025
bolt_head_radius = 0.0032
bolt_head_length = 0.004
bolt_head_z = 0.024

roller_tilt_deg = 45.0
roller_center_radius = 0.079
roller_length = 0.056
roller_groove_width = 0.010
roller_end_radius = 0.0105
roller_crown_radius = 0.018
roller_shaft_radius = 0.0036
roller_bush_radius = 0.0068
roller_bush_length = 0.008
roller_endcap_radius = 0.0095
roller_endcap_length = 0.005
roller_endcap_offset = 0.0255

mount_lug_radius = 0.015
mount_lug_length = 0.010
mount_arm_length = 0.026
mount_arm_width = 0.016
mount_hole_radius = 0.0044

colors = {
    "gunmetal": (0.28, 0.31, 0.35),
    "rubber": (0.07, 0.07, 0.08),
    "steel": (0.64, 0.66, 0.70),
    "accent": (0.82, 0.46, 0.16),
}


def _normalize(vector):
    x, y, z = vector
    length = math.sqrt((x * x) + (y * y) + (z * z))
    return (x / length, y / length, z / length)


def _axis_to_rpy(axis):
    ax, ay, az = _normalize(axis)
    yaw = math.atan2(ay, ax)
    pitch = math.atan2(math.sqrt((ax * ax) + (ay * ay)), az)
    return (0.0, pitch, yaw)


def _translate_along(axis, distance):
    ax, ay, az = _normalize(axis)
    return (ax * distance, ay * distance, az * distance)


def _polar(radius, angle):
    return (radius * math.cos(angle), radius * math.sin(angle), 0.0)


def _roller_center(index):
    theta = (2.0 * math.pi * index) / num_rollers
    return _polar(roller_center_radius, theta)


def _roller_axis(index):
    theta = (2.0 * math.pi * index) / num_rollers
    tangent = (-math.sin(theta), math.cos(theta), 0.0)
    tilt = math.radians(roller_tilt_deg)
    return _normalize(
        (
            tangent[0] * math.sin(tilt),
            tangent[1] * math.sin(tilt),
            math.cos(tilt),
        )
    )


def _build_disc(radius, thickness, z_center):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - (thickness / 2.0)))
        .circle(radius)
        .extrude(thickness)
    )


def _build_annulus(outer_radius, inner_radius, thickness, z_center):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - (thickness / 2.0)))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _build_mount_lug():
    lug = (
        cq.Workplane("XY", origin=(0.0, 0.0, -(mount_lug_length / 2.0)))
        .circle(mount_lug_radius)
        .extrude(mount_lug_length)
    )
    arm = (
        cq.Workplane("XY")
        .box(mount_arm_length, mount_arm_width, mount_lug_length)
        .translate((-(mount_arm_length / 2.0), 0.0, 0.0))
    )
    gusset = (
        cq.Workplane("XY")
        .box(mount_arm_length * 0.65, mount_arm_width * 0.75, mount_lug_length * 1.5)
        .translate((-(mount_arm_length * 0.55), 0.0, 0.0))
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, -((mount_lug_length + 0.004) / 2.0)))
        .circle(mount_hole_radius)
        .extrude(mount_lug_length + 0.004)
    )
    return lug.union(arm).union(gusset).cut(hole)


def _build_roller_tread_shape():
    half_span = (roller_length - roller_groove_width) / 2.0
    left = (
        cq.Workplane("XY", origin=(0.0, 0.0, -(roller_length / 2.0)))
        .circle(roller_end_radius)
        .workplane(offset=half_span)
        .circle(roller_crown_radius)
        .loft(combine=True, ruled=False)
    )
    right = (
        cq.Workplane("XY", origin=(0.0, 0.0, roller_groove_width / 2.0))
        .circle(roller_crown_radius)
        .workplane(offset=half_span)
        .circle(roller_end_radius)
        .loft(combine=True, ruled=False)
    )
    return left.union(right)


def _build_hub_shape():
    hub = _build_disc(hub_drum_radius, hub_drum_thickness, 0.0)
    hub = hub.union(_build_disc(hub_core_radius, hub_thickness, 0.0))
    hub = hub.union(
        _build_annulus(
            hub_flange_outer_radius,
            hub_flange_inner_radius,
            hub_flange_thickness,
            hub_flange_offset,
        )
    )
    hub = hub.union(
        _build_annulus(
            hub_flange_outer_radius,
            hub_flange_inner_radius,
            hub_flange_thickness,
            -hub_flange_offset,
        )
    )

    for index in range(num_rollers):
        angle = (2.0 * math.pi * index) / num_rollers
        angle_deg = math.degrees(angle)
        spoke = (
            cq.Workplane("XY")
            .box(hub_spoke_length, hub_spoke_width, hub_spoke_height)
            .translate((hub_spoke_center_radius, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        lug = (
            _build_mount_lug()
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -roller_tilt_deg)
            .translate((roller_center_radius, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        hub = hub.union(spoke).union(lug)

    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, -((hub_thickness + 0.004) / 2.0)))
        .circle(hub_bore_radius)
        .extrude(hub_thickness + 0.004)
    )
    return hub.cut(bore)


def _orient(shape, rpy_deg):
    roll, pitch, yaw = rpy_deg
    oriented = shape
    if roll:
        oriented = oriented.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), roll)
    if pitch:
        oriented = oriented.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), pitch)
    if yaw:
        oriented = oriented.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw)
    return oriented


hub_shape = _build_hub_shape()
roller_tread_shape = _build_roller_tread_shape()

result = cq.Assembly(name="mecanum_wheel")
result.add(
    hub_shape,
    name="hub",
    color=cq.Color(*colors["gunmetal"]),
)

for side in (-1.0, 1.0):
    result.add(
        _build_disc(0.024, 0.004, side * 0.026),
        name=f"accent_disc_{'front' if side > 0 else 'back'}",
        color=cq.Color(*colors["accent"]),
    )

for index in range(num_bolts):
    angle = (2.0 * math.pi * index) / num_bolts
    x = bolt_circle_radius * math.cos(angle)
    y = bolt_circle_radius * math.sin(angle)
    for side in (-1.0, 1.0):
        result.add(
            cq.Workplane("XY", origin=(x, y, (side * bolt_head_z) - (bolt_head_length / 2.0)))
            .circle(bolt_head_radius)
            .extrude(bolt_head_length),
            name=f"bolt_{index}_{'front' if side > 0 else 'back'}",
            color=cq.Color(*colors["accent"]),
        )

for index in range(num_rollers):
    center = _roller_center(index)
    axis = _roller_axis(index)
    roller_rpy_deg = tuple(math.degrees(value) for value in _axis_to_rpy(axis))
    cap_offset = _translate_along(axis, roller_endcap_offset)
    bush_offset = _translate_along(axis, (roller_length - roller_bush_length) / 2.0)

    tread = _orient(roller_tread_shape, roller_rpy_deg).translate(center)
    shaft = _orient(
        cq.Workplane("XY", origin=(0.0, 0.0, -(roller_length / 2.0)))
        .circle(roller_shaft_radius)
        .extrude(roller_length),
        roller_rpy_deg,
    ).translate(center)

    result.add(
        tread,
        name=f"roller_tread_{index}",
        color=cq.Color(*colors["rubber"]),
    )
    result.add(
        shaft,
        name=f"roller_shaft_{index}",
        color=cq.Color(*colors["steel"]),
    )

    for part_name, radius, length, offset in (
        ("bush_a", roller_bush_radius, roller_bush_length, bush_offset),
        (
            "bush_b",
            roller_bush_radius,
            roller_bush_length,
            (-bush_offset[0], -bush_offset[1], -bush_offset[2]),
        ),
        ("cap_a", roller_endcap_radius, roller_endcap_length, cap_offset),
        (
            "cap_b",
            roller_endcap_radius,
            roller_endcap_length,
            (-cap_offset[0], -cap_offset[1], -cap_offset[2]),
        ),
    ):
        part = _orient(
            cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
            .circle(radius)
            .extrude(length),
            roller_rpy_deg,
        ).translate(
            (
                center[0] + offset[0],
                center[1] + offset[1],
                center[2] + offset[2],
            )
        )
        result.add(
            part,
            name=f"roller_{index}_{part_name}",
            color=cq.Color(*colors["steel"]),
        )
```
