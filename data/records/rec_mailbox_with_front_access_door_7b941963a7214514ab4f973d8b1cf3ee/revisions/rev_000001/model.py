from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate(center)
    )


def _fixed_postbox_shell() -> cq.Workplane:
    """One connected fixed metal shell, including the post and hinge leaves."""

    width = 0.30
    depth = 0.20
    height = 0.70
    t = 0.015
    zc = 1.10
    bottom = zc - height / 2.0
    top = zc + height / 2.0

    pieces = [
        # Hollow mailbox body: back, sides, top, bottom and front frame.
        _box((width, t, height), (0.0, depth / 2.0 - t / 2.0, zc)),
        _box((t, depth, height), (-width / 2.0 + t / 2.0, 0.0, zc)),
        _box((t, depth, height), (width / 2.0 - t / 2.0, 0.0, zc)),
        _box((width, depth, t), (0.0, 0.0, top - t / 2.0)),
        _box((width, depth, t), (0.0, 0.0, bottom + t / 2.0)),
        _box((0.020, t, height), (-width / 2.0 + 0.010, -depth / 2.0 + t / 2.0, zc)),
        _box((0.020, t, height), (width / 2.0 - 0.010, -depth / 2.0 + t / 2.0, zc)),
        _box((width, t, 0.025), (0.0, -depth / 2.0 + t / 2.0, top - 0.0125)),
        _box((width, t, 0.025), (0.0, -depth / 2.0 + t / 2.0, bottom + 0.0125)),
        # Upper front face broken around the letter slot.
        _box((0.245, t, 0.032), (0.0, -depth / 2.0 + t / 2.0, 1.205)),
        _box((0.245, t, 0.030), (0.0, -depth / 2.0 + t / 2.0, 1.298)),
        _box((0.018, t, 0.110), (-0.122, -depth / 2.0 + t / 2.0, 1.252)),
        _box((0.018, t, 0.110), (0.122, -depth / 2.0 + t / 2.0, 1.252)),
        # A slightly proud rain cap gives the narrow body a postbox silhouette.
        _box((0.330, 0.230, 0.024), (0.0, 0.0, top + 0.012)),
        # Door and hood fixed hinge leaves mounted to the front frame.
        _box((0.250, 0.008, 0.018), (0.0, -0.099, 1.310)),
        # Post, base foot, and two straps visibly fixing the body to the post.
        _cylinder_z(0.026, 1.52, (0.0, 0.150, 0.760)),
        _cylinder_z(0.065, 0.024, (0.0, 0.150, 0.012)),
        _box((0.210, 0.075, 0.035), (0.0, 0.120, 0.960)),
        _box((0.210, 0.075, 0.035), (0.0, 0.120, 1.285)),
    ]

    shell = pieces[0]
    for piece in pieces[1:]:
        shell = shell.union(piece)
    return shell


def _door_shell() -> cq.Workplane:
    door_w = 0.250
    door_h = 0.390
    door_t = 0.018
    border = 0.018

    pieces = [
        _box((door_w, door_t, door_h), (door_w / 2.0, -door_t / 2.0, 0.0)),
        # Proud stamped frame on the outside face.
        _box((door_w - 0.020, 0.005, border), (door_w / 2.0, -door_t - 0.0025, door_h / 2.0 - border / 2.0 - 0.012)),
        _box((door_w - 0.020, 0.005, border), (door_w / 2.0, -door_t - 0.0025, -door_h / 2.0 + border / 2.0 + 0.012)),
        _box((border, 0.005, door_h - 0.070), (0.035, -door_t - 0.0025, 0.0)),
        _box((border, 0.005, door_h - 0.070), (door_w - 0.035, -door_t - 0.0025, 0.0)),
        # Two moving hinge knuckles, leaving space for the fixed-side center leaf.
        _cylinder_z(0.010, 0.105, (0.0, -0.012, 0.140)),
        _cylinder_z(0.010, 0.105, (0.0, -0.012, -0.140)),
    ]
    door = pieces[0]
    for piece in pieces[1:]:
        door = door.union(piece)
    return door


def _hood_shell() -> cq.Workplane:
    hood_w = 0.260
    hood_l = 0.095
    hood_t = 0.010
    closed_angle_deg = 25.0

    plate = _box((hood_w, hood_l, hood_t), (0.0, -hood_l / 2.0, -0.006))
    front_lip = _box((hood_w, 0.010, 0.032), (0.0, -hood_l, -0.020))
    cheek_0 = _box((0.008, hood_l, 0.030), (-hood_w / 2.0 + 0.004, -hood_l / 2.0, -0.016))
    cheek_1 = _box((0.008, hood_l, 0.030), (hood_w / 2.0 - 0.004, -hood_l / 2.0, -0.016))
    hinge_barrel = _cylinder_x(0.008, hood_w + 0.020, (0.0, -0.006, 0.0))

    hood = plate.union(front_lip).union(cheek_0).union(cheek_1).union(hinge_barrel)
    return hood.rotate((0, 0, 0), (1, 0, 0), closed_angle_deg)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_postbox")

    green = Material("powder_coated_green", color=(0.05, 0.20, 0.12, 1.0))
    dark = Material("dark_slot", color=(0.01, 0.01, 0.012, 1.0))
    brass = Material("aged_brass", color=(0.76, 0.56, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_fixed_postbox_shell(), "postbox_body"),
        material=green,
        name="body_shell",
    )
    body.visual(
        Box((0.240, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, -0.082, 1.252)),
        material=dark,
        name="mail_slot",
    )
    body.visual(
        Box((0.014, 0.012, 0.350)),
        origin=Origin(xyz=(-0.145, -0.1055, 1.000)),
        material=green,
        name="door_hinge_leaf",
    )
    body.visual(
        Box((0.016, 0.030, 0.020)),
        origin=Origin(xyz=(-0.148, -0.115, 1.307)),
        material=green,
        name="hood_lug_0",
    )
    body.visual(
        Box((0.016, 0.030, 0.020)),
        origin=Origin(xyz=(0.148, -0.115, 1.307)),
        material=green,
        name="hood_lug_1",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shell(), "door_panel"),
        material=green,
        name="door_panel",
    )
    door.visual(
        Cylinder(0.014, 0.026),
        origin=Origin(xyz=(0.215, -0.031, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="door_knob",
    )
    door.visual(
        Box((0.010, 0.004, 0.026)),
        origin=Origin(xyz=(0.215, -0.021, -0.052)),
        material=dark,
        name="key_slot",
    )

    hood = model.part("slot_hood")
    hood.visual(
        mesh_from_cadquery(_hood_shell(), "slot_hood"),
        material=green,
        name="hood_shell",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.137, -0.108, 1.000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.0, -0.132, 1.310)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hood = object_model.get_part("slot_hood")
    door_hinge = object_model.get_articulation("door_hinge")
    hood_hinge = object_model.get_articulation("hood_hinge")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.20,
        name="front door covers the lower box opening",
    )
    ctx.expect_overlap(
        hood,
        body,
        axes="x",
        elem_a="hood_shell",
        elem_b="mail_slot",
        min_overlap=0.18,
        name="weather hood spans the mail slot",
    )
    ctx.expect_gap(
        hood,
        door,
        axis="z",
        positive_elem="hood_shell",
        negative_elem="door_panel",
        min_gap=0.035,
        name="slot hood sits above the main door",
    )
    ctx.check(
        "door hinge is vertical",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "hood hinge is horizontal",
        tuple(round(v, 6) for v in hood_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={hood_hinge.axis}",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    closed_hood = ctx.part_element_world_aabb(hood, elem="hood_shell")
    with ctx.pose({door_hinge: 1.20}):
        open_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hood_hinge: 0.85}):
        open_hood = ctx.part_element_world_aabb(hood, elem="hood_shell")

    ctx.check(
        "front door swings outward from the box",
        closed_door is not None
        and open_door is not None
        and open_door[0][1] < closed_door[0][1] - 0.070,
        details=f"closed={closed_door}, open={open_door}",
    )
    ctx.check(
        "weather hood rotates upward",
        closed_hood is not None
        and open_hood is not None
        and open_hood[1][2] > closed_hood[1][2] + 0.025,
        details=f"closed={closed_hood}, open={open_hood}",
    )

    return ctx.report()


object_model = build_object_model()
