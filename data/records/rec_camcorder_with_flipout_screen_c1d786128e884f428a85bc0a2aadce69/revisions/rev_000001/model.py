from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LEN = 0.118
BODY_W = 0.050
BODY_H = 0.060

LENS_Z = 0.006
LENS_BARREL_R = 0.0148
LENS_BARREL_L = 0.026
ZOOM_INNER_R = 0.0162
ZOOM_OUTER_R = 0.0218
ZOOM_L = 0.014
ZOOM_X = BODY_LEN * 0.5 + 0.007
SHROUD_X = BODY_LEN * 0.5 + 0.023
SHROUD_L = 0.011

DISPLAY_LEN = 0.070
DISPLAY_T = 0.0035
DISPLAY_H = 0.046
DISPLAY_GAP = 0.0017
DISPLAY_HINGE_X = -0.030

LATCH_W = 0.012
LATCH_T = 0.0020
LATCH_H = 0.018
LATCH_Y = -0.010
LATCH_Z = 0.018


def _tube_shape(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .extrude(length)
        .translate((-length * 0.5, 0.0, 0.0))
    )
    inner_length = length + 0.003
    inner = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(inner_length)
        .translate((-inner_length * 0.5, 0.0, 0.0))
    )
    return outer.cut(inner)


def _body_shell_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BODY_LEN, BODY_W, BODY_H)
    base = base.edges("|Z").fillet(0.007)

    top_hump = cq.Workplane("XY").box(0.050, BODY_W * 0.84, 0.012).translate(
        (-0.004, 0.0, BODY_H * 0.5 + 0.004)
    )
    battery_bulge = cq.Workplane("XY").box(0.024, 0.022, 0.032).translate(
        (-BODY_LEN * 0.5 + 0.011, -0.005, 0.006)
    )
    nose_cheek = cq.Workplane("XY").box(0.018, 0.032, 0.036).translate(
        (BODY_LEN * 0.5 - 0.012, 0.0, 0.002)
    )

    door_pocket = cq.Workplane("XY").box(DISPLAY_LEN - 0.006, 0.0022, DISPLAY_H - 0.006).translate(
        (DISPLAY_HINGE_X + DISPLAY_LEN * 0.5 + 0.002, BODY_W * 0.5 - 0.0011, -0.001)
    )

    return base.union(top_hump).union(battery_bulge).union(nose_cheek).cut(door_pocket)


def _display_door_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(DISPLAY_LEN, DISPLAY_T, DISPLAY_H)
    door = door.edges("|Y").fillet(0.0045)

    screen_recess_depth = 0.0016
    screen_recess = cq.Workplane("XY").box(
        DISPLAY_LEN - 0.016,
        screen_recess_depth,
        DISPLAY_H - 0.014,
    ).translate((0.004, -DISPLAY_T * 0.5 + screen_recess_depth * 0.5, 0.0))

    hinge_spine = (
        cq.Workplane("XY")
        .circle(0.0030)
        .extrude(DISPLAY_H * 0.80)
        .translate((-DISPLAY_LEN * 0.5 + 0.0012, 0.0, -DISPLAY_H * 0.40))
    )

    return door.cut(screen_recess).union(hinge_spine)


def _eyecup_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(0.0125)
        .workplane(offset=0.018)
        .circle(0.0078)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .circle(0.0088)
        .workplane(offset=0.018)
        .circle(0.0048)
        .loft(combine=True)
    )
    cup = outer.cut(inner).translate((-0.009, 0.0, 0.0))
    mount_lip = _tube_shape(0.004, 0.0096, 0.0064).translate((0.006, 0.0, 0.0))
    return cup.union(mount_lip)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_camcorder")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    door_finish = model.material("door_finish", rgba=(0.24, 0.25, 0.28, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.50, 0.52, 0.56, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.12, 0.14, 0.17, 0.70))
    indicator_finish = model.material("indicator_finish", rgba=(0.88, 0.90, 0.92, 1.0))
    record_finish = model.material("record_finish", rgba=(0.72, 0.11, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "camcorder_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.0050, 0.0012, 0.040)),
        origin=Origin(xyz=(DISPLAY_HINGE_X + 0.0012, 0.02485, 0.0)),
        material=body_finish,
        name="hinge_mount",
    )
    body.visual(
        Cylinder(radius=LENS_BARREL_R, length=LENS_BARREL_L),
        origin=Origin(xyz=(BODY_LEN * 0.5 + 0.009, 0.0, LENS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber_finish,
        name="lens_barrel",
    )
    body.visual(
        mesh_from_cadquery(_tube_shape(SHROUD_L, 0.0255, 0.0102), "camcorder_front_shroud"),
        origin=Origin(xyz=(SHROUD_X, 0.0, LENS_Z)),
        material=body_finish,
        name="front_shroud",
    )
    body.visual(
        Cylinder(radius=0.0104, length=0.004),
        origin=Origin(xyz=(BODY_LEN * 0.5 + 0.028, 0.0, LENS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=glass_finish,
        name="lens_glass",
    )
    body.visual(
        Box((0.012, 0.016, 0.012)),
        origin=Origin(xyz=(-0.058, 0.009, 0.012)),
        material=body_finish,
        name="viewfinder_mount",
    )
    body.visual(
        mesh_from_cadquery(_eyecup_shape(), "camcorder_eyecup"),
        origin=Origin(xyz=(-0.068, 0.009, 0.012)),
        material=rubber_finish,
        name="eyecup",
    )
    body.visual(
        Box((0.060, 0.0025, 0.020)),
        origin=Origin(xyz=(0.004, -BODY_W * 0.5 - 0.0010, -0.004)),
        material=rubber_finish,
        name="strap_pad",
    )
    body.visual(
        Cylinder(radius=0.0040, length=0.004),
        origin=Origin(xyz=(-0.010, -0.011, BODY_H * 0.5 + 0.008), rpy=(0.0, 0.0, 0.0)),
        material=record_finish,
        name="record_button",
    )

    display_door = model.part("display_door")
    display_door.visual(
        mesh_from_cadquery(_display_door_shape(), "camcorder_display_door"),
        origin=Origin(xyz=(DISPLAY_LEN * 0.5, DISPLAY_T * 0.5, 0.0)),
        material=door_finish,
        name="door_shell",
    )
    display_door.visual(
        Box((0.050, 0.0007, 0.034)),
        origin=Origin(xyz=(0.038, 0.00065, 0.0)),
        material=glass_finish,
        name="screen_glass",
    )

    zoom_collar = model.part("zoom_collar")
    zoom_collar.visual(
        mesh_from_cadquery(_tube_shape(ZOOM_L, ZOOM_OUTER_R, ZOOM_INNER_R), "camcorder_zoom_collar"),
        material=rubber_finish,
        name="collar_ring",
    )
    zoom_collar.visual(
        Box((0.0030, 0.0055, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, ZOOM_OUTER_R + 0.0003)),
        material=indicator_finish,
        name="index_mark",
    )

    battery_latch = model.part("battery_latch")
    battery_latch.visual(
        Box((LATCH_T, LATCH_W, LATCH_H)),
        origin=Origin(xyz=(-LATCH_T * 0.5, 0.0, -0.0083)),
        material=trim_finish,
        name="latch_paddle",
    )
    battery_latch.visual(
        Cylinder(radius=0.0014, length=LATCH_W + 0.001),
        origin=Origin(
            xyz=(-0.0007, 0.0, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=trim_finish,
        name="latch_barrel",
    )

    model.articulation(
        "display_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_door,
        origin=Origin(xyz=(DISPLAY_HINGE_X, BODY_W * 0.5 + DISPLAY_GAP, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.2, effort=0.4, velocity=3.5),
    )
    model.articulation(
        "zoom_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=zoom_collar,
        origin=Origin(xyz=(ZOOM_X, 0.0, LENS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "battery_release",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_latch,
        origin=Origin(xyz=(-BODY_LEN * 0.5 - 0.0006, LATCH_Y, LATCH_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.75, effort=0.1, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    display_door = object_model.get_part("display_door")
    zoom_collar = object_model.get_part("zoom_collar")
    battery_latch = object_model.get_part("battery_latch")

    display_hinge = object_model.get_articulation("display_hinge")
    zoom_rotation = object_model.get_articulation("zoom_rotation")
    battery_release = object_model.get_articulation("battery_release")

    ctx.expect_gap(
        display_door,
        body,
        axis="y",
        positive_elem="door_shell",
        negative_elem="body_shell",
        min_gap=0.0004,
        max_gap=0.0030,
        name="display door sits just off the body sidewall",
    )
    ctx.expect_overlap(
        display_door,
        body,
        axes="xz",
        elem_a="door_shell",
        elem_b="body_shell",
        min_overlap=0.035,
        name="display door covers the camcorder side",
    )
    ctx.expect_overlap(
        zoom_collar,
        body,
        axes="yz",
        elem_a="collar_ring",
        elem_b="lens_barrel",
        min_overlap=0.028,
        name="zoom collar stays concentric with the lens barrel",
    )

    closed_door_aabb = ctx.part_element_world_aabb(display_door, elem="door_shell")
    with ctx.pose({display_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(display_door, elem="door_shell")
    ctx.check(
        "display door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.030,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_mark = _aabb_center(ctx.part_element_world_aabb(zoom_collar, elem="index_mark"))
    with ctx.pose({zoom_rotation: math.pi * 0.5}):
        turned_mark = _aabb_center(ctx.part_element_world_aabb(zoom_collar, elem="index_mark"))
    ctx.check(
        "zoom collar marker orbits the viewing axis",
        closed_mark is not None
        and turned_mark is not None
        and abs(closed_mark[0] - turned_mark[0]) < 0.002
        and math.hypot(closed_mark[1] - turned_mark[1], closed_mark[2] - turned_mark[2]) > 0.015,
        details=f"closed={closed_mark}, turned={turned_mark}",
    )

    closed_latch = ctx.part_element_world_aabb(battery_latch, elem="latch_paddle")
    with ctx.pose({battery_release: 0.60}):
        open_latch = ctx.part_element_world_aabb(battery_latch, elem="latch_paddle")
    ctx.check(
        "battery latch flips off the rear wall",
        closed_latch is not None
        and open_latch is not None
        and open_latch[0][0] < closed_latch[0][0] - 0.004,
        details=f"closed={closed_latch}, open={open_latch}",
    )

    return ctx.report()


object_model = build_object_model()
