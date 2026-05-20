from __future__ import annotations

from math import pi

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

BODY_D = 0.42
BODY_W = 0.58
BODY_H = 0.50
BODY_Z0 = 0.08
BODY_TOP_Z = BODY_Z0 + BODY_H
BODY_WALL = 0.020

BIN_FLOOR_Z = BODY_Z0 + 0.18

DRAWER_OPEN_W = 0.470
DRAWER_OPEN_H = 0.110
DRAWER_BAY_DEPTH = 0.206
DRAWER_FRONT_PLANE_X = BODY_D / 2.0 - BODY_WALL
DRAWER_CENTER_Z = 0.150
DRAWER_TRAVEL = 0.120

LID_D = 0.428
LID_W = 0.584
LID_H = 0.078
LID_WALL = 0.012
LID_TOP = 0.014
LID_CLOSED_Z = BODY_TOP_Z
LID_OPEN = 1.16

HANDLE_RAIL_Y = 0.175
HANDLE_SLEEVE_SIZE = (0.038, 0.034, 0.220)
HANDLE_SLEEVE_X = -BODY_D / 2.0 - 0.018
HANDLE_SLEEVE_Z0 = 0.280
HANDLE_ENTRY_Z = HANDLE_SLEEVE_Z0 + HANDLE_SLEEVE_SIZE[2]
HANDLE_UPPER = 0.280

WHEEL_RADIUS = 0.075
WHEEL_WIDTH = 0.044
WHEEL_Y = BODY_W / 2.0 + WHEEL_WIDTH / 2.0
WHEEL_X = -0.166
WHEEL_Z = 0.078

LATCH_PIVOT_X = 0.115
LATCH_PIVOT_Y = BODY_W / 2.0 + 0.009
LATCH_PIVOT_Z = 0.490
LATCH_OPEN = 1.10


def _rect_tube(size: tuple[float, float, float], wall: float):
    sx, sy, sz = size
    outer = cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(sx - 2.0 * wall, sy - 2.0 * wall, sz + 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, -0.005))
    )
    return outer.cut(inner)


def _body_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_Z0))
        .edges("|Z")
        .fillet(0.028)
    )

    main_cavity = (
        cq.Workplane("XY")
        .box(
            BODY_D - 2.0 * BODY_WALL,
            BODY_W - 2.0 * BODY_WALL,
            BODY_TOP_Z - BIN_FLOOR_Z + 0.030,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BIN_FLOOR_Z))
    )

    drawer_bay = (
        cq.Workplane("XY")
        .box(DRAWER_BAY_DEPTH + 0.040, DRAWER_OPEN_W, DRAWER_OPEN_H, centered=(True, True, True))
        .translate((BODY_D / 2.0 - DRAWER_BAY_DEPTH / 2.0 + 0.020, 0.0, DRAWER_CENTER_Z))
    )

    return outer.cut(main_cavity).cut(drawer_bay)


def _lid_shape():
    outer = (
        cq.Workplane("XY")
        .box(LID_D, LID_W, LID_H, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    inner = (
        cq.Workplane("XY")
        .box(LID_D - 2.0 * LID_WALL, LID_W - 2.0 * LID_WALL, LID_H - LID_TOP + 0.010, centered=(False, True, False))
        .translate((LID_WALL, 0.0, -0.005))
    )
    recess = (
        cq.Workplane("XY")
        .box(0.250, 0.360, 0.014, centered=(False, True, False))
        .translate((0.095, 0.0, LID_H - 0.014))
    )
    return outer.cut(inner).cut(recess)


def _drawer_shape():
    outer = (
        cq.Workplane("XY")
        .box(0.208, 0.454, 0.102, centered=(True, True, True))
        .translate((-0.080, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )
    inner = (
        cq.Workplane("XY")
        .box(0.176, 0.418, 0.102, centered=(True, True, True))
        .translate((-0.086, 0.0, 0.008))
    )
    finger_pull = (
        cq.Workplane("XY")
        .box(0.032, 0.220, 0.024, centered=(True, True, True))
        .translate((0.016, 0.0, 0.018))
    )
    return outer.cut(inner).cut(finger_pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox_base")

    body_polymer = model.material("body_polymer", rgba=(0.19, 0.20, 0.22, 1.0))
    lid_polymer = model.material("lid_polymer", rgba=(0.13, 0.13, 0.14, 1.0))
    drawer_polymer = model.material("drawer_polymer", rgba=(0.16, 0.17, 0.18, 1.0))
    latch_accent = model.material("latch_accent", rgba=(0.86, 0.66, 0.14, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.07, 0.07, 0.08, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.38, 0.40, 0.42, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell_shape(), "toolbox_body_shell"), material=body_polymer, name="body_shell")
    body.visual(
        mesh_from_cadquery(_rect_tube(HANDLE_SLEEVE_SIZE, wall=0.005), "toolbox_handle_sleeve_left"),
        origin=Origin(xyz=(HANDLE_SLEEVE_X, -HANDLE_RAIL_Y, HANDLE_SLEEVE_Z0)),
        material=handle_metal,
        name="left_handle_sleeve",
    )
    body.visual(
        mesh_from_cadquery(_rect_tube(HANDLE_SLEEVE_SIZE, wall=0.005), "toolbox_handle_sleeve_right"),
        origin=Origin(xyz=(HANDLE_SLEEVE_X, HANDLE_RAIL_Y, HANDLE_SLEEVE_Z0)),
        material=handle_metal,
        name="right_handle_sleeve",
    )
    body.visual(
        Box((0.050, 0.028, 0.090)),
        origin=Origin(xyz=(-0.174, -0.276, 0.105)),
        material=body_polymer,
        name="left_axle_boss",
    )
    body.visual(
        Box((0.050, 0.028, 0.090)),
        origin=Origin(xyz=(-0.174, 0.276, 0.105)),
        material=body_polymer,
        name="right_axle_boss",
    )
    body.visual(
        Box((0.062, 0.054, 0.080)),
        origin=Origin(xyz=(0.158, -0.188, 0.040)),
        material=dark_trim,
        name="front_foot_0",
    )
    body.visual(
        Box((0.062, 0.054, 0.080)),
        origin=Origin(xyz=(0.158, 0.188, 0.040)),
        material=dark_trim,
        name="front_foot_1",
    )
    body.visual(
        Box((0.032, 0.020, 0.036)),
        origin=Origin(xyz=(LATCH_PIVOT_X, -0.286, LATCH_PIVOT_Z)),
        material=body_polymer,
        name="left_latch_mount",
    )
    body.visual(
        Box((0.032, 0.020, 0.036)),
        origin=Origin(xyz=(LATCH_PIVOT_X, 0.286, LATCH_PIVOT_Z)),
        material=body_polymer,
        name="right_latch_mount",
    )
    body.visual(
        Box((0.150, 0.014, 0.012)),
        origin=Origin(xyz=(0.095, -0.229, 0.094)),
        material=dark_trim,
        name="left_drawer_guide",
    )
    body.visual(
        Box((0.150, 0.014, 0.012)),
        origin=Origin(xyz=(0.095, 0.229, 0.094)),
        material=dark_trim,
        name="right_drawer_guide",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "toolbox_lid_shell"), material=lid_polymer, name="lid_shell")

    drawer = model.part("drawer")
    drawer.visual(mesh_from_cadquery(_drawer_shape(), "toolbox_drawer_shell"), material=drawer_polymer, name="drawer_shell")
    drawer.visual(
        Box((0.150, 0.012, 0.018)),
        origin=Origin(xyz=(-0.095, -0.222, -0.041)),
        material=dark_trim,
        name="left_runner",
    )
    drawer.visual(
        Box((0.150, 0.012, 0.018)),
        origin=Origin(xyz=(-0.095, 0.222, -0.041)),
        material=dark_trim,
        name="right_runner",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.024, 0.014, 0.580)),
        origin=Origin(xyz=(0.0, -HANDLE_RAIL_Y, -0.110)),
        material=handle_metal,
        name="left_rail",
    )
    handle.visual(
        Box((0.024, 0.014, 0.580)),
        origin=Origin(xyz=(0.0, HANDLE_RAIL_Y, -0.110)),
        material=handle_metal,
        name="right_rail",
    )
    handle.visual(
        Box((0.030, 0.388, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=handle_metal,
        name="crossbar",
    )
    handle.visual(
        Box((0.040, 0.240, 0.034)),
        origin=Origin(xyz=(-0.006, 0.0, 0.206)),
        material=dark_trim,
        name="grip",
    )
    handle.visual(
        Box((0.034, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -HANDLE_RAIL_Y, 0.012)),
        material=dark_trim,
        name="left_stop",
    )
    handle.visual(
        Box((0.034, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, HANDLE_RAIL_Y, 0.012)),
        material=dark_trim,
        name="right_stop",
    )

    left_latch = model.part("latch_0")
    left_latch.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_accent,
        name="pivot_barrel",
    )
    left_latch.visual(
        Box((0.028, 0.012, 0.090)),
        origin=Origin(xyz=(0.004, -0.004, 0.036)),
        material=latch_accent,
        name="latch_body",
    )
    left_latch.visual(
        Box((0.026, 0.016, 0.020)),
        origin=Origin(xyz=(0.008, -0.0005, 0.078)),
        material=latch_accent,
        name="hook",
    )

    right_latch = model.part("latch_1")
    right_latch.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_accent,
        name="pivot_barrel",
    )
    right_latch.visual(
        Box((0.028, 0.012, 0.090)),
        origin=Origin(xyz=(0.004, 0.004, 0.036)),
        material=latch_accent,
        name="latch_body",
    )
    right_latch.visual(
        Box((0.026, 0.016, 0.020)),
        origin=Origin(xyz=(0.008, 0.0005, 0.078)),
        material=latch_accent,
        name="hook",
    )

    left_wheel = model.part("wheel_0")
    left_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.053, length=WHEEL_WIDTH + 0.004),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hub",
    )

    right_wheel = model.part("wheel_1")
    right_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.053, length=WHEEL_WIDTH + 0.004),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hub",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.004, 0.0, LID_CLOSED_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=0.0, upper=LID_OPEN),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_FRONT_PLANE_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_SLEEVE_X, 0.0, HANDLE_ENTRY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=HANDLE_UPPER),
    )
    model.articulation(
        "body_to_latch_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, -LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=LATCH_OPEN),
    )
    model.articulation(
        "body_to_latch_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=LATCH_OPEN),
    )
    model.articulation(
        "body_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(WHEEL_X, -WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=30.0),
    )
    model.articulation(
        "body_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("handle")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    handle_slide = object_model.get_articulation("body_to_handle")
    latch_joint_0 = object_model.get_articulation("body_to_latch_0")
    latch_joint_1 = object_model.get_articulation("body_to_latch_1")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            name="closed lid sits tightly above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.34,
            name="closed lid covers the toolbox footprint",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: LID_OPEN}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12
        and open_lid_aabb[0][0] < closed_lid_aabb[0][0] - 0.06,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.010,
            name="closed drawer stays aligned within the lower shell opening",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.18,
            name="closed drawer remains deeply inserted",
        )
        closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.010,
            name="extended drawer stays aligned within the lower shell opening",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.06,
            name="extended drawer retains insertion on its runners",
        )
        open_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides forward",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[0] > closed_drawer_pos[0] + 0.10,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    with ctx.pose({handle_slide: 0.0}):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rail",
            outer_elem="left_handle_sleeve",
            margin=0.003,
            name="left telescoping rail stays centered in its sleeve when collapsed",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="right_rail",
            outer_elem="right_handle_sleeve",
            margin=0.003,
            name="right telescoping rail stays centered in its sleeve when collapsed",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rail",
            elem_b="left_handle_sleeve",
            min_overlap=0.18,
            name="collapsed left rail remains inserted in its sleeve",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="right_rail",
            elem_b="right_handle_sleeve",
            min_overlap=0.18,
            name="collapsed right rail remains inserted in its sleeve",
        )
        collapsed_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_UPPER}):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rail",
            outer_elem="left_handle_sleeve",
            margin=0.003,
            name="left telescoping rail stays centered in its sleeve when extended",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="right_rail",
            outer_elem="right_handle_sleeve",
            margin=0.003,
            name="right telescoping rail stays centered in its sleeve when extended",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rail",
            elem_b="left_handle_sleeve",
            min_overlap=0.09,
            name="extended left rail still retains insertion",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="right_rail",
            elem_b="right_handle_sleeve",
            min_overlap=0.09,
            name="extended right rail still retains insertion",
        )
        extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle extends upward",
        collapsed_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > collapsed_handle_pos[2] + 0.20,
        details=f"collapsed={collapsed_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({latch_joint_0: 0.0, latch_joint_1: 0.0}):
        closed_latch_0 = ctx.part_element_world_aabb(latch_0, elem="latch_body")
        closed_latch_1 = ctx.part_element_world_aabb(latch_1, elem="latch_body")
    with ctx.pose({latch_joint_0: LATCH_OPEN, latch_joint_1: LATCH_OPEN}):
        open_latch_0 = ctx.part_element_world_aabb(latch_0, elem="latch_body")
        open_latch_1 = ctx.part_element_world_aabb(latch_1, elem="latch_body")
    ctx.check(
        "side latches swing outward from the lid seam",
        closed_latch_0 is not None
        and closed_latch_1 is not None
        and open_latch_0 is not None
        and open_latch_1 is not None
        and open_latch_0[0][1] < closed_latch_0[0][1] - 0.030
        and open_latch_1[1][1] > closed_latch_1[1][1] + 0.030,
        details=f"left_closed={closed_latch_0}, left_open={open_latch_0}, right_closed={closed_latch_1}, right_open={open_latch_1}",
    )

    return ctx.report()


object_model = build_object_model()
