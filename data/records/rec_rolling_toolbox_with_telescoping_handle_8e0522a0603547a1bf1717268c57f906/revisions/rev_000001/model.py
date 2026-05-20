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


CASE_L = 0.400
CASE_W = 0.240
CASE_BOTTOM = 0.028
CASE_H = 0.285
CASE_TOP = CASE_BOTTOM + CASE_H
CASE_RADIUS = 0.030

BASE_H = 0.232
BASE_TOP = CASE_BOTTOM + BASE_H
CRADLE_H = 0.024
CRADLE_TOP = BASE_TOP + CRADLE_H
SIDE_RAIL_W = 0.028

CAVITY_L = 0.255
CAVITY_W = 0.184
CAVITY_X = -0.015
CAVITY_DEPTH = 0.042
CAVITY_FLOOR_Z = BASE_TOP

TRAY_L = 0.232
TRAY_BIN_W = 0.160
TRAY_TOTAL_W = 0.176
TRAY_H = 0.026
TRAY_WALL = 0.004
TRAY_FLANGE_W = (TRAY_TOTAL_W - TRAY_BIN_W) / 2.0
TRAY_RUNNER_L = 0.206
TRAY_TRAVEL = 0.130
TRAY_BOTTOM_Z = CRADLE_TOP + 0.002
RUNNER_H = 0.006
RUNNER_W = 0.014
RUNNER_Z = TRAY_BOTTOM_Z - RUNNER_H * 0.5
RUNNER_Y = TRAY_BIN_W * 0.5 - RUNNER_W * 0.5

HINGE_X = CAVITY_X - CAVITY_L * 0.5 - 0.006
HINGE_Z = CRADLE_TOP + 0.035
HINGE_R = 0.007
BODY_KNUCKLE_L = 0.046
BODY_KNUCKLE_Y = 0.070

COVER_L = 0.268
COVER_W = 0.194
COVER_T = 0.024
COVER_REAR_GAP = 0.010
COVER_PANEL_L = COVER_L - COVER_REAR_GAP
COVER_BARREL_L = 0.086
COVER_BRIDGE_W = 0.082

BUTTON_L = 0.024
BUTTON_W = 0.044
BUTTON_H = 0.014
BUTTON_TRAVEL = 0.008
BUTTON_Z = 0.006

HANDLE_TRAVEL = 0.170
HANDLE_ORIGIN_X = -CASE_L * 0.5 - 0.004
HANDLE_ORIGIN_Z = 0.286
HANDLE_RAIL_L = 0.210
HANDLE_RAIL_X = -0.006
HANDLE_RAIL_Y = 0.046
HANDLE_RAIL_THICK_X = 0.010
HANDLE_RAIL_THICK_Y = 0.012
HANDLE_GRIP_R = 0.009
HANDLE_GRIP_L = 0.120
HANDLE_GRIP_X = -0.014
HANDLE_GRIP_Z = 0.018
HANDLE_POCKET_D = 0.034
HANDLE_POCKET_W = 0.138
HANDLE_POCKET_H = 0.208
HANDLE_POCKET_Z = 0.202
HANDLE_SLOT_D = 0.020
HANDLE_SLOT_W = 0.016
HANDLE_SLOT_H = 0.220
HANDLE_SLOT_Z = 0.191

AXLE_X = -0.150
AXLE_Z = 0.038
AXLE_R = 0.0065
AXLE_L = 0.272
WHEEL_Y = CASE_W * 0.5 + 0.013
WHEEL_R = 0.036
WHEEL_W = 0.024
WHEEL_HUB_R = 0.021
WHEEL_BORE_R = AXLE_R + 0.0015


def _body_shape():
    return (
        cq.Workplane("XY")
        .box(CASE_L, CASE_W, BASE_H)
        .translate((0.0, 0.0, CASE_BOTTOM + BASE_H * 0.5))
        .edges("|Z")
        .fillet(CASE_RADIUS)
        .cut(
            cq.Workplane("XY")
            .box(HANDLE_POCKET_D, HANDLE_POCKET_W, HANDLE_POCKET_H)
            .translate((-(CASE_L * 0.5) + HANDLE_POCKET_D * 0.5 - 0.001, 0.0, HANDLE_POCKET_Z))
        )
    )


def _cover_shape():
    cover_panel = (
        cq.Workplane("XY")
        .box(COVER_PANEL_L, COVER_W, COVER_T)
        .translate((COVER_REAR_GAP + COVER_PANEL_L * 0.5, 0.0, 0.007))
        .edges("|Z")
        .fillet(0.010)
    )
    barrel = cq.Workplane("XZ").cylinder(COVER_BARREL_L, HINGE_R).translate((0.0, 0.0, 0.004))
    bridge = (
        cq.Workplane("XY")
        .box(0.016, COVER_BRIDGE_W, 0.012)
        .translate((0.008, 0.0, 0.003))
    )
    cover = cover_panel.union(barrel).union(bridge)

    underside_cavity = (
        cq.Workplane("XY")
        .box(COVER_PANEL_L - 0.014, COVER_W - 0.014, COVER_T)
        .translate((COVER_REAR_GAP + (COVER_PANEL_L - 0.014) * 0.5 + 0.003, 0.0, -0.007))
    )
    button_pocket = (
        cq.Workplane("XY")
        .box(0.022, BUTTON_W + 0.004, BUTTON_H + 0.004)
        .translate((COVER_L - 0.011, 0.0, BUTTON_Z))
    )
    return cover.cut(underside_cavity).cut(button_pocket)


def _tray_shape():
    tray_shell = (
        cq.Workplane("XY")
        .box(TRAY_L, TRAY_TOTAL_W - 0.004, TRAY_H)
        .translate((0.0, 0.0, TRAY_H * 0.5))
        .edges("|Z")
        .fillet(0.006)
    )
    tray_pocket = (
        cq.Workplane("XY")
        .box(TRAY_L - 0.012, TRAY_TOTAL_W - 0.016, TRAY_H)
        .translate((0.0, 0.0, TRAY_WALL + TRAY_H * 0.5))
    )
    pull_lip = (
        cq.Workplane("XY")
        .box(0.018, 0.054, 0.010)
        .translate((TRAY_L * 0.5 + 0.009, 0.0, 0.016))
    )
    underside_runner_0 = (
        cq.Workplane("XY")
        .box(TRAY_RUNNER_L, 0.012, 0.004)
        .translate((0.0, 0.060, 0.004))
    )
    underside_runner_1 = (
        cq.Workplane("XY")
        .box(TRAY_RUNNER_L, 0.012, 0.004)
        .translate((0.0, -0.060, 0.004))
    )
    return tray_shell.cut(tray_pocket).union(pull_lip).union(underside_runner_0).union(underside_runner_1)


def _handle_shape():
    rail_0 = (
        cq.Workplane("XY")
        .box(HANDLE_RAIL_THICK_X, HANDLE_RAIL_THICK_Y, HANDLE_RAIL_L)
        .translate((HANDLE_RAIL_X, HANDLE_RAIL_Y, -0.095))
    )
    rail_1 = (
        cq.Workplane("XY")
        .box(HANDLE_RAIL_THICK_X, HANDLE_RAIL_THICK_Y, HANDLE_RAIL_L)
        .translate((HANDLE_RAIL_X, -HANDLE_RAIL_Y, -0.095))
    )
    grip = cq.Workplane("XZ").cylinder(HANDLE_GRIP_L, HANDLE_GRIP_R).translate((HANDLE_GRIP_X, 0.0, HANDLE_GRIP_Z))
    lug_0 = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.026)
        .translate((HANDLE_GRIP_X, HANDLE_RAIL_Y, 0.002))
    )
    lug_1 = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.026)
        .translate((HANDLE_GRIP_X, -HANDLE_RAIL_Y, 0.002))
    )
    return rail_0.union(rail_1).union(grip).union(lug_0).union(lug_1)


def _wheel_shape():
    tire = cq.Workplane("XZ").cylinder(WHEEL_W, WHEEL_R)
    hub = cq.Workplane("XZ").cylinder(WHEEL_W + 0.004, WHEEL_HUB_R)
    side_0 = cq.Workplane("XZ").cylinder(0.004, WHEEL_R * 0.82).translate((0.0, WHEEL_W * 0.5 - 0.002, 0.0))
    side_1 = cq.Workplane("XZ").cylinder(0.004, WHEEL_R * 0.82).translate((0.0, -(WHEEL_W * 0.5 - 0.002), 0.0))
    bore = cq.Workplane("XZ").cylinder(WHEEL_W + 0.010, WHEEL_BORE_R)
    return tire.union(hub).union(side_0).union(side_1).cut(bore)


def _button_shape():
    return (
        cq.Workplane("XY")
        .box(BUTTON_L, BUTTON_W, BUTTON_H)
        .translate((-BUTTON_L * 0.5 + 0.001, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_case")

    case_shell = model.material("case_shell", rgba=(0.13, 0.14, 0.16, 1.0))
    cover_smoke = model.material("cover_smoke", rgba=(0.20, 0.24, 0.28, 0.82))
    tray_color = model.material("tray_color", rgba=(0.86, 0.54, 0.12, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    button_red = model.material("button_red", rgba=(0.79, 0.14, 0.12, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "tool_case_body"), material=case_shell, name="body_shell")
    body.visual(
        Box((CASE_L - CAVITY_L, CASE_W, CRADLE_H)),
        origin=Origin(
            xyz=(
                (CASE_L * 0.5 + (CAVITY_X + CAVITY_L * 0.5)) * 0.5,
                0.0,
                BASE_TOP + CRADLE_H * 0.5,
            )
        ),
        material=case_shell,
        name="front_lip",
    )
    body.visual(
        Box(((CAVITY_X - CAVITY_L * 0.5) - (-CASE_L * 0.5), CASE_W, CRADLE_H)),
        origin=Origin(
            xyz=(
                (-CASE_L * 0.5 + (CAVITY_X - CAVITY_L * 0.5)) * 0.5,
                0.0,
                BASE_TOP + CRADLE_H * 0.5,
            )
        ),
        material=case_shell,
        name="rear_lip",
    )
    body.visual(
        Box((CAVITY_L, SIDE_RAIL_W, CRADLE_H)),
        origin=Origin(xyz=(CAVITY_X, CASE_W * 0.5 - SIDE_RAIL_W * 0.5, BASE_TOP + CRADLE_H * 0.5)),
        material=case_shell,
        name="side_rail_0",
    )
    body.visual(
        Box((CAVITY_L, SIDE_RAIL_W, CRADLE_H)),
        origin=Origin(xyz=(CAVITY_X, -(CASE_W * 0.5 - SIDE_RAIL_W * 0.5), BASE_TOP + CRADLE_H * 0.5)),
        material=case_shell,
        name="side_rail_1",
    )
    body.visual(
        Box((TRAY_RUNNER_L, RUNNER_W, RUNNER_H)),
        origin=Origin(xyz=(CAVITY_X, RUNNER_Y, RUNNER_Z)),
        material=case_shell,
        name="runner_0",
    )
    body.visual(
        Box((TRAY_RUNNER_L, RUNNER_W, RUNNER_H)),
        origin=Origin(xyz=(CAVITY_X, -RUNNER_Y, RUNNER_Z)),
        material=case_shell,
        name="runner_1",
    )
    body.visual(
        Box((0.040, 0.050, CASE_BOTTOM)),
        origin=Origin(xyz=(0.145, 0.065, CASE_BOTTOM * 0.5)),
        material=case_shell,
        name="front_foot_0",
    )
    body.visual(
        Box((0.040, 0.050, CASE_BOTTOM)),
        origin=Origin(xyz=(0.145, -0.065, CASE_BOTTOM * 0.5)),
        material=case_shell,
        name="front_foot_1",
    )
    body.visual(
        Cylinder(radius=AXLE_R, length=AXLE_L),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=case_shell,
        name="axle",
    )
    body.visual(
        Box((0.010, 0.018, 0.204)),
        origin=Origin(xyz=(-0.205, HANDLE_RAIL_Y, 0.188)),
        material=case_shell,
        name="guide_0",
    )
    body.visual(
        Box((0.010, 0.018, 0.204)),
        origin=Origin(xyz=(-0.205, -HANDLE_RAIL_Y, 0.188)),
        material=case_shell,
        name="guide_1",
    )
    body.visual(
        Box((0.016, 0.100, 0.030)),
        origin=Origin(xyz=(HINGE_X + 0.008, 0.0, CRADLE_TOP + 0.015)),
        material=case_shell,
        name="hinge_bridge",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_L),
        origin=Origin(xyz=(HINGE_X, BODY_KNUCKLE_Y, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=case_shell,
        name="knuckle_0",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_L),
        origin=Origin(xyz=(HINGE_X, -BODY_KNUCKLE_Y, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=case_shell,
        name="knuckle_1",
    )

    cover = model.part("cover")
    cover.visual(mesh_from_cadquery(_cover_shape(), "tool_case_cover"), material=cover_smoke, name="cover_shell")

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_shape(), "tool_case_tray"), material=tray_color, name="tray_insert")

    pull_handle = model.part("pull_handle")
    pull_handle.visual(mesh_from_cadquery(_handle_shape(), "tool_case_handle"), material=handle_metal, name="handle_frame")

    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_0.visual(mesh_from_cadquery(_wheel_shape(), "tool_case_rear_wheel_0"), material=wheel_rubber, name="wheel_shell")

    rear_wheel_1 = model.part("rear_wheel_1")
    rear_wheel_1.visual(mesh_from_cadquery(_wheel_shape(), "tool_case_rear_wheel_1"), material=wheel_rubber, name="wheel_shell")

    latch_button = model.part("latch_button")
    latch_button.visual(mesh_from_cadquery(_button_shape(), "tool_case_latch_button"), material=button_red, name="button_cap")

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(CAVITY_X, 0.0, TRAY_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=20.0, velocity=0.18),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_handle,
        origin=Origin(xyz=(HANDLE_ORIGIN_X, 0.0, HANDLE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=HANDLE_TRAVEL, effort=25.0, velocity=0.24),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_0,
        origin=Origin(xyz=(AXLE_X, WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_1,
        origin=Origin(xyz=(AXLE_X, -WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "latch_press",
        ArticulationType.PRISMATIC,
        parent=cover,
        child=latch_button,
        origin=Origin(xyz=(COVER_L, 0.0, BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=4.0, velocity=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    tray = object_model.get_part("tray")
    pull_handle = object_model.get_part("pull_handle")
    latch_button = object_model.get_part("latch_button")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")

    cover_hinge = object_model.get_articulation("cover_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    handle_slide = object_model.get_articulation("handle_slide")
    latch_press = object_model.get_articulation("latch_press")
    rear_wheel_0_spin = object_model.get_articulation("rear_wheel_0_spin")
    rear_wheel_1_spin = object_model.get_articulation("rear_wheel_1_spin")

    ctx.allow_overlap(
        cover,
        latch_button,
        elem_a="cover_shell",
        elem_b="button_cap",
        reason="The push-button latch is intentionally represented as a plunger nested into the cover nose.",
    )

    ctx.expect_within(
        tray,
        body,
        axes="yz",
        margin=0.012,
        name="tray stays within the organizer bay width and height",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        min_overlap=0.12,
        name="tray remains inserted when closed",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.10}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover opens upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.070,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({cover_hinge: 1.10, tray_slide: TRAY_TRAVEL}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.014,
            name="extended tray stays guided between the case walls",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.05,
            name="extended tray keeps retained insertion",
        )
    ctx.check(
        "tray slides forward under the opened cover",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > closed_tray_pos[0] + 0.090,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    closed_handle_pos = ctx.part_world_position(pull_handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        extended_handle_pos = ctx.part_world_position(pull_handle)
        ctx.expect_overlap(
            pull_handle,
            body,
            axes="z",
            min_overlap=0.050,
            name="extended handle remains engaged in the rear guides",
        )
    ctx.check(
        "pull handle raises upward",
        closed_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > closed_handle_pos[2] + 0.120,
        details=f"closed={closed_handle_pos}, extended={extended_handle_pos}",
    )

    released_button_pos = ctx.part_world_position(latch_button)
    with ctx.pose({latch_press: BUTTON_TRAVEL}):
        pressed_button_pos = ctx.part_world_position(latch_button)
    ctx.check(
        "latch button presses into the lid nose",
        released_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < released_button_pos[0] - 0.005,
        details=f"released={released_button_pos}, pressed={pressed_button_pos}",
    )

    left_wheel_rest = ctx.part_world_position(rear_wheel_0)
    right_wheel_rest = ctx.part_world_position(rear_wheel_1)
    with ctx.pose({rear_wheel_0_spin: pi / 2.0, rear_wheel_1_spin: pi / 2.0}):
        left_wheel_spun = ctx.part_world_position(rear_wheel_0)
        right_wheel_spun = ctx.part_world_position(rear_wheel_1)
    ctx.check(
        "rear wheels spin about fixed axle centers",
        left_wheel_rest == left_wheel_spun and right_wheel_rest == right_wheel_spun,
        details=(
            f"left_rest={left_wheel_rest}, left_spun={left_wheel_spun}, "
            f"right_rest={right_wheel_rest}, right_spun={right_wheel_spun}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
