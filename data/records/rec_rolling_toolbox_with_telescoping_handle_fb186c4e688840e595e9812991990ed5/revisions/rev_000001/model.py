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


BODY_D = 0.36
BODY_W = 0.56
BODY_H = 0.30
BODY_WALL = 0.006

LID_H = 0.105
LID_OUTER_D = 0.286
LID_OUTER_W = 0.554
LID_X = BODY_D * 0.5 - LID_OUTER_D * 0.5 - 0.003

DRAWER_W = 0.454
DRAWER_H = 0.074
DRAWER_D = 0.252
DRAWER_TRAVEL = 0.17
DRAWER_BOTTOM = 0.071

WHEEL_R = 0.087
WHEEL_T = 0.044
WHEEL_X = -0.146
WHEEL_Y = BODY_W * 0.5 + 0.026
WHEEL_Z = WHEEL_R

HANDLE_X = -BODY_D * 0.5 - 0.011
HANDLE_RAIL_SPAN = 0.300
HANDLE_SLEEVE_BOTTOM = 0.174
HANDLE_SLEEVE_H = 0.242
HANDLE_JOINT_Z = HANDLE_SLEEVE_BOTTOM + HANDLE_SLEEVE_H
HANDLE_TRAVEL = 0.18

COVER_DEPTH = 0.174
COVER_WIDTH = 0.214
COVER_HINGE_X = -0.086
COVER_Y = 0.120


def _lower_shell_shape() -> cq.Workplane:
    drawer_clear = 0.006
    drawer_cavity_d = DRAWER_D + 0.012
    drawer_cavity_w = DRAWER_W + 0.012
    drawer_cavity_h = DRAWER_H + 0.010

    body = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )

    main_cavity = (
        cq.Workplane("XY")
        .box(
            BODY_D - 2.0 * BODY_WALL,
            BODY_W - 2.0 * BODY_WALL,
            BODY_H - BODY_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )
    body = body.cut(main_cavity)

    drawer_cavity = (
        cq.Workplane("XY")
        .box(
            drawer_cavity_d + 0.020,
            drawer_cavity_w,
            drawer_cavity_h,
            centered=(True, True, False),
        )
        .translate(
            (
                BODY_D * 0.5 - drawer_cavity_d * 0.5 + 0.010,
                0.0,
                DRAWER_BOTTOM - drawer_clear,
            )
        )
    )
    body = body.cut(drawer_cavity)

    drawer_shelf = (
        cq.Workplane("XY")
        .box(
            drawer_cavity_d - 0.018,
            BODY_W - 2.0 * BODY_WALL,
            BODY_WALL,
            centered=(True, True, False),
        )
        .translate(
            (
                BODY_D * 0.5 - (drawer_cavity_d - 0.018) * 0.5 - 0.010,
                0.0,
                DRAWER_BOTTOM + DRAWER_H + 0.004,
            )
        )
    )
    body = body.union(drawer_shelf)

    runner_len = drawer_cavity_d - 0.048
    runner_w = 0.026
    runner_h = 0.010
    runner_y = DRAWER_W * 0.5 + 0.008
    runner_z = DRAWER_BOTTOM - 0.008
    runner_x = BODY_D * 0.5 - runner_len * 0.5 - 0.022
    for sign in (-1.0, 1.0):
        runner = (
            cq.Workplane("XY")
            .box(runner_len, runner_w, runner_h, centered=(True, True, False))
            .translate((runner_x, sign * runner_y, runner_z))
        )
        body = body.union(runner)

    sleeve_outer_d = 0.024
    sleeve_outer_w = 0.034
    sleeve_inner_d = 0.016
    sleeve_inner_w = 0.024
    sleeve_x = HANDLE_X
    for sign in (-1.0, 1.0):
        sleeve_outer = (
            cq.Workplane("XY")
            .box(sleeve_outer_d, sleeve_outer_w, HANDLE_SLEEVE_H, centered=(True, True, False))
            .translate((sleeve_x, sign * HANDLE_RAIL_SPAN * 0.5, HANDLE_SLEEVE_BOTTOM))
        )
        sleeve_inner = (
            cq.Workplane("XY")
            .box(
                sleeve_inner_d,
                sleeve_inner_w,
                HANDLE_SLEEVE_H + 0.012,
                centered=(True, True, False),
            )
            .translate((sleeve_x, sign * HANDLE_RAIL_SPAN * 0.5, HANDLE_SLEEVE_BOTTOM + 0.006))
        )
        body = body.union(sleeve_outer).cut(sleeve_inner)

    rear_panel = (
        cq.Workplane("XY")
        .box(0.012, 0.196, 0.170, centered=(True, True, False))
        .translate((HANDLE_X + 0.007, 0.0, 0.212))
    )
    body = body.union(rear_panel)

    side_pad_d = 0.042
    side_pad_w = 0.014
    side_pad_h = 0.068
    for sign in (-1.0, 1.0):
        pad = (
            cq.Workplane("XY")
            .box(side_pad_d, side_pad_w, side_pad_h, centered=(True, True, False))
            .translate((WHEEL_X + 0.010, sign * (BODY_W * 0.5 + side_pad_w * 0.5 - 0.002), 0.052))
        )
        body = body.union(pad)

    foot_d = 0.056
    foot_w = 0.090
    foot_h = 0.016
    for sign in (-1.0, 1.0):
        foot = (
            cq.Workplane("XY")
            .box(foot_d, foot_w, foot_h, centered=(True, True, False))
            .translate((BODY_D * 0.5 - 0.052, sign * 0.160, 0.0))
        )
        body = body.union(foot)

    return body


def _lid_shell_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_OUTER_D, LID_OUTER_W, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    lid_inner = (
        cq.Workplane("XY")
        .box(
            LID_OUTER_D - 0.014,
            LID_OUTER_W - 0.014,
            LID_H - BODY_WALL,
            centered=(True, True, False),
        )
    )
    lid = lid.cut(lid_inner)

    recess_depth = 0.030
    recess_size_d = COVER_DEPTH + 0.010
    recess_size_w = COVER_WIDTH + 0.008
    recess_center_x = COVER_HINGE_X + COVER_DEPTH * 0.5
    recess_bottom = LID_H - recess_depth
    for sign in (-1.0, 1.0):
        recess = (
            cq.Workplane("XY")
            .box(recess_size_d, recess_size_w, recess_depth, centered=(True, True, False))
            .translate((recess_center_x, sign * COVER_Y, recess_bottom))
        )
        lid = lid.cut(recess)

    front_bezel = (
        cq.Workplane("XY")
        .box(0.032, LID_OUTER_W - 0.070, 0.014, centered=(True, True, False))
        .translate((LID_OUTER_D * 0.5 - 0.030, 0.0, LID_H - 0.014))
    )
    lid = lid.union(front_bezel)

    return lid


def _drawer_shape() -> cq.Workplane:
    front_t = 0.016
    back_t = 0.008
    wall = 0.006
    drawer = (
        cq.Workplane("XY")
        .box(DRAWER_D, DRAWER_W, DRAWER_H, centered=(True, True, False))
        .translate((-DRAWER_D * 0.5, 0.0, 0.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            DRAWER_D - front_t - back_t,
            DRAWER_W - 2.0 * wall,
            DRAWER_H - wall,
            centered=(True, True, False),
        )
        .translate((-(front_t + (DRAWER_D - front_t - back_t) * 0.5), 0.0, wall))
    )
    drawer = drawer.cut(cavity)

    pull = (
        cq.Workplane("YZ")
        .center(0.0, DRAWER_H * 0.52)
        .slot2D(0.170, 0.028, 0.014)
        .extrude(0.010)
        .translate((-0.010, 0.0, 0.0))
    )
    drawer = drawer.cut(pull)

    return drawer


def _part_aabb_z(ctx: TestContext, part) -> tuple[float | None, float | None]:
    bounds = ctx.part_world_aabb(part)
    if bounds is None:
        return (None, None)
    return (bounds[0][2], bounds[1][2])


def _part_aabb_x(ctx: TestContext, part) -> tuple[float | None, float | None]:
    bounds = ctx.part_world_aabb(part)
    if bounds is None:
        return (None, None)
    return (bounds[0][0], bounds[1][0])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    shell_plastic = model.material("shell_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.22, 0.23, 0.24, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.65, 0.75, 0.83, 0.35))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.30, 0.31, 0.33, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_cadquery(_lower_shell_shape(), "lower_shell"),
        material=shell_plastic,
        name="lower_shell",
    )

    lid_shell = model.part("lid_shell")
    lid_shell.visual(
        mesh_from_cadquery(_lid_shell_shape(), "lid_shell"),
        material=lid_plastic,
        name="lid_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "drawer"),
        material=drawer_plastic,
        name="drawer_shell",
    )

    handle = model.part("handle")
    rail_len = 0.390
    rail_insert = 0.220
    rail_top = rail_len - rail_insert
    rail_center_z = (rail_top - rail_insert) * 0.5
    for index, sign in enumerate((-1.0, 1.0)):
        handle.visual(
            Box((0.015, 0.020, rail_len)),
            origin=Origin(xyz=(0.0, sign * HANDLE_RAIL_SPAN * 0.5, rail_center_z)),
            material=rail_aluminum,
            name=f"rail_{index}",
        )
        handle.visual(
            Box((0.022, 0.030, 0.008)),
            origin=Origin(xyz=(0.0, sign * HANDLE_RAIL_SPAN * 0.5, 0.004)),
            material=grip_black,
            name=f"guide_cap_{index}",
        )
    handle.visual(
        Box((0.030, HANDLE_RAIL_SPAN + 0.060, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, rail_top + 0.018)),
        material=grip_black,
        name="grip",
    )
    handle.visual(
        Box((0.020, HANDLE_RAIL_SPAN + 0.020, 0.014)),
        origin=Origin(xyz=(-0.004, 0.0, rail_top + 0.002)),
        material=grip_black,
        name="grip_bridge",
    )

    for wheel_name in ("left_wheel", "right_wheel"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_T),
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.060, length=WHEEL_T - 0.010),
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=wheel_hub,
            name="hub",
        )

    for cover_name, cover_y in (("organizer_cover_0", -COVER_Y), ("organizer_cover_1", COVER_Y)):
        cover = model.part(cover_name)
        cover.visual(
            Box((COVER_DEPTH, COVER_WIDTH, 0.006)),
            origin=Origin(xyz=(COVER_DEPTH * 0.5, 0.0, -0.002)),
            material=clear_smoke,
            name="cover_panel",
        )
        cover.visual(
            Box((0.016, COVER_WIDTH * 0.60, 0.010)),
            origin=Origin(xyz=(COVER_DEPTH - 0.012, 0.0, -0.002)),
            material=clear_smoke,
            name="pull_lip",
        )
        cover.visual(
            Cylinder(radius=0.004, length=COVER_WIDTH - 0.010),
            origin=Origin(xyz=(0.002, 0.0, -0.003), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=clear_smoke,
            name="hinge_barrel",
        )
        cover.visual(
            Box((0.012, 0.026, 0.008)),
            origin=Origin(xyz=(0.010, cover_y / abs(cover_y) * (COVER_WIDTH * 0.5 - 0.018), -0.003)),
            material=clear_smoke,
            name="hinge_tab",
        )

    model.articulation(
        "shell_to_lid_shell",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=lid_shell,
        origin=Origin(xyz=(LID_X, 0.0, BODY_H)),
    )
    model.articulation(
        "shell_to_drawer",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=drawer,
        origin=Origin(xyz=(BODY_D * 0.5, 0.0, DRAWER_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.30, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "shell_to_handle",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "shell_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=lower_shell,
        child="left_wheel",
        origin=Origin(xyz=(WHEEL_X, -(WHEEL_Y + 0.004), WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "shell_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=lower_shell,
        child="right_wheel",
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y + 0.004, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "lid_to_organizer_cover_0",
        ArticulationType.REVOLUTE,
        parent=lid_shell,
        child="organizer_cover_0",
        origin=Origin(xyz=(COVER_HINGE_X, -COVER_Y, LID_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "lid_to_organizer_cover_1",
        ArticulationType.REVOLUTE,
        parent=lid_shell,
        child="organizer_cover_1",
        origin=Origin(xyz=(COVER_HINGE_X, COVER_Y, LID_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_shell = object_model.get_part("lower_shell")
    lid_shell = object_model.get_part("lid_shell")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("handle")
    cover_0 = object_model.get_part("organizer_cover_0")
    cover_1 = object_model.get_part("organizer_cover_1")

    drawer_joint = object_model.get_articulation("shell_to_drawer")
    handle_joint = object_model.get_articulation("shell_to_handle")
    cover_joint_0 = object_model.get_articulation("lid_to_organizer_cover_0")
    cover_joint_1 = object_model.get_articulation("lid_to_organizer_cover_1")

    ctx.allow_overlap(
        lid_shell,
        lower_shell,
        reason="The closed lid shell is authored as a seated nested shell over the lower tub, so the shell volumes intentionally occupy the same sealed cavity region in the closed pose.",
    )
    ctx.allow_isolated_part(
        cover_0,
        reason="The transparent organizer cover is hinged by its articulation axis, and the modeled hinge barrel keeps a small visual clearance from the lid recess instead of using a literal touching pin.",
    )
    ctx.allow_isolated_part(
        cover_1,
        reason="The transparent organizer cover is hinged by its articulation axis, and the modeled hinge barrel keeps a small visual clearance from the lid recess instead of using a literal touching pin.",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    rest_handle_pos = ctx.part_world_position(handle)
    rest_drawer_x = _part_aabb_x(ctx, drawer)
    lower_shell_x = _part_aabb_x(ctx, lower_shell)
    rest_cover_0_z = _part_aabb_z(ctx, cover_0)
    rest_cover_1_z = _part_aabb_z(ctx, cover_1)
    lid_z = _part_aabb_z(ctx, lid_shell)

    ctx.expect_within(
        drawer,
        lower_shell,
        axes="yz",
        margin=0.010,
        name="drawer stays aligned inside lower shell at rest",
    )
    ctx.expect_overlap(
        drawer,
        lower_shell,
        axes="x",
        min_overlap=0.070,
        name="drawer remains inserted at rest",
    )
    ctx.expect_within(
        handle,
        lower_shell,
        axes="y",
        margin=0.016,
        name="handle stays centered on the twin rear guides at rest",
    )
    ctx.expect_overlap(
        handle,
        lower_shell,
        axes="z",
        min_overlap=0.120,
        name="collapsed handle remains deeply engaged in sleeves",
    )
    ctx.expect_overlap(
        cover_0,
        lid_shell,
        axes="xy",
        min_overlap=0.150,
        name="left organizer cover seats over its lid recess",
    )
    ctx.expect_overlap(
        cover_1,
        lid_shell,
        axes="xy",
        min_overlap=0.150,
        name="right organizer cover seats over its lid recess",
    )

    lid_top_ok = (
        lid_z[1] is not None
        and rest_cover_0_z[1] is not None
        and rest_cover_1_z[1] is not None
        and abs(rest_cover_0_z[1] - lid_z[1]) <= 0.008
        and abs(rest_cover_1_z[1] - lid_z[1]) <= 0.008
    )
    ctx.check(
        "organizer covers sit near the lid top plane when closed",
        lid_top_ok,
        details=f"lid_z={lid_z}, cover_0_z={rest_cover_0_z}, cover_1_z={rest_cover_1_z}",
    )

    drawer_limit = drawer_joint.motion_limits
    if drawer_limit is not None and drawer_limit.upper is not None:
        with ctx.pose({drawer_joint: drawer_limit.upper}):
            ext_drawer_pos = ctx.part_world_position(drawer)
            ext_drawer_x = _part_aabb_x(ctx, drawer)
            ctx.expect_within(
                drawer,
                lower_shell,
                axes="yz",
                margin=0.010,
                name="drawer stays aligned on runners when extended",
            )
            ctx.expect_overlap(
                drawer,
                lower_shell,
                axes="x",
                min_overlap=0.050,
                name="drawer retains runner engagement when extended",
            )
            ctx.check(
                "drawer extends forward",
                rest_drawer_pos is not None
                and ext_drawer_pos is not None
                and ext_drawer_pos[0] > rest_drawer_pos[0] + 0.12,
                details=f"rest={rest_drawer_pos}, extended={ext_drawer_pos}",
            )
            ctx.check(
                "drawer front moves past shell front at full extension",
                ext_drawer_x[1] is not None
                and lower_shell_x[1] is not None
                and ext_drawer_x[1] > lower_shell_x[1] + 0.10,
                details=f"drawer_x={ext_drawer_x}, shell_x={lower_shell_x}",
            )

    handle_limit = handle_joint.motion_limits
    if handle_limit is not None and handle_limit.upper is not None:
        with ctx.pose({handle_joint: handle_limit.upper}):
            ext_handle_pos = ctx.part_world_position(handle)
            ctx.expect_within(
                handle,
                lower_shell,
                axes="y",
                margin=0.016,
                name="extended handle stays on the rear rail axis",
            )
            ctx.expect_overlap(
                handle,
                lower_shell,
                axes="z",
                min_overlap=0.040,
                name="extended handle retains insertion in the guide sleeves",
            )
            ctx.check(
                "handle rises upward",
                rest_handle_pos is not None
                and ext_handle_pos is not None
                and ext_handle_pos[2] > rest_handle_pos[2] + 0.12,
                details=f"rest={rest_handle_pos}, extended={ext_handle_pos}",
            )

    cover_limit_0 = cover_joint_0.motion_limits
    if cover_limit_0 is not None and cover_limit_0.upper is not None:
        with ctx.pose({cover_joint_0: cover_limit_0.upper}):
            open_cover_0_z = _part_aabb_z(ctx, cover_0)
            ctx.check(
                "left organizer cover opens upward",
                rest_cover_0_z[1] is not None
                and open_cover_0_z[1] is not None
                and open_cover_0_z[1] > rest_cover_0_z[1] + 0.08,
                details=f"closed={rest_cover_0_z}, open={open_cover_0_z}",
            )

    cover_limit_1 = cover_joint_1.motion_limits
    if cover_limit_1 is not None and cover_limit_1.upper is not None:
        with ctx.pose({cover_joint_1: cover_limit_1.upper}):
            open_cover_1_z = _part_aabb_z(ctx, cover_1)
            ctx.check(
                "right organizer cover opens upward",
                rest_cover_1_z[1] is not None
                and open_cover_1_z[1] is not None
                and open_cover_1_z[1] > rest_cover_1_z[1] + 0.08,
                details=f"closed={rest_cover_1_z}, open={open_cover_1_z}",
            )

    return ctx.report()


object_model = build_object_model()
