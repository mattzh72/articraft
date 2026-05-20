from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.315
BODY_DEPTH = 0.355
BODY_HEIGHT = 0.375
FOOT_HEIGHT = 0.010

OPENING_WIDTH = 0.280
OPENING_HEIGHT = 0.236
OPENING_BOTTOM = 0.047

CHAMBER_WIDTH = 0.246
CHAMBER_DEPTH = 0.308
CHAMBER_HEIGHT = 0.220
CHAMBER_WALL = 0.0025
CHAMBER_BOTTOM = 0.055

DRAWER_WIDTH = 0.234
DRAWER_DEPTH = 0.300
DRAWER_HEIGHT = 0.132
DRAWER_WALL = 0.0025

BASKET_WIDTH = 0.214
BASKET_DEPTH = 0.265
BASKET_HEIGHT = 0.105
BASKET_WALL = 0.0020


def _body_shell_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT - FOOT_HEIGHT, centered=(False, True, False))
        .translate((-BODY_DEPTH + 0.008, 0.0, FOOT_HEIGHT))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.014)
    )

    lower_cavity = (
        cq.Workplane("XY")
        .box(0.342, OPENING_WIDTH, OPENING_HEIGHT, centered=(False, True, False))
        .translate((-0.334, 0.0, OPENING_BOTTOM))
    )
    housing = housing.cut(lower_cavity)

    feet = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.295, -0.116),
                (-0.295, 0.116),
                (-0.060, -0.116),
                (-0.060, 0.116),
            ]
        )
        .circle(0.013)
        .extrude(FOOT_HEIGHT)
    )
    return housing.union(feet)


def _chamber_shell_shape() -> cq.Workplane:
    liner_length = CHAMBER_DEPTH + 0.008
    side_center_y = (CHAMBER_WIDTH * 0.5) - (CHAMBER_WALL * 0.5)

    left_wall = (
        cq.Workplane("XY")
        .box(liner_length, CHAMBER_WALL, CHAMBER_HEIGHT, centered=(False, True, False))
        .translate((-CHAMBER_DEPTH, -side_center_y, CHAMBER_BOTTOM))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(liner_length, CHAMBER_WALL, CHAMBER_HEIGHT, centered=(False, True, False))
        .translate((-CHAMBER_DEPTH, side_center_y, CHAMBER_BOTTOM))
    )
    roof = (
        cq.Workplane("XY")
        .box(liner_length, CHAMBER_WIDTH, CHAMBER_WALL, centered=(False, True, False))
        .translate((-CHAMBER_DEPTH, 0.0, CHAMBER_BOTTOM + CHAMBER_HEIGHT - CHAMBER_WALL))
    )
    back_wall = (
        cq.Workplane("XY")
        .box(CHAMBER_WALL, CHAMBER_WIDTH, CHAMBER_HEIGHT, centered=(False, True, False))
        .translate((-CHAMBER_DEPTH, 0.0, CHAMBER_BOTTOM))
    )
    left_flange = (
        cq.Workplane("XY")
        .box(0.008, 0.018, CHAMBER_HEIGHT, centered=(False, True, False))
        .translate((0.0, -0.131, CHAMBER_BOTTOM))
    )
    right_flange = (
        cq.Workplane("XY")
        .box(0.008, 0.018, CHAMBER_HEIGHT, centered=(False, True, False))
        .translate((0.0, 0.131, CHAMBER_BOTTOM))
    )
    top_flange = (
        cq.Workplane("XY")
        .box(0.008, OPENING_WIDTH, 0.026, centered=(False, True, False))
        .translate((0.0, 0.0, OPENING_BOTTOM + OPENING_HEIGHT - 0.026))
    )

    return left_wall.union(right_wall).union(roof).union(back_wall).union(left_flange).union(right_flange).union(top_flange)


def _drawer_pan_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DRAWER_DEPTH, DRAWER_WIDTH, DRAWER_HEIGHT, centered=(False, True, False))
        .translate((-DRAWER_DEPTH, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .shell(-DRAWER_WALL)
    )


def _drawer_front_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.022, 0.276, 0.172, centered=(False, True, False))
        .translate((0.0, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">X")
        .fillet(0.0035)
    )


def _basket_insert_shape() -> cq.Workplane:
    basket = (
        cq.Workplane("XY")
        .box(BASKET_DEPTH, BASKET_WIDTH, BASKET_HEIGHT, centered=(False, True, False))
        .translate((-BASKET_DEPTH, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .shell(-BASKET_WALL)
    )

    for x in (-0.225, -0.185, -0.145, -0.105, -0.065):
        for y in (-0.072, -0.036, 0.0, 0.036, 0.072):
            slot = (
                cq.Workplane("XY")
                .box(0.026, 0.007, 0.010, centered=(True, True, False))
                .translate((x, y, -0.001))
            )
            basket = basket.cut(slot)

    for x in (-0.210, -0.072):
        for y in (-0.070, 0.070):
            foot = (
                cq.Workplane("XY")
                .box(0.024, 0.024, 0.0115, centered=(True, True, False))
                .translate((x, y, -0.0115))
            )
            basket = basket.union(foot)

    return basket


def _handle_shape() -> cq.Workplane:
    left_mount = (
        cq.Workplane("XY")
        .box(0.018, 0.024, 0.050, centered=(False, True, False))
        .translate((0.0, -0.043, 0.0))
    )
    right_mount = (
        cq.Workplane("XY")
        .box(0.018, 0.024, 0.050, centered=(False, True, False))
        .translate((0.0, 0.043, 0.0))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.034, 0.112, 0.024, centered=(False, True, False))
        .translate((0.014, 0.0, 0.012))
    )
    latch_podium = (
        cq.Workplane("XY")
        .box(0.020, 0.050, 0.012, centered=(False, True, False))
        .translate((0.010, 0.0, 0.036))
    )
    return left_mount.union(right_mount).union(grip).union(latch_podium)


def _latch_button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.020, 0.042, 0.008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
        .edges(">Z")
        .fillet(0.0015)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_air_fryer")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.11, 0.12, 0.13, 1.0))
    display_glass = model.material("display_glass", rgba=(0.12, 0.20, 0.22, 0.55))
    chamber_dark = model.material("chamber_dark", rgba=(0.19, 0.19, 0.20, 1.0))
    basket_dark = model.material("basket_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dial_black = model.material("dial_black", rgba=(0.13, 0.13, 0.14, 1.0))
    start_green = model.material("start_green", rgba=(0.18, 0.42, 0.20, 1.0))
    stop_red = model.material("stop_red", rgba=(0.46, 0.14, 0.16, 1.0))
    button_grey = model.material("button_grey", rgba=(0.58, 0.60, 0.63, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "air_fryer_body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        Box((0.004, 0.236, 0.110)),
        origin=Origin(xyz=(0.010, 0.0, 0.297)),
        material=fascia_black,
        name="front_fascia",
    )
    body.visual(
        Box((0.003, 0.154, 0.060)),
        origin=Origin(xyz=(0.0115, 0.0, 0.322)),
        material=fascia_black,
        name="display_bezel",
    )
    body.visual(
        Box((0.002, 0.136, 0.042)),
        origin=Origin(xyz=(0.012, 0.0, 0.322)),
        material=display_glass,
        name="display",
    )

    body.visual(
        Box((CHAMBER_DEPTH + 0.008, CHAMBER_WALL, CHAMBER_HEIGHT)),
        origin=Origin(
            xyz=(
                (-CHAMBER_DEPTH + 0.008) * 0.5,
                -((CHAMBER_WIDTH * 0.5) - (CHAMBER_WALL * 0.5)),
                CHAMBER_BOTTOM + (CHAMBER_HEIGHT * 0.5),
            )
        ),
        material=chamber_dark,
        name="chamber_left_wall",
    )
    body.visual(
        Box((CHAMBER_DEPTH + 0.008, CHAMBER_WALL, CHAMBER_HEIGHT)),
        origin=Origin(
            xyz=(
                (-CHAMBER_DEPTH + 0.008) * 0.5,
                (CHAMBER_WIDTH * 0.5) - (CHAMBER_WALL * 0.5),
                CHAMBER_BOTTOM + (CHAMBER_HEIGHT * 0.5),
            )
        ),
        material=chamber_dark,
        name="chamber_right_wall",
    )
    body.visual(
        Box((CHAMBER_DEPTH + 0.008, CHAMBER_WIDTH, CHAMBER_WALL)),
        origin=Origin(
            xyz=(
                (-CHAMBER_DEPTH + 0.008) * 0.5,
                0.0,
                CHAMBER_BOTTOM + CHAMBER_HEIGHT - (CHAMBER_WALL * 0.5),
            )
        ),
        material=chamber_dark,
        name="chamber_roof",
    )
    body.visual(
        Box((CHAMBER_WALL, CHAMBER_WIDTH, CHAMBER_HEIGHT)),
        origin=Origin(
            xyz=(
                -CHAMBER_DEPTH + (CHAMBER_WALL * 0.5),
                0.0,
                CHAMBER_BOTTOM + (CHAMBER_HEIGHT * 0.5),
            )
        ),
        material=chamber_dark,
        name="chamber_back",
    )
    body.visual(
        Box((0.008, 0.018, CHAMBER_HEIGHT)),
        origin=Origin(xyz=(0.004, -0.131, CHAMBER_BOTTOM + (CHAMBER_HEIGHT * 0.5))),
        material=chamber_dark,
        name="chamber_left_flange",
    )
    body.visual(
        Box((0.008, 0.018, CHAMBER_HEIGHT)),
        origin=Origin(xyz=(0.004, 0.131, CHAMBER_BOTTOM + (CHAMBER_HEIGHT * 0.5))),
        material=chamber_dark,
        name="chamber_right_flange",
    )
    body.visual(
        Box((0.008, OPENING_WIDTH, 0.026)),
        origin=Origin(xyz=(0.004, 0.0, OPENING_BOTTOM + OPENING_HEIGHT - 0.013)),
        material=chamber_dark,
        name="chamber_top_flange",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_pan_shape(), "air_fryer_drawer_pan"),
        material=basket_dark,
        name="drawer_pan",
    )
    drawer.visual(
        Box((0.230, 0.0035, 0.008)),
        origin=Origin(xyz=(-0.140, -0.11875, 0.104)),
        material=basket_dark,
        name="drawer_rail_0",
    )
    drawer.visual(
        Box((0.230, 0.0035, 0.008)),
        origin=Origin(xyz=(-0.140, 0.11875, 0.104)),
        material=basket_dark,
        name="drawer_rail_1",
    )
    drawer.visual(
        mesh_from_cadquery(_drawer_front_panel_shape(), "air_fryer_drawer_front_panel"),
        material=stainless,
        name="front_panel",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_insert_shape(), "air_fryer_basket_insert"),
        material=basket_dark,
        name="basket_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "air_fryer_handle"),
        material=handle_black,
        name="handle_shell",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(_latch_button_shape(), "air_fryer_latch_button"),
        material=button_grey,
        name="latch_cap",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.054,
                0.020,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "air_fryer_jog_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_black,
        name="dial_cap",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=start_green,
        name="start_cap",
    )

    stop_button = model.part("stop_button")
    stop_button.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stop_red,
        name="stop_cap",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.030, 0.0, CHAMBER_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=-0.030, upper=0.180),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-0.018, 0.0, 0.014)),
    )
    model.articulation(
        "drawer_to_handle",
        ArticulationType.FIXED,
        parent=drawer,
        child=handle,
        origin=Origin(xyz=(0.022, 0.0, 0.056)),
    )
    model.articulation(
        "handle_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=latch_button,
        origin=Origin(xyz=(0.020, 0.0, 0.048)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.012, 0.0, 0.271)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.012, -0.054, 0.232)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.0035),
    )
    model.articulation(
        "body_to_stop_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stop_button,
        origin=Origin(xyz=(0.012, 0.054, 0.232)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    start_button = object_model.get_part("start_button")
    stop_button = object_model.get_part("stop_button")
    latch_button = object_model.get_part("latch_button")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    start_joint = object_model.get_articulation("body_to_start_button")
    stop_joint = object_model.get_articulation("body_to_stop_button")
    latch_joint = object_model.get_articulation("handle_to_latch_button")
    dial_joint = object_model.get_articulation("body_to_dial")

    drawer_limits = drawer_joint.motion_limits
    if drawer_limits is not None and drawer_limits.lower is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_limits.lower}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_pan",
                margin=0.010,
                name="drawer pan stays guided in the chamber when seated",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_pan",
                min_overlap=0.290,
                name="drawer remains deeply inserted when seated",
            )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_pan",
                margin=0.012,
                name="drawer pan stays aligned while extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_pan",
                min_overlap=0.090,
                name="drawer retains insertion at full extension",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            "drawer extends forward",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.15,
            details=f"rest={rest_pos!r}, extended={extended_pos!r}",
        )

    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        inner_elem="basket_shell",
        outer_elem="drawer_pan",
        margin=0.012,
        name="basket insert nests inside the drawer pan",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="x",
        elem_a="basket_shell",
        elem_b="drawer_pan",
        min_overlap=0.220,
        name="basket insert remains retained in the drawer pan",
    )

    start_rest = ctx.part_world_position(start_button)
    start_limits = start_joint.motion_limits
    if start_limits is not None and start_limits.upper is not None:
        with ctx.pose({start_joint: start_limits.upper}):
            start_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses inward",
            start_rest is not None and start_pressed is not None and start_pressed[0] < start_rest[0] - 0.002,
            details=f"rest={start_rest!r}, pressed={start_pressed!r}",
        )

    stop_rest = ctx.part_world_position(stop_button)
    stop_limits = stop_joint.motion_limits
    if stop_limits is not None and stop_limits.upper is not None:
        with ctx.pose({stop_joint: stop_limits.upper}):
            stop_pressed = ctx.part_world_position(stop_button)
        ctx.check(
            "stop button presses inward",
            stop_rest is not None and stop_pressed is not None and stop_pressed[0] < stop_rest[0] - 0.002,
            details=f"rest={stop_rest!r}, pressed={stop_pressed!r}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    latch_limits = latch_joint.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({latch_joint: latch_limits.upper}):
            latch_pressed = ctx.part_world_position(latch_button)
        ctx.check(
            "latch button presses downward",
            latch_rest is not None and latch_pressed is not None and latch_pressed[2] < latch_rest[2] - 0.003,
            details=f"rest={latch_rest!r}, pressed={latch_pressed!r}",
        )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "jog dial is continuous",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
