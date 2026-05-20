from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.108
BODY_WIDTH = 0.088
BODY_HEIGHT = 0.148
BASE_DEPTH = 0.122
BASE_WIDTH = 0.100
BASE_HEIGHT = 0.010
WALL = 0.004
BODY_FRONT_X = BODY_DEPTH / 2.0
BODY_BACK_X = -BODY_DEPTH / 2.0

PORT_Y = 0.008
PORT_Z = 0.101
PORT_RADIUS = 0.0065
PORT_COLLAR_RADIUS = 0.012

SELECTOR_Y = -0.022
SELECTOR_Z_LOWER = 0.096
SELECTOR_TRAVEL = 0.014

DRAWER_LENGTH = 0.078
DRAWER_WIDTH = 0.068
DRAWER_HEIGHT = 0.038
DRAWER_OPENING_WIDTH = 0.072
DRAWER_OPENING_HEIGHT = 0.042
DRAWER_BOTTOM_Z = 0.013
DRAWER_REST_X = BODY_FRONT_X - 0.002 - DRAWER_LENGTH
DRAWER_TRAVEL = 0.040

CRANK_Z = 0.084
CRANK_JOINT_X = BODY_BACK_X - 0.005


def x_cylinder(radius: float, length: float, *, positive: bool) -> cq.Workplane:
    angle = 90 if positive else -90
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
    )


def y_cylinder(radius: float, length: float, *, positive: bool) -> cq.Workplane:
    angle = -90 if positive else 90
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
    )


def make_body_shape() -> cq.Workplane:
    housing_outer = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BODY_BACK_X, BASE_HEIGHT - 0.001),
                (BODY_BACK_X, BODY_HEIGHT),
                (-0.012, BODY_HEIGHT),
                (BODY_FRONT_X, 0.124),
                (BODY_FRONT_X, BASE_HEIGHT - 0.001),
            ]
        )
        .close()
        .extrude(BODY_WIDTH / 2.0, both=True)
    )

    base_plinth = cq.Workplane("XY").box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT / 2.0)
    )

    cavity = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BODY_BACK_X + WALL, BASE_HEIGHT + 0.004),
                (BODY_BACK_X + WALL, BODY_HEIGHT - WALL),
                (-0.010, BODY_HEIGHT - WALL),
                (BODY_FRONT_X - WALL, 0.119),
                (BODY_FRONT_X - WALL, BASE_HEIGHT + 0.004),
            ]
        )
        .close()
        .extrude((BODY_WIDTH - 2.0 * WALL) / 2.0, both=True)
    )

    drawer_cut = cq.Workplane("XY").box(
        0.030,
        DRAWER_OPENING_WIDTH,
        DRAWER_OPENING_HEIGHT,
    ).translate((BODY_FRONT_X - 0.011, 0.0, DRAWER_BOTTOM_Z + DRAWER_OPENING_HEIGHT / 2.0))
    drawer_bay = cq.Workplane("XY").box(
        DRAWER_LENGTH + 0.006,
        DRAWER_OPENING_WIDTH + 0.002,
        DRAWER_OPENING_HEIGHT + 0.004,
    ).translate(
        (
            (DRAWER_REST_X - 0.004 + BODY_FRONT_X + 0.002) / 2.0,
            0.0,
            DRAWER_BOTTOM_Z + (DRAWER_OPENING_HEIGHT + 0.004) / 2.0,
        )
    )

    selector_recess = cq.Workplane("XY").box(0.004, 0.016, 0.032).translate(
        (BODY_FRONT_X - 0.002, SELECTOR_Y, SELECTOR_Z_LOWER + SELECTOR_TRAVEL / 2.0)
    )
    selector_slot = cq.Workplane("XY").box(0.020, 0.005, 0.024).translate(
        (BODY_FRONT_X - 0.009, SELECTOR_Y, SELECTOR_Z_LOWER + SELECTOR_TRAVEL / 2.0)
    )

    port_collar = x_cylinder(PORT_COLLAR_RADIUS, 0.005, positive=True).translate(
        (BODY_FRONT_X - 0.001, PORT_Y, PORT_Z)
    )
    port_sleeve = x_cylinder(PORT_COLLAR_RADIUS * 0.92, 0.026, positive=False).translate(
        (BODY_FRONT_X - 0.001, PORT_Y, PORT_Z)
    )
    port_hole = x_cylinder(PORT_RADIUS, 0.034, positive=False).translate(
        (BODY_FRONT_X + 0.002, PORT_Y, PORT_Z)
    )

    crank_boss = x_cylinder(0.011, 0.005, positive=False).translate(
        (BODY_BACK_X, 0.0, CRANK_Z)
    )

    body = housing_outer.union(base_plinth)
    body = body.cut(cavity)
    body = body.cut(drawer_cut)
    body = body.cut(drawer_bay)
    body = body.cut(selector_recess)
    body = body.cut(selector_slot)
    body = body.union(port_collar).union(port_sleeve).union(crank_boss)
    body = body.cut(port_hole)
    return body


def make_drawer_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").box(
        DRAWER_LENGTH,
        DRAWER_WIDTH - 0.006,
        0.003,
    ).translate((DRAWER_LENGTH / 2.0, 0.0, 0.0015))
    left_wall = cq.Workplane("XY").box(DRAWER_LENGTH, 0.003, DRAWER_HEIGHT).translate(
        (DRAWER_LENGTH / 2.0, (DRAWER_WIDTH - 0.003) / 2.0, DRAWER_HEIGHT / 2.0)
    )
    right_wall = cq.Workplane("XY").box(DRAWER_LENGTH, 0.003, DRAWER_HEIGHT).translate(
        (DRAWER_LENGTH / 2.0, -(DRAWER_WIDTH - 0.003) / 2.0, DRAWER_HEIGHT / 2.0)
    )
    front_wall = cq.Workplane("XY").box(0.004, DRAWER_WIDTH, DRAWER_HEIGHT).translate(
        (DRAWER_LENGTH - 0.002, 0.0, DRAWER_HEIGHT / 2.0)
    )
    pull = cq.Workplane("XY").box(0.010, 0.040, 0.010).translate(
        (DRAWER_LENGTH + 0.005, 0.0, DRAWER_HEIGHT * 0.55)
    )

    return floor.union(left_wall).union(right_wall).union(front_wall).union(pull)


def make_selector_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(0.010, 0.014, 0.008)
    stem = cq.Workplane("XY").box(0.012, 0.004, 0.004).translate((-0.010, 0.0, 0.0))
    keeper = cq.Workplane("XY").box(0.004, 0.009, 0.012).translate((-0.015, 0.0, 0.0))
    return pad.union(stem).union(keeper)


def make_crank_shape() -> cq.Workplane:
    hub = x_cylinder(0.0075, 0.018, positive=False)
    arm = y_cylinder(0.0032, 0.050, positive=False).translate((-0.018, 0.0, 0.0))
    handle_post = cq.Workplane("XY").circle(0.003).extrude(0.028).translate(
        (-0.018, -0.050, -0.014)
    )
    knob = cq.Workplane("XY").circle(0.007).extrude(0.018).translate(
        (-0.018, -0.050, -0.009)
    )
    return hub.union(arm).union(handle_post).union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    enamel_red = model.material("enamel_red", rgba=(0.50, 0.10, 0.12, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.12, 0.12, 0.13, 1.0))
    selector_black = model.material("selector_black", rgba=(0.10, 0.10, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shape(), "sharpener_body"),
        material=enamel_red,
        name="housing",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(make_drawer_shape(), "sharpener_drawer"),
        material=drawer_black,
        name="drawer",
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_cadquery(make_selector_shape(), "sharpener_selector"),
        material=selector_black,
        name="selector",
    )

    crank = model.part("crank")
    crank.visual(
        mesh_from_cadquery(make_crank_shape(), "sharpener_crank"),
        material=steel,
        name="crank",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_REST_X, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=35.0,
            velocity=0.18,
        ),
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.PRISMATIC,
        parent=body,
        child=selector,
        origin=Origin(xyz=(BODY_FRONT_X, SELECTOR_Y + 0.001, SELECTOR_Z_LOWER)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SELECTOR_TRAVEL,
            effort=3.0,
            velocity=0.08,
        ),
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_JOINT_X, 0.0, CRANK_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    selector = object_model.get_part("selector")
    crank = object_model.get_part("crank")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    selector_joint = object_model.get_articulation("body_to_selector")
    crank_joint = object_model.get_articulation("body_to_crank")

    drawer_limits = drawer_joint.motion_limits
    selector_limits = selector_joint.motion_limits

    ctx.allow_overlap(
        body,
        drawer,
        reason="The shavings drawer is intentionally nested inside the sharpener's front collection bay.",
    )
    ctx.allow_overlap(
        body,
        selector,
        reason="The selector's hidden keeper is intentionally captured inside the short front guide slot.",
    )

    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        min_overlap=0.030,
        name="drawer stays centered in the front cavity footprint",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="y",
        min_overlap=0.006,
        name="selector remains beside the pencil port",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.030,
                name="drawer remains aligned when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.030,
                name="drawer keeps retained insertion at full extension",
            )
            open_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer pulls forward",
            closed_drawer_pos is not None
            and open_drawer_pos is not None
            and open_drawer_pos[0] > closed_drawer_pos[0] + 0.030,
            details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
        )

    closed_selector_pos = ctx.part_world_position(selector)
    if selector_limits is not None and selector_limits.upper is not None:
        with ctx.pose({selector_joint: selector_limits.upper}):
            high_selector_pos = ctx.part_world_position(selector)
            ctx.expect_overlap(
                selector,
                body,
                axes="y",
                min_overlap=0.006,
                name="selector stays on its guide at the upper stop",
            )
        ctx.check(
            "selector slides upward",
            closed_selector_pos is not None
            and high_selector_pos is not None
            and high_selector_pos[2] > closed_selector_pos[2] + 0.010,
            details=f"closed={closed_selector_pos}, high={high_selector_pos}",
        )

    rest_crank_aabb = ctx.part_element_world_aabb(crank, elem="crank")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        raised_crank_aabb = ctx.part_element_world_aabb(crank, elem="crank")
    ctx.check(
        "crank rotates around the rear axis",
        rest_crank_aabb is not None
        and raised_crank_aabb is not None
        and raised_crank_aabb[1][2] > rest_crank_aabb[1][2] + 0.020,
        details=f"rest={rest_crank_aabb}, raised={raised_crank_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
