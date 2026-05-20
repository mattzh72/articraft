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
    cadquery_local_aabb,
    mesh_from_cadquery,
)


DESK_WIDTH = 1.16
DESK_DEPTH = 0.70
DESK_HEIGHT = 0.77
KNEE_WIDTH = 0.60
KNEE_DEPTH = 0.50
KNEE_BASE = 0.07
KNEE_HEIGHT = 0.65
PEDESTAL_WIDTH = (DESK_WIDTH - KNEE_WIDTH) / 2.0

UPPER_WIDTH = 0.74
UPPER_DEPTH = 0.40
UPPER_HEIGHT = 0.23
UPPER_WALL = 0.035

DRAWER_Y = (KNEE_WIDTH / 2.0) + (PEDESTAL_WIDTH / 2.0)
DRAWER_BAY_WIDTH = 0.245
DRAWER_BAY_HEIGHT = 0.178
DRAWER_BAY_DEPTH = 0.525
DRAWER_BAY_BASE = 0.455
DRAWER_WIDTH = 0.233
DRAWER_HEIGHT = 0.166
DRAWER_DEPTH = 0.490
DRAWER_TRAVEL = 0.255

TAMBOUR_RADIUS = 0.19
TAMBOUR_THICKNESS = 0.017
TAMBOUR_WIDTH = UPPER_WIDTH - (2.0 * (UPPER_WALL + 0.006))
TAMBOUR_TRAVEL = 0.150


def front_box(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    front_x: float = 0.0,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        size_x,
        size_y,
        size_z,
        centered=(True, True, False),
    ).translate((front_x - (size_x / 2.0), center_y, base_z))


def quarter_shell(
    outer_radius: float,
    inner_radius: float,
    width_y: float,
) -> cq.Workplane:
    ring = cq.Workplane("XZ").circle(outer_radius).circle(inner_radius).extrude(width_y)
    quadrant = (
        cq.Workplane("XZ")
        .center(outer_radius / 2.0, outer_radius / 2.0)
        .rect(outer_radius + 0.04, outer_radius + 0.04)
        .extrude(width_y)
    )
    shell = ring.intersect(quadrant)
    aabb_min, aabb_max = cadquery_local_aabb(shell)
    center_y = (aabb_min[1] + aabb_max[1]) / 2.0
    return shell.translate((-aabb_max[0], -center_y, -aabb_min[2]))


def build_body_shape() -> cq.Workplane:
    body = front_box(DESK_DEPTH, DESK_WIDTH, DESK_HEIGHT)

    body = body.cut(
        front_box(
            KNEE_DEPTH,
            KNEE_WIDTH,
            KNEE_HEIGHT,
            front_x=0.006,
            base_z=KNEE_BASE,
        )
    )

    for drawer_center_y in (-DRAWER_Y, DRAWER_Y):
        body = body.cut(
            front_box(
                DRAWER_BAY_DEPTH,
                DRAWER_BAY_WIDTH,
                DRAWER_BAY_HEIGHT,
                front_x=0.006,
                center_y=drawer_center_y,
                base_z=DRAWER_BAY_BASE,
            )
        )
        for side_sign in (-1.0, 1.0):
            body = body.cut(
                front_box(
                    DRAWER_BAY_DEPTH - 0.025,
                    0.012,
                    0.016,
                    front_x=-0.002,
                    center_y=drawer_center_y
                    + side_sign * ((DRAWER_BAY_WIDTH / 2.0) - 0.004),
                    base_z=DRAWER_BAY_BASE + 0.067,
                )
            )

    upper_shell = front_box(
        UPPER_DEPTH,
        UPPER_WIDTH,
        UPPER_HEIGHT,
        base_z=DESK_HEIGHT,
    )
    body = body.union(upper_shell)

    body = body.cut(
        front_box(
            UPPER_DEPTH - 0.050,
            UPPER_WIDTH - (2.0 * UPPER_WALL),
            UPPER_HEIGHT - 0.006,
            front_x=0.005,
            base_z=DESK_HEIGHT - 0.004,
        )
    )

    track_outer = TAMBOUR_RADIUS + 0.004
    track_inner = TAMBOUR_RADIUS - TAMBOUR_THICKNESS - 0.004
    track_depth = UPPER_WALL + 0.010
    track = quarter_shell(track_outer, track_inner, track_depth)

    body = body.cut(
        track.translate(
            (0.0, -(UPPER_WIDTH / 2.0) + (track_depth / 2.0), DESK_HEIGHT)
        )
    )
    body = body.cut(
        track.translate(
            (0.0, (UPPER_WIDTH / 2.0) - (track_depth / 2.0), DESK_HEIGHT)
        )
    )

    return body


def build_tambour_shape() -> cq.Workplane:
    tambour = quarter_shell(
        TAMBOUR_RADIUS,
        TAMBOUR_RADIUS - TAMBOUR_THICKNESS,
        TAMBOUR_WIDTH,
    )

    groove_count = 11
    groove_radius = TAMBOUR_RADIUS - (TAMBOUR_THICKNESS * 0.45)
    for index in range(1, groove_count + 1):
        angle = (index / (groove_count + 1)) * (math.pi / 2.0)
        x = -TAMBOUR_RADIUS + (math.cos(angle) * groove_radius)
        z = math.sin(angle) * groove_radius
        groove = (
            cq.Workplane("XY")
            .box(
                0.010,
                TAMBOUR_WIDTH + 0.020,
                TAMBOUR_THICKNESS * 0.75,
                centered=(True, True, True),
            )
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(angle))
            .translate((x, 0.0, z))
        )
        tambour = tambour.cut(groove)

    pull = front_box(
        0.014,
        0.120,
        0.012,
        front_x=0.010,
        base_z=0.050,
    )

    tambour = tambour.union(pull)

    shoe_center_y = (TAMBOUR_WIDTH / 2.0) + 0.004
    for side_sign in (-1.0, 1.0):
        tambour = tambour.union(
            front_box(
                0.022,
                0.012,
                0.060,
                front_x=-0.030,
                center_y=side_sign * shoe_center_y,
                base_z=0.070,
            )
        )

    return tambour


def build_drawer_shape() -> cq.Workplane:
    wall = 0.010
    front_panel_thickness = 0.020

    outer = front_box(DRAWER_DEPTH, DRAWER_WIDTH, DRAWER_HEIGHT)
    cavity = front_box(
        DRAWER_DEPTH - front_panel_thickness - 0.018,
        DRAWER_WIDTH - (2.0 * wall),
        DRAWER_HEIGHT - wall,
        front_x=-(front_panel_thickness + 0.009),
        base_z=wall,
    )
    drawer = outer.cut(cavity)

    drawer_front = front_box(
        front_panel_thickness,
        DRAWER_WIDTH + 0.008,
        DRAWER_HEIGHT + 0.002,
        front_x=0.0,
        base_z=0.0,
    )
    drawer = drawer.union(drawer_front)

    pull = front_box(
        0.016,
        0.090,
        0.016,
        front_x=0.016,
        base_z=(DRAWER_HEIGHT / 2.0) - 0.008,
    )

    return drawer.union(pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_top_desk")

    dark_oak = model.material("dark_oak", rgba=(0.34, 0.24, 0.15, 1.0))
    medium_oak = model.material("medium_oak", rgba=(0.46, 0.33, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shape(), "roll_top_desk_body"),
        material=dark_oak,
        name="carcass",
    )

    tambour = model.part("tambour")
    tambour.visual(
        mesh_from_cadquery(build_tambour_shape(), "roll_top_desk_tambour"),
        material=medium_oak,
        name="cover",
    )

    left_drawer = model.part("left_drawer")
    left_drawer.visual(
        mesh_from_cadquery(build_drawer_shape(), "roll_top_desk_left_drawer"),
        material=medium_oak,
        name="box",
    )

    right_drawer = model.part("right_drawer")
    right_drawer.visual(
        mesh_from_cadquery(build_drawer_shape(), "roll_top_desk_right_drawer"),
        material=medium_oak,
        name="box",
    )

    model.articulation(
        "body_to_tambour",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=TAMBOUR_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_left_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_drawer,
        origin=Origin(xyz=(0.0, -DRAWER_Y, DRAWER_BAY_BASE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_right_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_drawer,
        origin=Origin(xyz=(0.0, DRAWER_Y, DRAWER_BAY_BASE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tambour = object_model.get_part("tambour")
    left_drawer = object_model.get_part("left_drawer")
    right_drawer = object_model.get_part("right_drawer")

    ctx.allow_overlap(
        body,
        left_drawer,
        elem_a="carcass",
        elem_b="box",
        reason=(
            "The pedestal carcass is represented as a simplified single shell mesh, "
            "while the left drawer is authored as a retained sliding member nested "
            "inside that proxy opening."
        ),
    )
    ctx.allow_overlap(
        body,
        right_drawer,
        elem_a="carcass",
        elem_b="box",
        reason=(
            "The pedestal carcass is represented as a simplified single shell mesh, "
            "while the right drawer is authored as a retained sliding member nested "
            "inside that proxy opening."
        ),
    )
    ctx.allow_overlap(
        body,
        tambour,
        elem_a="carcass",
        elem_b="cover",
        reason=(
            "The tambour is represented as a rigid curved shell sliding inside a "
            "simplified roll-top housing proxy, so the retained guide fit is "
            "intentionally approximated as a nested overlap."
        ),
    )

    tambour_slide = object_model.get_articulation("body_to_tambour")
    left_slide = object_model.get_articulation("body_to_left_drawer")
    right_slide = object_model.get_articulation("body_to_right_drawer")

    left_closed_pos = None
    right_closed_pos = None
    tambour_closed_pos = None

    with ctx.pose({tambour_slide: 0.0, left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_within(
            tambour,
            body,
            axes="yz",
            margin=0.012,
            name="closed tambour stays between the curved side guides",
        )
        ctx.expect_overlap(
            tambour,
            body,
            axes="x",
            min_overlap=0.20,
            name="closed tambour remains captured in the roll-top housing",
        )

        ctx.expect_within(
            left_drawer,
            body,
            axes="yz",
            margin=0.012,
            name="left drawer sits within the pedestal opening",
        )
        ctx.expect_within(
            right_drawer,
            body,
            axes="yz",
            margin=0.012,
            name="right drawer sits within the pedestal opening",
        )
        ctx.expect_overlap(
            left_drawer,
            body,
            axes="x",
            min_overlap=0.46,
            name="closed left drawer remains deeply inserted",
        )
        ctx.expect_overlap(
            right_drawer,
            body,
            axes="x",
            min_overlap=0.46,
            name="closed right drawer remains deeply inserted",
        )

        left_closed_pos = ctx.part_world_position(left_drawer)
        right_closed_pos = ctx.part_world_position(right_drawer)
        tambour_closed_pos = ctx.part_world_position(tambour)

    left_open_pos = None
    right_open_pos = None
    tambour_open_pos = None
    left_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    right_upper = right_slide.motion_limits.upper if right_slide.motion_limits is not None else None
    tambour_upper = (
        tambour_slide.motion_limits.upper
        if tambour_slide.motion_limits is not None
        else None
    )

    with ctx.pose(
        {
            tambour_slide: tambour_upper or 0.0,
            left_slide: left_upper or 0.0,
            right_slide: right_upper or 0.0,
        }
    ):
        ctx.expect_within(
            tambour,
            body,
            axes="yz",
            margin=0.012,
            name="open tambour still stays guided between the side tracks",
        )
        ctx.expect_overlap(
            tambour,
            body,
            axes="x",
            min_overlap=0.03,
            name="open tambour remains retained inside the upper housing",
        )
        ctx.expect_within(
            left_drawer,
            body,
            axes="yz",
            margin=0.012,
            name="extended left drawer stays aligned on its channels",
        )
        ctx.expect_within(
            right_drawer,
            body,
            axes="yz",
            margin=0.012,
            name="extended right drawer stays aligned on its channels",
        )
        ctx.expect_overlap(
            left_drawer,
            body,
            axes="x",
            min_overlap=0.22,
            name="extended left drawer keeps retained insertion",
        )
        ctx.expect_overlap(
            right_drawer,
            body,
            axes="x",
            min_overlap=0.22,
            name="extended right drawer keeps retained insertion",
        )

        left_open_pos = ctx.part_world_position(left_drawer)
        right_open_pos = ctx.part_world_position(right_drawer)
        tambour_open_pos = ctx.part_world_position(tambour)

    ctx.check(
        "left drawer extends toward the desk front",
        left_closed_pos is not None
        and left_open_pos is not None
        and left_open_pos[0] > left_closed_pos[0] + 0.20,
        details=f"closed={left_closed_pos}, open={left_open_pos}",
    )
    ctx.check(
        "right drawer extends toward the desk front",
        right_closed_pos is not None
        and right_open_pos is not None
        and right_open_pos[0] > right_closed_pos[0] + 0.20,
        details=f"closed={right_closed_pos}, open={right_open_pos}",
    )
    ctx.check(
        "tambour slides rearward into the roll-top housing",
        tambour_closed_pos is not None
        and tambour_open_pos is not None
        and tambour_open_pos[0] < tambour_closed_pos[0] - 0.10,
        details=f"closed={tambour_closed_pos}, open={tambour_open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
