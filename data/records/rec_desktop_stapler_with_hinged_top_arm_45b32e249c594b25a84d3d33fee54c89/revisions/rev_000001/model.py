from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.036
HANDLE_WIDTH = 0.029
BASE_DECK_Z = -0.0065
BASE_FRONT_X = 0.136
HANDLE_BARREL_RADIUS = 0.0042
HANDLE_BARREL_LENGTH = 0.017

FOLLOWER_CENTER_X = 0.084
FOLLOWER_CENTER_Z = 0.0034
FOLLOWER_TRAVEL = 0.050
FOLLOWER_BODY_LENGTH = 0.016
FOLLOWER_BODY_WIDTH = 0.0102
FOLLOWER_BODY_HEIGHT = 0.0036

PAPER_RAIL_LENGTH = 0.052
PAPER_RAIL_WIDTH = 0.0074
PAPER_RAIL_HEIGHT = 0.0022
PAPER_RAIL_CENTER_X = 0.090
PAPER_STOP_HOME_X = 0.076
PAPER_STOP_TRAVEL = 0.030


def _build_base_shape() -> cq.Workplane:
    body_profile = [
        (-0.010, -0.033),
        (-0.010, -0.012),
        (-0.002, -0.005),
        (0.025, -0.0065),
        (0.090, -0.0065),
        (0.122, -0.0115),
        (BASE_FRONT_X, -0.020),
        (BASE_FRONT_X, -0.030),
        (0.095, -0.032),
        (0.010, -0.033),
    ]
    body = cq.Workplane("XZ").polyline(body_profile).close().extrude(BASE_WIDTH / 2.0, both=True)

    rail = cq.Workplane("XY").box(
        PAPER_RAIL_LENGTH,
        PAPER_RAIL_WIDTH,
        PAPER_RAIL_HEIGHT,
    ).translate((PAPER_RAIL_CENTER_X, 0.0, BASE_DECK_Z + (PAPER_RAIL_HEIGHT / 2.0)))

    rear_saddle = cq.Workplane("XY").box(
        0.016,
        0.020,
        0.004,
    ).translate((0.004, 0.0, -0.0045))

    anvil = cq.Workplane("XY").box(
        0.020,
        0.020,
        0.0012,
    ).translate((0.121, 0.0, BASE_DECK_Z + 0.0006))

    nose_pad = cq.Workplane("XY").box(
        0.016,
        0.026,
        0.003,
    ).translate((0.123, 0.0, BASE_DECK_Z - 0.0015))

    return body.union(rail).union(rear_saddle).union(anvil).union(nose_pad)


def _build_handle_shape() -> cq.Workplane:
    shell_profile = [
        (0.010, -0.002),
        (0.015, 0.005),
        (0.032, 0.014),
        (0.060, 0.019),
        (0.098, 0.0175),
        (0.122, 0.013),
        (0.132, 0.007),
        (0.128, 0.0015),
        (0.090, -0.0015),
        (0.028, -0.0025),
    ]
    shell = cq.Workplane("XZ").polyline(shell_profile).close().extrude(HANDLE_WIDTH / 2.0, both=True)

    connector = cq.Workplane("XY").box(
        0.012,
        0.018,
        0.007,
    ).translate((0.006, 0.0, 0.001))

    barrel = (
        cq.Workplane("XZ")
        .circle(HANDLE_BARREL_RADIUS)
        .extrude(HANDLE_BARREL_LENGTH / 2.0, both=True)
        .translate((0.0, 0.0, 0.0))
    )

    body = shell.union(connector).union(barrel)

    lower_cavity = cq.Workplane("XY").box(
        0.104,
        0.0215,
        0.0105,
    ).translate((0.068, 0.0, 0.0014))
    top_slot = cq.Workplane("XY").box(
        0.062,
        0.0048,
        0.018,
    ).translate((0.058, 0.0, 0.010))
    nose_slot = cq.Workplane("XY").box(
        0.012,
        0.018,
        0.007,
    ).translate((0.126, 0.0, 0.0005))

    return body.cut(lower_cavity).cut(top_slot).cut(nose_slot)


def _build_follower_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        FOLLOWER_BODY_LENGTH,
        FOLLOWER_BODY_WIDTH,
        FOLLOWER_BODY_HEIGHT,
    )
    pusher = cq.Workplane("XY").box(
        0.0022,
        FOLLOWER_BODY_WIDTH * 0.92,
        0.006,
    ).translate((FOLLOWER_BODY_LENGTH / 2.0 - 0.0011, 0.0, -0.0002))
    tab = cq.Workplane("XY").box(
        0.006,
        0.0048,
        0.009,
    ).translate((-0.004, 0.0, 0.005))
    return body.union(pusher).union(tab)


def _build_paper_stop_shape() -> cq.Workplane:
    saddle = cq.Workplane("XY").box(
        0.016,
        0.018,
        0.0036,
    ).translate((0.0, 0.0, 0.0018))
    rail_relief = cq.Workplane("XY").box(
        0.022,
        PAPER_RAIL_WIDTH + 0.0012,
        0.0028,
    ).translate((0.0, 0.0, 0.0014))
    stop_fence = cq.Workplane("XY").box(
        0.003,
        0.016,
        0.0048,
    ).translate((0.0055, 0.0, 0.0024))
    finger_pad = cq.Workplane("XY").box(
        0.006,
        0.010,
        0.0018,
    ).translate((-0.0015, 0.0, 0.0045))
    return saddle.cut(rail_relief).union(stop_fence).union(finger_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_strip_stapler")

    model.material("body_red", rgba=(0.66, 0.10, 0.14, 1.0))
    model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("dark_steel", rgba=(0.40, 0.42, 0.46, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_shell"),
        material="body_red",
        name="base_shell",
    )
    base.visual(
        Box((0.020, 0.020, 0.0015)),
        origin=Origin(xyz=(0.121, 0.0, BASE_DECK_Z + 0.00075)),
        material="steel",
        name="anvil_plate",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "handle_shell"),
        material="body_red",
        name="handle_shell",
    )

    follower = model.part("follower")
    follower.visual(
        mesh_from_cadquery(_build_follower_shape(), "follower_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_steel",
        name="follower_body",
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        mesh_from_cadquery(_build_paper_stop_shape(), "paper_stop_body"),
        material="trim_black",
        name="paper_stop_body",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=18.0, velocity=3.0),
    )
    model.articulation(
        "follower_slide",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=follower,
        origin=Origin(xyz=(FOLLOWER_CENTER_X, 0.0, FOLLOWER_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FOLLOWER_TRAVEL, effort=8.0, velocity=0.25),
    )
    model.articulation(
        "paper_stop_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_stop,
        origin=Origin(xyz=(PAPER_STOP_HOME_X, 0.0, BASE_DECK_Z + 0.0006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=PAPER_STOP_TRAVEL, effort=3.0, velocity=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    follower = object_model.get_part("follower")
    paper_stop = object_model.get_part("paper_stop")

    handle_hinge = object_model.get_articulation("handle_hinge")
    follower_slide = object_model.get_articulation("follower_slide")
    paper_stop_slide = object_model.get_articulation("paper_stop_slide")

    handle_limits = handle_hinge.motion_limits
    follower_limits = follower_slide.motion_limits
    paper_stop_limits = paper_stop_slide.motion_limits

    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_hinge: handle_limits.lower}):
            ctx.expect_overlap(
                handle,
                base,
                axes="xy",
                min_overlap=0.020,
                name="closed handle covers the stapler deck",
            )
            closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")

        with ctx.pose({handle_hinge: handle_limits.upper}):
            opened_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")

        ctx.check(
            "handle opens upward",
            closed_handle_aabb is not None
            and opened_handle_aabb is not None
            and opened_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.025,
            details=f"closed={closed_handle_aabb}, opened={opened_handle_aabb}",
        )

    if follower_limits is not None and follower_limits.lower is not None and follower_limits.upper is not None:
        ctx.allow_overlap(
            handle,
            follower,
            elem_a="handle_shell",
            elem_b="follower_body",
            reason="The staple follower is intentionally represented as a nested magazine slider inside the simplified upper track shell.",
        )

        with ctx.pose({follower_slide: follower_limits.lower}):
            follower_home = ctx.part_world_position(follower)
            ctx.expect_within(
                follower,
                handle,
                axes="yz",
                margin=0.0,
                name="follower stays centered in the handle track at rest",
            )

        with ctx.pose({follower_slide: follower_limits.upper}):
            follower_retracted = ctx.part_world_position(follower)
            ctx.expect_within(
                follower,
                handle,
                axes="yz",
                margin=0.0,
                name="follower stays centered in the handle track when retracted",
            )

        ctx.check(
            "follower retracts toward the hinge",
            follower_home is not None
            and follower_retracted is not None
            and follower_retracted[0] < follower_home[0] - 0.030,
            details=f"home={follower_home}, retracted={follower_retracted}",
        )

    if paper_stop_limits is not None and paper_stop_limits.lower is not None and paper_stop_limits.upper is not None:
        with ctx.pose({paper_stop_slide: paper_stop_limits.lower}):
            stop_home = ctx.part_world_position(paper_stop)
            ctx.expect_within(
                paper_stop,
                base,
                axes="y",
                margin=0.0,
                name="paper stop stays centered on the deck rail at minimum depth",
            )

        with ctx.pose({paper_stop_slide: paper_stop_limits.upper}):
            stop_forward = ctx.part_world_position(paper_stop)
            ctx.expect_within(
                paper_stop,
                base,
                axes="y",
                margin=0.0,
                name="paper stop stays centered on the deck rail at maximum depth",
            )

        ctx.check(
            "paper stop slides toward the nose",
            stop_home is not None
            and stop_forward is not None
            and stop_forward[0] > stop_home[0] + 0.020,
            details=f"home={stop_home}, forward={stop_forward}",
        )

    return ctx.report()


object_model = build_object_model()
