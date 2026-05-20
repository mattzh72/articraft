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


SPOUT_OUTER_RADIUS = 0.016
SPOUT_INNER_RADIUS = 0.012
HEAD_INSERT_RADIUS = 0.0065

SPOUT_SOCKET_START = (0.182, 0.0, 0.180)
SPOUT_SOCKET_END = (0.199, 0.0, 0.126)


def _rotation_about_y_deg(direction: tuple[float, float, float]) -> float:
    return math.degrees(math.atan2(direction[0], direction[2]))


def _direction_and_length(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return ((dx / length, dy / length, dz / length), length)


def _build_body_shape() -> cq.Workplane:
    deck_plate = cq.Workplane("XY").rect(0.068, 0.022).extrude(0.008)
    stem = cq.Workplane("XY").circle(0.024).extrude(0.094)
    swivel_collar = cq.Workplane("XY").circle(0.029).extrude(0.014).translate((0.0, 0.0, 0.094))
    handle_boss = (
        cq.Workplane("XZ")
        .center(0.013, 0.070)
        .circle(0.010)
        .extrude(0.012)
        .translate((0.0, 0.024, 0.0))
    )
    return deck_plate.union(stem).union(swivel_collar).union(handle_boss)


def _build_spout_shell() -> cq.Workplane:
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .spline(
            [
                (0.0, 0.092),
                (0.030, 0.216),
                (0.118, 0.256),
                (SPOUT_SOCKET_START[0], SPOUT_SOCKET_START[2]),
            ]
        )
    )
    neck = cq.Workplane("XY").circle(SPOUT_OUTER_RADIUS).circle(SPOUT_INNER_RADIUS).sweep(path)
    swivel_sleeve = cq.Workplane("XY").circle(SPOUT_OUTER_RADIUS).circle(SPOUT_INNER_RADIUS).extrude(0.018)
    return swivel_sleeve.union(neck)


def _build_tip_socket() -> cq.Workplane:
    direction, length = _direction_and_length(SPOUT_SOCKET_START, SPOUT_SOCKET_END)
    inset = 0.020
    start = (
        SPOUT_SOCKET_START[0] - direction[0] * inset,
        SPOUT_SOCKET_START[1] - direction[1] * inset,
        SPOUT_SOCKET_START[2] - direction[2] * inset,
    )
    return (
        cq.Workplane("XY")
        .circle(SPOUT_OUTER_RADIUS)
        .circle(SPOUT_INNER_RADIUS)
        .extrude(length + inset)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), _rotation_about_y_deg(direction))
        .translate(start)
    )


def _build_handle_shape() -> cq.Workplane:
    pivot_cap = cq.Workplane("XZ").circle(0.0072).extrude(0.010).translate((0.0, 0.010, 0.0))
    arm = (
        cq.Workplane("XY")
        .circle(0.0048)
        .extrude(0.052)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -66.0)
        .translate((0.0, 0.0, 0.001))
    )
    paddle = (
        cq.Workplane("XY")
        .box(0.011, 0.026, 0.007)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0)
        .translate((0.0, 0.050, 0.022))
    )
    return pivot_cap.union(arm).union(paddle)


def _build_head_shell() -> cq.Workplane:
    seat = cq.Workplane("XY").circle(0.014).extrude(0.003)
    neck = cq.Workplane("XY").circle(0.013).extrude(0.015).translate((0.0, 0.0, 0.003))
    bell = cq.Workplane("XY").circle(0.018).extrude(0.030).translate((0.0, 0.0, 0.018))
    nose = cq.Workplane("XY").circle(0.016).extrude(0.008).translate((0.0, 0.0, 0.048))
    return seat.union(neck).union(bell).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pull_out_kitchen_faucet")

    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=chrome,
        name="body_shell",
    )

    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shell(), "spout_shell"),
        material=chrome,
        name="spout_shell",
    )
    spout.visual(
        mesh_from_cadquery(_build_tip_socket(), "tip_socket"),
        material=chrome,
        name="tip_socket",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "handle_shell"),
        material=chrome,
        name="handle_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(HEAD_INSERT_RADIUS).extrude(0.123).translate((0.0, 0.0, -0.118)),
            "insert_shank",
        ),
        material=dark_trim,
        name="insert_shank",
    )
    head.visual(
        mesh_from_cadquery(_build_head_shell(), "head_shell"),
        material=chrome,
        name="head_shell",
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.013, 0.0259, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-0.28,
            upper=0.82,
        ),
    )

    socket_direction, _ = _direction_and_length(SPOUT_SOCKET_START, SPOUT_SOCKET_END)
    model.articulation(
        "spout_to_head",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=head,
        origin=Origin(
            xyz=SPOUT_SOCKET_END,
            rpy=(0.0, math.atan2(socket_direction[0], socket_direction[2]), 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.28,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    head = object_model.get_part("head")

    spout_swivel = object_model.get_articulation("body_to_spout")
    handle_pivot = object_model.get_articulation("body_to_handle")
    head_slide = object_model.get_articulation("spout_to_head")

    ctx.allow_overlap(
        head,
        spout,
        elem_a="head_shell",
        elem_b="tip_socket",
        reason="The detachable spray head docks into a simplified tip socket proxy, so the parked shell nests slightly into the socket lip.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="body_shell",
        elem_b="handle_shell",
        reason="The side lever root is simplified as a pivot cap seated into a solid body boss proxy, so the parked lever slightly embeds at the mount.",
    )

    ctx.expect_overlap(
        head,
        spout,
        axes="z",
        elem_a="insert_shank",
        elem_b="tip_socket",
        min_overlap=0.025,
        name="parked spray head remains inserted in the tip socket",
    )

    parked_head_pos = ctx.part_world_position(head)
    with ctx.pose({head_slide: head_slide.motion_limits.upper}):
        ctx.expect_overlap(
            head,
            spout,
            axes="z",
            elem_a="insert_shank",
            elem_b="tip_socket",
            min_overlap=0.008,
            name="extended spray head retains insertion at full travel",
        )
        pulled_head_pos = ctx.part_world_position(head)

    ctx.check(
        "spray head pulls downward from the spout tip",
        parked_head_pos is not None
        and pulled_head_pos is not None
        and pulled_head_pos[2] < parked_head_pos[2] - 0.055,
        details=f"parked={parked_head_pos}, pulled={pulled_head_pos}",
    )

    parked_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: handle_pivot.motion_limits.upper}):
        lifted_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "side lever lifts on its short pivot",
        parked_handle_aabb is not None
        and lifted_handle_aabb is not None
        and lifted_handle_aabb[1][2] > parked_handle_aabb[1][2] + 0.018,
        details=f"parked={parked_handle_aabb}, lifted={lifted_handle_aabb}",
    )

    parked_tip_pos = ctx.part_world_position(head)
    with ctx.pose({spout_swivel: math.pi / 2.0}):
        turned_tip_pos = ctx.part_world_position(head)

    ctx.check(
        "spout swivels around the base axis",
        parked_tip_pos is not None
        and turned_tip_pos is not None
        and abs(turned_tip_pos[0] - parked_tip_pos[0]) > 0.10
        and abs(turned_tip_pos[1] - parked_tip_pos[1]) > 0.10,
        details=f"parked={parked_tip_pos}, turned={turned_tip_pos}",
    )

    return ctx.report()


object_model = build_object_model()
