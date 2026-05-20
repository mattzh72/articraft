from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _tube_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
):
    origin, length = _tube_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    frame_metal = model.material("frame_metal", rgba=(0.13, 0.13, 0.14, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.24, 0.24, 0.26, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    center_frame = model.part("center_frame")
    rail_r = 0.017
    brace_r = 0.014
    handle_r = 0.012

    _add_tube(center_frame, "lower_rail_0", (-0.48, 0.16, 0.07), (0.46, 0.16, 0.07), radius=rail_r, material=frame_metal)
    _add_tube(center_frame, "lower_rail_1", (-0.48, -0.16, 0.07), (0.46, -0.16, 0.07), radius=rail_r, material=frame_metal)
    _add_tube(center_frame, "front_crossmember", (-0.48, -0.19, 0.07), (-0.48, 0.19, 0.07), radius=rail_r, material=frame_metal)
    _add_tube(center_frame, "rear_crossmember", (0.48, -0.21, 0.07), (0.48, 0.21, 0.07), radius=rail_r, material=frame_metal)

    _add_tube(center_frame, "front_brace_0", (-0.48, 0.16, 0.07), (-0.12, 0.16, 0.296), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "front_brace_1", (-0.48, -0.16, 0.07), (-0.12, -0.16, 0.296), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "seat_post_0", (-0.12, 0.16, 0.07), (-0.12, 0.16, 0.296), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "seat_post_1", (-0.12, -0.16, 0.07), (-0.12, -0.16, 0.296), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "seat_crossmember", (-0.12, -0.16, 0.296), (-0.12, 0.16, 0.296), radius=brace_r, material=frame_metal)

    _add_tube(center_frame, "back_post_0", (0.02, 0.16, 0.07), (0.02, 0.16, 0.352), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "back_post_1", (0.02, -0.16, 0.07), (0.02, -0.16, 0.352), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "back_crossmember", (0.02, -0.16, 0.352), (0.02, 0.16, 0.352), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "rear_brace_0", (0.02, 0.16, 0.352), (0.48, 0.16, 0.07), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "rear_brace_1", (0.02, -0.16, 0.352), (0.48, -0.16, 0.07), radius=brace_r, material=frame_metal)

    _add_tube(center_frame, "support_post_0", (0.24, 0.16, 0.07), (0.24, 0.16, 0.17), radius=brace_r, material=frame_metal)
    _add_tube(center_frame, "support_post_1", (0.24, -0.16, 0.07), (0.24, -0.16, 0.17), radius=brace_r, material=frame_metal)

    _add_tube(center_frame, "handle_support_0", (-0.48, 0.05, 0.07), (-0.56, 0.05, 0.10), radius=handle_r, material=frame_metal)
    _add_tube(center_frame, "handle_support_1", (-0.48, -0.05, 0.07), (-0.56, -0.05, 0.10), radius=handle_r, material=frame_metal)
    _add_tube(center_frame, "handle_bar", (-0.56, -0.05, 0.10), (-0.56, 0.05, 0.10), radius=handle_r, material=frame_metal)

    center_frame.visual(
        Box((0.10, 0.07, 0.03)),
        origin=Origin(xyz=(-0.07, 0.125, 0.325)),
        material=bracket_metal,
        name="seat_tab_0",
    )
    center_frame.visual(
        Box((0.10, 0.07, 0.03)),
        origin=Origin(xyz=(-0.07, -0.125, 0.325)),
        material=bracket_metal,
        name="seat_tab_1",
    )
    center_frame.visual(
        Box((0.04, 0.07, 0.03)),
        origin=Origin(xyz=(0.03, 0.125, 0.375)),
        material=bracket_metal,
        name="back_tab_0",
    )
    center_frame.visual(
        Box((0.04, 0.07, 0.03)),
        origin=Origin(xyz=(0.03, -0.125, 0.375)),
        material=bracket_metal,
        name="back_tab_1",
    )
    center_frame.visual(
        Box((0.03, 0.07, 0.03)),
        origin=Origin(xyz=(0.24, 0.125, 0.185)),
        material=bracket_metal,
        name="support_tab_0",
    )
    center_frame.visual(
        Box((0.03, 0.07, 0.03)),
        origin=Origin(xyz=(0.24, -0.125, 0.185)),
        material=bracket_metal,
        name="support_tab_1",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.017, length=0.18),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=bracket_metal,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.70, 0.24, 0.018)),
        origin=Origin(xyz=(0.35, 0.0, 0.026)),
        material=bracket_metal,
        name="support_plate",
    )
    backrest.visual(
        Box((0.70, 0.30, 0.075)),
        origin=Origin(xyz=(0.35, 0.0, 0.0725)),
        material=pad_vinyl,
        name="cushion",
    )
    backrest.visual(
        Box((0.28, 0.06, 0.022)),
        origin=Origin(xyz=(0.32, 0.0, 0.006)),
        material=bracket_metal,
        name="support_rail",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.017, length=0.18),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=bracket_metal,
        name="hinge_barrel",
    )
    seat.visual(
        Box((0.30, 0.24, 0.018)),
        origin=Origin(xyz=(-0.15, 0.0, 0.026)),
        material=bracket_metal,
        name="support_plate",
    )
    seat.visual(
        Box((0.30, 0.30, 0.075)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0725)),
        material=pad_vinyl,
        name="cushion",
    )
    seat.visual(
        Box((0.18, 0.08, 0.022)),
        origin=Origin(xyz=(-0.19, 0.0, 0.006)),
        material=bracket_metal,
        name="front_bracket",
    )

    support_link = model.part("support_link")
    support_link.visual(
        Cylinder(radius=0.016, length=0.18),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=bracket_metal,
        name="pivot_barrel",
    )
    support_link.visual(
        Box((0.05, 0.15, 0.02)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=bracket_metal,
        name="pivot_web",
    )
    _add_tube(support_link, "side_bar_0", (0.024, 0.075, 0.0), (0.24, 0.09, 0.16), radius=0.013, material=bracket_metal)
    _add_tube(support_link, "side_bar_1", (0.024, -0.075, 0.0), (0.24, -0.09, 0.16), radius=0.013, material=bracket_metal)
    _add_tube(support_link, "top_bar", (0.24, -0.09, 0.16), (0.24, 0.09, 0.16), radius=0.013, material=bracket_metal)

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=backrest,
        origin=Origin(xyz=(0.04, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=seat,
        origin=Origin(xyz=(-0.02, 0.0, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=0.38),
    )
    model.articulation(
        "frame_to_support",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=support_link,
        origin=Origin(xyz=(0.24, 0.0, 0.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.75, upper=0.35),
    )
    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.48, 0.204, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=15.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.48, -0.204, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_frame = object_model.get_part("center_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_link = object_model.get_part("support_link")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    back_joint = object_model.get_articulation("frame_to_backrest")
    seat_joint = object_model.get_articulation("frame_to_seat")
    support_joint = object_model.get_articulation("frame_to_support")

    ctx.expect_origin_gap(seat, center_frame, axis="z", min_gap=0.33, name="seat hinge sits above the compact frame")
    ctx.expect_origin_gap(backrest, center_frame, axis="z", min_gap=0.38, name="backrest hinge sits above the compact frame")
    ctx.expect_origin_gap(wheel_0, center_frame, axis="y", min_gap=0.18, name="wheel_0 sits outboard of the frame")
    ctx.expect_origin_gap(center_frame, wheel_1, axis="y", min_gap=0.18, name="wheel_1 sits outboard of the frame")

    handle_aabb = ctx.part_element_world_aabb(center_frame, elem="handle_bar")
    wheel_0_aabb = ctx.part_element_world_aabb(wheel_0, elem="tire")
    wheel_1_aabb = ctx.part_element_world_aabb(wheel_1, elem="tire")
    ctx.check(
        "carry handle stays between the wheels",
        handle_aabb is not None
        and wheel_0_aabb is not None
        and wheel_1_aabb is not None
        and handle_aabb[0][1] > wheel_1_aabb[1][1]
        and handle_aabb[1][1] < wheel_0_aabb[0][1]
        and handle_aabb[0][0] < -0.50,
        details=f"handle={handle_aabb}, wheel_0={wheel_0_aabb}, wheel_1={wheel_1_aabb}",
    )

    backrest_rest = ctx.part_element_world_aabb(backrest, elem="cushion")
    seat_rest = ctx.part_element_world_aabb(seat, elem="cushion")
    support_rest = ctx.part_element_world_aabb(support_link, elem="top_bar")

    back_upper = back_joint.motion_limits.upper if back_joint.motion_limits is not None else None
    seat_upper = seat_joint.motion_limits.upper if seat_joint.motion_limits is not None else None
    support_upper = support_joint.motion_limits.upper if support_joint.motion_limits is not None else None

    if back_upper is not None:
        with ctx.pose({back_joint: back_upper}):
            backrest_open = ctx.part_element_world_aabb(backrest, elem="cushion")
        ctx.check(
            "backrest pivots upward",
            backrest_rest is not None
            and backrest_open is not None
            and backrest_open[1][2] > backrest_rest[1][2] + 0.22,
            details=f"rest={backrest_rest}, open={backrest_open}",
        )

    if seat_upper is not None:
        with ctx.pose({seat_joint: seat_upper}):
            seat_raised = ctx.part_element_world_aabb(seat, elem="cushion")
        ctx.check(
            "seat pivots upward at the front",
            seat_rest is not None
            and seat_raised is not None
            and seat_raised[1][2] > seat_rest[1][2] + 0.06
            and seat_raised[0][0] > seat_rest[0][0] + 0.03,
            details=f"rest={seat_rest}, raised={seat_raised}",
        )

    if support_upper is not None:
        with ctx.pose({support_joint: support_upper}):
            support_raised = ctx.part_element_world_aabb(support_link, elem="top_bar")
        ctx.check(
            "rear support link swings upward",
            support_rest is not None
            and support_raised is not None
            and support_raised[1][2] > support_rest[1][2] + 0.05,
            details=f"rest={support_rest}, raised={support_raised}",
        )

    return ctx.report()


object_model = build_object_model()
