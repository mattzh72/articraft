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


def _tube_origin(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _add_tube(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: str,
) -> None:
    length, origin = _tube_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    model.material("frame_powder", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("upholstery", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("subframe", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("tire", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("hub", rgba=(0.58, 0.60, 0.62, 1.0))

    frame = model.part("frame")
    _add_tube(frame, "rear_foot", (-0.36, -0.18, 0.05), (-0.36, 0.18, 0.05), 0.016, "frame_powder")
    _add_tube(frame, "left_rail", (-0.36, -0.16, 0.05), (0.34, -0.16, 0.075), 0.014, "frame_powder")
    _add_tube(frame, "right_rail", (-0.36, 0.16, 0.05), (0.34, 0.16, 0.075), 0.014, "frame_powder")
    _add_tube(frame, "front_beam", (0.34, -0.16, 0.075), (0.34, 0.16, 0.075), 0.015, "frame_powder")
    _add_tube(frame, "left_front_riser", (0.23, -0.16, 0.071), (0.015, -0.16, 0.315), 0.013, "frame_powder")
    _add_tube(frame, "right_front_riser", (0.23, 0.16, 0.071), (0.015, 0.16, 0.315), 0.013, "frame_powder")
    _add_tube(frame, "left_rear_riser", (-0.26, -0.16, 0.053), (-0.015, -0.16, 0.325), 0.013, "frame_powder")
    _add_tube(frame, "right_rear_riser", (-0.26, 0.16, 0.053), (-0.015, 0.16, 0.325), 0.013, "frame_powder")
    _add_tube(frame, "seat_bridge", (0.015, -0.16, 0.307), (0.015, 0.16, 0.307), 0.014, "frame_powder")
    _add_tube(frame, "back_bridge", (-0.015, -0.16, 0.317), (-0.015, 0.16, 0.317), 0.014, "frame_powder")
    _add_tube(frame, "left_top_link", (-0.015, -0.16, 0.317), (0.015, -0.16, 0.307), 0.012, "frame_powder")
    _add_tube(frame, "right_top_link", (-0.015, 0.16, 0.317), (0.015, 0.16, 0.307), 0.012, "frame_powder")
    _add_tube(frame, "seat_left_tab", (0.015, -0.122, 0.307), (0.015, -0.122, 0.333), 0.009, "subframe")
    _add_tube(frame, "seat_right_tab", (0.015, 0.122, 0.307), (0.015, 0.122, 0.333), 0.009, "subframe")
    _add_tube(frame, "back_left_tab", (-0.015, -0.122, 0.317), (-0.015, -0.122, 0.343), 0.009, "subframe")
    _add_tube(frame, "back_right_tab", (-0.015, 0.122, 0.317), (-0.015, 0.122, 0.343), 0.009, "subframe")
    _add_tube(frame, "left_support_brace", (-0.28, -0.16, 0.052), (-0.09, -0.11, 0.15), 0.012, "frame_powder")
    _add_tube(frame, "right_support_brace", (-0.28, 0.16, 0.052), (-0.09, 0.11, 0.15), 0.012, "frame_powder")
    _add_tube(frame, "left_support_lug", (-0.09, -0.135, 0.15), (-0.09, -0.075, 0.15), 0.012, "frame_powder")
    _add_tube(frame, "right_support_lug", (-0.09, 0.075, 0.15), (-0.09, 0.135, 0.15), 0.012, "frame_powder")
    _add_tube(frame, "wheel_axle", (0.34, -0.12, 0.034), (0.34, 0.12, 0.034), 0.009, "subframe")
    _add_tube(frame, "left_axle_stub", (0.34, -0.12, 0.034), (0.34, -0.136, 0.034), 0.009, "subframe")
    _add_tube(frame, "right_axle_stub", (0.34, 0.12, 0.034), (0.34, 0.136, 0.034), 0.009, "subframe")
    _add_tube(frame, "axle_drop", (0.34, 0.0, 0.034), (0.34, 0.0, 0.075), 0.009, "subframe")

    seat = model.part("seat")
    seat.visual(
        Box((0.30, 0.30, 0.06)),
        origin=Origin(xyz=(0.15, 0.0, 0.045)),
        material="upholstery",
        name="pad",
    )
    _add_tube(seat, "left_hinge_barrel", (0.0, -0.128, 0.0), (0.0, -0.088, 0.0), 0.012, "subframe")
    _add_tube(seat, "right_hinge_barrel", (0.0, 0.088, 0.0), (0.0, 0.128, 0.0), 0.012, "subframe")
    _add_tube(seat, "left_subrail", (0.0, -0.10, 0.005), (0.22, -0.10, 0.005), 0.010, "subframe")
    _add_tube(seat, "right_subrail", (0.0, 0.10, 0.005), (0.22, 0.10, 0.005), 0.010, "subframe")
    _add_tube(seat, "front_subbrace", (0.22, -0.09, 0.005), (0.22, 0.09, 0.005), 0.010, "subframe")

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.78, 0.30, 0.06)),
        origin=Origin(xyz=(-0.39, 0.0, 0.045)),
        material="upholstery",
        name="back_panel",
    )
    _add_tube(backrest, "left_hinge_barrel", (0.0, -0.128, 0.0), (0.0, -0.088, 0.0), 0.012, "subframe")
    _add_tube(backrest, "right_hinge_barrel", (0.0, 0.088, 0.0), (0.0, 0.128, 0.0), 0.012, "subframe")
    _add_tube(backrest, "left_subrail", (0.0, -0.10, 0.005), (-0.62, -0.10, 0.005), 0.010, "subframe")
    _add_tube(backrest, "right_subrail", (0.0, 0.10, 0.005), (-0.62, 0.10, 0.005), 0.010, "subframe")
    _add_tube(backrest, "mid_subbrace", (-0.28, -0.09, 0.005), (-0.28, 0.09, 0.005), 0.010, "subframe")

    support_link = model.part("support_link")
    _add_tube(support_link, "pivot_barrel", (0.0, -0.075, 0.0), (0.0, 0.075, 0.0), 0.015, "subframe")
    _add_tube(support_link, "main_link", (0.0, 0.0, 0.015), (-0.15, 0.0, 0.015), 0.013, "frame_powder")
    _add_tube(support_link, "link_tip", (-0.15, -0.055, 0.03), (-0.15, 0.055, 0.03), 0.016, "subframe")

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tire",
        name="tire",
    )
    wheel_0.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hub",
        name="hub",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tire",
        name="tire",
    )
    wheel_1.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hub",
        name="hub",
    )

    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.015, 0.0, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.32, effort=80.0, velocity=1.2),
    )
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.015, 0.0, 0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=100.0, velocity=1.0),
    )
    model.articulation(
        "frame_to_support_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_link,
        origin=Origin(xyz=(-0.09, 0.0, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=80.0, velocity=1.5),
    )
    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(0.34, -0.148, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(0.34, 0.148, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    support_link = object_model.get_part("support_link")
    wheel_0 = object_model.get_part("wheel_0")

    seat_hinge = object_model.get_articulation("frame_to_seat")
    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    support_hinge = object_model.get_articulation("frame_to_support_link")

    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_left_tab",
        elem_b="left_hinge_barrel",
        reason="Simplified hinge tab and barrel solids overlap locally to represent the seat pivot hardware.",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_right_tab",
        elem_b="right_hinge_barrel",
        reason="Simplified hinge tab and barrel solids overlap locally to represent the seat pivot hardware.",
    )
    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_left_tab",
        elem_b="left_hinge_barrel",
        reason="Simplified hinge tab and barrel solids overlap locally to represent the backrest pivot hardware.",
    )
    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_right_tab",
        elem_b="right_hinge_barrel",
        reason="Simplified hinge tab and barrel solids overlap locally to represent the backrest pivot hardware.",
    )

    with ctx.pose({seat_hinge: 0.0, backrest_hinge: 0.0}):
        ctx.expect_gap(
            seat,
            backrest,
            axis="x",
            positive_elem="pad",
            negative_elem="back_panel",
            max_gap=0.04,
            max_penetration=0.0,
            name="seat and backrest meet with a narrow center seam",
        )
        ctx.expect_overlap(
            seat,
            backrest,
            axes="y",
            elem_a="pad",
            elem_b="back_panel",
            min_overlap=0.26,
            name="seat and backrest stay aligned across bench width",
        )

    ctx.expect_gap(
        frame,
        wheel_0,
        axis="z",
        positive_elem="front_beam",
        negative_elem="tire",
        min_gap=0.0,
        max_gap=0.015,
        name="front transport wheel tucks just below the front crossmember",
    )
    ctx.expect_overlap(
        frame,
        wheel_0,
        axes="x",
        elem_a="front_beam",
        elem_b="tire",
        min_overlap=0.02,
        name="front transport wheel sits under the front beam footprint",
    )

    seat_upper = seat_hinge.motion_limits.upper if seat_hinge.motion_limits is not None else 0.0
    backrest_upper = backrest_hinge.motion_limits.upper if backrest_hinge.motion_limits is not None else 0.0
    support_pose = support_hinge.motion_limits.upper if support_hinge.motion_limits is not None else 0.95

    seat_rest = ctx.part_element_world_aabb(seat, elem="pad")
    backrest_rest = ctx.part_element_world_aabb(backrest, elem="back_panel")
    support_rest = ctx.part_element_world_aabb(support_link, elem="link_tip")

    with ctx.pose({seat_hinge: seat_upper, backrest_hinge: backrest_upper, support_hinge: support_pose}):
        seat_raised = ctx.part_element_world_aabb(seat, elem="pad")
        backrest_raised = ctx.part_element_world_aabb(backrest, elem="back_panel")
        support_raised = ctx.part_element_world_aabb(support_link, elem="link_tip")

        ctx.expect_gap(
            backrest,
            support_link,
            axis="z",
            positive_elem="back_panel",
            negative_elem="link_tip",
            min_gap=0.0,
            max_gap=0.12,
            name="raised support link stays tucked under the backrest underside",
        )
        ctx.expect_overlap(
            support_link,
            backrest,
            axes="xy",
            elem_a="link_tip",
            elem_b="back_panel",
            min_overlap=0.02,
            name="support link tip stays under the backrest support zone",
        )

    seat_rest_z = seat_rest[1][2] if seat_rest is not None else None
    seat_raised_z = seat_raised[1][2] if seat_raised is not None else None
    ctx.check(
        "seat front rises when the seat hinge opens",
        seat_rest_z is not None and seat_raised_z is not None and seat_raised_z > seat_rest_z + 0.05,
        details=f"rest_max_z={seat_rest_z}, raised_max_z={seat_raised_z}",
    )

    backrest_rest_z = backrest_rest[1][2] if backrest_rest is not None else None
    backrest_raised_z = backrest_raised[1][2] if backrest_raised is not None else None
    ctx.check(
        "backrest lifts into an incline when the hinge opens",
        backrest_rest_z is not None and backrest_raised_z is not None and backrest_raised_z > backrest_rest_z + 0.20,
        details=f"rest_max_z={backrest_rest_z}, raised_max_z={backrest_raised_z}",
    )

    support_rest_z = support_rest[1][2] if support_rest is not None else None
    support_raised_z = support_raised[1][2] if support_raised is not None else None
    ctx.check(
        "rear support link swings upward toward the backrest",
        support_rest_z is not None and support_raised_z is not None and support_raised_z > support_rest_z + 0.10,
        details=f"rest_max_z={support_rest_z}, raised_max_z={support_raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
