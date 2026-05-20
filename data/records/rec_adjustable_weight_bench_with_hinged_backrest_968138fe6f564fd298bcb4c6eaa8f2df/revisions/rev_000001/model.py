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


SEAT_REST_ANGLE = math.radians(7.0)
BACKREST_REST_ANGLE = math.radians(26.0)
LADDER_REST_ANGLE = math.radians(67.0)


def _hinged_box_center(length: float, pitch_up: float, lift_from_hinge: float) -> tuple[float, float, float]:
    return (
        0.5 * length * math.cos(pitch_up) - lift_from_hinge * math.sin(pitch_up),
        0.0,
        0.5 * length * math.sin(pitch_up) + lift_from_hinge * math.cos(pitch_up),
    )


def _ladder_point(distance: float) -> tuple[float, float, float]:
    return (
        -distance * math.cos(LADDER_REST_ANGLE),
        0.0,
        distance * math.sin(LADDER_REST_ANGLE),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_weight_bench")

    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    vinyl = model.material("vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    board = model.material("board", rgba=(0.18, 0.17, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.30, 0.31, 0.33, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.38, 0.04)),
        origin=Origin(xyz=(-0.56, 0.0, 0.02)),
        material=frame_paint,
        name="front_foot",
    )
    frame.visual(
        Box((0.26, 0.58, 0.04)),
        origin=Origin(xyz=(0.56, 0.0, 0.02)),
        material=frame_paint,
        name="rear_foot",
    )
    frame.visual(
        Box((0.06, 0.08, 0.24)),
        origin=Origin(xyz=(-0.47, 0.0, 0.14)),
        material=frame_paint,
        name="front_post",
    )
    frame.visual(
        Box((0.06, 0.08, 0.26)),
        origin=Origin(xyz=(0.49, 0.0, 0.15)),
        material=frame_paint,
        name="rear_post",
    )
    frame.visual(
        Box((1.06, 0.09, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.285)),
        material=frame_paint,
        name="spine",
    )
    frame.visual(
        Box((0.10, 0.22, 0.05)),
        origin=Origin(xyz=(-0.20, 0.0, 0.335)),
        material=frame_paint,
        name="seat_mount",
    )
    frame.visual(
        Box((0.16, 0.24, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.335)),
        material=frame_paint,
        name="backrest_mount",
    )
    frame.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(0.56, 0.225, 0.13)),
        material=frame_paint,
        name="ladder_tab_0",
    )
    frame.visual(
        Box((0.05, 0.05, 0.18)),
        origin=Origin(xyz=(0.56, -0.225, 0.13)),
        material=frame_paint,
        name="ladder_tab_1",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(-0.56, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_axle",
    )
    frame.visual(
        Box((0.025, 0.04, 0.06)),
        origin=Origin(xyz=(-0.56, 0.18, 0.055)),
        material=frame_paint,
        name="wheel_bracket_0",
    )
    frame.visual(
        Box((0.025, 0.04, 0.06)),
        origin=Origin(xyz=(-0.56, -0.18, 0.055)),
        material=frame_paint,
        name="wheel_bracket_1",
    )

    seat = model.part("seat")
    seat_board_center = _hinged_box_center(0.34, SEAT_REST_ANGLE, 0.0075)
    seat_pad_center = _hinged_box_center(0.34, SEAT_REST_ANGLE, 0.015 + 0.0275)
    seat.visual(
        Box((0.34, 0.29, 0.015)),
        origin=Origin(xyz=seat_board_center, rpy=(0.0, -SEAT_REST_ANGLE, 0.0)),
        material=board,
        name="seat_board",
    )
    seat.visual(
        Box((0.34, 0.31, 0.055)),
        origin=Origin(xyz=seat_pad_center, rpy=(0.0, -SEAT_REST_ANGLE, 0.0)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(
        Cylinder(radius=0.014, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="seat_hinge_tube",
    )

    backrest = model.part("backrest")
    backrest_board_center = _hinged_box_center(0.84, BACKREST_REST_ANGLE, 0.0075)
    backrest_pad_center = _hinged_box_center(0.84, BACKREST_REST_ANGLE, 0.015 + 0.0275)
    backrest.visual(
        Box((0.84, 0.29, 0.015)),
        origin=Origin(xyz=backrest_board_center, rpy=(0.0, -BACKREST_REST_ANGLE, 0.0)),
        material=board,
        name="backrest_board",
    )
    backrest.visual(
        Box((0.84, 0.31, 0.055)),
        origin=Origin(xyz=backrest_pad_center, rpy=(0.0, -BACKREST_REST_ANGLE, 0.0)),
        material=vinyl,
        name="backrest_pad",
    )
    backrest.visual(
        Box((0.05, 0.22, 0.03)),
        origin=Origin(xyz=_hinged_box_center(0.62, BACKREST_REST_ANGLE, 0.02), rpy=(0.0, -BACKREST_REST_ANGLE, 0.0)),
        material=steel,
        name="support_bar",
    )
    backrest.visual(
        Cylinder(radius=0.015, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="backrest_hinge_tube",
    )

    support_ladder = model.part("support_ladder")
    ladder_pitch = -(math.pi - LADDER_REST_ANGLE)
    rail_center = _ladder_point(0.215)
    support_ladder.visual(
        Cylinder(radius=0.018, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tube",
    )
    support_ladder.visual(
        Box((0.44, 0.025, 0.03)),
        origin=Origin(xyz=(rail_center[0], 0.18, rail_center[2]), rpy=(0.0, ladder_pitch, 0.0)),
        material=frame_paint,
        name="side_rail_0",
    )
    support_ladder.visual(
        Box((0.44, 0.025, 0.03)),
        origin=Origin(xyz=(rail_center[0], -0.18, rail_center[2]), rpy=(0.0, ladder_pitch, 0.0)),
        material=frame_paint,
        name="side_rail_1",
    )
    rung_0_point = _ladder_point(0.07)
    support_ladder.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=rung_0_point, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rung_0",
    )
    rung_1_point = _ladder_point(0.22)
    support_ladder.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=rung_1_point, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rung_1",
    )
    rung_2_point = _ladder_point(0.26)
    support_ladder.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=rung_2_point, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rung_2",
    )
    top_rung_point = _ladder_point(0.35)
    support_ladder.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=top_rung_point, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="top_rung",
    )

    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_0.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    front_wheel_0.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="hub",
    )
    front_wheel_0.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cap",
    )

    front_wheel_1 = model.part("front_wheel_1")
    front_wheel_1.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    front_wheel_1.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="hub",
    )
    front_wheel_1.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cap",
    )

    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.22, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-SEAT_REST_ANGLE,
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.16, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=-BACKREST_REST_ANGLE,
            upper=math.radians(54.0),
        ),
    )
    model.articulation(
        "ladder_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_ladder,
        origin=Origin(xyz=(0.56, 0.0, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.0,
            lower=math.radians(-30.0),
            upper=math.radians(14.0),
        ),
    )
    model.articulation(
        "front_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel_0,
        origin=Origin(xyz=(-0.56, 0.215, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel_1,
        origin=Origin(xyz=(-0.56, -0.215, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    support_ladder = object_model.get_part("support_ladder")
    wheel_0 = object_model.get_part("front_wheel_0")
    wheel_1 = object_model.get_part("front_wheel_1")

    seat_hinge = object_model.get_articulation("seat_hinge")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    ladder_pivot = object_model.get_articulation("ladder_pivot")
    wheel_spin_0 = object_model.get_articulation("front_wheel_0_spin")
    wheel_spin_1 = object_model.get_articulation("front_wheel_1_spin")

    ctx.expect_gap(
        backrest,
        support_ladder,
        axis="z",
        positive_elem="support_bar",
        negative_elem="top_rung",
        min_gap=0.0,
        max_gap=0.03,
        name="support ladder sits just under the backrest support bar",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="seat_board",
        negative_elem="seat_mount",
        max_gap=0.03,
        max_penetration=0.001,
        name="seat board is perched on the center frame mount",
    )
    ctx.expect_gap(
        backrest,
        frame,
        axis="z",
        positive_elem="backrest_board",
        negative_elem="backrest_mount",
        max_gap=0.03,
        max_penetration=0.001,
        name="backrest board is perched on the rear hinge mount",
    )
    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="z",
        elem_a="tire",
        elem_b="wheel_bracket_0",
        min_overlap=0.02,
        name="front wheel 0 stays aligned with its bracket height",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="z",
        elem_a="tire",
        elem_b="wheel_bracket_1",
        min_overlap=0.02,
        name="front wheel 1 stays aligned with its bracket height",
    )

    seat_limits = seat_hinge.motion_limits
    if seat_limits is not None and seat_limits.lower is not None and seat_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
        ctx.check(
            "seat rear lifts as the hinge opens",
            low_aabb is not None and high_aabb is not None and high_aabb[1][2] > low_aabb[1][2] + 0.05,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    backrest_limits = backrest_hinge.motion_limits
    if backrest_limits is not None and backrest_limits.lower is not None and backrest_limits.upper is not None:
        with ctx.pose({backrest_hinge: backrest_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
        with ctx.pose({backrest_hinge: backrest_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
        ctx.check(
            "backrest head end rises through its adjustment range",
            low_aabb is not None and high_aabb is not None and high_aabb[1][2] > low_aabb[1][2] + 0.24,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    ladder_limits = ladder_pivot.motion_limits
    if ladder_limits is not None and ladder_limits.lower is not None and ladder_limits.upper is not None:
        with ctx.pose({ladder_pivot: ladder_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(support_ladder, elem="top_rung")
        with ctx.pose({ladder_pivot: ladder_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(support_ladder, elem="top_rung")
        ctx.check(
            "support ladder swings upward from its rear pivot",
            low_aabb is not None and high_aabb is not None and high_aabb[1][2] > low_aabb[1][2] + 0.10,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    ctx.check(
        "transport wheels use continuous spin joints",
        wheel_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin_1.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin_0.motion_limits is not None
        and wheel_spin_0.motion_limits.lower is None
        and wheel_spin_0.motion_limits.upper is None
        and wheel_spin_1.motion_limits is not None
        and wheel_spin_1.motion_limits.lower is None
        and wheel_spin_1.motion_limits.upper is None,
        details=(
            f"wheel_0={wheel_spin_0.articulation_type}/{wheel_spin_0.motion_limits}, "
            f"wheel_1={wheel_spin_1.articulation_type}/{wheel_spin_1.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
