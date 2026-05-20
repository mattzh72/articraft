from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_h_frame_easel")

    ash = model.material("ash", rgba=(0.73, 0.63, 0.47, 1.0))
    walnut = model.material("walnut", rgba=(0.45, 0.29, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.03, 0.24, 0.025)),
        origin=Origin(xyz=(-0.145, 0.02, 0.0125)),
        material=ash,
        name="left_foot",
    )
    frame.visual(
        Box((0.03, 0.24, 0.025)),
        origin=Origin(xyz=(0.145, 0.02, 0.0125)),
        material=ash,
        name="right_foot",
    )
    frame.visual(
        Box((0.03, 0.04, 0.58)),
        origin=Origin(xyz=(-0.145, -0.01, 0.29)),
        material=ash,
        name="left_stile",
    )
    frame.visual(
        Box((0.03, 0.04, 0.58)),
        origin=Origin(xyz=(0.145, -0.01, 0.29)),
        material=ash,
        name="right_stile",
    )
    frame.visual(
        Box((0.35, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, -0.01, 0.11)),
        material=ash,
        name="lower_rail",
    )
    frame.visual(
        Box((0.35, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, -0.01, 0.575)),
        material=ash,
        name="top_rail",
    )
    frame.visual(
        Box((0.045, 0.05, 0.48)),
        origin=Origin(xyz=(0.0, 0.02, 0.34)),
        material=walnut,
        name="mast",
    )
    rear_brace_angle = -math.atan2(0.11, 0.53)
    frame.visual(
        Box((0.028, 0.03, 0.55)),
        origin=Origin(xyz=(-0.145, 0.045, 0.289), rpy=(rear_brace_angle, 0.0, 0.0)),
        material=ash,
        name="left_brace",
    )
    frame.visual(
        Box((0.028, 0.03, 0.55)),
        origin=Origin(xyz=(0.145, 0.045, 0.289), rpy=(rear_brace_angle, 0.0, 0.0)),
        material=ash,
        name="right_brace",
    )

    upper_carriage = model.part("upper_carriage")
    upper_carriage.visual(
        Box((0.11, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, -0.03, 0.0)),
        material=graphite,
        name="carriage_body",
    )
    upper_carriage.visual(
        Box((0.20, 0.035, 0.03)),
        origin=Origin(xyz=(0.0, -0.058, -0.055)),
        material=steel,
        name="hinge_bar",
    )

    board = model.part("board")
    board.visual(
        Box((0.30, 0.015, 0.18)),
        origin=Origin(xyz=(0.0, -0.0075, -0.09)),
        material=walnut,
        name="panel",
    )
    board.visual(
        Box((0.28, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, -0.171)),
        material=ash,
        name="lower_ledge",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.11, 0.038, 0.10)),
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
        material=graphite,
        name="carriage_body",
    )
    tray.visual(
        Box((0.29, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, -0.064, -0.040)),
        material=ash,
        name="shelf",
    )
    tray.visual(
        Box((0.27, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.085, -0.027)),
        material=ash,
        name="front_lip",
    )

    model.articulation(
        "frame_to_upper_carriage",
        ArticulationType.FIXED,
        parent=frame,
        child=upper_carriage,
        origin=Origin(xyz=(0.0, -0.005, 0.50)),
    )
    model.articulation(
        "upper_carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=upper_carriage,
        child=board,
        origin=Origin(xyz=(0.0, -0.071, -0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "frame_to_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, -0.005, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.09,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_carriage = object_model.get_part("upper_carriage")
    board = object_model.get_part("board")
    tray = object_model.get_part("tray")
    board_tilt = object_model.get_articulation("upper_carriage_to_board")
    tray_slide = object_model.get_articulation("frame_to_tray")

    tray_upper = 0.09
    board_upper = math.radians(28.0)
    if tray_slide.motion_limits is not None and tray_slide.motion_limits.upper is not None:
        tray_upper = tray_slide.motion_limits.upper
    if board_tilt.motion_limits is not None and board_tilt.motion_limits.upper is not None:
        board_upper = board_tilt.motion_limits.upper

    ctx.expect_gap(
        frame,
        tray,
        axis="y",
        positive_elem="mast",
        negative_elem="carriage_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="tray carriage seats against mast front",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="z",
        elem_a="carriage_body",
        elem_b="mast",
        min_overlap=0.08,
        name="tray carriage remains engaged on mast at rest",
    )
    ctx.expect_gap(
        board,
        tray,
        axis="z",
        positive_elem="lower_ledge",
        negative_elem="shelf",
        min_gap=0.04,
        name="board clears tray at rest",
    )
    ctx.expect_gap(
        upper_carriage,
        tray,
        axis="z",
        positive_elem="hinge_bar",
        negative_elem="carriage_body",
        min_gap=0.08,
        name="tray clears upper carriage",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_upper}):
        ctx.expect_overlap(
            tray,
            frame,
            axes="z",
            elem_a="carriage_body",
            elem_b="mast",
            min_overlap=0.08,
            name="raised tray carriage remains engaged on mast",
        )
        ctx.expect_gap(
            board,
            tray,
            axis="z",
            positive_elem="lower_ledge",
            negative_elem="shelf",
            min_gap=0.02,
            name="raised tray stays below board",
        )
        raised_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray slides upward along mast",
        rest_tray_pos is not None
        and raised_tray_pos is not None
        and raised_tray_pos[2] > rest_tray_pos[2] + 0.07,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    rest_board_aabb = ctx.part_world_aabb(board)
    with ctx.pose({board_tilt: board_upper}):
        tilted_board_aabb = ctx.part_world_aabb(board)

    ctx.check(
        "board tilts forward from upper carriage hinge",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[0][1] < rest_board_aabb[0][1] - 0.035,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
