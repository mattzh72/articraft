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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_room_drafting_table")

    powder_coat = model.material("powder_coat", rgba=(0.20, 0.22, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.59, 0.63, 1.0))
    board_laminate = model.material("board_laminate", rgba=(0.84, 0.80, 0.72, 1.0))
    drawer_paint = model.material("drawer_paint", rgba=(0.73, 0.76, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    board_w = 1.10
    board_d = 0.76
    board_t = 0.018

    base = model.part("base")
    base.visual(
        Box((0.16, 0.52, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=powder_coat,
        name="base_spine",
    )
    base.visual(
        Box((0.80, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.22, 0.0175)),
        material=powder_coat,
        name="front_foot",
    )
    base.visual(
        Box((0.80, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, -0.22, 0.0175)),
        material=powder_coat,
        name="rear_foot",
    )

    sleeve_outer = 0.118
    sleeve_wall = 0.015
    sleeve_inner = sleeve_outer - (2.0 * sleeve_wall)
    sleeve_h = 0.55
    sleeve_center_z = 0.05 + (sleeve_h / 2.0)
    sleeve_side_pos = (sleeve_outer / 2.0) - (sleeve_wall / 2.0)
    sleeve_front_pos = (sleeve_outer / 2.0) - (sleeve_wall / 2.0)
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_h)),
        origin=Origin(xyz=(0.0, sleeve_front_pos, sleeve_center_z)),
        material=steel,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_h)),
        origin=Origin(xyz=(0.0, -sleeve_front_pos, sleeve_center_z)),
        material=steel,
        name="sleeve_back",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_h)),
        origin=Origin(xyz=(sleeve_side_pos, 0.0, sleeve_center_z)),
        material=steel,
        name="sleeve_right",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_h)),
        origin=Origin(xyz=(-sleeve_side_pos, 0.0, sleeve_center_z)),
        material=steel,
        name="sleeve_left",
    )
    base.visual(
        Box((0.86, 0.06, 0.008)),
        origin=Origin(xyz=(0.0, 0.22, 0.004)),
        material=rubber,
        name="front_glide",
    )
    base.visual(
        Box((0.86, 0.06, 0.008)),
        origin=Origin(xyz=(0.0, -0.22, 0.004)),
        material=rubber,
        name="rear_glide",
    )

    column = model.part("column")
    column.visual(
        Box((0.078, 0.078, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="column_shaft",
    )
    column.visual(
        Box((0.11, 0.11, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2125)),
        material=steel,
        name="column_cap",
    )
    column.visual(
        Box((0.11, 0.11, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=steel,
        name="column_collar",
    )

    support = model.part("support")
    support.visual(
        Box((0.14, 0.12, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=powder_coat,
        name="support_plate",
    )
    support.visual(
        Box((0.10, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, -0.015, 0.0675)),
        material=powder_coat,
        name="support_neck",
    )
    support.visual(
        Box((0.03, 0.16, 0.11)),
        origin=Origin(xyz=(-0.105, 0.02, 0.08)),
        material=powder_coat,
        name="support_cheek_0",
    )
    support.visual(
        Box((0.03, 0.16, 0.11)),
        origin=Origin(xyz=(0.105, 0.02, 0.08)),
        material=powder_coat,
        name="support_cheek_1",
    )
    support.visual(
        Box((0.28, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.07, 0.13)),
        material=powder_coat,
        name="support_crosshead",
    )
    support.visual(
        Box((0.18, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.025, 0.105)),
        material=powder_coat,
        name="support_bridge",
    )
    support.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.0, 0.07, 0.138), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    board = model.part("board")
    board.visual(
        Box((1.06, 0.72, board_t)),
        origin=Origin(xyz=(0.0, 0.37, board_t / 2.0)),
        material=board_laminate,
        name="board_panel",
    )
    board.visual(
        Box((board_w, 0.04, 0.03)),
        origin=Origin(xyz=(0.0, 0.02, 0.015)),
        material=steel,
        name="rear_rail",
    )
    board.visual(
        Box((board_w, 0.04, 0.03)),
        origin=Origin(xyz=(0.0, 0.74, 0.015)),
        material=steel,
        name="front_rail",
    )
    board.visual(
        Box((0.04, board_d, 0.03)),
        origin=Origin(xyz=(-(board_w / 2.0) + 0.02, board_d / 2.0, 0.015)),
        material=steel,
        name="side_rail_0",
    )
    board.visual(
        Box((0.04, board_d, 0.03)),
        origin=Origin(xyz=((board_w / 2.0) - 0.02, board_d / 2.0, 0.015)),
        material=steel,
        name="side_rail_1",
    )
    board.visual(
        Box((0.96, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.70, -0.006)),
        material=steel,
        name="tray_floor",
    )
    board.visual(
        Box((1.00, 0.022, 0.04)),
        origin=Origin(xyz=(0.0, 0.769, 0.02)),
        material=steel,
        name="tray_front_lip",
    )
    board.visual(
        Box((0.02, 0.10, 0.04)),
        origin=Origin(xyz=(-0.49, 0.70, 0.02)),
        material=steel,
        name="tray_cheek_0",
    )
    board.visual(
        Box((0.02, 0.10, 0.04)),
        origin=Origin(xyz=(0.49, 0.70, 0.02)),
        material=steel,
        name="tray_cheek_1",
    )
    board.visual(
        Box((0.04, 0.34, 0.025)),
        origin=Origin(xyz=(-0.23, 0.39, -0.0125)),
        material=steel,
        name="runner_left",
    )
    board.visual(
        Box((0.04, 0.34, 0.025)),
        origin=Origin(xyz=(0.23, 0.39, -0.0125)),
        material=steel,
        name="runner_right",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.40, 0.26, 0.008)),
        origin=Origin(xyz=(0.0, 0.13, -0.051)),
        material=drawer_paint,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.26, 0.047)),
        origin=Origin(xyz=(-0.194, 0.13, -0.0235)),
        material=drawer_paint,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.012, 0.26, 0.047)),
        origin=Origin(xyz=(0.194, 0.13, -0.0235)),
        material=drawer_paint,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.40, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, 0.253, -0.0275)),
        material=drawer_paint,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.40, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.005, -0.02)),
        material=drawer_paint,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.12, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.269, -0.022)),
        material=powder_coat,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.03, 0.22, 0.008)),
        origin=Origin(xyz=(-0.209, 0.11, -0.004)),
        material=steel,
        name="slide_left",
    )
    drawer.visual(
        Box((0.03, 0.22, 0.008)),
        origin=Origin(xyz=(0.209, 0.11, -0.004)),
        material=steel,
        name="slide_right",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.18, lower=0.0, upper=0.24),
    )
    model.articulation(
        "column_to_support",
        ArticulationType.FIXED,
        parent=column,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
    )
    model.articulation(
        "support_to_board",
        ArticulationType.REVOLUTE,
        parent=support,
        child=board,
        origin=Origin(xyz=(0.0, 0.07, 0.15)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(52.0),
        ),
    )
    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.21, -0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("column")
    support = object_model.get_part("support")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    lift = object_model.get_articulation("base_to_column")
    tilt = object_model.get_articulation("support_to_board")
    slide = object_model.get_articulation("board_to_drawer")

    ctx.expect_gap(
        board,
        support,
        axis="z",
        positive_elem="rear_rail",
        negative_elem="support_crosshead",
        max_gap=0.001,
        max_penetration=0.0,
        name="board rests on support head at the hinge line",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="xy",
        elem_a="slide_left",
        elem_b="runner_left",
        min_overlap=0.008,
        name="drawer left slide starts engaged with the runner",
    )

    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        rest_support_pos = ctx.part_world_position(support)
        with ctx.pose({lift: lift_limits.upper}):
            extended_support_pos = ctx.part_world_position(support)
        ctx.check(
            "column raises the support head",
            rest_support_pos is not None
            and extended_support_pos is not None
            and extended_support_pos[2] > rest_support_pos[2] + 0.20,
            details=f"rest={rest_support_pos}, extended={extended_support_pos}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_lip_aabb = ctx.part_element_world_aabb(board, elem="tray_front_lip")
        with ctx.pose({tilt: tilt_limits.upper}):
            raised_lip_aabb = ctx.part_element_world_aabb(board, elem="tray_front_lip")
        ctx.check(
            "board front edge rises when tilted",
            rest_lip_aabb is not None
            and raised_lip_aabb is not None
            and raised_lip_aabb[1][2] > rest_lip_aabb[1][2] + 0.20,
            details=f"rest={rest_lip_aabb}, raised={raised_lip_aabb}",
        )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: slide_limits.upper}):
            extended_drawer_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                board,
                axes="xy",
                elem_a="slide_left",
                elem_b="runner_left",
                min_overlap=0.006,
                name="drawer left slide stays retained on the runner when extended",
            )
        ctx.check(
            "drawer pulls outward from under the board",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.14,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    return ctx.report()


object_model = build_object_model()
