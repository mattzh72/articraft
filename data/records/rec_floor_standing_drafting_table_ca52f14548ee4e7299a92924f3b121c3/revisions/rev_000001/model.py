from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="studio_drafting_table")

    model.material("powder_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("silver", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("ivory", rgba=(0.91, 0.90, 0.85, 1.0))
    model.material("beech", rgba=(0.71, 0.60, 0.45, 1.0))
    model.material("charcoal", rgba=(0.23, 0.24, 0.27, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.82, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, 0.36, 0.015)),
        material="powder_black",
        name="front_foot",
    )
    stand.visual(
        Box((0.82, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, -0.36, 0.015)),
        material="powder_black",
        name="rear_foot",
    )
    stand.visual(
        Box((0.12, 0.74, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="dark_steel",
        name="center_stretcher",
    )
    stand.visual(
        Box((0.16, 0.12, 0.46)),
        origin=Origin(xyz=(0.0, -0.02, 0.260)),
        material="dark_steel",
        name="lower_upright",
    )
    stand.visual(
        Box((0.22, 0.08, 0.86)),
        origin=Origin(xyz=(0.0, -0.04, 0.900)),
        material="charcoal",
        name="carriage_rail",
    )
    stand.visual(
        Box((0.28, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, -0.04, 1.355)),
        material="powder_black",
        name="rail_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.26, 0.09, 0.24)),
        origin=Origin(xyz=(0.0, 0.045, 0.020)),
        material="silver",
        name="slide_body",
    )
    carriage.visual(
        Box((0.18, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.070, 0.152)),
        material="silver",
        name="head_block",
    )
    carriage.visual(
        Box((0.035, 0.14, 0.17)),
        origin=Origin(xyz=(-0.125, 0.070, 0.090)),
        material="silver",
        name="left_bracket",
    )
    carriage.visual(
        Box((0.035, 0.14, 0.17)),
        origin=Origin(xyz=(0.125, 0.070, 0.090)),
        material="silver",
        name="right_bracket",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(-0.1485, 0.100, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
        material="powder_black",
        name="left_pivot_cap",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.1485, 0.100, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
        material="powder_black",
        name="right_pivot_cap",
    )

    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=0.280,
        ),
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.022, length=0.195),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="powder_black",
        name="pivot_barrel",
    )
    board.visual(
        Box((0.18, 0.055, 0.080)),
        origin=Origin(xyz=(0.0, 0.045, -0.040)),
        material="powder_black",
        name="pivot_mount",
    )
    board.visual(
        Box((0.35, 0.045, 0.035)),
        origin=Origin(xyz=(-0.275, 0.065, -0.060)),
        material="dark_steel",
        name="left_top_frame",
    )
    board.visual(
        Box((0.35, 0.045, 0.035)),
        origin=Origin(xyz=(0.275, 0.065, -0.060)),
        material="dark_steel",
        name="right_top_frame",
    )
    board.visual(
        Box((0.90, 0.66, 0.020)),
        origin=Origin(xyz=(0.0, 0.390, -0.080)),
        material="ivory",
        name="drawing_surface",
    )
    board.visual(
        Box((0.030, 0.64, 0.040)),
        origin=Origin(xyz=(-0.435, 0.380, -0.075)),
        material="dark_steel",
        name="left_frame",
    )
    board.visual(
        Box((0.030, 0.64, 0.040)),
        origin=Origin(xyz=(0.435, 0.380, -0.075)),
        material="dark_steel",
        name="right_frame",
    )
    board.visual(
        Box((0.90, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, 0.700, -0.075)),
        material="dark_steel",
        name="lower_frame",
    )
    board.visual(
        Box((0.78, 0.065, 0.022)),
        origin=Origin(xyz=(0.0, 0.715, -0.048)),
        material="beech",
        name="lower_ledge",
    )
    board.visual(
        Box((0.022, 0.280, 0.050)),
        origin=Origin(xyz=(-0.250, 0.420, -0.115)),
        material="powder_black",
        name="left_runner",
    )
    board.visual(
        Box((0.022, 0.280, 0.050)),
        origin=Origin(xyz=(0.250, 0.420, -0.115)),
        material="powder_black",
        name="right_runner",
    )
    board.visual(
        Box((0.56, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.320, -0.105)),
        material="powder_black",
        name="runner_bridge",
    )

    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.123, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-0.18,
            upper=1.20,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.018, 0.200, 0.010)),
        origin=Origin(xyz=(-0.250, 0.000, -0.008)),
        material="dark_steel",
        name="left_slide",
    )
    drawer.visual(
        Box((0.018, 0.200, 0.010)),
        origin=Origin(xyz=(0.250, 0.000, -0.008)),
        material="dark_steel",
        name="right_slide",
    )
    drawer.visual(
        Box((0.56, 0.240, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, -0.054)),
        material="charcoal",
        name="tray_floor",
    )
    drawer.visual(
        Box((0.020, 0.240, 0.045)),
        origin=Origin(xyz=(-0.260, 0.010, -0.0325)),
        material="charcoal",
        name="left_wall",
    )
    drawer.visual(
        Box((0.020, 0.240, 0.045)),
        origin=Origin(xyz=(0.260, 0.010, -0.0325)),
        material="charcoal",
        name="right_wall",
    )
    drawer.visual(
        Box((0.56, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, -0.110, -0.0325)),
        material="charcoal",
        name="rear_wall",
    )
    drawer.visual(
        Box((0.60, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.130, -0.0275)),
        material="beech",
        name="front_panel",
    )
    drawer.visual(
        Box((0.20, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.146, -0.027)),
        material="powder_black",
        name="pull",
    )

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.420, -0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.22,
            lower=0.0,
            upper=0.180,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    carriage_slide = object_model.get_articulation("stand_to_carriage")
    board_tilt = object_model.get_articulation("carriage_to_board")
    drawer_slide = object_model.get_articulation("board_to_drawer")

    carriage_limits = carriage_slide.motion_limits
    board_limits = board_tilt.motion_limits
    drawer_limits = drawer_slide.motion_limits

    if carriage_limits is not None and carriage_limits.upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({carriage_slide: carriage_limits.upper}):
            raised_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage rises on rail",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    if board_limits is not None and board_limits.upper is not None:
        with ctx.pose({board_tilt: 0.0}):
            flat_ledge = ctx.part_element_world_aabb(board, elem="lower_ledge")
        with ctx.pose({board_tilt: board_limits.upper}):
            tilted_ledge = ctx.part_element_world_aabb(board, elem="lower_ledge")
        ctx.check(
            "board tilts upward at positive angle",
            flat_ledge is not None
            and tilted_ledge is not None
            and tilted_ledge[0][2] > flat_ledge[0][2] + 0.30,
            details=f"flat={flat_ledge}, tilted={tilted_ledge}",
        )

    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({board_tilt: 0.0, drawer_slide: 0.0}):
            closed_front = ctx.part_element_world_aabb(drawer, elem="front_panel")
        with ctx.pose({board_tilt: 0.0, drawer_slide: drawer_limits.upper}):
            open_front = ctx.part_element_world_aabb(drawer, elem="front_panel")
            ctx.expect_within(
                drawer,
                board,
                axes="x",
                inner_elem="left_slide",
                outer_elem="left_runner",
                margin=0.004,
                name="left drawer slide stays laterally aligned with runner",
            )
            ctx.expect_within(
                drawer,
                board,
                axes="x",
                inner_elem="right_slide",
                outer_elem="right_runner",
                margin=0.004,
                name="right drawer slide stays laterally aligned with runner",
            )
            ctx.expect_overlap(
                drawer,
                board,
                axes="y",
                elem_a="left_slide",
                elem_b="left_runner",
                min_overlap=0.045,
                name="left drawer slide retains insertion at full extension",
            )
            ctx.expect_overlap(
                drawer,
                board,
                axes="y",
                elem_a="right_slide",
                elem_b="right_runner",
                min_overlap=0.045,
                name="right drawer slide retains insertion at full extension",
            )
        ctx.check(
            "drawer extends forward",
            closed_front is not None
            and open_front is not None
            and open_front[1][1] > closed_front[1][1] + 0.15,
            details=f"closed={closed_front}, open={open_front}",
        )

    return ctx.report()


object_model = build_object_model()
