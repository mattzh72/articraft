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
    model = ArticulatedObject(name="drafting_table")

    steel = model.material("steel", rgba=(0.23, 0.24, 0.26, 1.0))
    trim = model.material("trim", rgba=(0.10, 0.11, 0.12, 1.0))
    board_surface = model.material("board_surface", rgba=(0.90, 0.89, 0.83, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.77, 0.79, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.02, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.44, 0.02)),
        material=steel,
        name="front_foot",
    )
    base.visual(
        Box((0.92, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, -0.34, 0.02)),
        material=steel,
        name="rear_foot",
    )
    base.visual(
        Box((0.14, 0.86, 0.06)),
        origin=Origin(xyz=(0.0, 0.04, 0.03)),
        material=steel,
        name="spine",
    )
    base.visual(
        Box((0.10, 0.08, 0.84)),
        origin=Origin(xyz=(0.0, -0.08, 0.46)),
        material=steel,
        name="mast",
    )
    base.visual(
        Box((0.22, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.08, 0.10)),
        material=steel,
        name="mast_base",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.03, 0.16, 0.26)),
        origin=Origin(xyz=(-0.075, -0.08, -0.10)),
        material=steel,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.03, 0.16, 0.26)),
        origin=Origin(xyz=(0.075, -0.08, -0.10)),
        material=steel,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.18, 0.03, 0.24)),
        origin=Origin(xyz=(0.0, -0.03, -0.10)),
        material=steel,
        name="front_bridge",
    )
    carriage.visual(
        Box((0.18, 0.03, 0.24)),
        origin=Origin(xyz=(0.0, -0.13, -0.10)),
        material=steel,
        name="rear_bridge",
    )
    carriage.visual(
        Box((0.04, 0.06, 0.06)),
        origin=Origin(xyz=(-0.08, -0.045, -0.005)),
        material=steel,
        name="left_mount",
    )
    carriage.visual(
        Box((0.04, 0.06, 0.06)),
        origin=Origin(xyz=(0.08, -0.045, -0.005)),
        material=steel,
        name="right_mount",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="tilt_axle",
    )

    board = model.part("board")
    board.visual(
        Box((0.90, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.03, 0.041)),
        material=steel,
        name="rear_frame",
    )
    board.visual(
        Box((0.90, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.73, 0.041)),
        material=steel,
        name="front_frame",
    )
    board.visual(
        Box((0.05, 0.76, 0.05)),
        origin=Origin(xyz=(-0.425, 0.38, 0.041)),
        material=steel,
        name="left_frame",
    )
    board.visual(
        Box((0.03, 0.76, 0.05)),
        origin=Origin(xyz=(0.435, 0.38, 0.041)),
        material=steel,
        name="right_frame",
    )
    board.visual(
        Box((0.84, 0.70, 0.018)),
        origin=Origin(xyz=(0.0, 0.38, 0.057)),
        material=board_surface,
        name="main_panel",
    )
    board.visual(
        Box((0.40, 0.05, 0.036)),
        origin=Origin(xyz=(0.0, 0.28, 0.028)),
        material=steel,
        name="drawer_crossmember",
    )
    board.visual(
        Box((0.03, 0.08, 0.042)),
        origin=Origin(xyz=(-0.16, 0.28, 0.031)),
        material=steel,
        name="hanger_left",
    )
    board.visual(
        Box((0.03, 0.08, 0.042)),
        origin=Origin(xyz=(0.16, 0.28, 0.031)),
        material=steel,
        name="hanger_right",
    )
    board.visual(
        Box((0.03, 0.26, 0.016)),
        origin=Origin(xyz=(-0.16, 0.40, 0.006)),
        material=trim,
        name="runner_left",
    )
    board.visual(
        Box((0.03, 0.26, 0.016)),
        origin=Origin(xyz=(0.16, 0.40, 0.006)),
        material=trim,
        name="runner_right",
    )

    wing = model.part("wing")
    wing.visual(
        Box((0.02, 0.72, 0.05)),
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
        material=steel,
        name="inner_frame",
    )
    wing.visual(
        Box((0.02, 0.72, 0.05)),
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        material=steel,
        name="outer_frame",
    )
    wing.visual(
        Box((0.20, 0.05, 0.05)),
        origin=Origin(xyz=(0.10, 0.335, 0.0)),
        material=steel,
        name="front_rail",
    )
    wing.visual(
        Box((0.20, 0.05, 0.05)),
        origin=Origin(xyz=(0.10, -0.335, 0.0)),
        material=steel,
        name="rear_rail",
    )
    wing.visual(
        Box((0.16, 0.64, 0.018)),
        origin=Origin(xyz=(0.10, 0.0, 0.016)),
        material=board_surface,
        name="wing_panel",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.03, 0.32, 0.012)),
        origin=Origin(xyz=(-0.16, 0.16, 0.0)),
        material=trim,
        name="slide_left",
    )
    drawer.visual(
        Box((0.03, 0.32, 0.012)),
        origin=Origin(xyz=(0.16, 0.16, 0.0)),
        material=trim,
        name="slide_right",
    )
    drawer.visual(
        Box((0.34, 0.32, 0.008)),
        origin=Origin(xyz=(0.0, 0.19, -0.035)),
        material=drawer_finish,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.015, 0.32, 0.05)),
        origin=Origin(xyz=(-0.17, 0.19, -0.017)),
        material=drawer_finish,
        name="left_wall",
    )
    drawer.visual(
        Box((0.015, 0.32, 0.05)),
        origin=Origin(xyz=(0.17, 0.19, -0.017)),
        material=drawer_finish,
        name="right_wall",
    )
    drawer.visual(
        Box((0.34, 0.015, 0.05)),
        origin=Origin(xyz=(0.0, 0.0375, -0.017)),
        material=drawer_finish,
        name="rear_wall",
    )
    drawer.visual(
        Box((0.38, 0.02, 0.07)),
        origin=Origin(xyz=(0.0, 0.345, -0.007)),
        material=drawer_finish,
        name="front_face",
    )
    drawer.visual(
        Box((0.16, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.365, -0.005)),
        material=trim,
        name="pull",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=0.0, upper=0.22),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "board_to_wing",
        ArticulationType.REVOLUTE,
        parent=board,
        child=wing,
        origin=Origin(xyz=(0.45, 0.38, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=0.0, upper=1.20),
    )
    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.27, -0.009)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.30, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    wing = object_model.get_part("wing")
    drawer = object_model.get_part("drawer")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    board_tilt = object_model.get_articulation("carriage_to_board")
    wing_hinge = object_model.get_articulation("board_to_wing")
    drawer_slide = object_model.get_articulation("board_to_drawer")

    board_tilt_limits = board_tilt.motion_limits
    wing_limits = wing_hinge.motion_limits
    carriage_limits = carriage_slide.motion_limits
    drawer_limits = drawer_slide.motion_limits

    ctx.expect_gap(
        wing,
        board,
        axis="x",
        positive_elem="inner_frame",
        negative_elem="right_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="wing hinge line seats at board edge",
    )
    ctx.expect_gap(
        board,
        drawer,
        axis="z",
        positive_elem="runner_left",
        negative_elem="slide_left",
        min_gap=0.0005,
        max_gap=0.003,
        name="left runner carries drawer just below frame",
    )
    ctx.expect_gap(
        board,
        drawer,
        axis="z",
        positive_elem="runner_right",
        negative_elem="slide_right",
        min_gap=0.0005,
        max_gap=0.003,
        name="right runner carries drawer just below frame",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a="slide_left",
        elem_b="runner_left",
        min_overlap=0.24,
        name="closed drawer left slide stays engaged with runner",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a="slide_right",
        elem_b="runner_right",
        min_overlap=0.24,
        name="closed drawer right slide stays engaged with runner",
    )

    if carriage_limits is not None and carriage_limits.upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({carriage_slide: carriage_limits.upper}):
            high_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage raises on vertical mast",
            rest_pos is not None
            and high_pos is not None
            and high_pos[2] > rest_pos[2] + 0.18,
            details=f"rest={rest_pos}, high={high_pos}",
        )

    if board_tilt_limits is not None and board_tilt_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(board, elem="front_frame")
        with ctx.pose({board_tilt: board_tilt_limits.upper}):
            tilted_front = ctx.part_element_world_aabb(board, elem="front_frame")
        ctx.check(
            "main board front edge rises in tilt motion",
            rest_front is not None
            and tilted_front is not None
            and tilted_front[1][2] > rest_front[1][2] + 0.30,
            details=f"rest={rest_front}, tilted={tilted_front}",
        )

    if wing_limits is not None and wing_limits.upper is not None:
        rest_outer = ctx.part_element_world_aabb(wing, elem="outer_frame")
        with ctx.pose({wing_hinge: wing_limits.upper}):
            raised_outer = ctx.part_element_world_aabb(wing, elem="outer_frame")
        ctx.check(
            "reference wing folds upward from board edge",
            rest_outer is not None
            and raised_outer is not None
            and raised_outer[1][2] > rest_outer[1][2] + 0.12,
            details=f"rest={rest_outer}, raised={raised_outer}",
        )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                board,
                axes="y",
                elem_a="slide_left",
                elem_b="runner_left",
                min_overlap=0.095,
                name="extended drawer left slide retains insertion",
            )
            ctx.expect_overlap(
                drawer,
                board,
                axes="y",
                elem_a="slide_right",
                elem_b="runner_right",
                min_overlap=0.095,
                name="extended drawer right slide retains insertion",
            )
        ctx.check(
            "drawer extends forward on runners",
            rest_pos is not None
            and open_pos is not None
            and open_pos[1] > rest_pos[1] + 0.12,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    return ctx.report()


object_model = build_object_model()
