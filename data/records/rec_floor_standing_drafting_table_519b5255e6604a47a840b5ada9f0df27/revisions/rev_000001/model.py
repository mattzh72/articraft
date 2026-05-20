from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table")

    powder = model.material("powder", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    maple = model.material("maple", rgba=(0.76, 0.68, 0.55, 1.0))
    drawer_paint = model.material("drawer_paint", rgba=(0.73, 0.74, 0.76, 1.0))

    post_spacing = 0.72
    post_x = post_spacing * 0.5
    foot_size = (0.14, 0.68, 0.04)
    outer_post_size = (0.10, 0.08, 0.72)
    wall = 0.012
    inner_column_size = (0.068, outer_post_size[1] - 2.0 * wall, 0.54)
    stage_travel = 0.16

    board_width = 1.18
    board_depth = 0.82
    board_thickness = 0.026

    base = model.part("base")
    for prefix, x_pos in (("left", -post_x), ("right", post_x)):
        base.visual(
            Box(foot_size),
            origin=Origin(xyz=(x_pos, 0.0, foot_size[2] * 0.5)),
            material=powder,
            name=f"{prefix}_foot",
        )
        base.visual(
            Box((outer_post_size[0], wall, outer_post_size[2])),
            origin=Origin(
                xyz=(
                    x_pos,
                    outer_post_size[1] * 0.5 - wall * 0.5,
                    foot_size[2] + outer_post_size[2] * 0.5,
                )
            ),
            material=powder,
            name=f"{prefix}_post_front",
        )
        base.visual(
            Box((outer_post_size[0], wall, outer_post_size[2])),
            origin=Origin(
                xyz=(
                    x_pos,
                    -outer_post_size[1] * 0.5 + wall * 0.5,
                    foot_size[2] + outer_post_size[2] * 0.5,
                )
            ),
            material=powder,
            name=f"{prefix}_post_back",
        )
        base.visual(
            Box((wall, outer_post_size[1] - 2.0 * wall, outer_post_size[2])),
            origin=Origin(
                xyz=(
                    x_pos - outer_post_size[0] * 0.5 + wall * 0.5,
                    0.0,
                    foot_size[2] + outer_post_size[2] * 0.5,
                )
            ),
            material=powder,
            name=f"{prefix}_post_inner",
        )
        base.visual(
            Box((wall, outer_post_size[1] - 2.0 * wall, outer_post_size[2])),
            origin=Origin(
                xyz=(
                    x_pos + outer_post_size[0] * 0.5 - wall * 0.5,
                    0.0,
                    foot_size[2] + outer_post_size[2] * 0.5,
                )
            ),
            material=powder,
            name=f"{prefix}_post_outer",
        )

    base.visual(
        Box((post_spacing - outer_post_size[0], 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=graphite,
        name="stretcher",
    )
    base.visual(
        Box((post_spacing - 0.02, 0.04, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="tie_bar",
    )

    def add_inner_stage(part_name: str) -> None:
        stage = model.part(part_name)
        stage.visual(
            Box(inner_column_size),
            origin=Origin(xyz=(0.0, 0.0, 0.01)),
            material=steel,
            name="column",
        )
        stage.visual(
            Box((0.094, 0.080, 0.05)),
            origin=Origin(xyz=(0.0, 0.0, 0.305)),
            material=graphite,
            name="top_cap",
        )

    add_inner_stage("left_inner")
    add_inner_stage("right_inner")

    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child="left_inner",
        origin=Origin(xyz=(-post_x, 0.0, foot_size[2] + outer_post_size[2])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.12,
            lower=0.0,
            upper=stage_travel,
        ),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child="right_inner",
        origin=Origin(xyz=(post_x, 0.0, foot_size[2] + outer_post_size[2])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.12,
            lower=0.0,
            upper=stage_travel,
        ),
        mimic=Mimic("left_lift"),
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((post_spacing, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.035, -0.05)),
        material=graphite,
        name="crossbeam",
    )
    support_head.visual(
        Box((0.060, 0.080, 0.088)),
        origin=Origin(xyz=(-post_x, -0.010, -0.044)),
        material=graphite,
        name="left_cheek",
    )
    support_head.visual(
        Box((0.060, 0.080, 0.088)),
        origin=Origin(xyz=(post_x, -0.010, -0.044)),
        material=graphite,
        name="right_cheek",
    )
    support_head.visual(
        Box((0.094, 0.080, 0.012)),
        origin=Origin(xyz=(-post_x, 0.0, -0.094)),
        material=steel,
        name="left_pad",
    )
    support_head.visual(
        Box((0.094, 0.080, 0.012)),
        origin=Origin(xyz=(post_x, 0.0, -0.094)),
        material=steel,
        name="right_pad",
    )
    support_head.visual(
        Box((0.26, 0.22, 0.06)),
        origin=Origin(xyz=(0.0, 0.07, -0.07)),
        material=steel,
        name="center_support",
    )
    support_head.visual(
        Cylinder(radius=0.018, length=0.28),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    support_head.visual(
        Box((0.20, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.030, -0.020)),
        material=steel,
        name="hinge_block",
    )

    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent="left_inner",
        child=support_head,
        origin=Origin(xyz=(post_x, 0.0, 0.43)),
    )

    board = model.part("board")
    board.visual(
        Box((board_width, board_depth, board_thickness)),
        origin=Origin(xyz=(0.0, board_depth * 0.5, board_thickness * 0.5)),
        material=maple,
        name="panel",
    )
    board.visual(
        Box((0.030, board_depth - 0.04, 0.05)),
        origin=Origin(
            xyz=(
                -board_width * 0.5 + 0.015,
                0.43,
                -0.025,
            )
        ),
        material=graphite,
        name="side_rail_0",
    )
    board.visual(
        Box((0.030, board_depth - 0.04, 0.05)),
        origin=Origin(
            xyz=(
                board_width * 0.5 - 0.015,
                0.43,
                -0.025,
            )
        ),
        material=graphite,
        name="side_rail_1",
    )
    for index, y_pos in enumerate((0.45, 0.71)):
        board.visual(
            Box((0.270, 0.03, 0.02)),
            origin=Origin(xyz=(-0.425, y_pos, -0.05)),
            material=graphite,
            name=f"runner_mount_left_{index}",
        )
        board.visual(
            Box((0.270, 0.03, 0.02)),
            origin=Origin(xyz=(0.425, y_pos, -0.05)),
            material=graphite,
            name=f"runner_mount_right_{index}",
        )
    board.visual(
        Box((0.018, 0.26, 0.008)),
        origin=Origin(xyz=(-0.283, 0.58, -0.046)),
        material=steel,
        name="runner_0",
    )
    board.visual(
        Box((0.018, 0.26, 0.008)),
        origin=Origin(xyz=(0.283, 0.58, -0.046)),
        material=steel,
        name="runner_1",
    )
    board.visual(
        Box((0.24, 0.24, 0.04)),
        origin=Origin(xyz=(0.0, 0.18, -0.02)),
        material=graphite,
        name="board_spine",
    )
    board.visual(
        Box((0.94, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, board_depth - 0.012, board_thickness + 0.009)),
        material=graphite,
        name="front_lip",
    )

    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=board,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=0.0,
            upper=1.12,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.54, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, 0.08, 0.005)),
        material=drawer_paint,
        name="bottom",
    )
    drawer.visual(
        Box((0.01, 0.28, 0.045)),
        origin=Origin(xyz=(-0.271, 0.08, 0.0225)),
        material=drawer_paint,
        name="side_0",
    )
    drawer.visual(
        Box((0.01, 0.28, 0.045)),
        origin=Origin(xyz=(0.271, 0.08, 0.0225)),
        material=drawer_paint,
        name="side_1",
    )
    drawer.visual(
        Box((0.54, 0.01, 0.045)),
        origin=Origin(xyz=(0.0, -0.055, 0.0225)),
        material=drawer_paint,
        name="back",
    )
    drawer.visual(
        Box((0.60, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, 0.229, 0.0225)),
        material=drawer_paint,
        name="front",
    )
    drawer.visual(
        Box((0.18, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.245, 0.0225)),
        material=graphite,
        name="pull",
    )
    drawer.visual(
        Box((0.014, 0.24, 0.012)),
        origin=Origin(xyz=(-0.283, 0.02, -0.006)),
        material=steel,
        name="drawer_runner_0",
    )
    drawer.visual(
        Box((0.014, 0.24, 0.012)),
        origin=Origin(xyz=(0.283, 0.02, -0.006)),
        material=steel,
        name="drawer_runner_1",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.58, -0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_inner = object_model.get_part("left_inner")
    right_inner = object_model.get_part("right_inner")
    support_head = object_model.get_part("support_head")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    left_lift = object_model.get_articulation("left_lift")
    board_tilt = object_model.get_articulation("board_tilt")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.expect_gap(
        support_head,
        right_inner,
        axis="z",
        positive_elem="right_pad",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="right post carries support head at rest",
    )

    lift_limits = left_lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        rest_left = ctx.part_world_position(left_inner)
        rest_right = ctx.part_world_position(right_inner)
        with ctx.pose({left_lift: lift_limits.upper}):
            ctx.expect_gap(
                support_head,
                right_inner,
                axis="z",
                positive_elem="right_pad",
                negative_elem="top_cap",
                max_gap=0.001,
                max_penetration=0.0,
                name="right post carries support head at full height",
            )
            high_left = ctx.part_world_position(left_inner)
            high_right = ctx.part_world_position(right_inner)
        ctx.check(
            "twin posts lift together",
            rest_left is not None
            and rest_right is not None
            and high_left is not None
            and high_right is not None
            and high_left[2] > rest_left[2] + 0.12
            and abs(high_left[2] - high_right[2]) < 1e-6
            and abs(rest_left[2] - rest_right[2]) < 1e-6,
            details=(
                f"rest_left={rest_left}, rest_right={rest_right}, "
                f"high_left={high_left}, high_right={high_right}"
            ),
        )

    tilt_limits = board_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_front = ctx.part_element_world_aabb(board, elem="front_lip")
        with ctx.pose({board_tilt: tilt_limits.upper}):
            raised_front = ctx.part_element_world_aabb(board, elem="front_lip")
        ctx.check(
            "drawing board tilts upward",
            rest_front is not None
            and raised_front is not None
            and raised_front[0][2] > rest_front[0][2] + 0.20,
            details=f"rest_front={rest_front}, raised_front={raised_front}",
        )

    ctx.expect_within(
        drawer,
        board,
        axes="x",
        inner_elem="front",
        outer_elem="panel",
        margin=0.0,
        name="drawer stays within board width",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="x",
        elem_a="drawer_runner_0",
        elem_b="runner_0",
        min_overlap=0.010,
        name="drawer runner stays laterally aligned",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a="drawer_runner_0",
        elem_b="runner_0",
        min_overlap=0.18,
        name="drawer stays engaged on runners when closed",
    )

    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        closed_front = ctx.part_element_world_aabb(drawer, elem="front")
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            open_front = ctx.part_element_world_aabb(drawer, elem="front")
            ctx.expect_overlap(
                drawer,
                board,
                axes="y",
                elem_a="drawer_runner_0",
                elem_b="runner_0",
                min_overlap=0.10,
                name="drawer retains insertion at full extension",
            )
        ctx.check(
            "drawer slides forward",
            closed_front is not None
            and open_front is not None
            and open_front[0][1] > closed_front[0][1] + 0.10,
            details=f"closed_front={closed_front}, open_front={open_front}",
        )

    return ctx.report()


object_model = build_object_model()
