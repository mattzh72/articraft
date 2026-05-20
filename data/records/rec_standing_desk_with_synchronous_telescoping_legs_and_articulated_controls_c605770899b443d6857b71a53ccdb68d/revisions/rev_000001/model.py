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


DESK_LENGTH = 1.80
DESK_DEPTH = 0.82
TOP_THICKNESS = 0.032

MODULE_OFFSET = 0.62

FOOT_WIDTH = 0.10
FOOT_DEPTH = 0.72
FOOT_THICKNESS = 0.03

OUTER_COL_X = 0.085
OUTER_COL_Y = 0.065
OUTER_COL_H = 0.55
OUTER_WALL = 0.006

INNER_STAGE_X = 0.070
INNER_STAGE_Y = 0.050
INNER_STAGE_H = 0.62
INNER_STAGE_CENTER_Z = -0.178

TOP_PLATE_X = 0.100
TOP_PLATE_Y = 0.078
TOP_PLATE_THICKNESS = 0.012
TOP_PLATE_CENTER_Z = 0.126
TOP_PLATE_TOP_Z = TOP_PLATE_CENTER_Z + TOP_PLATE_THICKNESS / 2.0

LIFT_TRAVEL = 0.36
LIFT_LIMITS = MotionLimits(
    effort=1600.0,
    velocity=0.04,
    lower=0.0,
    upper=LIFT_TRAVEL,
)


def _add_outer_column(part, material) -> None:
    part.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS / 2.0)),
        material=material,
        name="foot",
    )

    wall_center_z = FOOT_THICKNESS + OUTER_COL_H / 2.0
    front_back_y = OUTER_COL_Y / 2.0 - OUTER_WALL / 2.0
    side_x = OUTER_COL_X / 2.0 - OUTER_WALL / 2.0

    part.visual(
        Box((OUTER_COL_X, OUTER_WALL, OUTER_COL_H)),
        origin=Origin(xyz=(0.0, -front_back_y, wall_center_z)),
        material=material,
        name="front_wall",
    )
    part.visual(
        Box((OUTER_COL_X, OUTER_WALL, OUTER_COL_H)),
        origin=Origin(xyz=(0.0, front_back_y, wall_center_z)),
        material=material,
        name="rear_wall",
    )
    part.visual(
        Box((OUTER_WALL, OUTER_COL_Y - 2.0 * OUTER_WALL, OUTER_COL_H)),
        origin=Origin(xyz=(-side_x, 0.0, wall_center_z)),
        material=material,
        name="left_wall",
    )
    part.visual(
        Box((OUTER_WALL, OUTER_COL_Y - 2.0 * OUTER_WALL, OUTER_COL_H)),
        origin=Origin(xyz=(side_x, 0.0, wall_center_z)),
        material=material,
        name="right_wall",
    )


def _add_inner_stage(part, material) -> None:
    part.visual(
        Box((INNER_STAGE_X, INNER_STAGE_Y, INNER_STAGE_H)),
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_CENTER_Z)),
        material=material,
        name="inner_tube",
    )
    part.visual(
        Box((0.003, 0.016, 0.34)),
        origin=Origin(xyz=(-0.035, 0.0, -0.27)),
        material=material,
        name="left_guide",
    )
    part.visual(
        Box((0.003, 0.016, 0.34)),
        origin=Origin(xyz=(0.035, 0.0, -0.27)),
        material=material,
        name="right_guide",
    )
    part.visual(
        Box((0.020, 0.003, 0.34)),
        origin=Origin(xyz=(0.0, -0.025, -0.27)),
        material=material,
        name="front_guide",
    )
    part.visual(
        Box((0.020, 0.003, 0.34)),
        origin=Origin(xyz=(0.0, 0.025, -0.27)),
        material=material,
        name="rear_guide",
    )
    part.visual(
        Box((0.082, 0.058, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=material,
        name="head_block",
    )
    part.visual(
        Box((TOP_PLATE_X, TOP_PLATE_Y, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_CENTER_Z)),
        material=material,
        name="top_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_style_standing_desk")

    top_oak = model.material("top_oak", rgba=(0.69, 0.55, 0.39, 1.0))
    black_steel = model.material("black_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))

    center_base = model.part("center_base")
    _add_outer_column(center_base, black_steel)
    center_base.visual(
        Box((MODULE_OFFSET * 2.0, 0.028, 0.050)),
        origin=Origin(
            xyz=(
                0.0,
                OUTER_COL_Y / 2.0 + 0.014,
                FOOT_THICKNESS + OUTER_COL_H * 0.68,
            )
        ),
        material=black_steel,
        name="rear_stretcher",
    )

    left_base = model.part("left_base")
    _add_outer_column(left_base, black_steel)
    left_base.visual(
        Box((0.060, 0.028, 0.050)),
        origin=Origin(
            xyz=(
                -0.030,
                OUTER_COL_Y / 2.0 + 0.014,
                FOOT_THICKNESS + OUTER_COL_H * 0.68,
            )
        ),
        material=black_steel,
        name="rear_clamp",
    )

    right_base = model.part("right_base")
    _add_outer_column(right_base, black_steel)
    right_base.visual(
        Box((0.060, 0.028, 0.050)),
        origin=Origin(
            xyz=(
                0.030,
                OUTER_COL_Y / 2.0 + 0.014,
                FOOT_THICKNESS + OUTER_COL_H * 0.68,
            )
        ),
        material=black_steel,
        name="rear_clamp",
    )

    model.articulation(
        "center_base_to_left_base",
        ArticulationType.FIXED,
        parent=center_base,
        child=left_base,
        origin=Origin(xyz=(-MODULE_OFFSET, 0.0, 0.0)),
    )
    model.articulation(
        "center_base_to_right_base",
        ArticulationType.FIXED,
        parent=center_base,
        child=right_base,
        origin=Origin(xyz=(MODULE_OFFSET, 0.0, 0.0)),
    )

    center_stage = model.part("center_stage")
    _add_inner_stage(center_stage, graphite)
    left_stage = model.part("left_stage")
    _add_inner_stage(left_stage, graphite)
    right_stage = model.part("right_stage")
    _add_inner_stage(right_stage, graphite)

    lift_origin = Origin(xyz=(0.0, 0.0, FOOT_THICKNESS + OUTER_COL_H))
    center_lift = model.articulation(
        "center_lift",
        ArticulationType.PRISMATIC,
        parent=center_base,
        child=center_stage,
        origin=lift_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=LIFT_LIMITS,
    )
    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=left_base,
        child=left_stage,
        origin=lift_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=LIFT_LIMITS,
        mimic=Mimic(joint="center_lift"),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=right_base,
        child=right_stage,
        origin=lift_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=LIFT_LIMITS,
        mimic=Mimic(joint="center_lift"),
    )

    desktop_frame = model.part("desktop_frame")
    desktop_frame.visual(
        Box((DESK_LENGTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=top_oak,
        name="tabletop",
    )
    desktop_frame.visual(
        Box((1.58, 0.060, 0.044)),
        origin=Origin(xyz=(0.0, -0.230, 0.022)),
        material=graphite,
        name="front_rail",
    )
    desktop_frame.visual(
        Box((1.58, 0.060, 0.044)),
        origin=Origin(xyz=(0.0, 0.230, 0.022)),
        material=graphite,
        name="rear_rail",
    )
    for name, x_pos in (("left_pad", -MODULE_OFFSET), ("center_pad", 0.0), ("right_pad", MODULE_OFFSET)):
        desktop_frame.visual(
            Box((0.150, 0.140, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, 0.012)),
            material=graphite,
            name=name,
        )
        desktop_frame.visual(
            Box((0.140, 0.520, 0.044)),
            origin=Origin(xyz=(x_pos, 0.0, 0.022)),
            material=graphite,
            name=f"{name}_brace",
        )

    model.articulation(
        "center_stage_to_desktop_frame",
        ArticulationType.FIXED,
        parent=center_stage,
        child=desktop_frame,
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_TOP_Z)),
    )

    controller = model.part("controller")
    controller.visual(
        Box((0.092, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.012, -0.004)),
        material=charcoal,
        name="mounting_plate",
    )
    controller.visual(
        Box((0.132, 0.052, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=charcoal,
        name="housing",
    )
    controller.visual(
        Box((0.132, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.034, -0.015)),
        material=charcoal,
        name="front_lip",
    )
    controller.visual(
        Box((0.028, 0.014, 0.010)),
        origin=Origin(xyz=(-0.026, -0.026, -0.013)),
        material=charcoal,
        name="down_pivot_boss",
    )
    controller.visual(
        Box((0.028, 0.014, 0.010)),
        origin=Origin(xyz=(0.026, -0.026, -0.013)),
        material=charcoal,
        name="up_pivot_boss",
    )

    model.articulation(
        "desktop_frame_to_controller",
        ArticulationType.FIXED,
        parent=desktop_frame,
        child=controller,
        origin=Origin(xyz=(0.0, -0.352, 0.044)),
    )

    up_paddle = model.part("up_paddle")
    up_paddle.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=graphite,
        name="pivot_barrel",
    )
    up_paddle.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, -0.006)),
        material=graphite,
        name="neck",
    )
    up_paddle.visual(
        Box((0.032, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.006, -0.025)),
        material=graphite,
        name="blade",
    )

    down_paddle = model.part("down_paddle")
    down_paddle.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=graphite,
        name="pivot_barrel",
    )
    down_paddle.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, -0.006)),
        material=graphite,
        name="neck",
    )
    down_paddle.visual(
        Box((0.032, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.006, -0.025)),
        material=graphite,
        name="blade",
    )

    paddle_limits = MotionLimits(
        effort=3.0,
        velocity=4.0,
        lower=-0.28,
        upper=0.32,
    )
    model.articulation(
        "controller_to_up_paddle",
        ArticulationType.REVOLUTE,
        parent=controller,
        child=up_paddle,
        origin=Origin(xyz=(0.026, -0.046, -0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=paddle_limits,
    )
    model.articulation(
        "controller_to_down_paddle",
        ArticulationType.REVOLUTE,
        parent=controller,
        child=down_paddle,
        origin=Origin(xyz=(-0.026, -0.046, -0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=paddle_limits,
    )

    return model


def _check_stage_fit(
    ctx: TestContext,
    stage,
    base,
    *,
    label: str,
    pad_name: str,
    min_insertion: float,
) -> None:
    ctx.expect_gap(
        stage,
        base,
        axis="x",
        positive_elem="inner_tube",
        negative_elem="left_wall",
        min_gap=0.001,
        max_gap=0.004,
        name=f"{label} stage clears left sleeve wall",
    )
    ctx.expect_gap(
        base,
        stage,
        axis="x",
        positive_elem="right_wall",
        negative_elem="inner_tube",
        min_gap=0.001,
        max_gap=0.004,
        name=f"{label} stage clears right sleeve wall",
    )
    ctx.expect_gap(
        stage,
        base,
        axis="y",
        positive_elem="inner_tube",
        negative_elem="front_wall",
        min_gap=0.001,
        max_gap=0.004,
        name=f"{label} stage clears front sleeve wall",
    )
    ctx.expect_gap(
        base,
        stage,
        axis="y",
        positive_elem="rear_wall",
        negative_elem="inner_tube",
        min_gap=0.001,
        max_gap=0.004,
        name=f"{label} stage clears rear sleeve wall",
    )
    ctx.expect_overlap(
        stage,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="front_wall",
        min_overlap=min_insertion,
        name=f"{label} stage remains inserted in its outer column",
    )
    ctx.expect_contact(
        stage,
        "desktop_frame",
        elem_a="top_plate",
        elem_b=pad_name,
        name=f"{label} stage top plate supports the shared desktop frame",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_base = object_model.get_part("center_base")
    left_base = object_model.get_part("left_base")
    right_base = object_model.get_part("right_base")
    center_stage = object_model.get_part("center_stage")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    desktop_frame = object_model.get_part("desktop_frame")
    controller = object_model.get_part("controller")
    up_paddle = object_model.get_part("up_paddle")
    down_paddle = object_model.get_part("down_paddle")

    center_lift = object_model.get_articulation("center_lift")
    up_joint = object_model.get_articulation("controller_to_up_paddle")
    down_joint = object_model.get_articulation("controller_to_down_paddle")

    _check_stage_fit(ctx, left_stage, left_base, label="left", pad_name="left_pad", min_insertion=0.40)
    _check_stage_fit(ctx, center_stage, center_base, label="center", pad_name="center_pad", min_insertion=0.40)
    _check_stage_fit(ctx, right_stage, right_base, label="right", pad_name="right_pad", min_insertion=0.40)

    ctx.expect_gap(
        desktop_frame,
        controller,
        axis="z",
        positive_elem="tabletop",
        negative_elem="mounting_plate",
        max_gap=0.001,
        max_penetration=1e-6,
        name="controller mounting plate seats against the desk underside",
    )
    ctx.expect_gap(
        up_paddle,
        down_paddle,
        axis="x",
        positive_elem="blade",
        negative_elem="blade",
        min_gap=0.006,
        name="up and down paddles remain visibly split",
    )

    rest_center = ctx.part_world_position(center_stage)
    rest_left = ctx.part_world_position(left_stage)
    rest_right = ctx.part_world_position(right_stage)

    lift_limits = center_lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({center_lift: lift_limits.upper}):
            _check_stage_fit(
                ctx,
                left_stage,
                left_base,
                label="left extended",
                pad_name="left_pad",
                min_insertion=0.10,
            )
            _check_stage_fit(
                ctx,
                center_stage,
                center_base,
                label="center extended",
                pad_name="center_pad",
                min_insertion=0.10,
            )
            _check_stage_fit(
                ctx,
                right_stage,
                right_base,
                label="right extended",
                pad_name="right_pad",
                min_insertion=0.10,
            )

            extended_center = ctx.part_world_position(center_stage)
            extended_left = ctx.part_world_position(left_stage)
            extended_right = ctx.part_world_position(right_stage)

        ctx.check(
            "all three lifting stages rise together",
            rest_center is not None
            and rest_left is not None
            and rest_right is not None
            and extended_center is not None
            and extended_left is not None
            and extended_right is not None
            and extended_center[2] > rest_center[2] + 0.30
            and abs((extended_left[2] - rest_left[2]) - (extended_center[2] - rest_center[2])) < 1e-6
            and abs((extended_right[2] - rest_right[2]) - (extended_center[2] - rest_center[2])) < 1e-6,
            details=(
                f"rest center={rest_center}, left={rest_left}, right={rest_right}; "
                f"extended center={extended_center}, left={extended_left}, right={extended_right}"
            ),
        )

    up_rest_aabb = ctx.part_element_world_aabb(up_paddle, elem="blade")
    if up_joint.motion_limits is not None and up_joint.motion_limits.upper is not None:
        with ctx.pose({up_joint: up_joint.motion_limits.upper}):
            up_pressed_aabb = ctx.part_element_world_aabb(up_paddle, elem="blade")
        ctx.check(
            "up paddle rotates on its local pivot",
            up_rest_aabb is not None
            and up_pressed_aabb is not None
            and up_pressed_aabb[1][2] > up_rest_aabb[1][2] + 0.003,
            details=f"rest={up_rest_aabb}, pressed={up_pressed_aabb}",
        )

    down_rest_aabb = ctx.part_element_world_aabb(down_paddle, elem="blade")
    if down_joint.motion_limits is not None and down_joint.motion_limits.upper is not None:
        with ctx.pose({down_joint: down_joint.motion_limits.upper}):
            down_pressed_aabb = ctx.part_element_world_aabb(down_paddle, elem="blade")
        ctx.check(
            "down paddle rotates on its local pivot",
            down_rest_aabb is not None
            and down_pressed_aabb is not None
            and down_pressed_aabb[1][2] > down_rest_aabb[1][2] + 0.003,
            details=f"rest={down_rest_aabb}, pressed={down_pressed_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
