from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEG_Y = 0.38
COLUMN_TOP_Z = 0.655
LEG_TRAVEL = 0.22
TOP_HINGE_X = -0.30
TOP_HINGE_Z = 0.07


def _add_column_shell(
    part,
    *,
    prefix: str,
    center_y: float,
    outer_x: float,
    outer_y: float,
    wall: float,
    height: float,
    bottom_z: float,
    material,
) -> None:
    center_z = bottom_z + height / 2.0
    front_x = outer_x / 2.0 - wall / 2.0
    side_y = outer_y / 2.0 - wall / 2.0

    part.visual(
        Box((wall, outer_y, height)),
        origin=Origin(xyz=(front_x, center_y, center_z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((wall, outer_y, height)),
        origin=Origin(xyz=(-front_x, center_y, center_z)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((outer_x - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, center_y + side_y, center_z)),
        material=material,
        name=f"{prefix}_outer",
    )
    part.visual(
        Box((outer_x - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, center_y - side_y, center_z)),
        material=material,
        name=f"{prefix}_inner",
    )


def _add_key_button(
    model: ArticulatedObject,
    handset,
    *,
    name: str,
    center_xy: tuple[float, float],
    material,
) -> None:
    button = model.part(name)
    button.visual(
        Box((0.024, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=material,
        name="button_cap",
    )
    button.visual(
        Box((0.012, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=material,
        name="button_stem",
    )
    button.visual(
        Box((0.043, 0.023, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=material,
        name="button_retainer",
    )

    model.articulation(
        f"handset_to_{name}",
        ArticulationType.PRISMATIC,
        parent=handset,
        child=button,
        origin=Origin(xyz=(center_xy[0], center_xy[1], 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0035,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    powder_black = model.material("powder_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.30, 0.33, 1.0))
    warm_oak = model.material("warm_oak", rgba=(0.70, 0.55, 0.36, 1.0))
    handset_body = model.material("handset_body", rgba=(0.14, 0.15, 0.17, 1.0))
    handset_button = model.material("handset_button", rgba=(0.82, 0.84, 0.87, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.72, 0.08, 0.035)),
        origin=Origin(xyz=(0.0, -LEG_Y, 0.0175)),
        material=powder_black,
        name="left_foot",
    )
    frame.visual(
        Box((0.72, 0.08, 0.035)),
        origin=Origin(xyz=(0.0, LEG_Y, 0.0175)),
        material=powder_black,
        name="right_foot",
    )
    frame.visual(
        Box((0.10, 0.68, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=powder_black,
        name="base_tie",
    )
    _add_column_shell(
        frame,
        prefix="left_column",
        center_y=-LEG_Y,
        outer_x=0.11,
        outer_y=0.08,
        wall=0.006,
        height=0.62,
        bottom_z=0.035,
        material=graphite,
    )
    _add_column_shell(
        frame,
        prefix="right_column",
        center_y=LEG_Y,
        outer_x=0.11,
        outer_y=0.08,
        wall=0.006,
        height=0.62,
        bottom_z=0.035,
        material=graphite,
    )

    left_stage = model.part("left_stage")
    left_stage.visual(
        Box((0.098, 0.068, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=graphite,
        name="stage_body",
    )
    left_stage.visual(
        Box((0.10, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=powder_black,
        name="stage_cap",
    )

    right_stage = model.part("right_stage")
    right_stage.visual(
        Box((0.098, 0.068, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=graphite,
        name="stage_body",
    )
    right_stage.visual(
        Box((0.10, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=powder_black,
        name="stage_cap",
    )

    model.articulation(
        "frame_to_left_stage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_stage,
        origin=Origin(xyz=(0.0, -LEG_Y, COLUMN_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.05,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_right_stage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_stage,
        origin=Origin(xyz=(0.0, LEG_Y, COLUMN_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.05,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
        mimic=Mimic("frame_to_left_stage"),
    )

    upper_frame = model.part("upper_frame")
    upper_frame.visual(
        Box((0.10, 0.82, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=powder_black,
        name="cross_rail",
    )
    upper_frame.visual(
        Box((0.05, 0.86, 0.036)),
        origin=Origin(xyz=(TOP_HINGE_X, 0.0, TOP_HINGE_Z)),
        material=powder_black,
        name="rear_beam",
    )
    upper_frame.visual(
        Box((0.32, 0.05, 0.04)),
        origin=Origin(xyz=(-0.15, -0.355, 0.04)),
        material=powder_black,
        name="left_runner",
    )
    upper_frame.visual(
        Box((0.32, 0.05, 0.04)),
        origin=Origin(xyz=(-0.15, 0.355, 0.04)),
        material=powder_black,
        name="right_runner",
    )
    upper_frame.visual(
        Box((0.22, 0.07, 0.04)),
        origin=Origin(xyz=(-0.19, 0.0, 0.04)),
        material=powder_black,
        name="center_runner",
    )

    model.articulation(
        "left_stage_to_upper_frame",
        ArticulationType.FIXED,
        parent=left_stage,
        child=upper_frame,
        origin=Origin(xyz=(0.0, LEG_Y, 0.258)),
    )

    top = model.part("top")
    top.visual(
        Box((0.76, 1.12, 0.028)),
        origin=Origin(xyz=(0.41, 0.0, 0.04)),
        material=warm_oak,
        name="panel",
    )
    top.visual(
        Box((0.04, 1.06, 0.07)),
        origin=Origin(xyz=(0.77, 0.0, 0.005)),
        material=graphite,
        name="front_apron",
    )
    top.visual(
        Box((0.65, 0.03, 0.045)),
        origin=Origin(xyz=(0.39, -0.49, 0.0035)),
        material=graphite,
        name="left_rail",
    )
    top.visual(
        Box((0.65, 0.03, 0.045)),
        origin=Origin(xyz=(0.39, 0.49, 0.0035)),
        material=graphite,
        name="right_rail",
    )
    top.visual(
        Box((0.08, 0.98, 0.03)),
        origin=Origin(xyz=(0.10, 0.0, 0.011)),
        material=graphite,
        name="rear_cleat",
    )
    top.visual(
        Box((0.08, 0.05, 0.026)),
        origin=Origin(xyz=(0.02, -0.36, 0.031)),
        material=graphite,
        name="left_hinge_ear",
    )
    top.visual(
        Box((0.08, 0.05, 0.026)),
        origin=Origin(xyz=(0.02, 0.36, 0.031)),
        material=graphite,
        name="right_hinge_ear",
    )

    model.articulation(
        "upper_frame_to_top",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=top,
        origin=Origin(xyz=(TOP_HINGE_X, 0.0, TOP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=0.0,
            upper=0.95,
        ),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.05, 0.64, 0.018)),
        material=handset_body,
        name="strip_body",
    )

    model.articulation(
        "top_to_control_strip",
        ArticulationType.FIXED,
        parent=top,
        child=control_strip,
        origin=Origin(xyz=(0.74, 0.0, -0.039)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.094, 0.058, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=handset_body,
        name="back_plate",
    )
    handset.visual(
        Box((0.094, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.026, -0.001)),
        material=handset_body,
        name="bezel_top",
    )
    handset.visual(
        Box((0.094, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.026, -0.001)),
        material=handset_body,
        name="bezel_bottom",
    )
    handset.visual(
        Box((0.004, 0.046, 0.004)),
        origin=Origin(xyz=(-0.045, 0.0, -0.001)),
        material=handset_body,
        name="bezel_left",
    )
    handset.visual(
        Box((0.004, 0.046, 0.004)),
        origin=Origin(xyz=(0.045, 0.0, -0.001)),
        material=handset_body,
        name="bezel_right",
    )
    handset.visual(
        Box((0.004, 0.046, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=handset_body,
        name="bezel_center",
    )
    handset.visual(
        Box((0.086, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=handset_body,
        name="bezel_cross",
    )
    handset.visual(
        Box((0.094, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.027, -0.009)),
        material=handset_body,
        name="top_wall",
    )
    handset.visual(
        Box((0.094, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.027, -0.009)),
        material=handset_body,
        name="bottom_wall",
    )
    handset.visual(
        Box((0.004, 0.058, 0.014)),
        origin=Origin(xyz=(-0.045, 0.0, -0.009)),
        material=handset_body,
        name="left_wall",
    )
    handset.visual(
        Box((0.004, 0.058, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, -0.009)),
        material=handset_body,
        name="right_wall",
    )

    model.articulation(
        "control_strip_to_handset",
        ArticulationType.FIXED,
        parent=control_strip,
        child=handset,
        origin=Origin(xyz=(0.0, 0.24, -0.026), rpy=(math.pi, 0.0, 0.0)),
    )

    _add_key_button(
        model,
        handset,
        name="button_0_0",
        center_xy=(-0.022, 0.013),
        material=handset_button,
    )
    _add_key_button(
        model,
        handset,
        name="button_0_1",
        center_xy=(0.022, 0.013),
        material=handset_button,
    )
    _add_key_button(
        model,
        handset,
        name="button_1_0",
        center_xy=(-0.022, -0.013),
        material=handset_button,
    )
    _add_key_button(
        model,
        handset,
        name="button_1_1",
        center_xy=(0.022, -0.013),
        material=handset_button,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    top = object_model.get_part("top")
    handset = object_model.get_part("handset")
    button_0_0 = object_model.get_part("button_0_0")

    left_lift = object_model.get_articulation("frame_to_left_stage")
    top_hinge = object_model.get_articulation("upper_frame_to_top")
    button_joint = object_model.get_articulation("handset_to_button_0_0")

    ctx.expect_origin_distance(
        left_stage,
        right_stage,
        axes="z",
        max_dist=0.002,
        name="leg stages stay level at rest",
    )
    ctx.expect_overlap(
        left_stage,
        frame,
        axes="z",
        min_overlap=0.28,
        name="collapsed left stage remains deeply inserted",
    )
    ctx.expect_overlap(
        button_0_0,
        handset,
        axes="xy",
        min_overlap=0.010,
        name="button footprint stays inside the handset face",
    )

    rest_left = ctx.part_world_position(left_stage)
    rest_right = ctx.part_world_position(right_stage)
    rest_front = ctx.part_element_world_aabb(top, elem="front_apron")
    rest_button = ctx.part_world_position(button_0_0)

    lift_limits = left_lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({left_lift: lift_limits.upper}):
            ctx.expect_origin_distance(
                left_stage,
                right_stage,
                axes="z",
                max_dist=0.002,
                name="leg stages stay level when extended",
            )
            ctx.expect_overlap(
                left_stage,
                frame,
                axes="z",
                min_overlap=0.095,
                name="extended left stage still retains insertion",
            )
            extended_left = ctx.part_world_position(left_stage)
            extended_right = ctx.part_world_position(right_stage)
        ctx.check(
            "desk lifts upward on the telescoping legs",
            rest_left is not None
            and extended_left is not None
            and extended_left[2] > rest_left[2] + 0.18,
            details=f"rest={rest_left}, extended={extended_left}",
        )
        ctx.check(
            "right leg follows the driven left leg",
            rest_right is not None
            and extended_right is not None
            and rest_left is not None
            and extended_left is not None
            and abs(extended_left[2] - extended_right[2]) < 0.002
            and extended_right[2] > rest_right[2] + 0.18,
            details=(
                f"left_rest={rest_left}, right_rest={rest_right}, "
                f"left_extended={extended_left}, right_extended={extended_right}"
            ),
        )

    hinge_limits = top_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.upper is not None:
        with ctx.pose({top_hinge: hinge_limits.upper}):
            tilted_front = ctx.part_element_world_aabb(top, elem="front_apron")
        ctx.check(
            "tilting top lifts the front edge",
            rest_front is not None
            and tilted_front is not None
            and tilted_front[1][2] > rest_front[1][2] + 0.25,
            details=f"rest={rest_front}, tilted={tilted_front}",
        )

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_button = ctx.part_world_position(button_0_0)
        ctx.check(
            "handset button depresses inward",
            rest_button is not None
            and pressed_button is not None
            and pressed_button[2] > rest_button[2] + 0.002,
            details=f"rest={rest_button}, pressed={pressed_button}",
        )

    return ctx.report()


object_model = build_object_model()
