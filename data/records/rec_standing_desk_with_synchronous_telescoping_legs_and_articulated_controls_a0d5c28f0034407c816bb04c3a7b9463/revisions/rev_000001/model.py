from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.62
FOOT_WIDTH = 0.09
FOOT_THICKNESS = 0.03

COLUMN_CENTER_X = 0.39
COLUMN_CENTER_Y = -0.03
COLUMN_OUTER_X = 0.092
COLUMN_OUTER_Y = 0.062
COLUMN_HEIGHT = 0.58
COLUMN_WALL = 0.010
COLUMN_BASE = 0.012

STAGE_X = 0.064
STAGE_Y = 0.034
STAGE_LENGTH = 0.58
STAGE_TOP_Z = STAGE_LENGTH * 0.5
STAGE_TOP_PAD_Z = 0.299
STAGE_TOP_PLANE_Z = 0.308

DESK_WIDTH = 1.14
DESK_DEPTH = 0.72
WORKTOP_THICKNESS = 0.03


def _column_shell() -> cq.Workplane:
    inner_x = COLUMN_OUTER_X - (2.0 * COLUMN_WALL)
    inner_y = COLUMN_OUTER_Y - (2.0 * COLUMN_WALL)
    return (
        cq.Workplane("XY")
        .box(
            COLUMN_OUTER_X,
            COLUMN_OUTER_Y,
            COLUMN_HEIGHT,
            centered=(True, True, False),
        )
        .faces(">Z")
        .workplane()
        .rect(inner_x, inner_y)
        .cutBlind(-(COLUMN_HEIGHT - COLUMN_BASE))
    )


def _add_stage(model: ArticulatedObject, name: str, material: str) -> None:
    stage = model.part(name)
    stage.visual(
        Box((STAGE_X, STAGE_Y, STAGE_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="stage_tube",
    )
    stage.visual(
        Box((0.076, 0.046, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.271)),
        material=material,
        name="mount_head",
    )
    stage.visual(
        Box((0.110, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_TOP_PAD_Z)),
        material=material,
        name="top_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    base_steel = model.material("base_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    lift_steel = model.material("lift_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    desk_black = model.material("desk_black", rgba=(0.13, 0.14, 0.15, 1.0))
    top_finish = model.material("top_finish", rgba=(0.68, 0.56, 0.42, 1.0))
    trim_black = model.material("trim_black", rgba=(0.11, 0.11, 0.12, 1.0))
    control_grey = model.material("control_grey", rgba=(0.32, 0.34, 0.37, 1.0))
    button_grey = model.material("button_grey", rgba=(0.55, 0.57, 0.60, 1.0))

    base_frame = model.part("base_frame")
    for side, x_pos in (("left", -COLUMN_CENTER_X), ("right", COLUMN_CENTER_X)):
        base_frame.visual(
            Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_THICKNESS)),
            origin=Origin(xyz=(x_pos, 0.0, FOOT_THICKNESS * 0.5)),
            material=base_steel,
            name=f"{side}_foot",
        )
        base_frame.visual(
            Box((0.120, 0.094, 0.080)),
            origin=Origin(xyz=(x_pos, COLUMN_CENTER_Y, 0.070)),
            material=base_steel,
            name=f"{side}_motor_box",
        )
        base_frame.visual(
            mesh_from_cadquery(_column_shell(), f"{side}_column_shell"),
            origin=Origin(xyz=(x_pos, COLUMN_CENTER_Y, FOOT_THICKNESS)),
            material=desk_black,
            name=f"{side}_column_shell",
        )

    base_frame.visual(
        Box((0.720, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, COLUMN_CENTER_Y, 0.110)),
        material=base_steel,
        name="lower_stretcher",
    )
    base_frame.visual(
        Box((0.720, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, COLUMN_CENTER_Y, 0.405)),
        material=base_steel,
        name="upper_stretcher",
    )
    base_frame.visual(
        Box((0.780, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, 0.170, 0.040)),
        material=base_steel,
        name="front_tie",
    )

    _add_stage(model, "left_stage", lift_steel.name)
    _add_stage(model, "right_stage", lift_steel.name)

    left_leg_slide = model.articulation(
        "left_leg_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child="left_stage",
        origin=Origin(
            xyz=(
                -COLUMN_CENTER_X,
                COLUMN_CENTER_Y,
                FOOT_THICKNESS + COLUMN_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.050,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "right_leg_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child="right_stage",
        origin=Origin(
            xyz=(
                COLUMN_CENTER_X,
                COLUMN_CENTER_Y,
                FOOT_THICKNESS + COLUMN_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.050,
            lower=0.0,
            upper=0.18,
        ),
        mimic=Mimic("left_leg_slide"),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.080, 0.480, 0.040)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, -0.010, 0.020)),
        material=desk_black,
        name="left_side_rail",
    )
    top_frame.visual(
        Box((0.080, 0.480, 0.040)),
        origin=Origin(xyz=(COLUMN_CENTER_X, -0.010, 0.020)),
        material=desk_black,
        name="right_side_rail",
    )
    top_frame.visual(
        Box((0.900, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.220, 0.020)),
        material=desk_black,
        name="front_rail",
    )
    top_frame.visual(
        Box((0.900, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.230, 0.020)),
        material=desk_black,
        name="rear_rail",
    )
    top_frame.visual(
        Box((0.760, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.010, 0.014)),
        material=desk_black,
        name="center_brace",
    )
    top_frame.visual(
        Box((0.040, 0.030, 0.072)),
        origin=Origin(xyz=(-0.430, -0.275, 0.076)),
        material=trim_black,
        name="left_hinge_post",
    )
    top_frame.visual(
        Box((0.040, 0.030, 0.072)),
        origin=Origin(xyz=(0.430, -0.275, 0.076)),
        material=trim_black,
        name="right_hinge_post",
    )
    top_frame.visual(
        Cylinder(radius=0.016, length=0.920),
        origin=Origin(
            xyz=(0.0, -0.285, 0.128),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lift_steel,
        name="hinge_beam",
    )
    top_frame.visual(
        Box((0.024, 0.220, 0.010)),
        origin=Origin(xyz=(-0.350, 0.170, -0.005)),
        material=trim_black,
        name="left_guide",
    )
    top_frame.visual(
        Box((0.024, 0.220, 0.010)),
        origin=Origin(xyz=(0.350, 0.170, -0.005)),
        material=trim_black,
        name="right_guide",
    )

    model.articulation(
        "left_stage_to_top_frame",
        ArticulationType.FIXED,
        parent="left_stage",
        child=top_frame,
        origin=Origin(
            xyz=(
                COLUMN_CENTER_X,
                -COLUMN_CENTER_Y,
                STAGE_TOP_PLANE_Z,
            )
        ),
    )

    worktop = model.part("worktop")
    worktop.visual(
        Box((DESK_WIDTH, DESK_DEPTH, WORKTOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.370, 0.034)),
        material=top_finish,
        name="main_panel",
    )
    worktop.visual(
        Box((0.900, 0.050, 0.038)),
        origin=Origin(xyz=(0.0, 0.040, 0.000)),
        material=trim_black,
        name="rear_apron",
    )
    worktop.visual(
        Box((DESK_WIDTH - 0.020, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.708, -0.006)),
        material=trim_black,
        name="front_apron",
    )
    worktop.visual(
        Box((0.050, 0.450, 0.040)),
        origin=Origin(xyz=(-0.360, 0.340, -0.001)),
        material=trim_black,
        name="left_stiffener",
    )
    worktop.visual(
        Box((0.050, 0.450, 0.040)),
        origin=Origin(xyz=(0.360, 0.340, -0.001)),
        material=trim_black,
        name="right_stiffener",
    )
    worktop.visual(
        Box((0.800, 0.050, 0.029)),
        origin=Origin(xyz=(0.0, 0.180, 0.0045)),
        material=trim_black,
        name="tray_mount",
    )

    model.articulation(
        "worktop_tilt",
        ArticulationType.REVOLUTE,
        parent=top_frame,
        child=worktop,
        origin=Origin(xyz=(0.0, -0.285, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=math.radians(52.0),
        ),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.180, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=control_grey,
        name="housing",
    )
    control_strip.visual(
        Box((0.090, 0.014, 0.003)),
        origin=Origin(xyz=(-0.022, -0.010, -0.0165)),
        material=button_grey,
        name="label_bar",
    )

    model.articulation(
        "worktop_to_control_strip",
        ArticulationType.FIXED,
        parent=worktop,
        child=control_strip,
        origin=Origin(xyz=(0.240, 0.676, -0.031)),
    )

    raise_button = model.part("raise_button")
    raise_button.visual(
        Box((0.020, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=button_grey,
        name="button_cap",
    )
    model.articulation(
        "control_raise_press",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=raise_button,
        origin=Origin(xyz=(-0.040, 0.010, -0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.020,
            lower=0.0,
            upper=0.0025,
        ),
    )

    lower_button = model.part("lower_button")
    lower_button.visual(
        Box((0.020, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=button_grey,
        name="button_cap",
    )
    model.articulation(
        "control_lower_press",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=lower_button,
        origin=Origin(xyz=(-0.010, 0.010, -0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.020,
            lower=0.0,
            upper=0.0025,
        ),
    )

    keyboard_tray = model.part("keyboard_tray")
    keyboard_tray.visual(
        Box((0.660, 0.300, 0.012)),
        origin=Origin(xyz=(0.0, 0.170, -0.006)),
        material=desk_black,
        name="tray_panel",
    )
    keyboard_tray.visual(
        Box((0.660, 0.015, 0.022)),
        origin=Origin(xyz=(0.0, 0.307, 0.011)),
        material=trim_black,
        name="front_lip",
    )
    keyboard_tray.visual(
        Box((0.015, 0.280, 0.018)),
        origin=Origin(xyz=(-0.332, 0.170, -0.010)),
        material=trim_black,
        name="left_flange",
    )
    keyboard_tray.visual(
        Box((0.015, 0.280, 0.018)),
        origin=Origin(xyz=(0.332, 0.170, -0.010)),
        material=trim_black,
        name="right_flange",
    )
    keyboard_tray.visual(
        Box((0.020, 0.260, 0.010)),
        origin=Origin(xyz=(-0.350, 0.120, -0.017)),
        material=lift_steel,
        name="left_runner",
    )
    keyboard_tray.visual(
        Box((0.020, 0.260, 0.010)),
        origin=Origin(xyz=(0.350, 0.120, -0.017)),
        material=lift_steel,
        name="right_runner",
    )
    keyboard_tray.visual(
        Box((0.022, 0.030, 0.020)),
        origin=Origin(xyz=(-0.341, 0.255, -0.011)),
        material=trim_black,
        name="left_runner_bracket",
    )
    keyboard_tray.visual(
        Box((0.022, 0.030, 0.020)),
        origin=Origin(xyz=(0.341, 0.255, -0.011)),
        material=trim_black,
        name="right_runner_bracket",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=top_frame,
        child=keyboard_tray,
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    top_frame = object_model.get_part("top_frame")
    worktop = object_model.get_part("worktop")
    keyboard_tray = object_model.get_part("keyboard_tray")
    raise_button = object_model.get_part("raise_button")

    left_leg_slide = object_model.get_articulation("left_leg_slide")
    worktop_tilt = object_model.get_articulation("worktop_tilt")
    tray_slide = object_model.get_articulation("tray_slide")
    control_raise_press = object_model.get_articulation("control_raise_press")

    left_leg_limits = left_leg_slide.motion_limits
    worktop_limits = worktop_tilt.motion_limits
    tray_limits = tray_slide.motion_limits
    raise_limits = control_raise_press.motion_limits

    with ctx.pose(
        {
            left_leg_slide: 0.0,
            worktop_tilt: 0.0,
            tray_slide: 0.0,
            control_raise_press: 0.0,
        }
    ):
        ctx.expect_within(
            left_stage,
            base_frame,
            axes="xy",
            inner_elem="stage_tube",
            outer_elem="left_column_shell",
            margin=0.004,
            name="left stage stays centered in left column",
        )
        ctx.expect_within(
            right_stage,
            base_frame,
            axes="xy",
            inner_elem="stage_tube",
            outer_elem="right_column_shell",
            margin=0.004,
            name="right stage stays centered in right column",
        )
        ctx.expect_overlap(
            left_stage,
            base_frame,
            axes="z",
            elem_a="stage_tube",
            elem_b="left_column_shell",
            min_overlap=0.24,
            name="left stage remains inserted at rest",
        )
        ctx.expect_overlap(
            right_stage,
            base_frame,
            axes="z",
            elem_a="stage_tube",
            elem_b="right_column_shell",
            min_overlap=0.24,
            name="right stage remains inserted at rest",
        )
        ctx.expect_gap(
            worktop,
            top_frame,
            axis="z",
            positive_elem="main_panel",
            negative_elem="hinge_beam",
            min_gap=0.001,
            max_gap=0.008,
            name="worktop clears the hinge beam at rest",
        )
        ctx.expect_overlap(
            keyboard_tray,
            top_frame,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide",
            min_overlap=0.20,
            name="tray runner stays engaged with left guide at rest",
        )
        ctx.expect_gap(
            top_frame,
            keyboard_tray,
            axis="z",
            positive_elem="left_guide",
            negative_elem="left_runner",
            min_gap=0.001,
            max_gap=0.004,
            name="left tray guide sits just above the runner",
        )

    if left_leg_limits is not None and left_leg_limits.upper is not None:
        rest_top_frame = ctx.part_world_position(top_frame)
        with ctx.pose({left_leg_slide: left_leg_limits.upper}):
            ctx.expect_within(
                left_stage,
                base_frame,
                axes="xy",
                inner_elem="stage_tube",
                outer_elem="left_column_shell",
                margin=0.004,
                name="left stage stays centered at full lift",
            )
            ctx.expect_within(
                right_stage,
                base_frame,
                axes="xy",
                inner_elem="stage_tube",
                outer_elem="right_column_shell",
                margin=0.004,
                name="right stage stays centered at full lift",
            )
            ctx.expect_overlap(
                left_stage,
                base_frame,
                axes="z",
                elem_a="stage_tube",
                elem_b="left_column_shell",
                min_overlap=0.10,
                name="left stage keeps retained insertion when raised",
            )
            ctx.expect_overlap(
                right_stage,
                base_frame,
                axes="z",
                elem_a="stage_tube",
                elem_b="right_column_shell",
                min_overlap=0.10,
                name="right stage keeps retained insertion when raised",
            )
            lifted_top_frame = ctx.part_world_position(top_frame)
        ctx.check(
            "lifting frame rises with the telescoping legs",
            rest_top_frame is not None
            and lifted_top_frame is not None
            and lifted_top_frame[2] > rest_top_frame[2] + 0.15,
            details=f"rest={rest_top_frame}, lifted={lifted_top_frame}",
        )

    if worktop_limits is not None and worktop_limits.upper is not None:
        closed_front = ctx.part_element_world_aabb(worktop, elem="front_apron")
        with ctx.pose({worktop_tilt: worktop_limits.upper}):
            open_front = ctx.part_element_world_aabb(worktop, elem="front_apron")
        ctx.check(
            "worktop front edge tilts upward",
            closed_front is not None
            and open_front is not None
            and open_front[1][2] > closed_front[1][2] + 0.22,
            details=f"closed={closed_front}, open={open_front}",
        )

    if tray_limits is not None and tray_limits.upper is not None:
        tray_rest = ctx.part_world_position(keyboard_tray)
        with ctx.pose({tray_slide: tray_limits.upper}):
            ctx.expect_overlap(
                keyboard_tray,
                top_frame,
                axes="y",
                elem_a="left_runner",
                elem_b="left_guide",
                min_overlap=0.08,
                name="tray runner stays engaged when extended",
            )
            tray_extended = ctx.part_world_position(keyboard_tray)
        ctx.check(
            "keyboard tray slides outward",
            tray_rest is not None
            and tray_extended is not None
            and tray_extended[1] > tray_rest[1] + 0.12,
            details=f"rest={tray_rest}, extended={tray_extended}",
        )

    if raise_limits is not None and raise_limits.upper is not None:
        raise_rest = ctx.part_world_position(raise_button)
        with ctx.pose({control_raise_press: raise_limits.upper}):
            raise_pressed = ctx.part_world_position(raise_button)
        ctx.check(
            "raise button presses inward",
            raise_rest is not None
            and raise_pressed is not None
            and raise_pressed[2] > raise_rest[2] + 0.001,
            details=f"rest={raise_rest}, pressed={raise_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
