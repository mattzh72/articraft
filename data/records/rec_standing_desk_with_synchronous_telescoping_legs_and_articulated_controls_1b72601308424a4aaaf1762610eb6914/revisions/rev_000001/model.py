from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


DESK_WIDTH = 1.40
DESK_DEPTH = 0.70
TOP_THICKNESS = 0.028

LEG_CENTER_X = 0.48
FOOT_LENGTH = 0.72
FOOT_WIDTH = 0.095
FOOT_HEIGHT = 0.030

OUTER_WIDTH = 0.090
OUTER_DEPTH = 0.070
OUTER_HEIGHT = 0.600
OUTER_WALL = 0.008

INNER_WIDTH = 0.060
INNER_DEPTH = 0.040
INNER_TOTAL_LENGTH = 0.624
INNER_CENTER_Z = -0.288
INNER_TOP_Z = 0.024

STAGE_CAP_WIDTH = 0.115
STAGE_CAP_DEPTH = 0.082
STAGE_CAP_HEIGHT = 0.012
STAGE_CAP_CENTER_Z = 0.030
STAGE_CAP_TOP_Z = STAGE_CAP_CENTER_Z + STAGE_CAP_HEIGHT / 2.0

LIFT_TRAVEL = 0.400

BASE_BEAM_LENGTH = 2.0 * LEG_CENTER_X - OUTER_WIDTH
BASE_BEAM_DEPTH = 0.050
BASE_BEAM_HEIGHT = 0.050
BASE_BEAM_CENTER_Z = 0.560

MOUNT_PAD_WIDTH = 0.120
MOUNT_PAD_DEPTH = 0.082
MOUNT_PAD_HEIGHT = 0.008

FRAME_RAIL_LENGTH = 1.020
FRAME_RAIL_DEPTH = 0.045
FRAME_RAIL_HEIGHT = 0.044
FRAME_RAIL_CENTER_Z = 0.030
FRONT_RAIL_Y = 0.190
REAR_RAIL_Y = -0.170

DESKTOP_CENTER_X = LEG_CENTER_X
DESKTOP_CENTER_Z = 0.066

GUIDE_WIDTH = 0.018
GUIDE_LENGTH = 0.220
GUIDE_HEIGHT = 0.030
GUIDE_CENTER_Z = -0.007
GUIDE_CENTER_Y = 0.090
LEFT_GUIDE_X = DESKTOP_CENTER_X - 0.370
RIGHT_GUIDE_X = DESKTOP_CENTER_X + 0.370

TRAY_WIDTH = 0.760
TRAY_DEPTH = 0.280
TRAY_HEIGHT = 0.018
TRAY_PANEL_CENTER_Z = -0.027

RUNNER_WIDTH = 0.012
RUNNER_LENGTH = 0.220
RUNNER_HEIGHT = 0.016
RUNNER_CENTER_Z = -0.010
RUNNER_OFFSET_X = 0.346

TRAY_TRAVEL = 0.160

HANDSET_WIDTH = 0.125
HANDSET_DEPTH = 0.048
HANDSET_HEIGHT = 0.018
HANDSET_X = DESKTOP_CENTER_X + 0.455
HANDSET_Y = 0.285


def rectangular_tube(width: float, depth: float, height: float, wall: float):
    return (
        cq.Workplane("XY")
        .rect(width, depth)
        .rect(width - 2.0 * wall, depth - 2.0 * wall)
        .extrude(height)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_standing_desk")

    frame_dark = model.material("frame_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    powder_gray = model.material("powder_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    top_wood = model.material("top_wood", rgba=(0.70, 0.60, 0.47, 1.0))
    tray_black = model.material("tray_black", rgba=(0.12, 0.12, 0.13, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_gray = model.material("button_gray", rgba=(0.74, 0.76, 0.78, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((BASE_BEAM_LENGTH, BASE_BEAM_DEPTH, BASE_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_BEAM_CENTER_Z)),
        material=frame_dark,
        name="crossbeam",
    )

    left_outer = model.part("left_outer")
    left_outer.visual(
        mesh_from_cadquery(
            rectangular_tube(OUTER_WIDTH, OUTER_DEPTH, OUTER_HEIGHT, OUTER_WALL),
            "left_outer",
        ),
        material=powder_gray,
        name="left_sleeve",
    )

    right_outer = model.part("right_outer")
    right_outer.visual(
        mesh_from_cadquery(
            rectangular_tube(OUTER_WIDTH, OUTER_DEPTH, OUTER_HEIGHT, OUTER_WALL),
            "right_outer",
        ),
        material=powder_gray,
        name="right_sleeve",
    )

    left_foot = model.part("left_foot")
    left_foot.visual(
        Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FOOT_HEIGHT / 2.0)),
        material=frame_dark,
        name="left_foot_bar",
    )

    right_foot = model.part("right_foot")
    right_foot.visual(
        Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FOOT_HEIGHT / 2.0)),
        material=frame_dark,
        name="right_foot_bar",
    )

    left_stage = model.part("left_stage")
    left_stage.visual(
        Box((INNER_WIDTH, INNER_DEPTH, INNER_TOTAL_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, INNER_CENTER_Z)),
        material=powder_gray,
        name="left_inner_member",
    )
    left_stage.visual(
        Box((STAGE_CAP_WIDTH, STAGE_CAP_DEPTH, STAGE_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_CAP_CENTER_Z)),
        material=frame_dark,
        name="left_stage_cap",
    )

    right_stage = model.part("right_stage")
    right_stage.visual(
        Box((INNER_WIDTH, INNER_DEPTH, INNER_TOTAL_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, INNER_CENTER_Z)),
        material=powder_gray,
        name="right_inner_member",
    )
    right_stage.visual(
        Box((STAGE_CAP_WIDTH, STAGE_CAP_DEPTH, STAGE_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_CAP_CENTER_Z)),
        material=frame_dark,
        name="right_stage_cap",
    )

    top_assembly = model.part("top_assembly")
    top_assembly.visual(
        Box((MOUNT_PAD_WIDTH, MOUNT_PAD_DEPTH, MOUNT_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_PAD_HEIGHT / 2.0)),
        material=frame_dark,
        name="left_mount_pad",
    )
    top_assembly.visual(
        Box((MOUNT_PAD_WIDTH, MOUNT_PAD_DEPTH, MOUNT_PAD_HEIGHT)),
        origin=Origin(xyz=(2.0 * LEG_CENTER_X, 0.0, MOUNT_PAD_HEIGHT / 2.0)),
        material=frame_dark,
        name="right_mount_pad",
    )
    top_assembly.visual(
        Box((0.060, 0.320, FRAME_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.010, FRAME_RAIL_CENTER_Z)),
        material=frame_dark,
        name="left_frame_plate",
    )
    top_assembly.visual(
        Box((0.060, 0.320, FRAME_RAIL_HEIGHT)),
        origin=Origin(xyz=(2.0 * LEG_CENTER_X, 0.010, FRAME_RAIL_CENTER_Z)),
        material=frame_dark,
        name="right_frame_plate",
    )
    top_assembly.visual(
        Box((FRAME_RAIL_LENGTH, FRAME_RAIL_DEPTH, FRAME_RAIL_HEIGHT)),
        origin=Origin(xyz=(DESKTOP_CENTER_X, FRONT_RAIL_Y, FRAME_RAIL_CENTER_Z)),
        material=frame_dark,
        name="front_rail",
    )
    top_assembly.visual(
        Box((FRAME_RAIL_LENGTH, FRAME_RAIL_DEPTH, FRAME_RAIL_HEIGHT)),
        origin=Origin(xyz=(DESKTOP_CENTER_X, REAR_RAIL_Y, FRAME_RAIL_CENTER_Z)),
        material=frame_dark,
        name="rear_rail",
    )
    top_assembly.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(DESKTOP_CENTER_X, 0.0, DESKTOP_CENTER_Z)),
        material=top_wood,
        name="desktop",
    )
    top_assembly.visual(
        Box((GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(LEFT_GUIDE_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material=frame_dark,
        name="left_guide",
    )
    top_assembly.visual(
        Box((GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(RIGHT_GUIDE_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material=frame_dark,
        name="right_guide",
    )

    keyboard_tray = model.part("keyboard_tray")
    keyboard_tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_PANEL_CENTER_Z)),
        material=tray_black,
        name="tray_panel",
    )
    keyboard_tray.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(-RUNNER_OFFSET_X, 0.0, RUNNER_CENTER_Z)),
        material=frame_dark,
        name="left_runner",
    )
    keyboard_tray.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(RUNNER_OFFSET_X, 0.0, RUNNER_CENTER_Z)),
        material=frame_dark,
        name="right_runner",
    )

    handset = model.part("handset")
    handset.visual(
        Box((HANDSET_WIDTH, HANDSET_DEPTH, HANDSET_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -HANDSET_HEIGHT / 2.0)),
        material=plastic_black,
        name="handset_shell",
    )
    handset.visual(
        Box((0.020, 0.016, 0.004)),
        origin=Origin(xyz=(-0.022, 0.0, -0.020)),
        material=button_gray,
        name="up_button",
    )
    handset.visual(
        Box((0.020, 0.016, 0.004)),
        origin=Origin(xyz=(0.022, 0.0, -0.020)),
        material=button_gray,
        name="down_button",
    )

    model.articulation(
        "base_frame_to_left_outer",
        ArticulationType.FIXED,
        parent=base_frame,
        child=left_outer,
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, FOOT_HEIGHT)),
    )
    model.articulation(
        "base_frame_to_right_outer",
        ArticulationType.FIXED,
        parent=base_frame,
        child=right_outer,
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, FOOT_HEIGHT)),
    )
    model.articulation(
        "left_outer_to_left_foot",
        ArticulationType.FIXED,
        parent=left_outer,
        child=left_foot,
        origin=Origin(),
    )
    model.articulation(
        "right_outer_to_right_foot",
        ArticulationType.FIXED,
        parent=right_outer,
        child=right_foot,
        origin=Origin(),
    )

    model.articulation(
        "left_outer_to_left_stage",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.060,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "right_outer_to_right_stage",
        ArticulationType.PRISMATIC,
        parent=right_outer,
        child=right_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.060,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic("left_outer_to_left_stage", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "left_stage_to_top_assembly",
        ArticulationType.FIXED,
        parent=left_stage,
        child=top_assembly,
        origin=Origin(xyz=(0.0, 0.0, STAGE_CAP_TOP_Z)),
    )
    model.articulation(
        "top_assembly_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=top_assembly,
        child=keyboard_tray,
        origin=Origin(xyz=(DESKTOP_CENTER_X, GUIDE_CENTER_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.250,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "top_assembly_to_handset",
        ArticulationType.FIXED,
        parent=top_assembly,
        child=handset,
        origin=Origin(xyz=(HANDSET_X, HANDSET_Y, 0.052)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_outer = object_model.get_part("left_outer")
    right_outer = object_model.get_part("right_outer")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    top_assembly = object_model.get_part("top_assembly")
    keyboard_tray = object_model.get_part("keyboard_tray")

    lift = object_model.get_articulation("left_outer_to_left_stage")
    tray_slide = object_model.get_articulation("top_assembly_to_keyboard_tray")

    lift_limits = lift.motion_limits
    tray_limits = tray_slide.motion_limits

    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift: 0.0}):
            ctx.expect_within(
                left_stage,
                left_outer,
                axes="xy",
                inner_elem="left_inner_member",
                outer_elem="left_sleeve",
                margin=0.0,
                name="left stage stays centered in the left sleeve",
            )
            ctx.expect_within(
                right_stage,
                right_outer,
                axes="xy",
                inner_elem="right_inner_member",
                outer_elem="right_sleeve",
                margin=0.0,
                name="right stage stays centered in the right sleeve",
            )
            ctx.expect_overlap(
                left_stage,
                left_outer,
                axes="z",
                elem_a="left_inner_member",
                elem_b="left_sleeve",
                min_overlap=0.55,
                name="left lift stage remains deeply inserted at rest",
            )
            ctx.expect_overlap(
                right_stage,
                right_outer,
                axes="z",
                elem_a="right_inner_member",
                elem_b="right_sleeve",
                min_overlap=0.55,
                name="right lift stage remains deeply inserted at rest",
            )
            ctx.expect_gap(
                top_assembly,
                right_stage,
                axis="z",
                positive_elem="right_mount_pad",
                negative_elem="right_stage_cap",
                max_gap=0.001,
                max_penetration=0.0,
                name="right lift stage supports the top assembly at rest",
            )
            rest_top_pos = ctx.part_world_position(top_assembly)

        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_overlap(
                left_stage,
                left_outer,
                axes="z",
                elem_a="left_inner_member",
                elem_b="left_sleeve",
                min_overlap=0.20,
                name="left lift stage retains insertion at full height",
            )
            ctx.expect_overlap(
                right_stage,
                right_outer,
                axes="z",
                elem_a="right_inner_member",
                elem_b="right_sleeve",
                min_overlap=0.20,
                name="right lift stage retains insertion at full height",
            )
            ctx.expect_gap(
                top_assembly,
                right_stage,
                axis="z",
                positive_elem="right_mount_pad",
                negative_elem="right_stage_cap",
                max_gap=0.001,
                max_penetration=0.0,
                name="right lift stage stays aligned with the top assembly at full height",
            )
            high_top_pos = ctx.part_world_position(top_assembly)

        ctx.check(
            "desktop rises upward across the lift stroke",
            rest_top_pos is not None
            and high_top_pos is not None
            and high_top_pos[2] > rest_top_pos[2] + 0.30,
            details=f"rest={rest_top_pos}, high={high_top_pos}",
        )

    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_slide: 0.0}):
            ctx.expect_overlap(
                keyboard_tray,
                top_assembly,
                axes="y",
                elem_a="left_runner",
                elem_b="left_guide",
                min_overlap=0.21,
                name="keyboard tray starts fully engaged on the left guide",
            )
            ctx.expect_overlap(
                keyboard_tray,
                top_assembly,
                axes="y",
                elem_a="right_runner",
                elem_b="right_guide",
                min_overlap=0.21,
                name="keyboard tray starts fully engaged on the right guide",
            )
            rest_tray_pos = ctx.part_world_position(keyboard_tray)

        with ctx.pose({tray_slide: tray_limits.upper}):
            ctx.expect_overlap(
                keyboard_tray,
                top_assembly,
                axes="y",
                elem_a="left_runner",
                elem_b="left_guide",
                min_overlap=0.05,
                name="left tray runner remains captured at full extension",
            )
            ctx.expect_overlap(
                keyboard_tray,
                top_assembly,
                axes="y",
                elem_a="right_runner",
                elem_b="right_guide",
                min_overlap=0.05,
                name="right tray runner remains captured at full extension",
            )
            extended_tray_pos = ctx.part_world_position(keyboard_tray)

        ctx.check(
            "keyboard tray extends outward from the desk front",
            rest_tray_pos is not None
            and extended_tray_pos is not None
            and extended_tray_pos[1] > rest_tray_pos[1] + 0.10,
            details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
        )

    return ctx.report()


object_model = build_object_model()
