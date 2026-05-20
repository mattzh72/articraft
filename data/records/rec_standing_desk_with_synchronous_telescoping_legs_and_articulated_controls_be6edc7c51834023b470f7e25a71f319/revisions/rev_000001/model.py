from __future__ import annotations

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


LEG_X = (-0.320, 0.320)
LIFT_TRAVEL = 0.230


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    top_finish = model.material("top_finish", rgba=(0.70, 0.57, 0.41, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    column_metal = model.material("column_metal", rgba=(0.30, 0.31, 0.34, 1.0))
    handset_dark = model.material("handset_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    button_light = model.material("button_light", rgba=(0.83, 0.85, 0.88, 1.0))
    glide_dark = model.material("glide_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.000, 0.550, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=top_finish,
        name="top",
    )
    frame.visual(
        Box((0.720, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.180, -0.0145)),
        material=frame_metal,
        name="front_rail",
    )
    frame.visual(
        Box((0.720, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.180, -0.0145)),
        material=frame_metal,
        name="rear_rail",
    )
    frame.visual(
        Box((0.060, 0.360, 0.030)),
        origin=Origin(xyz=(-0.320, 0.0, -0.0145)),
        material=frame_metal,
        name="side_brace_0",
    )
    frame.visual(
        Box((0.060, 0.360, 0.030)),
        origin=Origin(xyz=(0.320, 0.0, -0.0145)),
        material=frame_metal,
        name="side_brace_1",
    )
    frame.visual(
        Box((0.140, 0.100, 0.012)),
        origin=Origin(xyz=(-0.320, 0.0, -0.0350)),
        material=frame_metal,
        name="frame_mount_0",
    )
    frame.visual(
        Box((0.140, 0.100, 0.012)),
        origin=Origin(xyz=(0.320, 0.0, -0.0350)),
        material=frame_metal,
        name="frame_mount_1",
    )

    for index, x_pos in enumerate(LEG_X):
        outer = model.part(f"leg_outer_{index}")
        outer.visual(
            Box((0.120, 0.085, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=column_metal,
            name="top_cap",
        )
        outer.visual(
            Box((0.086, 0.006, 0.400)),
            origin=Origin(xyz=(0.0, 0.029, -0.212)),
            material=column_metal,
            name="front_wall",
        )
        outer.visual(
            Box((0.086, 0.006, 0.400)),
            origin=Origin(xyz=(0.0, -0.029, -0.212)),
            material=column_metal,
            name="rear_wall",
        )
        outer.visual(
            Box((0.006, 0.060, 0.400)),
            origin=Origin(xyz=(-0.040, 0.0, -0.212)),
            material=column_metal,
            name="side_wall_0",
        )
        outer.visual(
            Box((0.006, 0.060, 0.400)),
            origin=Origin(xyz=(0.040, 0.0, -0.212)),
            material=column_metal,
            name="side_wall_1",
        )
        outer.visual(
            Box((0.074, 0.005, 0.080)),
            origin=Origin(xyz=(0.0, 0.0245, -0.052)),
            material=column_metal,
            name="guide_pad_front",
        )
        outer.visual(
            Box((0.074, 0.005, 0.080)),
            origin=Origin(xyz=(0.0, -0.0245, -0.052)),
            material=column_metal,
            name="guide_pad_rear",
        )
        outer.visual(
            Box((0.005, 0.050, 0.080)),
            origin=Origin(xyz=(-0.0375, 0.0, -0.052)),
            material=column_metal,
            name="guide_pad_0",
        )
        outer.visual(
            Box((0.005, 0.050, 0.080)),
            origin=Origin(xyz=(0.0375, 0.0, -0.052)),
            material=column_metal,
            name="guide_pad_1",
        )
        model.articulation(
            f"frame_to_leg_outer_{index}",
            ArticulationType.FIXED,
            parent=frame,
            child=outer,
            origin=Origin(xyz=(x_pos, 0.0, -0.041)),
        )

        stage = model.part(f"leg_stage_{index}")
        stage.visual(
            Box((0.070, 0.044, 0.520)),
            origin=Origin(xyz=(0.0, 0.0, -0.260)),
            material=column_metal,
            name="inner_stage",
        )
        stage.visual(
            Box((0.096, 0.070, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.507)),
            material=column_metal,
            name="foot_shoe",
        )
        stage.visual(
            Box((0.078, 0.580, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.535)),
            material=glide_dark,
            name="foot_bar",
        )
        stage.visual(
            Box((0.078, 0.050, 0.010)),
            origin=Origin(xyz=(0.0, 0.250, -0.555)),
            material=glide_dark,
            name="front_glide",
        )
        stage.visual(
            Box((0.078, 0.050, 0.010)),
            origin=Origin(xyz=(0.0, -0.250, -0.555)),
            material=glide_dark,
            name="rear_glide",
        )

        model.articulation(
            f"leg_{index}_lift",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=stage,
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.060,
                lower=0.0,
                upper=LIFT_TRAVEL,
            ),
            mimic=Mimic(joint="leg_0_lift") if index == 1 else None,
        )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.120, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=handset_dark,
        name="mount_flange",
    )
    control_pod.visual(
        Box((0.082, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=handset_dark,
        name="pod_body",
    )
    control_pod.visual(
        Box((0.070, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, -0.042)),
        material=handset_dark,
        name="front_lip",
    )
    model.articulation(
        "frame_to_control_pod",
        ArticulationType.FIXED,
        parent=frame,
        child=control_pod,
        origin=Origin(xyz=(0.425, 0.235, 0.0)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.018, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.012, -0.009)),
        material=handset_dark,
        name="hanger",
    )
    handset.visual(
        Box((0.088, 0.005, 0.128)),
        origin=Origin(xyz=(0.0, 0.0025, -0.082)),
        material=handset_dark,
        name="back_plate",
    )
    handset.visual(
        Box((0.006, 0.019, 0.128)),
        origin=Origin(xyz=(-0.041, 0.0095, -0.082)),
        material=handset_dark,
        name="side_wall_0",
    )
    handset.visual(
        Box((0.006, 0.019, 0.128)),
        origin=Origin(xyz=(0.041, 0.0095, -0.082)),
        material=handset_dark,
        name="side_wall_1",
    )
    handset.visual(
        Box((0.088, 0.019, 0.012)),
        origin=Origin(xyz=(0.0, 0.0095, -0.024)),
        material=handset_dark,
        name="top_wall",
    )
    handset.visual(
        Box((0.088, 0.019, 0.012)),
        origin=Origin(xyz=(0.0, 0.0095, -0.140)),
        material=handset_dark,
        name="bottom_wall",
    )
    handset.visual(
        Box((0.012, 0.004, 0.094)),
        origin=Origin(xyz=(-0.038, 0.020, -0.082)),
        material=handset_dark,
        name="face_side_0",
    )
    handset.visual(
        Box((0.012, 0.004, 0.094)),
        origin=Origin(xyz=(0.038, 0.020, -0.082)),
        material=handset_dark,
        name="face_side_1",
    )
    handset.visual(
        Box((0.010, 0.004, 0.094)),
        origin=Origin(xyz=(0.0, 0.020, -0.082)),
        material=handset_dark,
        name="face_divider",
    )
    handset.visual(
        Box((0.088, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, -0.046)),
        material=handset_dark,
        name="face_top",
    )
    handset.visual(
        Box((0.088, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, -0.082)),
        material=handset_dark,
        name="face_mid",
    )
    handset.visual(
        Box((0.088, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, -0.118)),
        material=handset_dark,
        name="face_bottom",
    )
    model.articulation(
        "pod_to_handset",
        ArticulationType.FIXED,
        parent=control_pod,
        child=handset,
        origin=Origin(xyz=(0.0, 0.025, -0.042)),
    )

    button_centers = {
        (0, 0): (-0.0185, -0.0645),
        (0, 1): (0.0185, -0.0645),
        (1, 0): (-0.0185, -0.0995),
        (1, 1): (0.0185, -0.0995),
    }
    for (row, col), (x_pos, z_pos) in button_centers.items():
        button = model.part(f"button_{row}_{col}")
        button.visual(
            Box((0.027, 0.004, 0.025)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_light,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_light,
            name="button_plunger",
        )
        model.articulation(
            f"handset_to_button_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x_pos, 0.018, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.040,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    outer_0 = object_model.get_part("leg_outer_0")
    outer_1 = object_model.get_part("leg_outer_1")
    stage_0 = object_model.get_part("leg_stage_0")
    stage_1 = object_model.get_part("leg_stage_1")
    lift_0 = object_model.get_articulation("leg_0_lift")
    button_0_0 = object_model.get_part("button_0_0")
    button_0_1 = object_model.get_part("button_0_1")
    button_joint_0_0 = object_model.get_articulation("handset_to_button_0_0")

    ctx.expect_gap(
        frame,
        outer_0,
        axis="z",
        positive_elem="side_brace_0",
        negative_elem="top_cap",
        min_gap=0.010,
        max_gap=0.015,
        name="top frame stays visibly above the first lifting column",
    )
    ctx.expect_gap(
        frame,
        outer_1,
        axis="z",
        positive_elem="side_brace_1",
        negative_elem="top_cap",
        min_gap=0.010,
        max_gap=0.015,
        name="top frame stays visibly above the second lifting column",
    )

    ctx.expect_within(
        stage_0,
        outer_0,
        axes="xy",
        inner_elem="inner_stage",
        margin=0.0,
        name="first telescoping stage stays centered in its outer column at rest",
    )
    ctx.expect_within(
        stage_1,
        outer_1,
        axes="xy",
        inner_elem="inner_stage",
        margin=0.0,
        name="second telescoping stage stays centered in its outer column at rest",
    )
    ctx.expect_overlap(
        stage_0,
        outer_0,
        axes="z",
        elem_a="inner_stage",
        min_overlap=0.300,
        name="first telescoping stage remains deeply inserted at rest",
    )

    lift_limits = lift_0.motion_limits
    button_limits = button_joint_0_0.motion_limits

    rest_stage_0 = ctx.part_world_position(stage_0)
    rest_button_0_0 = ctx.part_world_position(button_0_0)
    rest_button_0_1 = ctx.part_world_position(button_0_1)

    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift_0: lift_limits.upper}):
            ctx.expect_within(
                stage_0,
                outer_0,
                axes="xy",
                inner_elem="inner_stage",
                margin=0.0,
                name="first telescoping stage stays centered at full travel",
            )
            ctx.expect_within(
                stage_1,
                outer_1,
                axes="xy",
                inner_elem="inner_stage",
                margin=0.0,
                name="second telescoping stage stays centered at full travel",
            )
            ctx.expect_overlap(
                stage_0,
                outer_0,
                axes="z",
                elem_a="inner_stage",
                min_overlap=0.160,
                name="first telescoping stage retains insertion at full travel",
            )
            ctx.expect_overlap(
                stage_1,
                outer_1,
                axes="z",
                elem_a="inner_stage",
                min_overlap=0.160,
                name="second telescoping stage retains insertion at full travel",
            )
            extended_stage_0 = ctx.part_world_position(stage_0)
            extended_stage_1 = ctx.part_world_position(stage_1)

        ctx.check(
            "driven lift stage extends downward",
            rest_stage_0 is not None
            and extended_stage_0 is not None
            and extended_stage_0[2] < rest_stage_0[2] - 0.150,
            details=f"rest={rest_stage_0}, extended={extended_stage_0}",
        )
        ctx.check(
            "twin lift stages stay synchronized",
            extended_stage_0 is not None
            and extended_stage_1 is not None
            and abs(extended_stage_0[2] - extended_stage_1[2]) <= 1e-6,
            details=f"stage_0={extended_stage_0}, stage_1={extended_stage_1}",
        )

    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint_0_0: button_limits.upper}):
            pressed_button_0_0 = ctx.part_world_position(button_0_0)
            pressed_button_0_1 = ctx.part_world_position(button_0_1)

        ctx.check(
            "a handset button presses inward",
            rest_button_0_0 is not None
            and pressed_button_0_0 is not None
            and pressed_button_0_0[1] < rest_button_0_0[1] - 0.0015,
            details=f"rest={rest_button_0_0}, pressed={pressed_button_0_0}",
        )
        ctx.check(
            "adjacent handset button stays independent",
            rest_button_0_1 is not None
            and pressed_button_0_1 is not None
            and abs(pressed_button_0_1[1] - rest_button_0_1[1]) <= 1e-6,
            details=f"rest={rest_button_0_1}, pressed={pressed_button_0_1}",
        )

    return ctx.report()


object_model = build_object_model()
