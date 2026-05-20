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
    Cylinder,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    powder_black = model.material("powder_black", rgba=(0.17, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    warm_oak = model.material("warm_oak", rgba=(0.76, 0.64, 0.47, 1.0))
    satin_white = model.material("satin_white", rgba=(0.90, 0.91, 0.92, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.08, 0.08, 0.09, 1.0))

    leg_span = 0.62
    column_opening_z = 0.59
    travel = 0.32

    base_beam = model.part("base_beam")
    base_beam.visual(
        Box((0.70, 0.07, 0.06)),
        origin=Origin(xyz=(0.0, -0.10, 0.56)),
        material=powder_black,
        name="rear_beam",
    )
    base_beam.visual(
        Box((0.09, 0.07, 0.05)),
        origin=Origin(xyz=(-leg_span / 2.0, -0.065, 0.545)),
        material=powder_black,
        name="end_bracket_0",
    )
    base_beam.visual(
        Box((0.09, 0.07, 0.05)),
        origin=Origin(xyz=(leg_span / 2.0, -0.065, 0.545)),
        material=powder_black,
        name="end_bracket_1",
    )
    base_beam.visual(
        Box((0.18, 0.10, 0.07)),
        origin=Origin(xyz=(0.0, -0.10, 0.550)),
        material=graphite,
        name="motor_housing",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        column = model.part(f"column_{index}")
        x_center = 0.0335
        y_center = 0.0285
        wall_t = 0.005
        shell_h = 0.56
        shell_cz = -shell_h / 2.0

        column.visual(
            Box((0.072, wall_t, shell_h)),
            origin=Origin(xyz=(0.0, y_center, shell_cz)),
            material=satin_white,
            name="front_wall",
        )
        column.visual(
            Box((0.072, wall_t, shell_h)),
            origin=Origin(xyz=(0.0, -y_center, shell_cz)),
            material=satin_white,
            name="rear_wall",
        )
        column.visual(
            Box((wall_t, 0.052, shell_h)),
            origin=Origin(xyz=(-x_center, 0.0, shell_cz)),
            material=satin_white,
            name="side_wall_0",
        )
        column.visual(
            Box((wall_t, 0.052, shell_h)),
            origin=Origin(xyz=(x_center, 0.0, shell_cz)),
            material=satin_white,
            name="side_wall_1",
        )
        column.visual(
            Box((0.085, 0.48, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, -0.5775)),
            material=powder_black,
            name="foot",
        )

        model.articulation(
            f"base_beam_to_column_{index}",
            ArticulationType.FIXED,
            parent=base_beam,
            child=column,
            origin=Origin(xyz=(x_sign * leg_span / 2.0, 0.0, column_opening_z)),
        )

    stage_0 = model.part("stage_0")
    stage_0.visual(
        Box((0.056, 0.040, 0.54)),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=graphite,
        name="tube",
    )
    stage_0.visual(
        Box((0.092, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=powder_black,
        name="top_cap",
    )
    stage_0.visual(
        Box((0.003, 0.018, 0.10)),
        origin=Origin(xyz=(-0.0295, 0.0, -0.10)),
        material=plastic_black,
        name="guide_pad_0",
    )
    stage_0.visual(
        Box((0.003, 0.018, 0.10)),
        origin=Origin(xyz=(0.0295, 0.0, -0.10)),
        material=plastic_black,
        name="guide_pad_1",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        Box((0.056, 0.040, 0.54)),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=graphite,
        name="tube",
    )
    stage_1.visual(
        Box((0.092, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=powder_black,
        name="top_cap",
    )
    stage_1.visual(
        Box((0.003, 0.018, 0.10)),
        origin=Origin(xyz=(-0.0295, 0.0, -0.10)),
        material=plastic_black,
        name="guide_pad_0",
    )
    stage_1.visual(
        Box((0.003, 0.018, 0.10)),
        origin=Origin(xyz=(0.0295, 0.0, -0.10)),
        material=plastic_black,
        name="guide_pad_1",
    )

    model.articulation(
        "column_0_to_stage_0",
        ArticulationType.PRISMATIC,
        parent="column_0",
        child=stage_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.05,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "column_1_to_stage_1",
        ArticulationType.PRISMATIC,
        parent="column_1",
        child=stage_1,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.05,
            lower=0.0,
            upper=travel,
        ),
        mimic=Mimic("column_0_to_stage_0"),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.74, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.14, 0.02)),
        material=powder_black,
        name="front_rail",
    )
    top_frame.visual(
        Box((0.74, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.14, 0.02)),
        material=powder_black,
        name="rear_rail",
    )
    top_frame.visual(
        Box((0.05, 0.28, 0.04)),
        origin=Origin(xyz=(-0.345, 0.0, 0.02)),
        material=powder_black,
        name="side_rail_0",
    )
    top_frame.visual(
        Box((0.05, 0.28, 0.04)),
        origin=Origin(xyz=(0.345, 0.0, 0.02)),
        material=powder_black,
        name="side_rail_1",
    )
    top_frame.visual(
        Box((0.10, 0.10, 0.008)),
        origin=Origin(xyz=(-leg_span / 2.0, 0.0, 0.004)),
        material=graphite,
        name="mount_plate_0",
    )
    top_frame.visual(
        Box((0.10, 0.10, 0.008)),
        origin=Origin(xyz=(leg_span / 2.0, 0.0, 0.004)),
        material=graphite,
        name="mount_plate_1",
    )
    top_frame.visual(
        Box((0.018, 0.24, 0.018)),
        origin=Origin(xyz=(-0.17, 0.06, -0.015)),
        material=graphite,
        name="guide_0",
    )
    top_frame.visual(
        Box((0.018, 0.24, 0.018)),
        origin=Origin(xyz=(0.17, 0.06, -0.015)),
        material=graphite,
        name="guide_1",
    )
    top_frame.visual(
        Box((0.018, 0.03, 0.012)),
        origin=Origin(xyz=(-0.17, 0.165, -0.006)),
        material=graphite,
        name="guide_bracket_0",
    )
    top_frame.visual(
        Box((0.018, 0.03, 0.012)),
        origin=Origin(xyz=(0.17, 0.165, -0.006)),
        material=graphite,
        name="guide_bracket_1",
    )

    model.articulation(
        "stage_0_to_top_frame",
        ArticulationType.FIXED,
        parent=stage_0,
        child=top_frame,
        origin=Origin(xyz=(leg_span / 2.0, 0.0, 0.09)),
    )

    top = model.part("top")
    top.visual(
        Box((1.00, 0.55, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=warm_oak,
        name="desktop",
    )

    model.articulation(
        "top_frame_to_top",
        ArticulationType.FIXED,
        parent=top_frame,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.10, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=plastic_black,
        name="body",
    )
    control_pod.visual(
        Box((0.085, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.010, -0.024)),
        material=plastic_black,
        name="fascia",
    )

    model.articulation(
        "top_frame_to_control_pod",
        ArticulationType.FIXED,
        parent=top_frame,
        child=control_pod,
        origin=Origin(xyz=(0.42, 0.18, 0.050)),
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=graphite,
        name="cap",
    )
    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=graphite,
        name="cap",
    )

    model.articulation(
        "control_pod_to_button_0",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=button_0,
        origin=Origin(xyz=(-0.018, 0.010, -0.030)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.04,
            lower=0.0,
            upper=0.002,
        ),
    )
    model.articulation(
        "control_pod_to_button_1",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=button_1,
        origin=Origin(xyz=(0.018, 0.010, -0.030)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.04,
            lower=0.0,
            upper=0.002,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.56, 0.25, 0.014)),
        origin=Origin(xyz=(0.0, 0.010, -0.013)),
        material=graphite,
        name="deck",
    )
    tray.visual(
        Box((0.56, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.134, -0.004)),
        material=powder_black,
        name="front_lip",
    )
    tray.visual(
        Box((0.018, 0.32, 0.012)),
        origin=Origin(xyz=(-0.17, 0.0, 0.0)),
        material=graphite,
        name="runner_0",
    )
    tray.visual(
        Box((0.018, 0.32, 0.012)),
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        material=graphite,
        name="runner_1",
    )

    model.articulation(
        "top_frame_to_tray",
        ArticulationType.PRISMATIC,
        parent=top_frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.055, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.15,
            lower=0.0,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column_0 = object_model.get_part("column_0")
    column_1 = object_model.get_part("column_1")
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    base_beam = object_model.get_part("base_beam")
    top_frame = object_model.get_part("top_frame")
    top = object_model.get_part("top")
    tray = object_model.get_part("tray")
    button_0 = object_model.get_part("button_0")

    lift = object_model.get_articulation("column_0_to_stage_0")
    tray_slide = object_model.get_articulation("top_frame_to_tray")
    button_press = object_model.get_articulation("control_pod_to_button_0")

    ctx.expect_origin_distance(
        stage_0,
        column_0,
        axes="xy",
        max_dist=0.001,
        name="stage_0 stays centered in column_0",
    )
    ctx.expect_origin_distance(
        stage_1,
        column_1,
        axes="xy",
        max_dist=0.001,
        name="stage_1 stays centered in column_1",
    )
    ctx.expect_overlap(
        stage_0,
        column_0,
        axes="z",
        min_overlap=0.40,
        name="collapsed stage_0 remains inserted in column_0",
    )
    ctx.expect_overlap(
        stage_1,
        column_1,
        axes="z",
        min_overlap=0.40,
        name="collapsed stage_1 remains inserted in column_1",
    )
    ctx.expect_gap(
        top_frame,
        column_0,
        axis="z",
        min_gap=0.05,
        name="top frame stays visibly above column_0",
    )
    ctx.expect_gap(
        top_frame,
        column_1,
        axis="z",
        min_gap=0.05,
        name="top frame stays visibly above column_1",
    )
    ctx.expect_gap(
        top,
        top_frame,
        axis="z",
        min_gap=0.008,
        max_gap=0.020,
        name="desktop sits slightly above the top frame",
    )
    ctx.expect_gap(
        top_frame,
        base_beam,
        axis="z",
        min_gap=0.05,
        name="top frame clears the lifting beam at rest",
    )
    ctx.expect_overlap(
        tray,
        top_frame,
        axes="y",
        elem_a="runner_0",
        elem_b="guide_0",
        min_overlap=0.18,
        name="collapsed tray keeps runner_0 engaged in guide_0",
    )
    ctx.expect_overlap(
        tray,
        top_frame,
        axes="y",
        elem_a="runner_1",
        elem_b="guide_1",
        min_overlap=0.18,
        name="collapsed tray keeps runner_1 engaged in guide_1",
    )

    rest_top = ctx.part_world_position(top)
    rest_stage_0 = ctx.part_world_position(stage_0)
    rest_stage_1 = ctx.part_world_position(stage_1)
    rest_tray = ctx.part_world_position(tray)
    rest_button = ctx.part_world_position(button_0)

    with ctx.pose({lift: 0.32}):
        ctx.expect_origin_distance(
            stage_0,
            stage_1,
            axes="z",
            max_dist=0.001,
            name="both lifting stages stay synchronized at full height",
        )
        ctx.expect_overlap(
            stage_0,
            column_0,
            axes="z",
            min_overlap=0.12,
            name="extended stage_0 still retains insertion in column_0",
        )
        ctx.expect_overlap(
            stage_1,
            column_1,
            axes="z",
            min_overlap=0.12,
            name="extended stage_1 still retains insertion in column_1",
        )
        extended_top = ctx.part_world_position(top)
        extended_stage_0 = ctx.part_world_position(stage_0)
        extended_stage_1 = ctx.part_world_position(stage_1)

    ctx.check(
        "desktop rises when the lift extends",
        rest_top is not None
        and extended_top is not None
        and extended_top[2] > rest_top[2] + 0.25,
        details=f"rest={rest_top}, extended={extended_top}",
    )
    ctx.check(
        "both stages rise together",
        rest_stage_0 is not None
        and rest_stage_1 is not None
        and extended_stage_0 is not None
        and extended_stage_1 is not None
        and abs(extended_stage_0[2] - extended_stage_1[2]) <= 0.001
        and extended_stage_0[2] > rest_stage_0[2] + 0.25
        and extended_stage_1[2] > rest_stage_1[2] + 0.25,
        details=(
            f"rest_0={rest_stage_0}, rest_1={rest_stage_1}, "
            f"extended_0={extended_stage_0}, extended_1={extended_stage_1}"
        ),
    )

    with ctx.pose({tray_slide: 0.16}):
        ctx.expect_overlap(
            tray,
            top_frame,
            axes="y",
            elem_a="runner_0",
            elem_b="guide_0",
            min_overlap=0.12,
            name="extended tray keeps runner_0 retained in guide_0",
        )
        ctx.expect_overlap(
            tray,
            top_frame,
            axes="y",
            elem_a="runner_1",
            elem_b="guide_1",
            min_overlap=0.12,
            name="extended tray keeps runner_1 retained in guide_1",
        )
        extended_tray = ctx.part_world_position(tray)

    ctx.check(
        "keyboard tray slides toward the front",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1] > rest_tray[1] + 0.10,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    with ctx.pose({button_press: 0.002}):
        pressed_button = ctx.part_world_position(button_0)

    ctx.check(
        "control button depresses downward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.001,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
