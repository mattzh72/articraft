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
    model = ArticulatedObject(name="teaching_microscope")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_cream = model.material("arm_cream", rgba=(0.86, 0.86, 0.82, 1.0))
    stage_black = model.material("stage_black", rgba=(0.10, 0.11, 0.12, 1.0))
    optics_black = model.material("optics_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_grey = model.material("knob_grey", rgba=(0.26, 0.27, 0.30, 1.0))
    carriage_metal = model.material("carriage_metal", rgba=(0.66, 0.68, 0.72, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.45, 0.47, 0.50, 1.0))
    glass = model.material("glass", rgba=(0.66, 0.78, 0.85, 0.35))

    base = model.part("base")
    base.visual(
        Box((0.240, 0.190, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=base_dark,
        name="foot",
    )
    base.visual(
        Box((0.120, 0.115, 0.022)),
        origin=Origin(xyz=(-0.040, 0.000, 0.035)),
        material=base_dark,
        name="heel",
    )
    base.visual(
        Box((0.070, 0.096, 0.034)),
        origin=Origin(xyz=(-0.055, 0.000, 0.063)),
        material=base_dark,
        name="pedestal",
    )
    base.visual(
        Box((0.088, 0.074, 0.016)),
        origin=Origin(xyz=(0.050, 0.000, 0.032)),
        material=accent_grey,
        name="front_pad",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base.visual(
                Cylinder(radius=0.014, length=0.004),
                origin=Origin(
                    xyz=(0.092 * x_sign, 0.070 * y_sign, 0.002),
                ),
                material=accent_grey,
                name=f"foot_pad_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )

    arm = model.part("arm")
    arm.visual(
        Box((0.050, 0.070, 0.190)),
        origin=Origin(xyz=(-0.010, 0.000, 0.095)),
        material=arm_cream,
        name="rear_spine",
    )
    arm.visual(
        Box((0.100, 0.080, 0.050)),
        origin=Origin(xyz=(0.020, 0.000, 0.028)),
        material=arm_cream,
        name="lower_shoulder",
    )
    arm.visual(
        Box((0.036, 0.060, 0.180)),
        origin=Origin(xyz=(0.060, 0.000, 0.145)),
        material=arm_cream,
        name="focus_guide",
    )
    arm.visual(
        Box((0.122, 0.052, 0.032)),
        origin=Origin(xyz=(0.028, 0.000, 0.110), rpy=(0.000, -0.88, 0.000)),
        material=arm_cream,
        name="diagonal_brace",
    )
    arm.visual(
        Box((0.086, 0.074, 0.050)),
        origin=Origin(xyz=(0.014, 0.000, 0.212)),
        material=arm_cream,
        name="upper_bridge",
    )
    arm.visual(
        Box((0.070, 0.045, 0.060)),
        origin=Origin(xyz=(0.082, 0.000, 0.042)),
        material=arm_cream,
        name="stage_bracket",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.FIXED,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.055, 0.000, 0.080)),
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.026, 0.040, 0.050)),
        origin=Origin(xyz=(0.013, 0.000, 0.000)),
        material=stage_black,
        name="mount_block",
    )
    stage.visual(
        Box((0.128, 0.138, 0.008)),
        origin=Origin(xyz=(0.074, 0.000, 0.024)),
        material=stage_black,
        name="stage_plate",
    )
    stage.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(xyz=(0.030, 0.000, 0.012)),
        material=stage_black,
        name="underside_support",
    )
    stage.visual(
        Box((0.120, 0.018, 0.022)),
        origin=Origin(xyz=(0.072, 0.060, 0.011)),
        material=accent_grey,
        name="stage_guide",
    )
    stage.visual(
        Box((0.010, 0.014, 0.020)),
        origin=Origin(xyz=(0.016, 0.060, 0.010)),
        material=accent_grey,
        name="guide_stop_rear",
    )
    stage.visual(
        Box((0.010, 0.014, 0.020)),
        origin=Origin(xyz=(0.128, 0.060, 0.010)),
        material=accent_grey,
        name="guide_stop_front",
    )
    stage.visual(
        Box((0.094, 0.010, 0.006)),
        origin=Origin(xyz=(0.066, 0.042, 0.031)),
        material=accent_grey,
        name="slide_bar",
    )

    model.articulation(
        "arm_to_stage",
        ArticulationType.FIXED,
        parent=arm,
        child=stage,
        origin=Origin(xyz=(0.117, 0.000, 0.042)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.020, 0.020, 0.038)),
        origin=Origin(xyz=(0.010, 0.010, 0.019)),
        material=carriage_metal,
        name="side_block",
    )
    carriage.visual(
        Box((0.060, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, -0.007, 0.006)),
        material=carriage_metal,
        name="runner",
    )
    carriage.visual(
        Box((0.060, 0.032, 0.008)),
        origin=Origin(xyz=(0.020, -0.016, 0.036)),
        material=carriage_metal,
        name="slide_holder",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.052, 0.010, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_grey,
        name="carriage_knob",
    )
    carriage.visual(
        Box((0.028, 0.010, 0.020)),
        origin=Origin(xyz=(0.034, 0.010, 0.038)),
        material=carriage_metal,
        name="knob_stem",
    )

    model.articulation(
        "stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=carriage,
        origin=Origin(xyz=(0.005, 0.083, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.060,
            lower=0.000,
            upper=0.055,
        ),
    )

    optical_body = model.part("optical_body")
    optical_body.visual(
        Box((0.034, 0.064, 0.090)),
        origin=Origin(xyz=(0.017, 0.000, 0.045)),
        material=arm_cream,
        name="slider",
    )
    optical_body.visual(
        Box((0.090, 0.078, 0.045)),
        origin=Origin(xyz=(0.060, 0.000, 0.052)),
        material=arm_cream,
        name="body_casting",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.125),
        origin=Origin(xyz=(0.020, 0.000, 0.120), rpy=(0.000, -math.pi / 4.0, 0.000)),
        material=optics_black,
        name="ocular_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.011, length=0.045),
        origin=Origin(xyz=(-0.040, 0.000, 0.180), rpy=(0.000, -math.pi / 4.0, 0.000)),
        material=knob_grey,
        name="eyepiece",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.090, 0.000, 0.020)),
        material=optics_black,
        name="nosepiece_mount",
    )
    optical_body.visual(
        Box((0.040, 0.034, 0.010)),
        origin=Origin(xyz=(0.046, 0.000, 0.086)),
        material=glass,
        name="tube_window",
    )

    model.articulation(
        "arm_to_optical_body",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=optical_body,
        origin=Origin(xyz=(0.078, 0.000, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.080,
            lower=0.000,
            upper=0.055,
        ),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=optics_black,
        name="turret_disc",
    )
    turret.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=knob_grey,
        name="turret_cap",
    )
    objective_specs = (
        ("objective_0", 0.016, 0.000, 0.045, 0.0060),
        ("objective_1", -0.008, 0.014, 0.037, 0.0052),
        ("objective_2", -0.008, -0.014, 0.030, 0.0046),
    )
    for name, x_pos, y_pos, length, radius in objective_specs:
        turret.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, y_pos, -(0.010 + length * 0.5))),
            material=accent_grey,
            name=name,
        )

    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=turret,
        origin=Origin(xyz=(0.090, 0.000, 0.007)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
        ),
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.000, 0.009, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_grey,
        name="wheel",
    )
    coarse_knob.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.000, 0.012, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_grey,
        name="hub",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.000, 0.007, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_grey,
        name="wheel",
    )
    fine_knob.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.000, 0.009, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_grey,
        name="hub",
    )

    model.articulation(
        "arm_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=coarse_knob,
        origin=Origin(xyz=(0.060, 0.030, 0.126)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=8.0,
        ),
    )
    model.articulation(
        "arm_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=fine_knob,
        origin=Origin(xyz=(0.060, 0.030, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("carriage")
    optical_body = object_model.get_part("optical_body")
    coarse_knob = object_model.get_part("coarse_knob")
    fine_knob = object_model.get_part("fine_knob")

    focus = object_model.get_articulation("arm_to_optical_body")
    carriage_slide = object_model.get_articulation("stage_to_carriage")
    turret_spin = object_model.get_articulation("body_to_turret")
    coarse_spin = object_model.get_articulation("arm_to_coarse_knob")
    fine_spin = object_model.get_articulation("arm_to_fine_knob")

    ctx.expect_gap(
        stage,
        base,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="pedestal",
        min_gap=0.050,
        name="stage stands clearly above the base",
    )
    ctx.expect_gap(
        optical_body,
        arm,
        axis="x",
        positive_elem="slider",
        negative_elem="focus_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="optical body rides on the front guide face",
    )
    ctx.expect_gap(
        carriage,
        stage,
        axis="y",
        positive_elem="runner",
        negative_elem="stage_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage runner bears on the stage guide",
    )
    ctx.expect_overlap(
        carriage,
        stage,
        axes="x",
        elem_a="runner",
        elem_b="stage_guide",
        min_overlap=0.040,
        name="rest carriage stays captured by the stage guide",
    )

    focus_limits = focus.motion_limits
    if focus_limits is not None and focus_limits.upper is not None:
        rest_body_pos = ctx.part_world_position(optical_body)
        with ctx.pose({focus: focus_limits.upper}):
            ctx.expect_gap(
                optical_body,
                arm,
                axis="x",
                positive_elem="slider",
                negative_elem="focus_guide",
                max_gap=0.001,
                max_penetration=0.0,
                name="raised optical body still bears on the guide",
            )
            ctx.expect_overlap(
                optical_body,
                arm,
                axes="y",
                elem_a="slider",
                elem_b="focus_guide",
                min_overlap=0.055,
                name="raised optical body stays laterally aligned to the guide",
            )
            ctx.expect_overlap(
                optical_body,
                arm,
                axes="z",
                elem_a="slider",
                elem_b="focus_guide",
                min_overlap=0.030,
                name="raised optical body remains retained on the guide",
            )
            raised_body_pos = ctx.part_world_position(optical_body)
        ctx.check(
            "optical body moves upward on the column",
            rest_body_pos is not None
            and raised_body_pos is not None
            and raised_body_pos[2] > rest_body_pos[2] + 0.040,
            details=f"rest={rest_body_pos}, raised={raised_body_pos}",
        )

    carriage_limits = carriage_slide.motion_limits
    if carriage_limits is not None and carriage_limits.upper is not None:
        rest_carriage_pos = ctx.part_world_position(carriage)
        with ctx.pose({carriage_slide: carriage_limits.upper}):
            ctx.expect_gap(
                carriage,
                stage,
                axis="y",
                positive_elem="runner",
                negative_elem="stage_guide",
                max_gap=0.001,
                max_penetration=0.0,
                name="extended carriage still bears on the stage guide",
            )
            ctx.expect_overlap(
                carriage,
                stage,
                axes="x",
                elem_a="runner",
                elem_b="stage_guide",
                min_overlap=0.040,
                name="extended carriage remains captured by the stage guide",
            )
            extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.check(
            "side carriage slides along the stage length",
            rest_carriage_pos is not None
            and extended_carriage_pos is not None
            and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.040,
            details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
        )

    knob_positions = (
        ctx.part_world_position(coarse_knob),
        ctx.part_world_position(fine_knob),
    )
    ctx.check(
        "coarse and fine knobs are separate visible controls",
        knob_positions[0] is not None
        and knob_positions[1] is not None
        and math.dist(knob_positions[0], knob_positions[1]) > 0.035,
        details=f"coarse={knob_positions[0]}, fine={knob_positions[1]}",
    )

    ctx.check(
        "turret and focus knobs use continuous rotary joints",
        turret_spin.articulation_type == ArticulationType.CONTINUOUS
        and coarse_spin.articulation_type == ArticulationType.CONTINUOUS
        and fine_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"turret={turret_spin.articulation_type}, "
            f"coarse={coarse_spin.articulation_type}, "
            f"fine={fine_spin.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
