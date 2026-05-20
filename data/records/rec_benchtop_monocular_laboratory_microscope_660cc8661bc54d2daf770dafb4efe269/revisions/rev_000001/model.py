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
    model = ArticulatedObject(name="bench_monocular_microscope")

    dark_metal = model.material("dark_metal", rgba=(0.21, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.60, 0.64, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.21, 0.13, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_metal,
        name="foot",
    )
    base.visual(
        Box((0.15, 0.11, 0.022)),
        origin=Origin(xyz=(0.01, 0.0, 0.049)),
        material=steel,
        name="deck",
    )
    base.visual(
        Box((0.05, 0.05, 0.255)),
        origin=Origin(xyz=(-0.058, 0.0, 0.1655)),
        material=steel,
        name="column",
    )
    base.visual(
        Box((0.11, 0.05, 0.05)),
        origin=Origin(xyz=(-0.028, 0.0, 0.102), rpy=(0.0, -0.50, 0.0)),
        material=steel,
        name="brace",
    )
    base.visual(
        Box((0.14, 0.092, 0.012)),
        origin=Origin(xyz=(0.037, 0.0, 0.145)),
        material=steel,
        name="stage_support",
    )
    base.visual(
        Box((0.12, 0.014, 0.004)),
        origin=Origin(xyz=(0.045, -0.028, 0.153)),
        material=chrome,
        name="rail_0",
    )
    base.visual(
        Box((0.12, 0.014, 0.004)),
        origin=Origin(xyz=(0.045, 0.028, 0.153)),
        material=chrome,
        name="rail_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.055, 0.064, 0.060)),
        origin=Origin(xyz=(0.0275, 0.0, 0.030)),
        material=steel,
        name="saddle_block",
    )
    carriage.visual(
        Box((0.062, 0.032, 0.024)),
        origin=Origin(xyz=(0.086, 0.0, 0.064)),
        material=steel,
        name="neck",
    )
    carriage.visual(
        Box((0.042, 0.042, 0.050)),
        origin=Origin(xyz=(0.108, 0.0, 0.095)),
        material=steel,
        name="body_housing",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.108, 0.0, 0.056)),
        material=chrome,
        name="nose_mount",
    )
    carriage.visual(
        Cylinder(radius=0.015, length=0.122),
        origin=Origin(xyz=(0.095, 0.0, 0.155), rpy=(0.0, -0.47, 0.0)),
        material=matte_black,
        name="tube",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(xyz=(0.058, 0.0, 0.2295), rpy=(0.0, -0.47, 0.0)),
        material=rubber,
        name="eyepiece",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=chrome,
        name="turret",
    )
    nosepiece.visual(
        Cylinder(radius=0.0065, length=0.048),
        origin=Origin(xyz=(0.014, 0.0, -0.038)),
        material=chrome,
        name="objective_0",
    )
    nosepiece.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(xyz=(-0.007, 0.012, -0.034)),
        material=chrome,
        name="objective_1",
    )
    nosepiece.visual(
        Cylinder(radius=0.0050, length=0.032),
        origin=Origin(xyz=(-0.007, -0.012, -0.030)),
        material=chrome,
        name="objective_2",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.025, 0.095, 0.008)),
        origin=Origin(xyz=(0.0425, 0.0, 0.008)),
        material=matte_black,
        name="front_bar",
    )
    stage.visual(
        Box((0.025, 0.095, 0.008)),
        origin=Origin(xyz=(-0.0425, 0.0, 0.008)),
        material=matte_black,
        name="rear_bar",
    )
    stage.visual(
        Box((0.060, 0.025, 0.008)),
        origin=Origin(xyz=(0.0, -0.035, 0.008)),
        material=matte_black,
        name="side_bar_0",
    )
    stage.visual(
        Box((0.060, 0.025, 0.008)),
        origin=Origin(xyz=(0.0, 0.035, 0.008)),
        material=matte_black,
        name="side_bar_1",
    )
    stage.visual(
        Box((0.092, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.028, 0.002)),
        material=chrome,
        name="skid_0",
    )
    stage.visual(
        Box((0.092, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.028, 0.002)),
        material=chrome,
        name="skid_1",
    )
    stage.visual(
        Box((0.038, 0.004, 0.006)),
        origin=Origin(xyz=(0.020, -0.018, 0.015)),
        material=chrome,
        name="clip_0",
    )
    stage.visual(
        Box((0.038, 0.004, 0.006)),
        origin=Origin(xyz=(-0.020, 0.018, 0.015)),
        material=chrome,
        name="clip_1",
    )
    stage.visual(
        Cylinder(radius=0.0038, length=0.022),
        origin=Origin(xyz=(0.018, 0.0585, 0.006), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="guide_shaft_0",
    )
    stage.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.018, 0.073, 0.006), rpy=(pi / 2, 0.0, 0.0)),
        material=matte_black,
        name="guide_control_0",
    )
    stage.visual(
        Cylinder(radius=0.0032, length=0.020),
        origin=Origin(xyz=(-0.012, 0.0575, 0.006), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="guide_shaft_1",
    )
    stage.visual(
        Cylinder(radius=0.0075, length=0.008),
        origin=Origin(xyz=(-0.012, 0.0715, 0.006), rpy=(pi / 2, 0.0, 0.0)),
        material=matte_black,
        name="guide_control_1",
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=matte_black,
        name="coarse_wheel",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=matte_black,
        name="fine_wheel",
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.033, 0.0, 0.202)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.10, lower=0.0, upper=0.060),
    )
    model.articulation(
        "nosepiece_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.108, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.045, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=-0.015, upper=0.015),
    )
    model.articulation(
        "coarse_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=coarse_knob,
        origin=Origin(xyz=(-0.056, 0.025, 0.196)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "fine_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=fine_knob,
        origin=Origin(xyz=(-0.043, 0.025, 0.158)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    nosepiece = object_model.get_part("nosepiece")
    stage = object_model.get_part("stage")
    coarse_knob = object_model.get_part("coarse_knob")
    fine_knob = object_model.get_part("fine_knob")

    focus_slide = object_model.get_articulation("focus_slide")
    stage_slide = object_model.get_articulation("stage_slide")

    ctx.expect_gap(
        stage,
        base,
        axis="z",
        positive_elem="skid_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage skid 0 sits on its guide rail",
    )
    ctx.expect_gap(
        stage,
        base,
        axis="z",
        positive_elem="skid_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage skid 1 sits on its guide rail",
    )

    stage_limits = stage_slide.motion_limits
    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            ctx.expect_overlap(
                stage,
                base,
                axes="x",
                elem_a="skid_0",
                elem_b="rail_0",
                min_overlap=0.060,
                name="stage retains insertion on rail 0 at lower travel",
            )
            ctx.expect_overlap(
                stage,
                base,
                axes="x",
                elem_a="skid_1",
                elem_b="rail_1",
                min_overlap=0.060,
                name="stage retains insertion on rail 1 at lower travel",
            )
            lower_pos = ctx.part_world_position(stage)
        with ctx.pose({stage_slide: stage_limits.upper}):
            ctx.expect_overlap(
                stage,
                base,
                axes="x",
                elem_a="skid_0",
                elem_b="rail_0",
                min_overlap=0.060,
                name="stage retains insertion on rail 0 at upper travel",
            )
            ctx.expect_overlap(
                stage,
                base,
                axes="x",
                elem_a="skid_1",
                elem_b="rail_1",
                min_overlap=0.060,
                name="stage retains insertion on rail 1 at upper travel",
            )
            upper_pos = ctx.part_world_position(stage)

        ctx.check(
            "stage carriage moves forward on positive travel",
            lower_pos is not None and upper_pos is not None and upper_pos[0] > lower_pos[0] + 0.020,
            details=f"lower={lower_pos}, upper={upper_pos}",
        )

    focus_limits = focus_slide.motion_limits
    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({focus_slide: focus_limits.lower}):
            ctx.expect_gap(
                nosepiece,
                stage,
                axis="z",
                positive_elem="objective_0",
                min_gap=0.008,
                max_gap=0.030,
                name="lower focus pose still clears the stage",
            )
            rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({focus_slide: focus_limits.upper}):
            raised_pos = ctx.part_world_position(carriage)

        ctx.check(
            "focus carriage rises on positive travel",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.050,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    ctx.expect_gap(
        coarse_knob,
        base,
        axis="y",
        positive_elem="coarse_wheel",
        negative_elem="column",
        max_gap=0.001,
        max_penetration=0.0,
        name="coarse focus knob seats against the arm side",
    )
    ctx.expect_gap(
        fine_knob,
        base,
        axis="y",
        positive_elem="fine_wheel",
        negative_elem="column",
        max_gap=0.001,
        max_penetration=0.0,
        name="fine focus knob seats against the arm side",
    )
    ctx.expect_origin_distance(
        coarse_knob,
        fine_knob,
        axes="xz",
        min_dist=0.030,
        max_dist=0.060,
        name="focus knobs occupy distinct positions on the arm side",
    )

    return ctx.report()


object_model = build_object_model()
