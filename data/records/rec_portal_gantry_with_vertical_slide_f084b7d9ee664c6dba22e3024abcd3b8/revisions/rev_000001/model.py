from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_portal_module")

    dark = Material("dark_anodized", color=(0.08, 0.09, 0.10, 1.0))
    aluminum = Material("brushed_aluminum", color=(0.68, 0.70, 0.68, 1.0))
    rail = Material("linear_rail_steel", color=(0.18, 0.20, 0.22, 1.0))
    carriage_blue = Material("carriage_blue", color=(0.05, 0.18, 0.48, 1.0))
    stage_black = Material("stage_black", color=(0.03, 0.035, 0.04, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.45, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="base_plate",
    )
    frame.visual(
        Box((1.25, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=aluminum,
        name="raised_plinth",
    )
    for x, name in ((-0.62, "side_plate_0"), (0.62, "side_plate_1")):
        frame.visual(
            Box((0.075, 0.16, 0.40)),
            origin=Origin(xyz=(x, 0.0, 0.245)),
            material=aluminum,
            name=name,
        )
    frame.visual(
        Box((1.24, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
        material=aluminum,
        name="beam",
    )
    frame.visual(
        Box((1.10, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.058, 0.455)),
        material=rail,
        name="front_linear_rail",
    )
    frame.visual(
        Box((1.10, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.058, 0.455)),
        material=rail,
        name="rear_linear_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.20, 0.22, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=carriage_blue,
        name="top_saddle",
    )
    carriage.visual(
        Box((0.20, 0.026, 0.095)),
        origin=Origin(xyz=(0.0, -0.090, -0.0075)),
        material=carriage_blue,
        name="front_cheek",
    )
    carriage.visual(
        Box((0.20, 0.026, 0.095)),
        origin=Origin(xyz=(0.0, 0.090, -0.0075)),
        material=carriage_blue,
        name="rear_cheek",
    )
    carriage.visual(
        Box((0.18, 0.22, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=carriage_blue,
        name="bottom_saddle",
    )
    carriage.visual(
        Box((0.13, 0.105, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=carriage_blue,
        name="z_sleeve",
    )
    carriage.visual(
        Box((0.16, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, -0.108, -0.010)),
        material=rail,
        name="front_bearing_pad",
    )
    carriage.visual(
        Box((0.16, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, 0.108, -0.010)),
        material=rail,
        name="rear_bearing_pad",
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.32, 0.0, 0.445)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.64),
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.060, 0.040, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=stage_black,
        name="slide_tongue",
    )
    stage.visual(
        Box((0.105, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.025, -0.185)),
        material=dark,
        name="front_face",
    )
    stage.visual(
        Box((0.120, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.2175)),
        material=dark,
        name="tool_plate",
    )

    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.20, lower=0.0, upper=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    beam_slide = object_model.get_articulation("beam_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_stage")

    ctx.allow_overlap(
        carriage,
        stage,
        elem_a="z_sleeve",
        elem_b="slide_tongue",
        reason="The compact Z stage tongue is intentionally represented as sliding inside the fixed sleeve proxy.",
    )

    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="top_saddle",
        negative_elem="beam",
        name="carriage saddle rides on the beam",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="xy",
        min_overlap=0.08,
        elem_a="top_saddle",
        elem_b="beam",
        name="carriage saddle remains over the beam footprint",
    )
    ctx.expect_within(
        stage,
        carriage,
        axes="xy",
        margin=0.0,
        inner_elem="slide_tongue",
        outer_elem="z_sleeve",
        name="vertical tongue is contained by the sleeve in plan",
    )
    ctx.expect_overlap(
        stage,
        carriage,
        axes="z",
        min_overlap=0.12,
        elem_a="slide_tongue",
        elem_b="z_sleeve",
        name="retracted vertical stage stays inserted",
    )

    carriage_rest = ctx.part_world_position(carriage)
    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({beam_slide: 0.64, z_slide: 0.08}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="top_saddle",
            negative_elem="beam",
            name="carriage remains seated at far travel",
        )
        ctx.expect_overlap(
            stage,
            carriage,
            axes="z",
            min_overlap=0.05,
            elem_a="slide_tongue",
            elem_b="z_sleeve",
            name="extended vertical stage retains insertion",
        )
        carriage_far = ctx.part_world_position(carriage)
        stage_low = ctx.part_world_position(stage)

    ctx.check(
        "bridge carriage translates along the beam",
        carriage_rest is not None
        and carriage_far is not None
        and carriage_far[0] > carriage_rest[0] + 0.60,
        details=f"rest={carriage_rest}, far={carriage_far}",
    )
    ctx.check(
        "vertical stage travels downward",
        stage_rest is not None
        and stage_low is not None
        and stage_low[2] < stage_rest[2] - 0.07,
        details=f"rest={stage_rest}, low={stage_low}",
    )

    return ctx.report()


object_model = build_object_model()
