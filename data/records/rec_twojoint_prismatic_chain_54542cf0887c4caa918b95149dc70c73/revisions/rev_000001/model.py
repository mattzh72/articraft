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
    model = ArticulatedObject(name="two_stage_linear_slide")

    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_rail = Material("hardened_dark_rail", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_anodized", rgba=(0.02, 0.025, 0.03, 1.0))
    rubber = Material("black_rubber_wiper", rgba=(0.01, 0.01, 0.01, 1.0))
    red = Material("red_end_plate", rgba=(0.86, 0.08, 0.04, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((0.88, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=aluminum,
        name="base_plate",
    )
    outer_guide.visual(
        Box((0.78, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.065, 0.0525)),
        material=dark_rail,
        name="rail_0",
    )
    outer_guide.visual(
        Box((0.78, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.065, 0.0525)),
        material=dark_rail,
        name="rail_1",
    )
    outer_guide.visual(
        Box((0.025, 0.22, 0.040)),
        origin=Origin(xyz=(-0.42, 0.0, 0.055)),
        material=black,
        name="rear_stop",
    )
    outer_guide.visual(
        Box((0.025, 0.22, 0.040)),
        origin=Origin(xyz=(0.42, 0.0, 0.055)),
        material=black,
        name="front_stop",
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((0.28, 0.17, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="carriage_bridge",
    )
    first_carriage.visual(
        Box((0.18, 0.050, 0.015)),
        origin=Origin(xyz=(0.0, -0.065, -0.0175)),
        material=aluminum,
        name="bearing_shoe_0",
    )
    first_carriage.visual(
        Box((0.18, 0.050, 0.015)),
        origin=Origin(xyz=(0.0, 0.065, -0.0175)),
        material=aluminum,
        name="bearing_shoe_1",
    )
    first_carriage.visual(
        Box((0.24, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, -0.045, 0.0275)),
        material=dark_rail,
        name="upper_way_0",
    )
    first_carriage.visual(
        Box((0.24, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, 0.045, 0.0275)),
        material=dark_rail,
        name="upper_way_1",
    )
    first_carriage.visual(
        Box((0.015, 0.17, 0.020)),
        origin=Origin(xyz=(-0.1325, 0.0, 0.0275)),
        material=rubber,
        name="rear_wiper",
    )

    second_guide = model.part("second_guide")
    second_guide.visual(
        Box((0.34, 0.090, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="short_guide_bar",
    )
    second_guide.visual(
        Box((0.025, 0.150, 0.095)),
        origin=Origin(xyz=(0.1825, 0.0, -0.010)),
        material=red,
        name="end_plate",
    )
    second_guide.visual(
        Box((0.040, 0.018, 0.060)),
        origin=Origin(xyz=(0.185, -0.025, -0.020)),
        material=red,
        name="gusset_0",
    )
    second_guide.visual(
        Box((0.040, 0.018, 0.060)),
        origin=Origin(xyz=(0.185, 0.025, -0.020)),
        material=red,
        name="gusset_1",
    )

    model.articulation(
        "outer_to_carriage",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=first_carriage,
        origin=Origin(xyz=(-0.20, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.40),
    )
    model.articulation(
        "carriage_to_guide",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_guide,
        origin=Origin(xyz=(-0.02, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_guide = object_model.get_part("outer_guide")
    first_carriage = object_model.get_part("first_carriage")
    second_guide = object_model.get_part("second_guide")
    outer_slide = object_model.get_articulation("outer_to_carriage")
    second_slide = object_model.get_articulation("carriage_to_guide")

    ctx.expect_gap(
        first_carriage,
        outer_guide,
        axis="z",
        positive_elem="bearing_shoe_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="first carriage shoe rides on fixed rail 0",
    )
    ctx.expect_gap(
        first_carriage,
        outer_guide,
        axis="z",
        positive_elem="bearing_shoe_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="first carriage shoe rides on fixed rail 1",
    )
    ctx.expect_overlap(
        first_carriage,
        outer_guide,
        axes="x",
        elem_a="bearing_shoe_0",
        elem_b="rail_0",
        min_overlap=0.16,
        name="first stage has retained rail engagement when retracted",
    )
    ctx.expect_gap(
        second_guide,
        first_carriage,
        axis="z",
        positive_elem="short_guide_bar",
        negative_elem="upper_way_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="second guide rides on carriage way",
    )
    ctx.expect_overlap(
        second_guide,
        first_carriage,
        axes="x",
        elem_a="short_guide_bar",
        elem_b="upper_way_0",
        min_overlap=0.08,
        name="second stage has retained guide engagement when retracted",
    )
    ctx.expect_gap(
        second_guide,
        first_carriage,
        axis="x",
        positive_elem="end_plate",
        negative_elem="carriage_bridge",
        min_gap=0.005,
        name="end plate sits ahead of the carriage face",
    )

    first_rest = ctx.part_world_position(first_carriage)
    with ctx.pose({outer_slide: 0.40}):
        first_extended = ctx.part_world_position(first_carriage)
        ctx.expect_overlap(
            first_carriage,
            outer_guide,
            axes="x",
            elem_a="bearing_shoe_0",
            elem_b="rail_0",
            min_overlap=0.16,
            name="first stage keeps rail engagement at full travel",
        )
    ctx.check(
        "first carriage translates along the fixed guide axis",
        first_rest is not None
        and first_extended is not None
        and first_extended[0] > first_rest[0] + 0.35,
        details=f"rest={first_rest}, extended={first_extended}",
    )

    second_rest = ctx.part_world_position(second_guide)
    with ctx.pose({second_slide: 0.20}):
        second_extended = ctx.part_world_position(second_guide)
        ctx.expect_overlap(
            second_guide,
            first_carriage,
            axes="x",
            elem_a="short_guide_bar",
            elem_b="upper_way_0",
            min_overlap=0.08,
            name="second stage keeps guide engagement at full travel",
        )
    ctx.check(
        "second guide translates along the carriage guide axis",
        second_rest is not None
        and second_extended is not None
        and second_extended[0] > second_rest[0] + 0.15,
        details=f"rest={second_rest}, extended={second_extended}",
    )

    with ctx.pose({outer_slide: 0.40, second_slide: 0.20}):
        ctx.expect_gap(
            second_guide,
            outer_guide,
            axis="x",
            positive_elem="end_plate",
            negative_elem="front_stop",
            min_gap=0.05,
            name="end plate projects beyond the fixed guide at full extension",
        )

    return ctx.report()


object_model = build_object_model()
