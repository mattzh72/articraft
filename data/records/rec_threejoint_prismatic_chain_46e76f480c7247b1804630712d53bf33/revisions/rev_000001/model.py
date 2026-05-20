from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_three_carriage_linear_chain")

    dark_steel = Material("dark_steel", color=(0.08, 0.09, 0.10, 1.0))
    rail_steel = Material("ground_steel", color=(0.55, 0.58, 0.60, 1.0))
    large_blue = Material("anodized_blue", color=(0.05, 0.22, 0.62, 1.0))
    medium_teal = Material("anodized_teal", color=(0.02, 0.46, 0.50, 1.0))
    small_orange = Material("anodized_orange", color=(0.88, 0.34, 0.06, 1.0))
    rubber = Material("rubber_stop", color=(0.015, 0.015, 0.018, 1.0))

    guide = model.part("guide_frame")
    guide.visual(
        Box((1.25, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    guide.visual(
        Box((1.12, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -0.11, 0.0675)),
        material=rail_steel,
        name="guide_rail_0",
    )
    guide.visual(
        Box((1.12, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.11, 0.0675)),
        material=rail_steel,
        name="guide_rail_1",
    )
    for idx, x in enumerate((-0.595, 0.595)):
        guide.visual(
            Box((0.035, 0.34, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.090)),
            material=dark_steel,
            name=f"end_stop_{idx}",
        )
        guide.visual(
            Box((0.012, 0.22, 0.030)),
            origin=Origin(xyz=(x - math.copysign(0.017, x), 0.0, 0.115)),
            material=rubber,
            name=f"bumper_{idx}",
        )

    large = model.part("large_carriage")
    large.visual(
        Box((0.72, 0.28, 0.050)),
        origin=Origin(),
        material=large_blue,
        name="large_plate",
    )
    large.visual(
        Box((0.66, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.11, -0.016)),
        material=rail_steel,
        name="large_runner_0",
    )
    large.visual(
        Box((0.66, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, 0.11, -0.016)),
        material=rail_steel,
        name="large_runner_1",
    )
    large.visual(
        Box((0.58, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.092, 0.045)),
        material=rail_steel,
        name="large_inner_rail_0",
    )
    large.visual(
        Box((0.58, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.092, 0.045)),
        material=rail_steel,
        name="large_inner_rail_1",
    )
    large.visual(
        Box((0.030, 0.22, 0.050)),
        origin=Origin(xyz=(-0.345, 0.0, 0.050)),
        material=large_blue,
        name="large_cross_stop",
    )

    medium = model.part("medium_carriage")
    medium.visual(
        Box((0.44, 0.205, 0.040)),
        origin=Origin(),
        material=medium_teal,
        name="medium_plate",
    )
    medium.visual(
        Box((0.40, 0.025, 0.016)),
        origin=Origin(xyz=(0.0, -0.092, -0.026)),
        material=rail_steel,
        name="medium_runner_0",
    )
    medium.visual(
        Box((0.40, 0.025, 0.016)),
        origin=Origin(xyz=(0.0, 0.092, -0.026)),
        material=rail_steel,
        name="medium_runner_1",
    )
    medium.visual(
        Box((0.34, 0.018, 0.025)),
        origin=Origin(xyz=(0.0, -0.062, 0.0325)),
        material=rail_steel,
        name="medium_inner_rail_0",
    )
    medium.visual(
        Box((0.34, 0.018, 0.025)),
        origin=Origin(xyz=(0.0, 0.062, 0.0325)),
        material=rail_steel,
        name="medium_inner_rail_1",
    )
    medium.visual(
        Box((0.024, 0.125, 0.040)),
        origin=Origin(xyz=(-0.212, 0.0, 0.030)),
        material=medium_teal,
        name="medium_cross_stop",
    )

    small = model.part("small_carriage")
    small.visual(
        Box((0.24, 0.130, 0.032)),
        origin=Origin(),
        material=small_orange,
        name="small_plate",
    )
    small.visual(
        Box((0.20, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.062, -0.020)),
        material=rail_steel,
        name="small_runner_0",
    )
    small.visual(
        Box((0.20, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.062, -0.020)),
        material=rail_steel,
        name="small_runner_1",
    )
    small.visual(
        Box((0.020, 0.105, 0.058)),
        origin=Origin(xyz=(0.130, 0.0, 0.004)),
        material=small_orange,
        name="output_face",
    )
    small.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.180, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="output_pin",
    )

    model.articulation(
        "guide_to_large",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=large,
        origin=Origin(xyz=(-0.20, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.28),
    )
    model.articulation(
        "large_to_medium",
        ArticulationType.PRISMATIC,
        parent=large,
        child=medium,
        origin=Origin(xyz=(-0.12, 0.0, 0.099)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.22),
    )
    model.articulation(
        "medium_to_small",
        ArticulationType.PRISMATIC,
        parent=medium,
        child=small,
        origin=Origin(xyz=(-0.065, 0.0, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    guide = object_model.get_part("guide_frame")
    large = object_model.get_part("large_carriage")
    medium = object_model.get_part("medium_carriage")
    small = object_model.get_part("small_carriage")
    guide_to_large = object_model.get_articulation("guide_to_large")
    large_to_medium = object_model.get_articulation("large_to_medium")
    medium_to_small = object_model.get_articulation("medium_to_small")

    ctx.check(
        "three serial prismatic joints",
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (guide_to_large, large_to_medium, medium_to_small)
        ),
        details="Expected exactly three prismatic articulations in the nested chain.",
    )
    ctx.check(
        "common slide axis",
        guide_to_large.axis == large_to_medium.axis == medium_to_small.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes={guide_to_large.axis}, {large_to_medium.axis}, {medium_to_small.axis}"
        ),
    )

    with ctx.pose({guide_to_large: 0.0, large_to_medium: 0.0, medium_to_small: 0.0}):
        ctx.expect_contact(
            large,
            guide,
            elem_a="large_runner_0",
            elem_b="guide_rail_0",
            name="large carriage rests on the fixed guide rail",
        )
        ctx.expect_contact(
            medium,
            large,
            elem_a="medium_runner_0",
            elem_b="large_inner_rail_0",
            name="medium carriage rests on the large carriage rail",
        )
        ctx.expect_contact(
            small,
            medium,
            elem_a="small_runner_0",
            elem_b="medium_inner_rail_0",
            name="small output carriage rests on the medium rail",
        )

    rest_large = ctx.part_world_position(large)
    rest_medium = ctx.part_world_position(medium)
    rest_small = ctx.part_world_position(small)

    with ctx.pose({guide_to_large: 0.28, large_to_medium: 0.22, medium_to_small: 0.16}):
        extended_large = ctx.part_world_position(large)
        extended_medium = ctx.part_world_position(medium)
        extended_small = ctx.part_world_position(small)

        ctx.expect_overlap(
            large,
            guide,
            axes="x",
            min_overlap=0.50,
            elem_a="large_runner_0",
            elem_b="guide_rail_0",
            name="large carriage remains captured at full travel",
        )
        ctx.expect_overlap(
            medium,
            large,
            axes="x",
            min_overlap=0.30,
            elem_a="medium_runner_0",
            elem_b="large_inner_rail_0",
            name="medium carriage remains captured at full travel",
        )
        ctx.expect_overlap(
            small,
            medium,
            axes="x",
            min_overlap=0.14,
            elem_a="small_runner_0",
            elem_b="medium_inner_rail_0",
            name="small carriage remains captured at full travel",
        )

    ctx.check(
        "large carriage translates along x",
        rest_large is not None
        and extended_large is not None
        and extended_large[0] > rest_large[0] + 0.25
        and abs(extended_large[1] - rest_large[1]) < 1e-9
        and abs(extended_large[2] - rest_large[2]) < 1e-9,
        details=f"rest={rest_large}, extended={extended_large}",
    )
    ctx.check(
        "medium carriage follows the serial x travel",
        rest_medium is not None
        and extended_medium is not None
        and extended_medium[0] > rest_medium[0] + 0.45
        and abs(extended_medium[1] - rest_medium[1]) < 1e-9
        and abs(extended_medium[2] - rest_medium[2]) < 1e-9,
        details=f"rest={rest_medium}, extended={extended_medium}",
    )
    ctx.check(
        "small output carriage accumulates all three x travels",
        rest_small is not None
        and extended_small is not None
        and extended_small[0] > rest_small[0] + 0.63
        and abs(extended_small[1] - rest_small[1]) < 1e-9
        and abs(extended_small[2] - rest_small[2]) < 1e-9,
        details=f"rest={rest_small}, extended={extended_small}",
    )

    return ctx.report()


object_model = build_object_model()
