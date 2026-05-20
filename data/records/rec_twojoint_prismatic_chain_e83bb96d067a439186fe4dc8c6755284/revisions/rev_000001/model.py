from __future__ import annotations

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
    model = ArticulatedObject(name="two_stage_service_fixture")

    bead_blasted = model.material("bead_blasted_aluminum", rgba=(0.63, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel_ways", rgba=(0.12, 0.13, 0.14, 1.0))
    blue_anodized = model.material("blue_anodized_carriages", rgba=(0.08, 0.26, 0.72, 1.0))
    black_oxide = model.material("black_oxide_fasteners", rgba=(0.015, 0.015, 0.018, 1.0))
    brass = model.material("brass_reference_pins", rgba=(0.86, 0.62, 0.25, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        Box((0.76, 0.26, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=bead_blasted,
        name="base_plate",
    )
    for rail_name, y in (("lower_rail_0", -0.078), ("lower_rail_1", 0.078)):
        guide.visual(
            Box((0.62, 0.035, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0725)),
            material=dark_steel,
            name=rail_name,
        )
    for x in (-0.29, 0.29):
        for y in (-0.112, 0.112):
            guide.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(x, y, 0.048)),
                material=black_oxide,
                name=f"base_screw_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    first = model.part("first_carriage")
    for shoe_name, y in (("first_shoe_0", -0.078), ("first_shoe_1", 0.078)):
        first.visual(
            Box((0.36, 0.024, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=dark_steel,
            name=shoe_name,
        )
    first.visual(
        Box((0.43, 0.18, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=blue_anodized,
        name="first_plate",
    )
    for rail_name, y in (("upper_rail_0", -0.052), ("upper_rail_1", 0.052)):
        first.visual(
            Box((0.36, 0.025, 0.038)),
            origin=Origin(xyz=(0.0, y, 0.083)),
            material=dark_steel,
            name=rail_name,
        )
    for x in (-0.145, 0.145):
        for y in (-0.083, 0.083):
            first.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, 0.067)),
                material=black_oxide,
                name=f"first_screw_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    second = model.part("second_carriage")
    for shoe_name, y in (("second_shoe_0", -0.052), ("second_shoe_1", 0.052)):
        second.visual(
            Box((0.235, 0.016, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.012)),
            material=dark_steel,
            name=shoe_name,
        )
    second.visual(
        Box((0.29, 0.13, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=blue_anodized,
        name="second_plate",
    )
    second.visual(
        Box((0.13, 0.082, 0.020)),
        origin=Origin(xyz=(-0.035, 0.0, 0.068)),
        material=bead_blasted,
        name="fixture_pad",
    )
    second.visual(
        Box((0.026, 0.12, 0.070)),
        origin=Origin(xyz=(0.132, 0.0, 0.093)),
        material=bead_blasted,
        name="front_work_stop",
    )
    for x in (-0.082, 0.012):
        for y in (-0.032, 0.032):
            second.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(x, y, 0.081)),
                material=brass,
                name=f"pad_pin_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    model.articulation(
        "guide_to_first_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=first,
        origin=Origin(xyz=(-0.12, 0.0, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.28, effort=220.0, velocity=0.35),
    )
    model.articulation(
        "first_to_second_slide",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(-0.055, 0.0, 0.102)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.18, effort=120.0, velocity=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    first_slide = object_model.get_articulation("guide_to_first_slide")
    second_slide = object_model.get_articulation("first_to_second_slide")

    ctx.check(
        "two serial prismatic stages",
        first_slide.articulation_type == ArticulationType.PRISMATIC
        and second_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={first_slide.articulation_type}, {second_slide.articulation_type}",
    )

    ctx.expect_contact(
        first,
        guide,
        elem_a="first_shoe_0",
        elem_b="lower_rail_0",
        name="first carriage shoe rests on fixed rail",
    )
    ctx.expect_contact(
        second,
        first,
        elem_a="second_shoe_0",
        elem_b="upper_rail_0",
        name="second carriage shoe rests on first carriage rail",
    )
    ctx.expect_overlap(
        first,
        guide,
        axes="xy",
        min_overlap=0.020,
        elem_a="first_shoe_0",
        elem_b="lower_rail_0",
        name="first stage has retained rail overlap",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="xy",
        min_overlap=0.014,
        elem_a="second_shoe_0",
        elem_b="upper_rail_0",
        name="second stage has retained rail overlap",
    )
    ctx.expect_within(
        first,
        guide,
        axes="y",
        margin=0.002,
        inner_elem="first_shoe_0",
        outer_elem="lower_rail_0",
        name="first shoe is captured laterally by fixed rail",
    )
    ctx.expect_within(
        second,
        first,
        axes="y",
        margin=0.002,
        inner_elem="second_shoe_0",
        outer_elem="upper_rail_0",
        name="second shoe is captured laterally by upper rail",
    )

    rest_first = ctx.part_world_position(first)
    rest_second = ctx.part_world_position(second)
    with ctx.pose({first_slide: 0.28, second_slide: 0.18}):
        ctx.expect_overlap(
            first,
            guide,
            axes="x",
            min_overlap=0.30,
            elem_a="first_shoe_0",
            elem_b="lower_rail_0",
            name="first stage remains overlapped at full travel",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="x",
            min_overlap=0.13,
            elem_a="second_shoe_0",
            elem_b="upper_rail_0",
            name="second stage remains overlapped at full travel",
        )
        extended_first = ctx.part_world_position(first)
        extended_second = ctx.part_world_position(second)

    ctx.check(
        "first carriage translates along guide",
        rest_first is not None
        and extended_first is not None
        and extended_first[0] > rest_first[0] + 0.25,
        details=f"rest={rest_first}, extended={extended_first}",
    )
    ctx.check(
        "second carriage stacks both slide motions",
        rest_second is not None
        and extended_second is not None
        and extended_second[0] > rest_second[0] + 0.43,
        details=f"rest={rest_second}, extended={extended_second}",
    )

    return ctx.report()


object_model = build_object_model()
