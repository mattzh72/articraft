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
    model = ArticulatedObject(name="single_prismatic_slider")

    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.20, 0.48, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bolt_dark = model.material("black_oxide_fasteners", rgba=(0.02, 0.022, 0.025, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((0.500, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_aluminum,
        name="base_plate",
    )
    base.visual(
        Box((0.430, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=steel,
        name="rail_web",
    )
    base.visual(
        Box((0.430, 0.045, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=steel,
        name="rail_cap",
    )
    for x, name in ((-0.179, "stop_block_0"), (0.179, "stop_block_1")):
        base.visual(
            Box((0.020, 0.100, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.044)),
            material=dark_aluminum,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(-0.162, 0.0, 0.059), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="bumper_0",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.172, 0.0, 0.059), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="bumper_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.140, 0.095, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=carriage_blue,
        name="saddle_top",
    )
    carriage.visual(
        Box((0.120, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.035, 0.040)),
        material=carriage_blue,
        name="guide_block_0",
    )
    carriage.visual(
        Box((0.120, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, -0.035, 0.040)),
        material=carriage_blue,
        name="guide_block_1",
    )
    carriage.visual(
        Box((0.105, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0245, 0.049)),
        material=bolt_dark,
        name="wear_pad_0",
    )
    carriage.visual(
        Box((0.105, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.0245, 0.049)),
        material=bolt_dark,
        name="wear_pad_1",
    )
    carriage.visual(
        Box((0.010, 0.105, 0.060)),
        origin=Origin(xyz=(0.075, 0.0, 0.089)),
        material=carriage_blue,
        name="end_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.045, -0.030), (-0.045, 0.030), (0.045, -0.030), (0.045, 0.030))
    ):
        carriage.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.078)),
            material=bolt_dark,
            name=f"bolt_{i}",
        )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.150),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")
    limits = slide.motion_limits

    ctx.check(
        "single prismatic slider joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "150 mm carriage travel",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - 0.150) < 1e-6,
        details=f"limits={limits}",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="saddle_top",
        negative_elem="rail_cap",
        min_gap=0.003,
        max_gap=0.006,
        name="carriage saddle clears rail cap",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="y",
        positive_elem="wear_pad_0",
        negative_elem="rail_cap",
        min_gap=0.0,
        max_gap=0.0005,
        name="positive guide pad runs on rail side",
    )
    ctx.expect_gap(
        base,
        carriage,
        axis="y",
        positive_elem="rail_cap",
        negative_elem="wear_pad_1",
        min_gap=0.0,
        max_gap=0.0005,
        name="negative guide pad runs on rail side",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="x",
        positive_elem="saddle_top",
        negative_elem="bumper_0",
        min_gap=0.006,
        max_gap=0.014,
        name="left stop is visibly clear at lower limit",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.150}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="guide_block_0",
            elem_b="rail_cap",
            min_overlap=0.080,
            name="extended guide block remains on rail",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="bumper_1",
            negative_elem="end_plate",
            min_gap=0.006,
            max_gap=0.014,
            name="right stop is visibly clear at upper limit",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along rail",
        rest_pos is not None
        and extended_pos is not None
        and abs((extended_pos[0] - rest_pos[0]) - 0.150) < 1e-6
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
