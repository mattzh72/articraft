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


TRAVEL = 0.40


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_carriage_slide")

    model.material("painted_wall", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("dark_recess", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("brushed_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("blackened_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("anodized_carriage", rgba=(0.86, 0.18, 0.10, 1.0))
    model.material("bronze_wear", rgba=(0.72, 0.48, 0.22, 1.0))

    backplate = model.part("backplate")

    # Grounded wall plate: a broad vertical mounting face behind the guide.
    backplate.visual(
        Box((0.92, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material="painted_wall",
        name="wall_plate",
    )
    backplate.visual(
        Box((0.80, 0.024, 0.22)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material="dark_recess",
        name="rail_bed",
    )

    # Straight guideway: two fixed rails on the front of the wall-backed plate.
    backplate.visual(
        Box((0.76, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.017, 0.075)),
        material="brushed_steel",
        name="top_rail",
    )
    backplate.visual(
        Box((0.76, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.017, -0.075)),
        material="brushed_steel",
        name="bottom_rail",
    )
    backplate.visual(
        Box((0.03, 0.050, 0.24)),
        origin=Origin(xyz=(-0.405, 0.012, 0.0)),
        material="blackened_steel",
        name="end_stop_0",
    )
    backplate.visual(
        Box((0.03, 0.050, 0.24)),
        origin=Origin(xyz=(0.405, 0.012, 0.0)),
        material="blackened_steel",
        name="end_stop_1",
    )

    bolt_rpy = (-math.pi / 2.0, 0.0, 0.0)
    for i, (x, z) in enumerate(
        ((-0.38, -0.135), (-0.38, 0.135), (0.38, -0.135), (0.38, 0.135))
    ):
        backplate.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(x, -0.0135, z), rpy=bolt_rpy),
            material="blackened_steel",
            name=f"wall_bolt_{i}",
        )

    carriage = model.part("carriage")

    # Moving front carriage: a colored load plate, held just in front of the
    # guideway by two bronze bearing shoes that track the fixed rails.
    carriage.visual(
        Box((0.18, 0.045, 0.17)),
        origin=Origin(xyz=(0.0, 0.078, 0.0)),
        material="anodized_carriage",
        name="front_plate",
    )
    carriage.visual(
        Box((0.150, 0.019, 0.036)),
        origin=Origin(xyz=(0.0, 0.0395, 0.075)),
        material="bronze_wear",
        name="upper_pad",
    )
    carriage.visual(
        Box((0.150, 0.019, 0.036)),
        origin=Origin(xyz=(0.0, 0.0395, -0.075)),
        material="bronze_wear",
        name="lower_pad",
    )
    carriage.visual(
        Box((0.150, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.053, 0.075)),
        material="anodized_carriage",
        name="upper_bridge",
    )
    carriage.visual(
        Box((0.150, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.053, -0.075)),
        material="anodized_carriage",
        name="lower_bridge",
    )
    carriage.visual(
        Box((0.102, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.106, 0.0)),
        material="anodized_carriage",
        name="load_boss",
    )

    for i, (x, z) in enumerate(
        ((-0.035, -0.025), (-0.035, 0.025), (0.035, -0.025), (0.035, 0.025))
    ):
        carriage.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, 0.1145, z), rpy=bolt_rpy),
            material="blackened_steel",
            name=f"carriage_bolt_{i}",
        )

    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("slide")

    ctx.check(
        "one prismatic carriage joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_gap(
        carriage,
        backplate,
        axis="y",
        positive_elem="upper_pad",
        negative_elem="top_rail",
        min_gap=0.0,
        max_gap=0.001,
        name="upper bearing shoe clears top rail",
    )
    ctx.expect_gap(
        carriage,
        backplate,
        axis="y",
        positive_elem="lower_pad",
        negative_elem="bottom_rail",
        min_gap=0.0,
        max_gap=0.001,
        name="lower bearing shoe clears bottom rail",
    )
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="xz",
        elem_a="upper_pad",
        elem_b="top_rail",
        min_overlap=0.020,
        name="upper shoe is aligned with guide rail",
    )
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="xz",
        elem_a="lower_pad",
        elem_b="bottom_rail",
        min_overlap=0.020,
        name="lower shoe is aligned with guide rail",
    )
    ctx.expect_within(
        carriage,
        backplate,
        axes="x",
        inner_elem="front_plate",
        outer_elem="rail_bed",
        margin=0.0,
        name="carriage starts inside the guideway length",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: TRAVEL}):
        ctx.expect_within(
            carriage,
            backplate,
            axes="x",
            inner_elem="front_plate",
            outer_elem="rail_bed",
            margin=0.0,
            name="carriage remains inside the guideway length at full travel",
        )
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="xz",
            elem_a="upper_pad",
            elem_b="top_rail",
            min_overlap=0.020,
            name="upper shoe remains engaged at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along positive guide direction",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
