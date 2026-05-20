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
    model = ArticulatedObject(name="three_stage_linear_slide_chain")

    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = Material("ground_rail_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    carriage_blue = Material("anodized_blue_carriage", rgba=(0.05, 0.22, 0.52, 1.0))
    carriage_orange = Material("orange_second_carriage", rgba=(0.85, 0.36, 0.08, 1.0))
    guide_yellow = Material("yellow_third_guide", rgba=(0.95, 0.78, 0.12, 1.0))
    black = Material("black_end_stops", rgba=(0.02, 0.02, 0.02, 1.0))

    # Fixed X-axis base guide: low bed, twin rails, and rigid end stops.
    base = model.part("base_guide")
    base.visual(
        Box((0.95, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="bed_plate",
    )
    for rail_name, y in (("base_rail_0", -0.085), ("base_rail_1", 0.085)):
        base.visual(
            Box((0.86, 0.035, 0.037)),
            origin=Origin(xyz=(0.0, y, 0.0525)),
            material=rail_steel,
            name=rail_name,
        )
    for idx, x in enumerate((-0.43, 0.43)):
        base.visual(
            Box((0.040, 0.235, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.064)),
            material=black,
            name=f"base_end_stop_{idx}",
        )
    base.visual(
        Box((0.66, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=black,
        name="center_scale_strip",
    )

    # First carriage rides on the base rails and carries the second Y-axis guide.
    first = model.part("first_carriage")
    for shoe_name, y in (("lower_shoe_0", -0.085), ("lower_shoe_1", 0.085)):
        first.visual(
            Box((0.225, 0.050, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0175)),
            material=carriage_blue,
            name=shoe_name,
        )
    first.visual(
        Box((0.260, 0.235, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=carriage_blue,
        name="first_saddle",
    )
    first.visual(
        Box((0.170, 0.310, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=dark_steel,
        name="second_guide_base",
    )
    for rail_name, x in (("second_rail_0", -0.055), ("second_rail_1", 0.055)):
        first.visual(
            Box((0.032, 0.440, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.102)),
            material=rail_steel,
            name=rail_name,
        )
    for idx, y in enumerate((-0.220, 0.220)):
        first.visual(
            Box((0.168, 0.032, 0.056)),
            origin=Origin(xyz=(0.0, y, 0.115)),
            material=black,
            name=f"second_end_stop_{idx}",
        )

    # Second carriage rides on the Y guide and includes a U-shaped vertical way.
    second = model.part("second_carriage")
    for shoe_name, x in (("upper_shoe_0", -0.055), ("upper_shoe_1", 0.055)):
        second.visual(
            Box((0.048, 0.155, 0.032)),
            origin=Origin(xyz=(x, 0.0, 0.016)),
            material=carriage_orange,
            name=shoe_name,
        )
    second.visual(
        Box((0.170, 0.190, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=carriage_orange,
        name="second_saddle",
    )
    for idx, x in enumerate((-0.085, 0.085)):
        second.visual(
            Box((0.035, 0.070, 0.030)),
            origin=Origin(xyz=(x, 0.115, 0.071)),
            material=carriage_orange,
            name=f"third_foot_{idx}",
        )
        second.visual(
            Box((0.032, 0.070, 0.150)),
            origin=Origin(xyz=(x, 0.120, 0.161)),
            material=dark_steel,
            name=f"third_side_way_{idx}",
        )
    second.visual(
        Box((0.200, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.085, 0.161)),
        material=dark_steel,
        name="third_rear_way",
    )
    second.visual(
        Box((0.200, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.085, 0.248)),
        material=black,
        name="third_top_tie",
    )

    # Short third Z-axis guide, capped by the terminal end plate.
    third = model.part("third_guide")
    for rod_name, x in (("guide_rod_0", -0.035), ("guide_rod_1", 0.035)):
        third.visual(
            Box((0.024, 0.024, 0.340)),
            origin=Origin(xyz=(x, 0.0, 0.110)),
            material=rail_steel,
            name=rod_name,
        )
    third.visual(
        Box((0.108, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.274)),
        material=guide_yellow,
        name="rod_cap",
    )
    third.visual(
        Box((0.180, 0.095, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, 0.295)),
        material=guide_yellow,
        name="end_plate",
    )

    model.articulation(
        "base_to_first_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first,
        origin=Origin(xyz=(-0.270, 0.0, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.380),
    )
    model.articulation(
        "first_to_second_carriage",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.0, -0.125, 0.117)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.265),
    )
    model.articulation(
        "second_to_third_guide",
        ArticulationType.PRISMATIC,
        parent=second,
        child=third,
        origin=Origin(xyz=(0.0, 0.106, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.22, lower=0.0, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    third = object_model.get_part("third_guide")
    j1 = object_model.get_articulation("base_to_first_carriage")
    j2 = object_model.get_articulation("first_to_second_carriage")
    j3 = object_model.get_articulation("second_to_third_guide")

    for joint, axis in (
        (j1, (1.0, 0.0, 0.0)),
        (j2, (0.0, 1.0, 0.0)),
        (j3, (0.0, 0.0, 1.0)),
    ):
        ctx.check(
            f"{joint.name} is a prismatic slide",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == axis,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        first,
        base,
        axis="z",
        positive_elem="lower_shoe_0",
        negative_elem="base_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="first carriage sits on the base guide rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="upper_shoe_0",
        negative_elem="second_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="second carriage sits on the second guide rail",
    )
    ctx.expect_overlap(
        third,
        second,
        axes="z",
        elem_a="guide_rod_0",
        elem_b="third_rear_way",
        min_overlap=0.10,
        name="third guide is retained in the vertical way at rest",
    )

    rest_first = ctx.part_world_position(first)
    rest_second = ctx.part_world_position(second)
    rest_third = ctx.part_world_position(third)
    with ctx.pose({j1: 0.380, j2: 0.265, j3: 0.120}):
        ctx.expect_within(
            first,
            base,
            axes="x",
            inner_elem="lower_shoe_0",
            outer_elem="base_rail_0",
            margin=0.0,
            name="first shoe remains on the base rail at full travel",
        )
        ctx.expect_within(
            second,
            first,
            axes="y",
            inner_elem="upper_shoe_0",
            outer_elem="second_rail_0",
            margin=0.0,
            name="second shoe remains on the second rail at full travel",
        )
        ctx.expect_overlap(
            third,
            second,
            axes="z",
            elem_a="guide_rod_0",
            elem_b="third_rear_way",
            min_overlap=0.070,
            name="third guide remains captured at full travel",
        )
        full_first = ctx.part_world_position(first)
        full_second = ctx.part_world_position(second)
        full_third = ctx.part_world_position(third)

    ctx.check(
        "stage motions follow x then y then z",
        rest_first is not None
        and rest_second is not None
        and rest_third is not None
        and full_first is not None
        and full_second is not None
        and full_third is not None
        and full_first[0] > rest_first[0] + 0.30
        and full_second[1] > rest_second[1] + 0.20
        and full_third[2] > rest_third[2] + 0.09,
        details=f"rest={rest_first, rest_second, rest_third}, full={full_first, full_second, full_third}",
    )

    return ctx.report()


object_model = build_object_model()
