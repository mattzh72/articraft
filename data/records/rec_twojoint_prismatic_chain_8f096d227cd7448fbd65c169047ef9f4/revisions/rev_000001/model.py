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
    model = ArticulatedObject(name="two_joint_prismatic_chain")

    painted_steel = model.material("painted_steel", color=(0.12, 0.14, 0.16, 1.0))
    guide_steel = model.material("ground_guide_steel", color=(0.72, 0.76, 0.78, 1.0))
    aluminum = model.material("brushed_aluminum", color=(0.58, 0.62, 0.64, 1.0))
    black = model.material("black_bearing_blocks", color=(0.03, 0.035, 0.04, 1.0))
    red = model.material("red_end_stops", color=(0.70, 0.06, 0.04, 1.0))

    # Dimensions are in meters.  The layout reads as a small benchtop XY linear
    # slide: the base guide drives the lower carriage along X, and that carriage
    # carries a second guide whose stage travels along Y.
    base = model.part("base_guide")
    base.visual(
        Box((1.20, 0.34, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted_steel,
        name="base_plate",
    )
    for y, rail_name, pedestal_name in (
        (0.105, "rail_0", "rail_pedestal_0"),
        (-0.105, "rail_1", "rail_pedestal_1"),
    ):
        base.visual(
            Box((1.10, 0.055, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=painted_steel,
            name=pedestal_name,
        )
        base.visual(
            Box((1.10, 0.035, 0.036)),
            origin=Origin(xyz=(0.0, y, 0.068)),
            material=guide_steel,
            name=rail_name,
        )

    for x, idx in ((-0.565, 0), (0.565, 1)):
        base.visual(
            Box((0.050, 0.31, 0.100)),
            origin=Origin(xyz=(x, 0.0, 0.079)),
            material=red,
            name=f"end_stop_{idx}",
        )

    for idx, (x, y) in enumerate(
        (
            (-0.42, 0.145),
            (0.0, 0.145),
            (0.42, 0.145),
            (-0.42, -0.145),
            (0.0, -0.145),
            (0.42, -0.145),
        )
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.033)),
            material=black,
            name=f"base_screw_{idx}",
        )

    first = model.part("first_carriage")
    for y, pad_name in ((0.105, "bearing_pad_0"), (-0.105, "bearing_pad_1")):
        first.visual(
            Box((0.250, 0.052, 0.028)),
            origin=Origin(xyz=(0.0, y, -0.016)),
            material=black,
            name=pad_name,
        )

    for idx, y in enumerate((0.078, 0.132, -0.078, -0.132)):
        first.visual(
            Box((0.250, 0.010, 0.043)),
            origin=Origin(xyz=(0.0, y, -0.021)),
            material=black,
            name=f"rail_keeper_{idx}",
        )

    first.visual(
        Box((0.320, 0.280, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=aluminum,
        name="saddle_plate",
    )
    for x, rail_name, base_name in (
        (0.085, "upper_rail_0", "upper_rail_base_0"),
        (-0.085, "upper_rail_1", "upper_rail_base_1"),
    ):
        first.visual(
            Box((0.046, 0.440, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.0365)),
            material=aluminum,
            name=base_name,
        )
        first.visual(
            Box((0.032, 0.380, 0.032)),
            origin=Origin(xyz=(x, 0.0, 0.056)),
            material=guide_steel,
            name=rail_name,
        )

    second = model.part("second_stage")
    for x, pad_name in ((0.085, "stage_pad_0"), (-0.085, "stage_pad_1")):
        second.visual(
            Box((0.052, 0.145, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.015)),
            material=black,
            name=pad_name,
        )

    keeper_xs = (0.060, 0.110, -0.060, -0.110)
    for idx, x in enumerate(keeper_xs):
        second.visual(
            Box((0.008, 0.145, 0.044)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=black,
            name=f"stage_keeper_{idx}",
        )

    second.visual(
        Box((0.280, 0.200, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=aluminum,
        name="stage_table",
    )
    for idx, (x, y) in enumerate(
        ((-0.095, -0.060), (0.095, -0.060), (-0.095, 0.060), (0.095, 0.060))
    ):
        second.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.035)),
            material=black,
            name=f"table_screw_{idx}",
        )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first,
        origin=Origin(xyz=(-0.250, 0.0, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=0.0, upper=0.42),
    )
    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.0, -0.090, 0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_stage")
    first_slide = object_model.get_articulation("base_to_carriage")
    second_slide = object_model.get_articulation("carriage_to_stage")

    ctx.check(
        "two serial prismatic joints",
        len(object_model.articulations) == 2
        and first_slide.articulation_type == ArticulationType.PRISMATIC
        and second_slide.articulation_type == ArticulationType.PRISMATIC
        and first_slide.parent == "base_guide"
        and first_slide.child == "first_carriage"
        and second_slide.parent == "first_carriage"
        and second_slide.child == "second_stage",
        details=(
            f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in object_model.articulations]}"
        ),
    )

    ctx.expect_gap(
        first,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="bearing_pad_0",
        negative_elem="rail_0",
        name="first carriage pad sits on base rail",
    )
    ctx.expect_gap(
        first,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="bearing_pad_1",
        negative_elem="rail_1",
        name="first carriage second pad sits on base rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_pad_0",
        negative_elem="upper_rail_0",
        name="second stage pad sits on upper rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_pad_1",
        negative_elem="upper_rail_1",
        name="second stage second pad sits on upper rail",
    )

    rest_first = ctx.part_world_position(first)
    rest_second = ctx.part_world_position(second)

    with ctx.pose({first_slide: 0.42}):
        extended_first = ctx.part_world_position(first)
        ctx.expect_overlap(
            first,
            base,
            axes="x",
            min_overlap=0.20,
            elem_a="bearing_pad_0",
            elem_b="rail_0",
            name="first carriage remains supported at full travel",
        )

    with ctx.pose({second_slide: 0.18}):
        extended_second = ctx.part_world_position(second)
        ctx.expect_overlap(
            second,
            first,
            axes="y",
            min_overlap=0.12,
            elem_a="stage_pad_0",
            elem_b="upper_rail_0",
            name="second stage remains supported at full travel",
        )

    with ctx.pose({first_slide: 0.42, second_slide: 0.18}):
        serial_second = ctx.part_world_position(second)

    ctx.check(
        "first joint translates along base X axis",
        rest_first is not None
        and extended_first is not None
        and extended_first[0] > rest_first[0] + 0.40
        and abs(extended_first[1] - rest_first[1]) < 0.002
        and abs(extended_first[2] - rest_first[2]) < 0.002,
        details=f"rest={rest_first}, extended={extended_first}",
    )
    ctx.check(
        "second joint translates along carried Y axis",
        rest_second is not None
        and extended_second is not None
        and extended_second[1] > rest_second[1] + 0.17
        and abs(extended_second[0] - rest_second[0]) < 0.002
        and abs(extended_second[2] - rest_second[2]) < 0.002,
        details=f"rest={rest_second}, extended={extended_second}",
    )
    ctx.check(
        "second stage motion is serial with first carriage",
        rest_second is not None
        and serial_second is not None
        and serial_second[0] > rest_second[0] + 0.40
        and serial_second[1] > rest_second[1] + 0.17,
        details=f"rest={rest_second}, combined={serial_second}",
    )

    return ctx.report()


object_model = build_object_model()
