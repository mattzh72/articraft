from __future__ import annotations

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
    model = ArticulatedObject(name="compact_xy_axis")

    painted_casting = Material("painted_casting", rgba=(0.18, 0.22, 0.26, 1.0))
    linear_rail_steel = Material("linear_rail_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    carriage_blue = Material("carriage_blue", rgba=(0.05, 0.23, 0.55, 1.0))
    cross_slide_gray = Material("cross_slide_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    tooling_orange = Material("tooling_orange", rgba=(0.95, 0.48, 0.13, 1.0))
    fastener_black = Material("fastener_black", rgba=(0.02, 0.02, 0.025, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        Box((0.92, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_casting,
        name="ground_plate",
    )
    for rail_name, y in (("x_rail_0", -0.075), ("x_rail_1", 0.075)):
        lower_guide.visual(
            Box((0.78, 0.036, 0.046)),
            origin=Origin(xyz=(0.0, y, 0.062)),
            material=linear_rail_steel,
            name=rail_name,
        )
    for idx, x in enumerate((-0.430, 0.430)):
        lower_guide.visual(
            Box((0.040, 0.22, 0.092)),
            origin=Origin(xyz=(x, 0.0, 0.085)),
            material=painted_casting,
            name=f"end_stop_{idx}",
        )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((0.24, 0.23, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=carriage_blue,
        name="carriage_body",
    )
    for rail_name, x in (("y_rail_0", -0.055), ("y_rail_1", 0.055)):
        first_carriage.visual(
            Box((0.036, 0.31, 0.021)),
            origin=Origin(xyz=(x, 0.0, 0.0845)),
            material=linear_rail_steel,
            name=rail_name,
        )
    first_carriage.visual(
        Box((0.19, 0.16, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=fastener_black,
        name="top_wear_plate",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((0.18, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cross_slide_gray,
        name="cross_beam",
    )
    cross_slide.visual(
        Box((0.20, 0.20, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=tooling_orange,
        name="tooling_deck",
    )
    for idx, (x, y) in enumerate(
        ((-0.070, -0.070), (-0.070, 0.070), (0.070, -0.070), (0.070, 0.070))
    ):
        cross_slide.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x, y, 0.0745)),
            material=fastener_black,
            name=f"deck_bolt_{idx}",
        )

    model.articulation(
        "lower_to_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=first_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.24, upper=0.24),
    )
    model.articulation(
        "carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-0.14, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    first_carriage = object_model.get_part("first_carriage")
    cross_slide = object_model.get_part("cross_slide")
    x_joint = object_model.get_articulation("lower_to_carriage")
    y_joint = object_model.get_articulation("carriage_to_cross_slide")

    ctx.check(
        "two orthogonal prismatic axes",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and y_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"x_axis={x_joint.axis}, y_axis={y_joint.axis}",
    )

    ctx.expect_gap(
        first_carriage,
        lower_guide,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="carriage_body",
        negative_elem="x_rail_0",
        name="first carriage sits on lower rail",
    )
    ctx.expect_gap(
        cross_slide,
        first_carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="cross_beam",
        negative_elem="y_rail_0",
        name="cross slide sits on upper rail",
    )
    ctx.expect_overlap(
        first_carriage,
        lower_guide,
        axes="x",
        min_overlap=0.22,
        elem_a="carriage_body",
        elem_b="x_rail_0",
        name="x stage retained on lower guide at center",
    )
    ctx.expect_overlap(
        cross_slide,
        first_carriage,
        axes="y",
        min_overlap=0.25,
        elem_a="cross_beam",
        elem_b="y_rail_0",
        name="y stage retained on carriage at center",
    )

    rest_carriage = ctx.part_world_position(first_carriage)
    rest_slide = ctx.part_world_position(cross_slide)
    with ctx.pose({x_joint: 0.24, y_joint: 0.14}):
        extended_carriage = ctx.part_world_position(first_carriage)
        extended_slide = ctx.part_world_position(cross_slide)
        ctx.expect_overlap(
            first_carriage,
            lower_guide,
            axes="x",
            min_overlap=0.20,
            elem_a="carriage_body",
            elem_b="x_rail_0",
            name="x stage remains supported at travel limit",
        )
        ctx.expect_overlap(
            cross_slide,
            first_carriage,
            axes="y",
            min_overlap=0.20,
            elem_a="cross_beam",
            elem_b="y_rail_0",
            name="y stage remains supported at travel limit",
        )

    ctx.check(
        "upper pose moves carriage in x",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.20,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "upper pose moves slide in y",
        rest_slide is not None
        and extended_slide is not None
        and extended_slide[1] > rest_slide[1] + 0.10,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )

    return ctx.report()


object_model = build_object_model()
