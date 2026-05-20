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
    model = ArticulatedObject(name="bridge_backed_xy_translation_table")

    cast_iron = Material("dark_cast_iron", color=(0.09, 0.10, 0.11, 1.0))
    rail_steel = Material("ground_steel", color=(0.70, 0.72, 0.70, 1.0))
    carriage_blue = Material("machined_blue", color=(0.10, 0.22, 0.45, 1.0))
    saddle_steel = Material("satin_saddle", color=(0.50, 0.53, 0.55, 1.0))
    slot_black = Material("dark_t_slots", color=(0.02, 0.02, 0.025, 1.0))
    brass = Material("brass_scale", color=(0.80, 0.58, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.48, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_plate",
    )
    for y, rail_name in ((-0.145, "x_rail_0"), (0.145, "x_rail_1")):
        base.visual(
            Box((0.62, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0675)),
            material=rail_steel,
            name=rail_name,
        )

    # A rear bridge makes the table read as a rigid, backed machine fixture.
    for x, post_name in ((-0.315, "bridge_post_0"), (0.315, "bridge_post_1")):
        base.visual(
            Box((0.055, 0.050, 0.255)),
            origin=Origin(xyz=(x, -0.215, 0.1775)),
            material=cast_iron,
            name=post_name,
        )
    base.visual(
        Box((0.70, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.215, 0.3275)),
        material=cast_iron,
        name="bridge_beam",
    )
    base.visual(
        Box((0.58, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.234, 0.060)),
        material=brass,
        name="front_scale",
    )

    lower = model.part("lower_carriage")
    for y, bearing_name in ((-0.145, "x_bearing_0"), (0.145, "x_bearing_1")):
        lower.visual(
            Box((0.34, 0.045, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.009)),
            material=slot_black,
            name=bearing_name,
        )
    lower.visual(
        Box((0.44, 0.36, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=carriage_blue,
        name="lower_plate",
    )
    for x, rail_name in ((-0.125, "y_rail_0"), (0.125, "y_rail_1")):
        lower.visual(
            Box((0.035, 0.300, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0805)),
            material=rail_steel,
            name=rail_name,
        )
    lower.visual(
        Box((0.40, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.174, 0.053)),
        material=brass,
        name="side_scale",
    )

    upper = model.part("upper_saddle")
    for x, bearing_name in ((-0.125, "y_bearing_0"), (0.125, "y_bearing_1")):
        upper.visual(
            Box((0.045, 0.220, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.009)),
            material=slot_black,
            name=bearing_name,
        )
    upper.visual(
        Box((0.22, 0.30, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=saddle_steel,
        name="saddle_plate",
    )
    for x, slot_name in ((-0.060, "t_slot_0"), (0.0, "t_slot_1"), (0.060, "t_slot_2")):
        upper.visual(
            Box((0.010, 0.260, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.0645)),
            material=slot_black,
            name=slot_name,
        )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "lower_to_saddle",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.08, upper=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_saddle")
    x_slide = object_model.get_articulation("base_to_lower")
    y_slide = object_model.get_articulation("lower_to_saddle")

    ctx.check(
        "lower carriage is x prismatic",
        x_slide.articulation_type == ArticulationType.PRISMATIC and tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={x_slide.articulation_type}, axis={x_slide.axis}",
    )
    ctx.check(
        "upper saddle is y prismatic",
        y_slide.articulation_type == ArticulationType.PRISMATIC and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"type={y_slide.articulation_type}, axis={y_slide.axis}",
    )

    ctx.expect_contact(
        lower,
        base,
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        name="lower carriage sits on base rail",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        name="upper saddle sits on lower rail",
    )

    lower_rest = ctx.part_world_position(lower)
    upper_rest = ctx.part_world_position(upper)
    with ctx.pose({x_slide: 0.12, y_slide: 0.08}):
        ctx.expect_overlap(
            lower,
            base,
            axes="x",
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            min_overlap=0.20,
            name="x slide retains rail engagement",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.15,
            name="y slide retains rail engagement",
        )
        lower_shifted = ctx.part_world_position(lower)
        upper_shifted = ctx.part_world_position(upper)

    ctx.check(
        "lower carriage translates along x",
        lower_rest is not None
        and lower_shifted is not None
        and lower_shifted[0] > lower_rest[0] + 0.10
        and abs(lower_shifted[1] - lower_rest[1]) < 0.001,
        details=f"rest={lower_rest}, shifted={lower_shifted}",
    )
    ctx.check(
        "upper saddle translates along y",
        upper_rest is not None
        and upper_shifted is not None
        and upper_shifted[1] > upper_rest[1] + 0.07,
        details=f"rest={upper_rest}, shifted={upper_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
