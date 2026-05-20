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
    model = ArticulatedObject(name="rail_stack_xyz_stage")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    hardened_steel = Material("hardened_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    black_plastic = Material("black_end_caps", rgba=(0.015, 0.015, 0.018, 1.0))
    brass = Material("brass_bearing_wipers", rgba=(0.74, 0.55, 0.23, 1.0))

    # Grounded X stage: a long base with two exposed linear rails.
    x_base = model.part("x_base")
    x_base.visual(
        Box((0.82, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_anodized,
        name="base_plate",
    )
    for rail_y, rail_name in (
        (-0.105, "x_rail_negative"),
        (0.105, "x_rail_positive"),
    ):
        x_base.visual(
            Box((0.72, 0.024, 0.026)),
            origin=Origin(xyz=(0.0, rail_y, 0.053)),
            material=hardened_steel,
            name=rail_name,
        )
    for stop_x in (-0.385, 0.385):
        x_base.visual(
            Box((0.030, 0.270, 0.040)),
            origin=Origin(xyz=(stop_x, 0.0, 0.060)),
            material=black_plastic,
            name=f"x_end_stop_{'negative' if stop_x < 0 else 'positive'}",
        )
    x_base.visual(
        Box((0.68, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=brushed_aluminum,
        name="x_leadscrew_guard",
    )

    # X carriage: a crosswise saddle riding on the X rails and carrying the Y rails.
    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.220, 0.260, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="x_saddle_plate",
    )
    for shoe_y, shoe_name, wiper_name in (
        (-0.105, "x_bearing_shoe_negative", "x_wiper_strip_negative"),
        (0.105, "x_bearing_shoe_positive", "x_wiper_strip_positive"),
    ):
        x_carriage.visual(
            Box((0.110, 0.046, 0.026)),
            origin=Origin(xyz=(0.0, shoe_y, -0.0285)),
            material=hardened_steel,
            name=shoe_name,
        )
        x_carriage.visual(
            Box((0.092, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, shoe_y, -0.0075)),
            material=brass,
            name=wiper_name,
        )
    for rail_x, rail_name in (
        (-0.065, "y_rail_negative"),
        (0.065, "y_rail_positive"),
    ):
        x_carriage.visual(
            Box((0.022, 0.240, 0.018)),
            origin=Origin(xyz=(rail_x, 0.0, 0.0265)),
            material=hardened_steel,
            name=rail_name,
        )
    x_carriage.visual(
        Box((0.020, 0.220, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=black_plastic,
        name="y_leadscrew_cover",
    )

    # Y carriage: a compact moving table with an upright guide pair for the Z slide.
    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.150, 0.120, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_anodized,
        name="y_table",
    )
    for shoe_x, shoe_name in (
        (-0.065, "y_bearing_shoe_negative"),
        (0.065, "y_bearing_shoe_positive"),
    ):
        y_carriage.visual(
            Box((0.046, 0.070, 0.016)),
            origin=Origin(xyz=(shoe_x, 0.0, -0.023)),
            material=hardened_steel,
            name=shoe_name,
        )
    for post_x, post_name in (
        (-0.075, "z_guide_post_negative"),
        (0.075, "z_guide_post_positive"),
    ):
        y_carriage.visual(
            Box((0.025, 0.034, 0.180)),
            origin=Origin(xyz=(post_x, 0.0, 0.105)),
            material=brushed_aluminum,
            name=post_name,
        )
    y_carriage.visual(
        Box((0.170, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.048, 0.025)),
        material=black_plastic,
        name="z_motor_boss",
    )

    # Z slide: the vertical moving block and an intentionally plain top tooling plate.
    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.085, 0.054, 0.150)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed_aluminum,
        name="z_slider_block",
    )
    for pad_x, pad_name in (
        (-0.0525, "z_bearing_pad_negative"),
        (0.0525, "z_bearing_pad_positive"),
    ):
        z_slide.visual(
            Box((0.020, 0.054, 0.080)),
            origin=Origin(xyz=(pad_x, 0.0, 0.075)),
            material=hardened_steel,
            name=pad_name,
        )
    z_slide.visual(
        Box((0.050, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=brushed_aluminum,
        name="top_plate_neck",
    )
    z_slide.visual(
        Box((0.180, 0.140, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1925)),
        material=dark_anodized,
        name="top_plate",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0665)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=-0.065, upper=0.065),
    )
    model.articulation(
        "z_slide_joint",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.18, lower=0.0, upper=0.080),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    x_base = object_model.get_part("x_base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    z_joint = object_model.get_articulation("z_slide_joint")

    ctx.check(
        "three orthogonal prismatic stages",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_joint.articulation_type == ArticulationType.PRISMATIC
        and x_slide.axis == (1.0, 0.0, 0.0)
        and y_slide.axis == (0.0, 1.0, 0.0)
        and z_joint.axis == (0.0, 0.0, 1.0),
    )

    ctx.expect_gap(
        x_carriage,
        x_base,
        axis="z",
        positive_elem="x_bearing_shoe_positive",
        negative_elem="x_rail_positive",
        min_gap=0.0,
        max_gap=0.001,
        name="x bearing shoe sits on the grounded rail",
    )
    ctx.expect_gap(
        y_carriage,
        x_carriage,
        axis="z",
        positive_elem="y_bearing_shoe_positive",
        negative_elem="y_rail_positive",
        max_gap=0.001,
        max_penetration=0.00001,
        name="y bearing shoe sits on the cross rail",
    )
    ctx.expect_gap(
        z_slide,
        y_carriage,
        axis="z",
        positive_elem="top_plate",
        negative_elem="z_guide_post_positive",
        min_gap=0.004,
        max_gap=0.007,
        name="plain top plate clears the fixed z guide posts",
    )
    ctx.expect_overlap(
        z_slide,
        y_carriage,
        axes="z",
        elem_a="z_bearing_pad_positive",
        elem_b="z_guide_post_positive",
        min_overlap=0.060,
        name="z pad remains engaged in the guide at the lowered pose",
    )

    rest_top = ctx.part_element_world_aabb(z_slide, elem="top_plate")
    with ctx.pose({x_slide: 0.18, y_slide: 0.055, z_joint: 0.070}):
        moved_top = ctx.part_element_world_aabb(z_slide, elem="top_plate")
        ctx.expect_within(
            x_carriage,
            x_base,
            axes="y",
            inner_elem="x_saddle_plate",
            outer_elem="base_plate",
            margin=0.002,
            name="x saddle stays over the base width at travel",
        )
        ctx.expect_overlap(
            x_carriage,
            x_base,
            axes="x",
            elem_a="x_bearing_shoe_positive",
            elem_b="x_rail_positive",
            min_overlap=0.08,
            name="x bearing remains captured on the rail at travel",
        )
        ctx.expect_overlap(
            y_carriage,
            x_carriage,
            axes="y",
            elem_a="y_bearing_shoe_positive",
            elem_b="y_rail_positive",
            min_overlap=0.050,
            name="y bearing remains captured on the rail at travel",
        )
        ctx.expect_overlap(
            z_slide,
            y_carriage,
            axes="z",
            elem_a="z_bearing_pad_positive",
            elem_b="z_guide_post_positive",
            min_overlap=0.045,
            name="z pad remains engaged in the guide at raised travel",
        )
    ctx.check(
        "top plate moves in all three commanded directions",
        rest_top is not None
        and moved_top is not None
        and moved_top[0][0] > rest_top[0][0] + 0.15
        and moved_top[0][1] > rest_top[0][1] + 0.04
        and moved_top[0][2] > rest_top[0][2] + 0.05,
        details=f"rest_top={rest_top}, moved_top={moved_top}",
    )

    return ctx.report()


object_model = build_object_model()
