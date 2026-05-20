from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_xz_stage")

    dark_anodized = model.material("dark_anodized", rgba=(0.06, 0.065, 0.07, 1.0))
    black_rail = model.material("black_linear_rail", rgba=(0.015, 0.017, 0.02, 1.0))
    blue_carriage = model.material("blue_carriage", rgba=(0.05, 0.17, 0.38, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    caution = model.material("etched_scale", rgba=(0.92, 0.78, 0.22, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        Box((0.52, 0.23, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_anodized,
        name="base_plate",
    )
    guide_body.visual(
        Box((0.44, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.075, 0.060)),
        material=black_rail,
        name="x_rail_0",
    )
    guide_body.visual(
        Box((0.44, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.075, 0.060)),
        material=black_rail,
        name="x_rail_1",
    )
    guide_body.visual(
        Box((0.022, 0.190, 0.055)),
        origin=Origin(xyz=(-0.246, 0.0, 0.0725)),
        material=dark_anodized,
        name="end_stop_0",
    )
    guide_body.visual(
        Box((0.022, 0.190, 0.055)),
        origin=Origin(xyz=(0.246, 0.0, 0.0725)),
        material=dark_anodized,
        name="end_stop_1",
    )
    guide_body.visual(
        Box((0.34, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.116, 0.048)),
        material=caution,
        name="x_scale_strip",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((0.24, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=blue_carriage,
        name="carriage_plate",
    )
    lower_carriage.visual(
        Box((0.095, 0.018, 0.210)),
        origin=Origin(xyz=(0.0, -0.060, 0.140)),
        material=black_rail,
        name="z_way_0",
    )
    lower_carriage.visual(
        Box((0.095, 0.018, 0.210)),
        origin=Origin(xyz=(0.0, 0.060, 0.140)),
        material=black_rail,
        name="z_way_1",
    )
    lower_carriage.visual(
        Box((0.095, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.060, 0.254)),
        material=blue_carriage,
        name="z_cap_0",
    )
    lower_carriage.visual(
        Box((0.095, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.060, 0.254)),
        material=blue_carriage,
        name="z_cap_1",
    )
    lower_carriage.visual(
        Box((0.105, 0.014, 0.160)),
        origin=Origin(xyz=(0.056, 0.076, 0.128)),
        material=caution,
        name="z_scale_strip",
    )

    upper_guide = model.part("upper_guide")
    upper_guide.visual(
        Box((0.065, 0.065, 0.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=brushed_aluminum,
        name="guide_mast",
    )
    upper_guide.visual(
        Box((0.110, 0.017, 0.120)),
        origin=Origin(xyz=(0.0, -0.041, 0.170)),
        material=brushed_aluminum,
        name="front_tool_plate",
    )
    upper_guide.visual(
        Box((0.084, 0.084, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.269)),
        material=dark_anodized,
        name="top_cap",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.08, upper=0.08),
    )
    model.articulation(
        "carriage_to_guide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_guide,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.14, lower=0.0, upper=0.14),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_guide = object_model.get_part("upper_guide")
    x_slide = object_model.get_articulation("body_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_guide")

    ctx.check(
        "orthogonal prismatic joints",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and x_slide.axis == (1.0, 0.0, 0.0)
        and z_slide.axis == (0.0, 0.0, 1.0),
        details=f"x={x_slide.articulation_type}/{x_slide.axis}, z={z_slide.articulation_type}/{z_slide.axis}",
    )

    ctx.expect_gap(
        lower_carriage,
        guide_body,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower carriage rests on x rails",
    )
    ctx.expect_overlap(
        lower_carriage,
        guide_body,
        axes="x",
        elem_a="carriage_plate",
        elem_b="x_rail_0",
        min_overlap=0.20,
        name="lower carriage is broad over guide body",
    )
    ctx.expect_gap(
        upper_guide,
        lower_carriage,
        axis="z",
        positive_elem="guide_mast",
        negative_elem="carriage_plate",
        max_gap=0.001,
        max_penetration=1e-6,
        name="vertical guide seats on carriage",
    )
    ctx.expect_gap(
        upper_guide,
        lower_carriage,
        axis="y",
        positive_elem="guide_mast",
        negative_elem="z_way_0",
        min_gap=0.006,
        max_gap=0.025,
        name="mast clears lower guide way",
    )

    rest_lower = ctx.part_world_position(lower_carriage)
    rest_upper = ctx.part_world_position(upper_guide)
    with ctx.pose({x_slide: 0.08}):
        extended_lower = ctx.part_world_position(lower_carriage)
    with ctx.pose({z_slide: 0.14}):
        raised_upper = ctx.part_world_position(upper_guide)
        ctx.expect_overlap(
            upper_guide,
            lower_carriage,
            axes="z",
            elem_a="guide_mast",
            elem_b="z_way_1",
            min_overlap=0.060,
            name="raised guide remains engaged in z ways",
        )

    ctx.check(
        "lower carriage travels in x",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[0] > rest_lower[0] + 0.075
        and abs(extended_lower[2] - rest_lower[2]) < 0.001,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )
    ctx.check(
        "upper guide travels in z",
        rest_upper is not None
        and raised_upper is not None
        and raised_upper[2] > rest_upper[2] + 0.13
        and abs(raised_upper[0] - rest_upper[0]) < 0.001,
        details=f"rest={rest_upper}, raised={raised_upper}",
    )

    return ctx.report()


object_model = build_object_model()
