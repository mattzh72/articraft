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
    model = ArticulatedObject(name="split_level_xy_stage")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("black_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = model.material("ground_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    blue = model.material("blue_anodized", rgba=(0.12, 0.28, 0.62, 1.0))

    lower_slide = model.part("lower_slide")
    lower_slide.visual(
        Box((1.10, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="ground_plate",
    )
    for y, name in [(-0.095, "lower_rail_0"), (0.095, "lower_rail_1")]:
        lower_slide.visual(
            Box((0.96, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0725)),
            material=steel,
            name=name,
        )
    for x, name in [(-0.515, "lower_end_stop_0"), (0.515, "lower_end_stop_1")]:
        lower_slide.visual(
            Box((0.030, 0.235, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.090)),
            material=dark,
            name=name,
        )
    for x, name in [(-0.30, "left_mount_bar"), (0.30, "right_mount_bar")]:
        lower_slide.visual(
            Box((0.070, 0.300, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.061)),
            material=aluminum,
            name=name,
        )

    first_carriage = model.part("first_carriage")
    for y, name in [(-0.095, "lower_bearing_0"), (0.095, "lower_bearing_1")]:
        first_carriage.visual(
            Box((0.430, 0.050, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=blue,
            name=name,
        )
    first_carriage.visual(
        Box((0.620, 0.220, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0485)),
        material=aluminum,
        name="long_carriage",
    )
    first_carriage.visual(
        Box((0.345, 0.285, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark,
        name="upper_rail_deck",
    )
    for x, name in [(-0.085, "upper_rail_0"), (0.085, "upper_rail_1")]:
        first_carriage.visual(
            Box((0.034, 0.420, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.112)),
            material=steel,
            name=name,
        )
    for y, name in [(-0.235, "upper_end_stop_0"), (0.235, "upper_end_stop_1")]:
        first_carriage.visual(
            Box((0.245, 0.050, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.116)),
            material=dark,
            name=name,
        )

    upper_carriage = model.part("upper_carriage")
    for x, name in [(-0.085, "upper_bearing_0"), (0.085, "upper_bearing_1")]:
        upper_carriage.visual(
            Box((0.050, 0.235, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=blue,
            name=name,
        )
    upper_carriage.visual(
        Box((0.245, 0.245, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=aluminum,
        name="saddle_block",
    )
    upper_carriage.visual(
        Box((0.315, 0.275, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0735)),
        material=aluminum,
        name="plain_top_plate",
    )

    model.articulation(
        "lower_x",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=first_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "upper_y",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=upper_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.080, upper=0.080),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_slide = object_model.get_part("lower_slide")
    first_carriage = object_model.get_part("first_carriage")
    upper_carriage = object_model.get_part("upper_carriage")
    lower_x = object_model.get_articulation("lower_x")
    upper_y = object_model.get_articulation("upper_y")

    ctx.check(
        "lower and upper slides are orthogonal",
        abs(sum(a * b for a, b in zip(lower_x.axis, upper_y.axis))) < 1.0e-6,
        details=f"lower_axis={lower_x.axis}, upper_axis={upper_y.axis}",
    )
    ctx.expect_contact(
        first_carriage,
        lower_slide,
        elem_a="lower_bearing_0",
        elem_b="lower_rail_0",
        name="first carriage rests on lower rail 0",
    )
    ctx.expect_contact(
        first_carriage,
        lower_slide,
        elem_a="lower_bearing_1",
        elem_b="lower_rail_1",
        name="first carriage rests on lower rail 1",
    )
    ctx.expect_contact(
        upper_carriage,
        first_carriage,
        elem_a="upper_bearing_0",
        elem_b="upper_rail_0",
        name="upper carriage rests on cross rail 0",
    )
    ctx.expect_contact(
        upper_carriage,
        first_carriage,
        elem_a="upper_bearing_1",
        elem_b="upper_rail_1",
        name="upper carriage rests on cross rail 1",
    )

    rest_first = ctx.part_world_position(first_carriage)
    with ctx.pose({lower_x: 0.18}):
        ctx.expect_overlap(
            first_carriage,
            lower_slide,
            axes="xy",
            elem_a="lower_bearing_0",
            elem_b="lower_rail_0",
            min_overlap=0.030,
            name="lower X slide remains captured at positive travel",
        )
        shifted_first = ctx.part_world_position(first_carriage)
    ctx.check(
        "lower stage translates along X",
        rest_first is not None and shifted_first is not None and shifted_first[0] > rest_first[0] + 0.15,
        details=f"rest={rest_first}, shifted={shifted_first}",
    )

    rest_upper = ctx.part_world_position(upper_carriage)
    with ctx.pose({upper_y: 0.080}):
        ctx.expect_overlap(
            upper_carriage,
            first_carriage,
            axes="xy",
            elem_a="upper_bearing_0",
            elem_b="upper_rail_0",
            min_overlap=0.030,
            name="upper Y slide remains captured at positive travel",
        )
        shifted_upper = ctx.part_world_position(upper_carriage)
    ctx.check(
        "upper stage translates along Y",
        rest_upper is not None and shifted_upper is not None and shifted_upper[1] > rest_upper[1] + 0.060,
        details=f"rest={rest_upper}, shifted={shifted_upper}",
    )

    return ctx.report()


object_model = build_object_model()
