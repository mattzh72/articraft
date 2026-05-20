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
    model = ArticulatedObject(name="column_carried_yz_slide")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_casting = model.material("dark_casting", rgba=(0.06, 0.065, 0.07, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    moving_blue = model.material("blue_carriage_casting", rgba=(0.08, 0.22, 0.42, 1.0))
    slide_steel = model.material("brushed_slide_steel", rgba=(0.66, 0.68, 0.66, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.46, 0.36, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_casting,
        name="floor_base",
    )
    column.visual(
        Box((0.11, 0.11, 1.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=painted_steel,
        name="upright_column",
    )
    column.visual(
        Box((0.19, 0.17, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=dark_casting,
        name="top_cap",
    )
    for y, suffix in ((-0.038, "lower"), (0.038, "upper")):
        column.visual(
            Cylinder(radius=0.012, length=0.96),
            origin=Origin(xyz=(0.061, y, 0.56)),
            material=rail_steel,
            name=f"vertical_rail_{suffix}",
        )
        for z, block in ((0.095, "bottom"), (1.025, "top")):
            column.visual(
                Box((0.036, 0.034, 0.030)),
                origin=Origin(xyz=(0.055, y, z)),
                material=dark_casting,
                name=f"{block}_rail_clamp_{suffix}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.030, 0.180, 0.220)),
        origin=Origin(xyz=(0.113, 0.0, 0.0)),
        material=moving_blue,
        name="carriage_face",
    )
    for y, suffix in ((-0.038, "lower"), (0.038, "upper")):
        carriage.visual(
            Box((0.018, 0.050, 0.185)),
            origin=Origin(xyz=(0.091, y, 0.0)),
            material=moving_blue,
            name=f"front_bearing_bridge_{suffix}",
        )
        carriage.visual(
            Box((0.040, 0.010, 0.185)),
            origin=Origin(xyz=(0.070, y - 0.028, 0.0)),
            material=moving_blue,
            name=f"outer_bearing_cheek_{suffix}",
        )
        carriage.visual(
            Box((0.040, 0.010, 0.185)),
            origin=Origin(xyz=(0.070, y + 0.028, 0.0)),
            material=moving_blue,
            name=f"inner_bearing_cheek_{suffix}",
        )
    carriage.visual(
        Box((0.064, 0.220, 0.020)),
        origin=Origin(xyz=(0.158, 0.000, 0.052)),
        material=moving_blue,
        name="side_slide_top_way",
    )
    carriage.visual(
        Box((0.064, 0.220, 0.020)),
        origin=Origin(xyz=(0.158, 0.000, -0.052)),
        material=moving_blue,
        name="side_slide_bottom_way",
    )
    carriage.visual(
        Box((0.022, 0.220, 0.124)),
        origin=Origin(xyz=(0.131, 0.000, 0.0)),
        material=moving_blue,
        name="side_slide_back_web",
    )

    side_slide = model.part("side_slide")
    side_slide.visual(
        Box((0.036, 0.340, 0.046)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=slide_steel,
        name="slide_bar",
    )
    side_slide.visual(
        Box((0.105, 0.020, 0.125)),
        origin=Origin(xyz=(0.0, 0.230, 0.0)),
        material=dark_casting,
        name="end_plate",
    )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.20, lower=0.0, upper=0.44),
    )
    model.articulation(
        "carriage_to_side_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=side_slide,
        origin=Origin(xyz=(0.158, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    side_slide = object_model.get_part("side_slide")
    vertical_joint = object_model.get_articulation("column_to_carriage")
    side_joint = object_model.get_articulation("carriage_to_side_slide")

    ctx.check(
        "carriage joint is vertical",
        tuple(vertical_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={vertical_joint.axis}",
    )
    ctx.check(
        "side slide joint is orthogonal",
        tuple(side_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={side_joint.axis}",
    )
    ctx.expect_within(
        side_slide,
        carriage,
        axes="x",
        inner_elem="slide_bar",
        outer_elem="side_slide_top_way",
        margin=0.030,
        name="slide bar stays laterally centered in the ways",
    )
    ctx.expect_gap(
        carriage,
        side_slide,
        axis="z",
        min_gap=0.014,
        max_gap=0.026,
        positive_elem="side_slide_top_way",
        negative_elem="slide_bar",
        name="upper way clears the slide bar",
    )
    ctx.expect_gap(
        side_slide,
        carriage,
        axis="z",
        min_gap=0.014,
        max_gap=0.026,
        positive_elem="slide_bar",
        negative_elem="side_slide_bottom_way",
        name="lower way clears the slide bar",
    )
    ctx.expect_overlap(
        side_slide,
        carriage,
        axes="y",
        elem_a="slide_bar",
        elem_b="side_slide_top_way",
        min_overlap=0.12,
        name="side slide retains insertion at rest",
    )

    carriage_low = ctx.part_world_position(carriage)
    slide_retracted = ctx.part_world_position(side_slide)
    with ctx.pose({vertical_joint: 0.44, side_joint: 0.18}):
        carriage_high = ctx.part_world_position(carriage)
        slide_extended = ctx.part_world_position(side_slide)
        ctx.expect_overlap(
            side_slide,
            carriage,
            axes="y",
            elem_a="slide_bar",
            elem_b="side_slide_top_way",
            min_overlap=0.035,
            name="side slide remains captured when extended",
        )

    ctx.check(
        "carriage moves upward on the column",
        carriage_low is not None
        and carriage_high is not None
        and carriage_high[2] > carriage_low[2] + 0.40,
        details=f"low={carriage_low}, high={carriage_high}",
    )
    ctx.check(
        "side slide moves out along Y",
        slide_retracted is not None
        and slide_extended is not None
        and slide_extended[1] > slide_retracted[1] + 0.16,
        details=f"retracted={slide_retracted}, extended={slide_extended}",
    )

    return ctx.report()


object_model = build_object_model()
