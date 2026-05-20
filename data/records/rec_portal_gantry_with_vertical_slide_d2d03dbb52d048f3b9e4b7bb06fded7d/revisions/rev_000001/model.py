from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="portal_gantry")

    aluminum = Material("anodized_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    dark = Material("dark_composite", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_black = Material("blackened_steel", rgba=(0.015, 0.015, 0.018, 1.0))
    safety_orange = Material("safety_orange", rgba=(1.0, 0.42, 0.08, 1.0))

    frame = model.part("frame")
    # Two rigid side uprights on broad feet.
    for suffix, x in (("0", -1.05), ("1", 1.05)):
        frame.visual(
            Box((0.42, 0.44, 0.06)),
            origin=Origin(xyz=(x, 0.0, 0.03)),
            material=dark,
            name=f"foot_{suffix}",
        )
        frame.visual(
            Box((0.13, 0.13, 1.39)),
            origin=Origin(xyz=(x, 0.0, 0.745)),
            material=aluminum,
            name=f"upright_{suffix}",
        )
        frame.visual(
            Box((0.24, 0.20, 0.06)),
            origin=Origin(xyz=(x, -0.015, 1.405)),
            material=aluminum,
            name=f"top_corner_{suffix}",
        )

    frame.visual(
        Box((2.36, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=aluminum,
        name="top_beam",
    )
    frame.visual(
        Box((2.20, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.09, 0.21)),
        material=aluminum,
        name="rear_tie",
    )
    # Linear guide rails and a belt/screw line on the front of the beam.
    frame.visual(
        Box((1.88, 0.032, 0.025)),
        origin=Origin(xyz=(0.0, -0.094, 1.525)),
        material=rail_black,
        name="upper_linear_rail",
    )
    frame.visual(
        Box((1.88, 0.032, 0.025)),
        origin=Origin(xyz=(0.0, -0.094, 1.465)),
        material=rail_black,
        name="lower_linear_rail",
    )
    frame.visual(
        Cylinder(radius=0.014, length=1.86),
        origin=Origin(xyz=(0.0, -0.310, 1.495), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_black,
        name="drive_screw",
    )
    for suffix, x in (("0", -0.94), ("1", 0.94)):
        frame.visual(
            Box((0.08, 0.34, 0.095)),
            origin=Origin(xyz=(x, -0.165, 1.495)),
            material=dark,
            name=f"screw_support_{suffix}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.36, 0.055, 0.30)),
        origin=Origin(xyz=(0.0, -0.033, 0.0)),
        material=dark,
        name="carriage_plate",
    )
    for bearing_name, x, z in (
        ("upper_bearing_0", -0.115, 0.030),
        ("upper_bearing_1", 0.115, 0.030),
        ("lower_bearing_0", -0.115, -0.030),
        ("lower_bearing_1", 0.115, -0.030),
    ):
        carriage.visual(
            Box((0.095, 0.070, 0.044)),
            origin=Origin(xyz=(x, 0.010, z)),
            material=rail_black,
            name=bearing_name,
        )
    carriage.visual(
        Box((0.27, 0.066, 0.78)),
        origin=Origin(xyz=(0.0, -0.070, -0.320)),
        material=dark,
        name="slide_backplate",
    )
    carriage.visual(
        Box((0.030, 0.026, 0.72)),
        origin=Origin(xyz=(-0.075, -0.116, -0.320)),
        material=rail_black,
        name="vertical_rail_0",
    )
    carriage.visual(
        Box((0.030, 0.026, 0.72)),
        origin=Origin(xyz=(0.075, -0.116, -0.320)),
        material=rail_black,
        name="vertical_rail_1",
    )
    carriage.visual(
        Box((0.28, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.125, -0.700)),
        material=safety_orange,
        name="lower_guard",
    )

    vertical_slide = model.part("vertical_slide")
    vertical_slide.visual(
        Box((0.14, 0.060, 0.86)),
        origin=Origin(xyz=(0.0, -0.037, -0.230)),
        material=aluminum,
        name="ram",
    )
    for shoe_name, x, z in (
        ("upper_shoe_0", -0.075, 0.110),
        ("upper_shoe_1", 0.075, 0.110),
        ("lower_shoe_0", -0.075, -0.060),
        ("lower_shoe_1", 0.075, -0.060),
    ):
        vertical_slide.visual(
            Box((0.086, 0.045, 0.072)),
            origin=Origin(xyz=(x, -0.0015, z)),
            material=rail_black,
            name=shoe_name,
        )
    vertical_slide.visual(
        Box((0.24, 0.042, 0.13)),
        origin=Origin(xyz=(0.0, -0.078, -0.690)),
        material=safety_orange,
        name="tool_plate",
    )
    for suffix, x, z in (
        ("0", -0.075, -0.660),
        ("1", 0.075, -0.660),
        ("2", -0.075, -0.720),
        ("3", 0.075, -0.720),
    ):
        vertical_slide.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, -0.101, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rail_black,
            name=f"face_bolt_{suffix}",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.155, 1.495)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.55, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=vertical_slide,
        origin=Origin(xyz=(0.0, -0.150, -0.295)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    vertical_slide = object_model.get_part("vertical_slide")
    x_axis = object_model.get_articulation("frame_to_carriage")
    z_axis = object_model.get_articulation("carriage_to_slide")

    ctx.expect_contact(
        carriage,
        frame,
        elem_a="upper_bearing_0",
        elem_b="upper_linear_rail",
        name="carriage bearing rides on upper rail",
    )
    ctx.expect_contact(
        vertical_slide,
        carriage,
        elem_a="upper_shoe_0",
        elem_b="vertical_rail_0",
        name="vertical shoe rides on guide rail",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="carriage_plate",
        outer_elem="top_beam",
        margin=0.0,
        name="centered carriage starts under beam span",
    )

    carriage_rest = ctx.part_world_position(carriage)
    slide_rest = ctx.part_world_position(vertical_slide)
    with ctx.pose({x_axis: 0.50}):
        carriage_shifted = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="upper_bearing_1",
            outer_elem="upper_linear_rail",
            margin=0.0,
            name="carriage keeps bearing on rail at travel",
        )
    with ctx.pose({z_axis: 0.30}):
        slide_lowered = ctx.part_world_position(vertical_slide)
        ctx.expect_overlap(
            vertical_slide,
            carriage,
            axes="z",
            elem_a="upper_shoe_0",
            elem_b="vertical_rail_0",
            min_overlap=0.04,
            name="lowered slide remains guided",
        )

    ctx.check(
        "carriage moves horizontally along gantry",
        carriage_rest is not None
        and carriage_shifted is not None
        and carriage_shifted[0] > carriage_rest[0] + 0.45,
        details=f"rest={carriage_rest}, shifted={carriage_shifted}",
    )
    ctx.check(
        "vertical slide moves downward",
        slide_rest is not None
        and slide_lowered is not None
        and slide_lowered[2] < slide_rest[2] - 0.25,
        details=f"rest={slide_rest}, lowered={slide_lowered}",
    )

    return ctx.report()


object_model = build_object_model()
