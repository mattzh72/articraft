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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_z_slide")

    frame_paint = model.material("powder_coated_frame", color=(0.16, 0.20, 0.24, 1.0))
    rail_steel = model.material("bright_linear_rail", color=(0.72, 0.74, 0.72, 1.0))
    bolt_dark = model.material("dark_fasteners", color=(0.03, 0.035, 0.04, 1.0))
    carriage_orange = model.material("safety_orange_carriage", color=(0.95, 0.42, 0.08, 1.0))
    bearing_black = model.material("black_bearing_rubber", color=(0.01, 0.01, 0.012, 1.0))
    slide_blue = model.material("blue_z_slide", color=(0.05, 0.17, 0.34, 1.0))
    tool_plate = model.material("machined_tool_plate", color=(0.62, 0.64, 0.62, 1.0))

    frame = model.part("frame")
    # Fixed portal: two grounded uprights tied together by a top beam.
    frame.visual(
        Box((0.30, 0.50, 0.055)),
        origin=Origin(xyz=(-1.05, 0.0, 0.0275)),
        material=frame_paint,
        name="foot_0",
    )
    frame.visual(
        Box((0.30, 0.50, 0.055)),
        origin=Origin(xyz=(1.05, 0.0, 0.0275)),
        material=frame_paint,
        name="foot_1",
    )
    frame.visual(
        Box((0.13, 0.18, 1.43)),
        origin=Origin(xyz=(-1.05, 0.0, 0.745)),
        material=frame_paint,
        name="upright_0",
    )
    frame.visual(
        Box((0.13, 0.18, 1.43)),
        origin=Origin(xyz=(1.05, 0.0, 0.745)),
        material=frame_paint,
        name="upright_1",
    )
    frame.visual(
        Box((2.32, 0.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.485)),
        material=frame_paint,
        name="top_beam",
    )
    # Linear rails bolted to the underside of the beam.  They provide a visible
    # support path for the horizontal carriage without making the carriage
    # interpenetrate the beam.
    frame.visual(
        Box((1.86, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, -0.074, 1.385)),
        material=rail_steel,
        name="front_linear_rail",
    )
    frame.visual(
        Box((1.86, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.074, 1.385)),
        material=rail_steel,
        name="rear_linear_rail",
    )
    # Knee plates and anchor bolts make the grounded frame read as a fabricated
    # machine structure rather than a set of abstract boxes.
    for sx in (-1.05, 1.05):
        frame.visual(
            Box((0.18, 0.030, 0.30)),
            origin=Origin(xyz=(sx, -0.105, 1.275), rpy=(0.0, 0.35 if sx < 0 else -0.35, 0.0)),
            material=frame_paint,
            name=f"knee_front_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Box((0.18, 0.030, 0.30)),
            origin=Origin(xyz=(sx, 0.105, 1.275), rpy=(0.0, 0.35 if sx < 0 else -0.35, 0.0)),
            material=frame_paint,
            name=f"knee_rear_{0 if sx < 0 else 1}",
        )
        for by in (-0.17, 0.17):
            for bx in (-0.07, 0.07):
                frame.visual(
                    Cylinder(radius=0.018, length=0.014),
                    origin=Origin(xyz=(sx + bx, by, 0.058)),
                    material=bolt_dark,
                    name=f"anchor_{0 if sx < 0 else 1}_{0 if by < 0 else 1}_{0 if bx < 0 else 1}",
                )

    carriage = model.part("beam_carriage")
    carriage.visual(
        Box((0.42, 0.23, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=carriage_orange,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.34, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=carriage_orange,
        name="lower_saddle",
    )
    # Four flanged rollers touch the two fixed rails from below.
    for roller_name, hub_name, x, y in (
        ("roller_0_0", "roller_hub_0_0", -0.13, -0.074),
        ("roller_0_1", "roller_hub_0_1", -0.13, 0.074),
        ("roller_1_0", "roller_hub_1_0", 0.13, -0.074),
        ("roller_1_1", "roller_hub_1_1", 0.13, 0.074),
    ):
            carriage.visual(
                Cylinder(radius=0.025, length=0.036),
                origin=Origin(xyz=(x, y, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bearing_black,
                name=roller_name,
            )
            carriage.visual(
                Cylinder(radius=0.014, length=0.042),
                origin=Origin(xyz=(x, y, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rail_steel,
                name=hub_name,
            )
    # Downward guide cheeks form the fixed half of the Z-axis slide.
    carriage.visual(
        Box((0.030, 0.14, 0.42)),
        origin=Origin(xyz=(-0.095, 0.0, -0.260)),
        material=carriage_orange,
        name="guide_0",
    )
    carriage.visual(
        Box((0.030, 0.14, 0.42)),
        origin=Origin(xyz=(0.095, 0.0, -0.260)),
        material=carriage_orange,
        name="guide_1",
    )
    carriage.visual(
        Box((0.22, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, 0.0875, -0.270)),
        material=carriage_orange,
        name="guide_backstrap",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.110, 0.075, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=slide_blue,
        name="slide_plate",
    )
    # Bearing shoes on both sides run against the orange guide cheeks.  The
    # shoes touch the cheeks in X while retaining clearance in Y.
    for shoe_name, x, z in (
        ("bearing_shoe_0_0", -0.0675, -0.125),
        ("bearing_shoe_0_1", -0.0675, -0.315),
        ("bearing_shoe_1_0", 0.0675, -0.125),
        ("bearing_shoe_1_1", 0.0675, -0.315),
    ):
            z_slide.visual(
                Box((0.025, 0.090, 0.070)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=rail_steel,
                name=shoe_name,
            )
    z_slide.visual(
        Box((0.20, 0.13, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.485)),
        material=tool_plate,
        name="tool_mount",
    )
    z_slide.visual(
        Cylinder(radius=0.018, length=0.30),
        origin=Origin(xyz=(0.0, -0.0555, -0.235)),
        material=rail_steel,
        name="lead_screw",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.65, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("beam_carriage")
    z_slide = object_model.get_part("z_slide")
    x_joint = object_model.get_articulation("frame_to_carriage")
    z_joint = object_model.get_articulation("carriage_to_z_slide")

    # The carriage is visibly carried by rollers touching the underside rails.
    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="front_linear_rail",
        negative_elem="roller_0_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="front roller touches front rail",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="rear_linear_rail",
        negative_elem="roller_0_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear roller touches rear rail",
    )
    ctx.expect_gap(
        carriage,
        z_slide,
        axis="x",
        positive_elem="guide_1",
        negative_elem="bearing_shoe_1_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="right slide shoe bears on guide",
    )
    ctx.expect_overlap(
        z_slide,
        carriage,
        axes="yz",
        elem_a="bearing_shoe_1_0",
        elem_b="guide_1",
        min_overlap=0.030,
        name="upper shoe remains engaged in guide",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({x_joint: 0.50}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="y",
            elem_a="roller_0_0",
            elem_b="front_linear_rail",
            min_overlap=0.015,
            name="carriage rollers stay aligned to rail",
        )
    ctx.check(
        "carriage translates along gantry",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.45,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_slide = ctx.part_world_position(z_slide)
    with ctx.pose({z_joint: 0.20}):
        extended_slide = ctx.part_world_position(z_slide)
        ctx.expect_overlap(
            z_slide,
            carriage,
            axes="z",
            elem_a="bearing_shoe_1_0",
            elem_b="guide_1",
            min_overlap=0.025,
            name="extended z shoe retained by guide",
        )
    ctx.check(
        "z slide moves downward",
        rest_slide is not None
        and extended_slide is not None
        and extended_slide[2] < rest_slide[2] - 0.18,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )

    return ctx.report()


object_model = build_object_model()
