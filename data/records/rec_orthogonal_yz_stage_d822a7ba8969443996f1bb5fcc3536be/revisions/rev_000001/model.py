from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="backplane_yz_stage")

    steel = model.material("dark_powder_coated_steel", color=(0.04, 0.045, 0.05, 1.0))
    rail_mat = model.material("polished_linear_rail", color=(0.70, 0.72, 0.70, 1.0))
    carriage_mat = model.material("blue_anodized_carriage", color=(0.05, 0.22, 0.62, 1.0))
    slide_mat = model.material("brushed_aluminum_slide", color=(0.62, 0.65, 0.66, 1.0))
    black = model.material("black_bearing_pads", color=(0.015, 0.015, 0.018, 1.0))
    tool_mat = model.material("warm_tool_face", color=(0.95, 0.67, 0.18, 1.0))
    tool_pad = model.material("dark_tool_insert", color=(0.02, 0.02, 0.025, 1.0))

    frame = model.part("rear_frame")
    # Grounded rectangular backplane frame in the YZ plane.  X is machine depth,
    # Y is the horizontal stage axis, and Z is vertical.
    frame.visual(
        Box((0.080, 0.060, 0.960)),
        origin=Origin(xyz=(0.0, -0.620, 0.500)),
        material=steel,
        name="side_upright_0",
    )
    frame.visual(
        Box((0.080, 0.060, 0.960)),
        origin=Origin(xyz=(0.0, 0.620, 0.500)),
        material=steel,
        name="side_upright_1",
    )
    frame.visual(
        Box((0.080, 1.300, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=steel,
        name="top_crossmember",
    )
    frame.visual(
        Box((0.080, 1.300, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=steel,
        name="bottom_crossmember",
    )
    frame.visual(
        Box((0.018, 1.220, 0.860)),
        origin=Origin(xyz=(-0.046, 0.0, 0.500)),
        material=model.material("recessed_rear_panel", color=(0.11, 0.12, 0.13, 1.0)),
        name="recessed_backplate",
    )

    # Two parallel horizontal rods carry the Y carriage.
    for z, name in ((0.760, "upper_rail"), (0.420, "lower_rail")):
        frame.visual(
            Cylinder(radius=0.015, length=1.180),
            origin=Origin(xyz=(0.080, 0.0, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rail_mat,
            name=name,
        )
        for y, suffix in ((-0.555, "neg"), (0.555, "pos")):
            frame.visual(
                Box((0.090, 0.070, 0.085)),
                origin=Origin(xyz=(0.060, y, z)),
                material=steel,
                name=f"{name}_end_block_{suffix}",
            )

    carriage = model.part("horizontal_carriage")
    carriage.visual(
        Box((0.035, 0.220, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_mat,
        name="carriage_plate",
    )
    # Split linear-bearing jaws leave clearance around the fixed rods while
    # visibly wrapping them from above and below.
    for rail_z, upper_name, lower_name in (
        (0.170, "upper_upper_jaw", "upper_lower_jaw"),
        (-0.170, "lower_upper_jaw", "lower_lower_jaw"),
    ):
        carriage.visual(
            Box((0.070, 0.170, 0.024)),
            origin=Origin(xyz=(-0.050, 0.0, rail_z + 0.027)),
            material=black,
            name=upper_name,
        )
        carriage.visual(
            Box((0.070, 0.170, 0.024)),
            origin=Origin(xyz=(-0.050, 0.0, rail_z - 0.027)),
            material=black,
            name=lower_name,
        )

    # Front guide cheeks carry the vertical slide without blocking its channel.
    for y, cheek_name in ((-0.037, "vertical_guide_cheek_neg"), (0.037, "vertical_guide_cheek_pos")):
        carriage.visual(
            Box((0.055, 0.014, 0.380)),
            origin=Origin(xyz=(0.040, y, -0.020)),
            material=carriage_mat,
            name=cheek_name,
        )
    carriage.visual(
        Box((0.055, 0.150, 0.030)),
        origin=Origin(xyz=(0.040, 0.0, 0.185)),
        material=carriage_mat,
        name="vertical_guide_cap",
    )

    slide = model.part("vertical_slide")
    slide.visual(
        Box((0.030, 0.060, 0.400)),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=slide_mat,
        name="slide_bar",
    )
    slide.visual(
        Box((0.070, 0.095, 0.070)),
        origin=Origin(xyz=(0.030, 0.0, -0.430)),
        material=slide_mat,
        name="tool_mount",
    )
    slide.visual(
        Box((0.020, 0.145, 0.105)),
        origin=Origin(xyz=(0.073, 0.0, -0.450)),
        material=tool_mat,
        name="tool_face",
    )
    slide.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.089, 0.0, -0.450), rpy=(0.0, pi / 2.0, 0.0)),
        material=tool_pad,
        name="tool_insert",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.130, 0.0, 0.590)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=-0.380, upper=0.380),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.040, 0.0, 0.170)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rear_frame")
    carriage = object_model.get_part("horizontal_carriage")
    slide = object_model.get_part("vertical_slide")
    y_joint = object_model.get_articulation("frame_to_carriage")
    z_joint = object_model.get_articulation("carriage_to_slide")

    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="upper_rail",
        negative_elem="upper_lower_jaw",
        max_gap=0.001,
        max_penetration=0.0005,
        name="upper rail rests on lower carriage jaw",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="upper_upper_jaw",
        negative_elem="upper_rail",
        max_gap=0.001,
        max_penetration=0.0005,
        name="upper carriage jaw caps the guide rail",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="y",
        elem_a="carriage_plate",
        elem_b="upper_rail",
        min_overlap=0.18,
        name="carriage spans the horizontal guide rail",
    )
    ctx.expect_overlap(
        slide,
        carriage,
        axes="z",
        elem_a="slide_bar",
        elem_b="vertical_guide_cheek_neg",
        min_overlap=0.18,
        name="vertical slide remains engaged in guide cheeks",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({y_joint: 0.300}):
        shifted_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "horizontal carriage moves along +Y",
        rest_carriage is not None
        and shifted_carriage is not None
        and shifted_carriage[1] > rest_carriage[1] + 0.250
        and abs(shifted_carriage[2] - rest_carriage[2]) < 0.001,
        details=f"rest={rest_carriage}, shifted={shifted_carriage}",
    )

    rest_slide = ctx.part_world_position(slide)
    with ctx.pose({z_joint: 0.180}):
        lowered_slide = ctx.part_world_position(slide)
        ctx.expect_overlap(
            slide,
            carriage,
            axes="z",
            elem_a="slide_bar",
            elem_b="vertical_guide_cheek_neg",
            min_overlap=0.12,
            name="lowered slide is still retained by the carriage guide",
        )
    ctx.check(
        "vertical slide moves downward",
        rest_slide is not None
        and lowered_slide is not None
        and lowered_slide[2] < rest_slide[2] - 0.150
        and abs(lowered_slide[1] - rest_slide[1]) < 0.001,
        details=f"rest={rest_slide}, lowered={lowered_slide}",
    )

    return ctx.report()


object_model = build_object_model()
