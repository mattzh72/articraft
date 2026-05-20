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
    model = ArticulatedObject(name="bridge_backed_transfer_axis")

    frame_iron = model.material("painted_frame_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    rail_steel = model.material("polished_linear_rail", rgba=(0.72, 0.74, 0.76, 1.0))
    stop_rubber = model.material("black_bumper_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    slide_blue = model.material("blue_carriage_casting", rgba=(0.04, 0.16, 0.42, 1.0))
    housing_black = model.material("black_motor_housing", rgba=(0.03, 0.035, 0.04, 1.0))
    tool_steel = model.material("brushed_output_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.01, 0.012, 0.014, 1.0))

    frame = model.part("rear_frame")
    frame.visual(
        Box((1.56, 0.44, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_iron,
        name="base_plate",
    )
    for x, name in ((-0.66, "post_0"), (0.66, "post_1")):
        frame.visual(
            Box((0.14, 0.16, 0.76)),
            origin=Origin(xyz=(x, 0.08, 0.43)),
            material=frame_iron,
            name=name,
        )
    frame.visual(
        Box((1.50, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, 0.08, 0.86)),
        material=frame_iron,
        name="top_bridge",
    )
    frame.visual(
        Box((1.36, 0.08, 0.32)),
        origin=Origin(xyz=(0.0, 0.04, 0.68)),
        material=frame_iron,
        name="rear_backbone",
    )
    for z, bed_name, rail_name in (
        (0.60, "lower_rail_bed", "lower_rail"),
        (0.76, "upper_rail_bed", "upper_rail"),
    ):
        frame.visual(
            Box((1.22, 0.12, 0.045)),
            origin=Origin(xyz=(0.0, -0.035, z)),
            material=frame_iron,
            name=bed_name,
        )
        frame.visual(
            Cylinder(radius=0.016, length=1.22),
            origin=Origin(xyz=(0.0, -0.100, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=rail_name,
        )
    for x, name in ((-0.625, "end_stop_0"), (0.625, "end_stop_1")):
        frame.visual(
            Box((0.07, 0.15, 0.26)),
            origin=Origin(xyz=(x, -0.05, 0.68)),
            material=frame_iron,
            name=name,
        )
        frame.visual(
            Box((0.018, 0.035, 0.13)),
            origin=Origin(xyz=(x * 0.985, -0.135, 0.68)),
            material=stop_rubber,
            name=f"bumper_{name[-1]}",
        )

    slide = model.part("slide_stage")
    slide.visual(
        Box((0.30, 0.055, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=slide_blue,
        name="carriage_plate",
    )
    slide.visual(
        Box((0.30, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.050, 0.080)),
        material=rail_steel,
        name="upper_bearing",
    )
    slide.visual(
        Box((0.30, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.050, -0.080)),
        material=rail_steel,
        name="lower_bearing",
    )
    slide.visual(
        Box((0.22, 0.11, 0.22)),
        origin=Origin(xyz=(0.0, -0.073, 0.0)),
        material=slide_blue,
        name="head_mount",
    )
    slide.visual(
        Cylinder(radius=0.105, length=0.150),
        origin=Origin(xyz=(0.0, -0.120, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="rotary_motor",
    )
    slide.visual(
        Cylinder(radius=0.115, length=0.045),
        origin=Origin(xyz=(0.0, -0.205, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="output_collar",
    )

    output = model.part("output_head")
    output.visual(
        Cylinder(radius=0.035, length=0.065),
        origin=Origin(xyz=(0.0, -0.0325, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=tool_steel,
        name="output_shaft",
    )
    output.visual(
        Cylinder(radius=0.078, length=0.040),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=tool_steel,
        name="drive_flange",
    )
    output.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, -0.100, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=tool_steel,
        name="pilot_boss",
    )
    for idx, (x, z) in enumerate(((0.050, 0.050), (-0.050, 0.050), (-0.050, -0.050), (0.050, -0.050))):
        output.visual(
            Cylinder(radius=0.0075, length=0.010),
            origin=Origin(xyz=(x, -0.096, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=screw_dark,
            name=f"flange_bolt_{idx}",
        )
    output.visual(
        Box((0.095, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.106, 0.0)),
        material=screw_dark,
        name="drive_key_slot",
    )

    slide_joint = model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide,
        origin=Origin(xyz=(-0.34, -0.1935, 0.68)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.55, lower=0.0, upper=0.68),
    )
    model.articulation(
        "slide_to_output",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=output,
        origin=Origin(xyz=(0.0, -0.2275, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0, lower=-3.14159, upper=3.14159),
    )
    # Keep a stable handle available for readable tests without hard-coding the
    # travel number in several places.
    slide_joint.meta["nominal_travel"] = 0.68
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rear_frame")
    slide = object_model.get_part("slide_stage")
    output = object_model.get_part("output_head")
    slide_joint = object_model.get_articulation("frame_to_slide")
    output_joint = object_model.get_articulation("slide_to_output")

    ctx.check(
        "one straight slide and one carried rotary head",
        len(object_model.articulations) == 2
        and slide_joint.articulation_type == ArticulationType.PRISMATIC
        and output_joint.articulation_type == ArticulationType.REVOLUTE
        and output_joint.parent == "slide_stage",
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_overlap(
        slide,
        frame,
        axes="xz",
        elem_a="upper_bearing",
        elem_b="upper_rail",
        min_overlap=0.02,
        name="upper bearing is aligned with upper rail",
    )
    ctx.expect_gap(
        frame,
        slide,
        axis="y",
        positive_elem="upper_rail",
        negative_elem="upper_bearing",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="upper bearing rides on upper rail",
    )
    ctx.expect_overlap(
        output,
        slide,
        axes="xz",
        elem_a="output_shaft",
        elem_b="output_collar",
        min_overlap=0.03,
        name="rotary shaft is coaxial with fixed collar",
    )
    ctx.expect_gap(
        slide,
        output,
        axis="y",
        positive_elem="output_collar",
        negative_elem="output_shaft",
        max_gap=0.001,
        max_penetration=0.00001,
        name="shaft seats at collar face without embed",
    )

    rest_slide_pos = ctx.part_world_position(slide)
    rest_output_pos = ctx.part_world_position(output)
    travel = slide_joint.motion_limits.upper or 0.0
    with ctx.pose({slide_joint: travel}):
        ctx.expect_overlap(
            slide,
            frame,
            axes="xz",
            elem_a="upper_bearing",
            elem_b="upper_rail",
            min_overlap=0.02,
            name="extended bearing remains on the rail",
        )
        ctx.expect_gap(
            frame,
            slide,
            axis="y",
            positive_elem="upper_rail",
            negative_elem="upper_bearing",
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            name="extended bearing rides on upper rail",
        )
        extended_slide_pos = ctx.part_world_position(slide)
        extended_output_pos = ctx.part_world_position(output)

    ctx.check(
        "slide moves along the transfer axis and carries the head",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and rest_output_pos is not None
        and extended_output_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.60
        and extended_output_pos[0] > rest_output_pos[0] + 0.60,
        details=(
            f"rest_slide={rest_slide_pos}, extended_slide={extended_slide_pos}, "
            f"rest_output={rest_output_pos}, extended_output={extended_output_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
