from __future__ import annotations

import math

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
    model = ArticulatedObject(name="bench_xyz_module")

    anodized = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    blue = Material("blue_anodized_carriage", color=(0.05, 0.18, 0.55, 1.0))
    steel = Material("brushed_steel", color=(0.70, 0.72, 0.72, 1.0))
    black = Material("black_bearing_blocks", color=(0.015, 0.015, 0.017, 1.0))
    face_plate = Material("machined_output_face", color=(0.86, 0.86, 0.82, 1.0))
    rubber = Material("matte_rubber_feet", color=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base_axis")
    base.visual(
        Box((1.20, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=anodized,
        name="ground_plate",
    )
    for i, x in enumerate((-0.48, 0.48)):
        for j, y in enumerate((-0.14, 0.14)):
            base.visual(
                Box((0.16, 0.060, 0.015)),
                origin=Origin(xyz=(x, y, 0.0075)),
                material=rubber,
                name=f"rubber_foot_{i}_{j}",
            )
    for y, rail_name in (
        (-0.105, "x_linear_rail_front"),
        (0.105, "x_linear_rail_rear"),
    ):
        base.visual(
            Box((1.06, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.0725)),
            material=steel,
            name=rail_name,
        )
    base.visual(
        Cylinder(radius=0.012, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="x_lead_screw",
    )
    for x in (-0.555, 0.555):
        base.visual(
            Box((0.045, 0.110, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.085)),
            material=black,
            name=f"x_end_bearing_{'neg' if x < 0 else 'pos'}",
        )

    lower = model.part("lower_stage")
    lower.visual(
        Box((0.28, 0.32, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=blue,
        name="x_carriage_plate",
    )
    for x, y, pad_name in (
        (-0.075, -0.105, "x_bearing_pad_neg_front"),
        (-0.075, 0.105, "x_bearing_pad_neg_rear"),
        (0.075, -0.105, "x_bearing_pad_pos_front"),
        (0.075, 0.105, "x_bearing_pad_pos_rear"),
    ):
        lower.visual(
            Box((0.095, 0.060, 0.028)),
            origin=Origin(xyz=(x, y, 0.014)),
            material=black,
            name=pad_name,
        )
    for x, rail_name in (
        (-0.075, "y_linear_rail_neg"),
        (0.075, "y_linear_rail_pos"),
    ):
        lower.visual(
            Box((0.032, 0.46, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0625)),
            material=steel,
            name=rail_name,
        )
    lower.visual(
        Cylinder(radius=0.010, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="y_lead_screw",
    )
    for y, block_name in (
        (-0.210, "y_end_bearing_front"),
        (0.210, "y_end_bearing_rear"),
    ):
        lower.visual(
            Box((0.230, 0.035, 0.065)),
            origin=Origin(xyz=(0.0, y, 0.0525)),
            material=black,
            name=block_name,
        )

    middle = model.part("middle_stage")
    middle.visual(
        Box((0.22, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=blue,
        name="y_carriage_plate",
    )
    for x, y, pad_name in (
        (-0.075, -0.060, "y_bearing_pad_neg_front"),
        (-0.075, 0.060, "y_bearing_pad_neg_rear"),
        (0.075, -0.060, "y_bearing_pad_pos_front"),
        (0.075, 0.060, "y_bearing_pad_pos_rear"),
    ):
        middle.visual(
            Box((0.060, 0.070, 0.022)),
            origin=Origin(xyz=(x, y, 0.011)),
            material=black,
            name=pad_name,
        )
    middle.visual(
        Box((0.13, 0.090, 0.055)),
        origin=Origin(xyz=(0.0, -0.060, 0.0625)),
        material=anodized,
        name="z_axis_foot",
    )
    middle.visual(
        Box((0.105, 0.045, 0.540)),
        origin=Origin(xyz=(0.0, -0.070, 0.305)),
        material=anodized,
        name="vertical_backbone",
    )
    for x, rail_name in (
        (-0.035, "z_linear_rail_neg"),
        (0.035, "z_linear_rail_pos"),
    ):
        middle.visual(
            Box((0.020, 0.020, 0.480)),
            origin=Origin(xyz=(x, -0.0375, 0.305)),
            material=steel,
            name=rail_name,
        )

    upper = model.part("upper_stage")
    upper.visual(
        Box((0.082, 0.035, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="vertical_slide",
    )
    upper.visual(
        Box((0.140, 0.024, 0.140)),
        origin=Origin(xyz=(0.0, 0.0295, 0.065)),
        material=face_plate,
        name="square_output_face",
    )
    upper.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.0465, 0.065), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="output_register_boss",
    )
    for x in (-0.045, 0.045):
        for z in (0.020, 0.110):
            upper.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, 0.0445, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"face_bolt_{'neg' if x < 0 else 'pos'}_{'low' if z < 0.05 else 'high'}",
            )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=-0.28, upper=0.28),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=-0.16, upper=0.16),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=upper,
        origin=Origin(xyz=(0.0, -0.010, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_axis")
    lower = object_model.get_part("lower_stage")
    middle = object_model.get_part("middle_stage")
    upper = object_model.get_part("upper_stage")
    x_joint = object_model.get_articulation("base_to_lower")
    y_joint = object_model.get_articulation("lower_to_middle")
    z_joint = object_model.get_articulation("middle_to_upper")

    ctx.check(
        "three orthogonal prismatic axes",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and y_joint.articulation_type == ArticulationType.PRISMATIC
        and z_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axes: {x_joint.axis}, {y_joint.axis}, {z_joint.axis}",
    )

    with ctx.pose({x_joint: 0.0, y_joint: 0.0, z_joint: 0.0}):
        ctx.expect_contact(
            lower,
            base,
            elem_a="x_bearing_pad_neg_front",
            elem_b="x_linear_rail_front",
            contact_tol=0.001,
            name="lower carriage sits on grounded x rail",
        )
        ctx.expect_contact(
            middle,
            lower,
            elem_a="y_bearing_pad_neg_front",
            elem_b="y_linear_rail_neg",
            contact_tol=0.001,
            name="crosswise carriage sits on y rail",
        )
        ctx.expect_contact(
            upper,
            middle,
            elem_a="vertical_slide",
            elem_b="z_linear_rail_neg",
            contact_tol=0.001,
            name="vertical stage rides on z rail",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="vertical_slide",
            elem_b="z_linear_rail_neg",
            min_overlap=0.18,
            name="vertical slide remains engaged at rest",
        )

    rest_lower = ctx.part_world_position(lower)
    rest_middle = ctx.part_world_position(middle)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({x_joint: 0.22, y_joint: 0.12, z_joint: 0.18}):
        moved_lower = ctx.part_world_position(lower)
        moved_middle = ctx.part_world_position(middle)
        moved_upper = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="vertical_slide",
            elem_b="z_linear_rail_neg",
            min_overlap=0.14,
            name="vertical slide retains insertion when raised",
        )

    ctx.check(
        "lower stage translates along x",
        rest_lower is not None and moved_lower is not None and moved_lower[0] > rest_lower[0] + 0.20,
        details=f"rest={rest_lower}, moved={moved_lower}",
    )
    ctx.check(
        "middle stage translates along y",
        rest_middle is not None and moved_middle is not None and moved_middle[1] > rest_middle[1] + 0.10,
        details=f"rest={rest_middle}, moved={moved_middle}",
    )
    ctx.check(
        "upper stage translates upward",
        rest_upper is not None and moved_upper is not None and moved_upper[2] > rest_upper[2] + 0.16,
        details=f"rest={rest_upper}, moved={moved_upper}",
    )

    return ctx.report()


object_model = build_object_model()
