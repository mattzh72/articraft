from __future__ import annotations

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


STEEL = Material("brushed_dark_steel", color=(0.22, 0.23, 0.24, 1.0))
RAIL = Material("polished_linear_rail", color=(0.72, 0.74, 0.75, 1.0))
ANODIZED = Material("blue_anodized_carriage", color=(0.08, 0.22, 0.55, 1.0))
PLATE = Material("matte_black_mounting_plate", color=(0.03, 0.035, 0.04, 1.0))
WALL = Material("powder_coated_wall_frame", color=(0.48, 0.50, 0.48, 1.0))
MOTOR = Material("black_motor_housing", color=(0.01, 0.01, 0.012, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_xyz_positioning_module")

    side_frame = model.part("side_frame")
    side_frame.visual(
        Box((1.80, 0.06, 1.30)),
        origin=Origin(xyz=(0.0, -0.030, 0.65)),
        material=WALL,
        name="wall_plate",
    )
    side_frame.visual(
        Box((1.80, 0.40, 0.06)),
        origin=Origin(xyz=(0.0, 0.120, 0.030)),
        material=WALL,
        name="base_foot",
    )
    for z, name in ((0.40, "x_rail_lower"), (0.80, "x_rail_upper")):
        side_frame.visual(
            Box((1.62, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, 0.025, z)),
            material=RAIL,
            name=name,
        )
    side_frame.visual(
        Cylinder(radius=0.014, length=1.55),
        origin=Origin(xyz=(0.0, 0.075, 0.60), rpy=(0.0, 1.57079632679, 0.0)),
        material=RAIL,
        name="x_lead_screw",
    )
    for x, name in ((-0.75, "x_bearing_left"), (0.75, "x_bearing_right")):
        side_frame.visual(
            Box((0.08, 0.09, 0.11)),
            origin=Origin(xyz=(x, 0.045, 0.60)),
            material=STEEL,
            name=name,
        )
    side_frame.visual(
        Box((0.13, 0.12, 0.13)),
        origin=Origin(xyz=(-0.86, 0.095, 0.60)),
        material=MOTOR,
        name="x_stepper_motor",
    )
    side_frame.visual(
        Box((0.10, 0.16, 0.17)),
        origin=Origin(xyz=(-0.81, 0.035, 0.60)),
        material=STEEL,
        name="x_motor_mount",
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((1.05, 0.056, 0.52)),
        origin=Origin(xyz=(0.0, 0.062, 0.0)),
        material=ANODIZED,
        name="long_x_plate",
    )
    for x, x_tag in ((-0.36, "0"), (0.36, "1")):
        for z, z_tag in ((-0.20, "lower"), (0.20, "upper")):
            first_carriage.visual(
                Box((0.22, 0.08, 0.08)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=STEEL,
                name=f"x_shoe_{z_tag}_{x_tag}",
            )
    for x, name in ((-0.17, "y_rail_0"), (0.17, "y_rail_1")):
        first_carriage.visual(
            Box((0.045, 0.58, 0.035)),
            origin=Origin(xyz=(x, 0.37, 0.040)),
            material=RAIL,
            name=name,
        )
    first_carriage.visual(
        Box((0.44, 0.05, 0.085)),
        origin=Origin(xyz=(0.0, 0.66, 0.040)),
        material=STEEL,
        name="y_front_crossbar",
    )

    second_carriage = model.part("second_carriage")
    for x, name in ((-0.17, "y_shoe_0"), (0.17, "y_shoe_1")):
        second_carriage.visual(
            Box((0.09, 0.14, 0.06)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=STEEL,
            name=name,
        )
    second_carriage.visual(
        Box((0.42, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=ANODIZED,
        name="compact_y_saddle",
    )
    second_carriage.visual(
        Box((0.30, 0.06, 0.40)),
        origin=Origin(xyz=(0.0, 0.070, 0.250)),
        material=PLATE,
        name="vertical_mast",
    )
    for x, name in ((-0.09, "z_rail_0"), (0.09, "z_rail_1")):
        second_carriage.visual(
            Box((0.035, 0.035, 0.40)),
            origin=Origin(xyz=(x, 0.115, 0.250)),
            material=RAIL,
            name=name,
        )

    third_stage = model.part("third_stage")
    for x, name in ((-0.09, "z_shoe_0"), (0.09, "z_shoe_1")):
        third_stage.visual(
            Box((0.07, 0.05, 0.12)),
            origin=Origin(xyz=(x, -0.020, 0.0)),
            material=STEEL,
            name=name,
        )
    third_stage.visual(
        Box((0.24, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=ANODIZED,
        name="vertical_tool_plate",
    )
    third_stage.visual(
        Box((0.18, 0.04, 0.18)),
        origin=Origin(xyz=(0.0, 0.080, 0.0)),
        material=PLATE,
        name="tool_mount_face",
    )
    for x in (-0.065, 0.065):
        for z in (-0.065, 0.065):
            third_stage.visual(
                Cylinder(radius=0.010, length=0.018),
                origin=Origin(xyz=(x, 0.107, z), rpy=(-1.57079632679, 0.0, 0.0)),
                material=RAIL,
                name=f"mount_bolt_{'p' if x > 0 else 'n'}x_{'p' if z > 0 else 'n'}z",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=side_frame,
        child=first_carriage,
        origin=Origin(xyz=(0.0, 0.090, 0.60)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=-0.28, upper=0.28),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_carriage,
        origin=Origin(xyz=(0.0, 0.180, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.35, lower=0.0, upper=0.34),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=second_carriage,
        child=third_stage,
        origin=Origin(xyz=(0.0, 0.1775, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=-0.08, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    third = object_model.get_part("third_stage")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.check(
        "three orthogonal prismatic slides",
        len(object_model.articulations) == 3
        and x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and x_slide.axis == (1.0, 0.0, 0.0)
        and y_slide.axis == (0.0, 1.0, 0.0)
        and z_slide.axis == (0.0, 0.0, 1.0),
        details="Expected X, Y, and Z prismatic axes.",
    )

    ctx.expect_contact(
        first,
        frame,
        elem_a="x_shoe_lower_0",
        elem_b="x_rail_lower",
        contact_tol=0.001,
        name="first carriage rides on fixed X rail",
    )
    ctx.expect_contact(
        second,
        first,
        elem_a="y_shoe_0",
        elem_b="y_rail_0",
        contact_tol=0.001,
        name="second carriage rides on first-stage Y rail",
    )
    ctx.expect_contact(
        third,
        second,
        elem_a="z_shoe_0",
        elem_b="z_rail_0",
        contact_tol=0.001,
        name="third stage rides on vertical Z rail",
    )

    first_rest = ctx.part_world_position(first)
    second_rest = ctx.part_world_position(second)
    third_rest = ctx.part_world_position(third)
    with ctx.pose({x_slide: 0.28}):
        first_extended = ctx.part_world_position(first)
        ctx.expect_contact(
            first,
            frame,
            elem_a="x_shoe_upper_1",
            elem_b="x_rail_upper",
            contact_tol=0.001,
            name="X slide remains supported at positive travel",
        )
        ctx.expect_within(
            first,
            frame,
            axes="x",
            inner_elem="x_shoe_upper_1",
            outer_elem="x_rail_upper",
            name="X carriage shoe remains within rail length",
        )
    with ctx.pose({y_slide: 0.34}):
        second_extended = ctx.part_world_position(second)
        ctx.expect_contact(
            second,
            first,
            elem_a="y_shoe_0",
            elem_b="y_rail_0",
            contact_tol=0.001,
            name="Y slide remains supported at full outreach",
        )
        ctx.expect_within(
            second,
            first,
            axes="y",
            inner_elem="y_shoe_0",
            outer_elem="y_rail_0",
            name="Y carriage shoe remains within rail length",
        )
    with ctx.pose({z_slide: 0.16}):
        third_extended = ctx.part_world_position(third)
        ctx.expect_contact(
            third,
            second,
            elem_a="z_shoe_0",
            elem_b="z_rail_0",
            contact_tol=0.001,
            name="Z stage remains captured at upper travel",
        )
        ctx.expect_within(
            third,
            second,
            axes="z",
            inner_elem="z_shoe_0",
            outer_elem="z_rail_0",
            name="Z shoe remains within vertical guide length",
        )

    ctx.check(
        "positive X command moves first carriage right",
        first_rest is not None
        and first_extended is not None
        and first_extended[0] > first_rest[0] + 0.25,
        details=f"rest={first_rest}, extended={first_extended}",
    )
    ctx.check(
        "positive Y command moves second carriage outward",
        second_rest is not None
        and second_extended is not None
        and second_extended[1] > second_rest[1] + 0.30,
        details=f"rest={second_rest}, extended={second_extended}",
    )
    ctx.check(
        "positive Z command raises third stage",
        third_rest is not None
        and third_extended is not None
        and third_extended[2] > third_rest[2] + 0.14,
        details=f"rest={third_rest}, extended={third_extended}",
    )

    return ctx.report()


object_model = build_object_model()
