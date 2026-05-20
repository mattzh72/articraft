from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_hinge_arm")

    cast_iron = model.material("dark_cast_iron", color=(0.14, 0.15, 0.16, 1.0))
    ground_steel = model.material("ground_steel", color=(0.68, 0.70, 0.70, 1.0))
    blue_anodized = model.material("blue_anodized_carriage", color=(0.07, 0.22, 0.42, 1.0))
    pivot_steel = model.material("polished_pivot_steel", color=(0.82, 0.82, 0.78, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(
        Box((0.74, 0.24, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_plate",
    )
    rail_base.visual(
        Box((0.64, 0.032, 0.042)),
        origin=Origin(xyz=(0.0, 0.075, 0.051)),
        material=cast_iron,
        name="slide_rail_0",
    )
    rail_base.visual(
        Box((0.64, 0.032, 0.042)),
        origin=Origin(xyz=(0.0, -0.075, 0.051)),
        material=cast_iron,
        name="slide_rail_1",
    )
    rail_base.visual(
        Box((0.64, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.075, 0.076)),
        material=ground_steel,
        name="wear_strip_0",
    )
    rail_base.visual(
        Box((0.64, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, -0.075, 0.076)),
        material=ground_steel,
        name="wear_strip_1",
    )
    rail_base.visual(
        Box((0.64, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=ground_steel,
        name="center_machined_face",
    )
    rail_base.visual(
        Box((0.050, 0.220, 0.095)),
        origin=Origin(xyz=(-0.345, 0.0, 0.0775)),
        material=cast_iron,
        name="left_stop",
    )
    rail_base.visual(
        Box((0.050, 0.220, 0.095)),
        origin=Origin(xyz=(0.345, 0.0, 0.0775)),
        material=cast_iron,
        name="right_stop",
    )
    rail_base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(-0.313, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="stop_bumper_0",
    )
    rail_base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.313, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="stop_bumper_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.170, 0.160, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=blue_anodized,
        name="saddle",
    )
    carriage.visual(
        Box((0.150, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.075, -0.131)),
        material=ground_steel,
        name="guide_shoe_0",
    )
    carriage.visual(
        Box((0.150, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.075, -0.131)),
        material=ground_steel,
        name="guide_shoe_1",
    )
    carriage.visual(
        Box((0.064, 0.022, 0.140)),
        origin=Origin(xyz=(-0.005, 0.052, -0.020)),
        material=blue_anodized,
        name="bracket_cheek_0",
    )
    carriage.visual(
        Box((0.064, 0.022, 0.140)),
        origin=Origin(xyz=(-0.005, -0.052, -0.020)),
        material=blue_anodized,
        name="bracket_cheek_1",
    )
    carriage.visual(
        Box((0.064, 0.104, 0.028)),
        origin=Origin(xyz=(-0.005, 0.0, -0.076)),
        material=blue_anodized,
        name="bracket_bridge",
    )
    carriage.visual(
        Box((0.026, 0.118, 0.075)),
        origin=Origin(xyz=(-0.045, 0.0, -0.047)),
        material=blue_anodized,
        name="rear_gusset",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pivot_pin",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pivot_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pivot_cap_1",
    )

    arm = model.part("output_arm")
    arm.visual(
        Cylinder(radius=0.035, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.235, 0.044, 0.032)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=pivot_steel,
        name="arm_bar",
    )
    arm.visual(
        Box((0.030, 0.090, 0.090)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=black_rubber,
        name="square_pad",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=carriage,
        origin=Origin(xyz=(-0.180, 0.0, 0.220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.360),
        motion_properties=MotionProperties(damping=4.0, friction=0.4),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.2, lower=0.0, upper=1.05),
        motion_properties=MotionProperties(damping=0.25, friction=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_base = object_model.get_part("rail_base")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("output_arm")
    slide = object_model.get_articulation("base_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

    ctx.allow_overlap(
        carriage,
        arm,
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        reason="The steel pivot pin is intentionally captured through the arm hub bore.",
    )

    ctx.expect_contact(
        carriage,
        rail_base,
        elem_a="guide_shoe_0",
        elem_b="wear_strip_0",
        contact_tol=0.001,
        name="upper guide shoe bears on the rail strip",
    )
    ctx.expect_contact(
        carriage,
        rail_base,
        elem_a="guide_shoe_1",
        elem_b="wear_strip_1",
        contact_tol=0.001,
        name="lower guide shoe bears on the rail strip",
    )
    ctx.expect_within(
        carriage,
        rail_base,
        axes="y",
        inner_elem="guide_shoe_0",
        outer_elem="wear_strip_0",
        margin=0.002,
        name="upper guide shoe stays centered on rail",
    )
    ctx.expect_within(
        carriage,
        rail_base,
        axes="y",
        inner_elem="guide_shoe_1",
        outer_elem="wear_strip_1",
        margin=0.002,
        name="lower guide shoe stays centered on rail",
    )
    ctx.expect_gap(
        carriage,
        rail_base,
        axis="x",
        positive_elem="saddle",
        negative_elem="left_stop",
        min_gap=0.035,
        name="carriage clears the left end stop at rest",
    )
    ctx.expect_within(
        carriage,
        arm,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_hub",
        margin=0.001,
        name="pivot pin is centered inside the arm hub",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        min_overlap=0.050,
        name="pivot pin spans the arm hub",
    )
    ctx.expect_gap(
        arm,
        rail_base,
        axis="z",
        min_gap=0.045,
        name="horizontal arm clears fixed rail and stops",
    )

    rest_carriage_position = ctx.part_world_position(carriage)
    rest_pad_aabb = ctx.part_element_world_aabb(arm, elem="square_pad")

    with ctx.pose({slide: 0.360}):
        extended_carriage_position = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            rail_base,
            elem_a="guide_shoe_0",
            elem_b="wear_strip_0",
            contact_tol=0.001,
            name="extended upper guide shoe remains on rail strip",
        )
        ctx.expect_contact(
            carriage,
            rail_base,
            elem_a="guide_shoe_1",
            elem_b="wear_strip_1",
            contact_tol=0.001,
            name="extended lower guide shoe remains on rail strip",
        )
        ctx.expect_gap(
            rail_base,
            carriage,
            axis="x",
            positive_elem="right_stop",
            negative_elem="saddle",
            min_gap=0.035,
            name="carriage clears the right end stop at full travel",
        )
        ctx.expect_gap(
            arm,
            rail_base,
            axis="z",
            min_gap=0.045,
            name="horizontal arm clears stops at full slide travel",
        )

    ctx.check(
        "carriage translates along the rail axis",
        rest_carriage_position is not None
        and extended_carriage_position is not None
        and extended_carriage_position[0] > rest_carriage_position[0] + 0.32
        and abs(extended_carriage_position[1] - rest_carriage_position[1]) < 1.0e-6
        and abs(extended_carriage_position[2] - rest_carriage_position[2]) < 1.0e-6,
        details=f"rest={rest_carriage_position}, extended={extended_carriage_position}",
    )

    with ctx.pose({hinge: 1.05}):
        raised_pad_aabb = ctx.part_element_world_aabb(arm, elem="square_pad")
        ctx.expect_gap(
            arm,
            rail_base,
            axis="z",
            min_gap=0.045,
            name="raised arm clears fixed rail and stops",
        )

    with ctx.pose({slide: 0.360, hinge: 1.05}):
        ctx.expect_gap(
            arm,
            rail_base,
            axis="z",
            min_gap=0.045,
            name="raised arm clears stops at full slide travel",
        )

    ctx.check(
        "arm swings upward from the carriage pivot",
        rest_pad_aabb is not None
        and raised_pad_aabb is not None
        and raised_pad_aabb[0][2] > rest_pad_aabb[0][2] + 0.14,
        details=f"rest_pad={rest_pad_aabb}, raised_pad={raised_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
