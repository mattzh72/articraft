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
    model = ArticulatedObject(name="compact_service_arm")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    dark = model.material("dark_metal", rgba=(0.07, 0.08, 0.09, 1.0))
    blue = model.material("powder_blue", rgba=(0.12, 0.34, 0.58, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    orange = model.material("service_orange", rgba=(0.92, 0.40, 0.10, 1.0))
    rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=graphite,
        name="floor_plate",
    )
    for x in (-0.13, 0.13):
        for y in (-0.10, 0.10):
            base.visual(
                Cylinder(radius=0.027, length=0.018),
                origin=Origin(xyz=(x, y, 0.009)),
                material=rubber,
                name="foot_pad",
            )
            base.visual(
                Cylinder(radius=0.010, length=0.020),
                origin=Origin(xyz=(x, y, 0.050)),
                material=steel,
                name="anchor_bolt",
            )
    base.visual(
        Cylinder(radius=0.055, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=graphite,
        name="pedestal_column",
    )
    for y in (-0.052, 0.052):
        base.visual(
            Box((0.12, 0.020, 0.30)),
            origin=Origin(xyz=(0.0, y, 0.19)),
            material=graphite,
            name="support_rib",
        )
    for x in (-0.052, 0.052):
        base.visual(
            Box((0.020, 0.12, 0.30)),
            origin=Origin(xyz=(x, 0.0, 0.19)),
            material=graphite,
            name="support_rib",
        )
    base.visual(
        Box((0.15, 0.13, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=dark,
        name="upper_collar",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=steel,
        name="slew_bearing",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.080, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="rotary_hub",
    )
    arm.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=blue,
        name="hub_cap",
    )
    arm.visual(
        Box((0.78, 0.095, 0.080)),
        origin=Origin(xyz=(0.39, 0.0, 0.075)),
        material=blue,
        name="boom_box",
    )
    arm.visual(
        Box((0.60, 0.075, 0.018)),
        origin=Origin(xyz=(0.43, 0.0, 0.123)),
        material=blue,
        name="top_stiffener",
    )
    arm.visual(
        Box((0.42, 0.040, 0.030)),
        origin=Origin(xyz=(0.60, 0.0, 0.020)),
        material=steel,
        name="linear_rail",
    )
    arm.visual(
        Box((0.030, 0.078, 0.075)),
        origin=Origin(xyz=(0.385, 0.0, 0.032)),
        material=dark,
        name="rail_stop",
    )
    arm.visual(
        Box((0.030, 0.078, 0.075)),
        origin=Origin(xyz=(0.815, 0.0, 0.032)),
        material=dark,
        name="rail_stop",
    )
    for x in (0.22, 0.53, 0.72):
        arm.visual(
            Cylinder(radius=0.012, length=0.098),
            origin=Origin(xyz=(x, 0.0, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="cross_bolt",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.150, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, -0.027, -0.010)),
        material=dark,
        name="slide_cheek",
    )
    carriage.visual(
        Box((0.150, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.027, -0.010)),
        material=dark,
        name="slide_cheek",
    )
    carriage.visual(
        Box((0.150, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=dark,
        name="lower_bridge",
    )
    carriage.visual(
        Box((0.110, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=dark,
        name="bottom_wear_shoe",
    )
    carriage.visual(
        Box((0.100, 0.082, 0.120)),
        origin=Origin(xyz=(0.040, 0.0, -0.085)),
        material=orange,
        name="head_block",
    )
    carriage.visual(
        Cylinder(radius=0.019, length=0.040),
        origin=Origin(xyz=(0.110, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_socket",
    )
    carriage.visual(
        Box((0.040, 0.060, 0.020)),
        origin=Origin(xyz=(0.090, 0.0, -0.040)),
        material=steel,
        name="head_face_plate",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.50, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.check(
        "one revolute and one prismatic joint",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and len(object_model.articulations) == 2,
    )
    ctx.expect_contact(
        arm,
        base,
        elem_a="rotary_hub",
        elem_b="slew_bearing",
        name="rotary hub sits on bearing",
    )
    ctx.expect_contact(
        carriage,
        arm,
        elem_a="slide_cheek",
        elem_b="linear_rail",
        name="carriage cheeks capture rail",
    )
    ctx.expect_within(
        carriage,
        arm,
        axes="x",
        inner_elem="lower_bridge",
        outer_elem="linear_rail",
        margin=0.002,
        name="carriage starts on rail length",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.18}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            arm,
            axes="x",
            inner_elem="lower_bridge",
            outer_elem="linear_rail",
            margin=0.002,
            name="carriage remains on rail at travel limit",
        )
    ctx.check(
        "prismatic joint moves carriage outward",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.15,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_arm_pos = ctx.part_world_position(carriage)
    with ctx.pose({yaw: 0.70}):
        swung_arm_pos = ctx.part_world_position(carriage)
    ctx.check(
        "revolute joint swings outer arm sideways",
        rest_arm_pos is not None
        and swung_arm_pos is not None
        and swung_arm_pos[1] > rest_arm_pos[1] + 0.25,
        details=f"rest={rest_arm_pos}, swung={swung_arm_pos}",
    )

    return ctx.report()


object_model = build_object_model()
