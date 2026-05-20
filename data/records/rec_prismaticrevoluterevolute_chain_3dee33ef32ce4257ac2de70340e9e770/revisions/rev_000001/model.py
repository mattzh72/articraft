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
    model = ArticulatedObject(name="fork_backed_slide_arm")

    painted_steel = model.material("painted_steel", color=(0.34, 0.36, 0.37, 1.0))
    rail_steel = model.material("brushed_rail_steel", color=(0.76, 0.76, 0.72, 1.0))
    carriage_red = model.material("carriage_red", color=(0.70, 0.12, 0.08, 1.0))
    arm_blue = model.material("arm_blue", color=(0.05, 0.20, 0.55, 1.0))
    pin_dark = model.material("dark_pins", color=(0.06, 0.06, 0.065, 1.0))
    rubber_black = model.material("rubber_black", color=(0.015, 0.014, 0.012, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.86, 0.36, 0.035)),
        origin=Origin(xyz=(0.43, 0.0, 0.0175)),
        material=painted_steel,
        name="base_plate",
    )
    frame.visual(
        Box((0.055, 0.060, 0.50)),
        origin=Origin(xyz=(0.035, -0.130, 0.285)),
        material=painted_steel,
        name="rear_post_0",
    )
    frame.visual(
        Box((0.055, 0.060, 0.50)),
        origin=Origin(xyz=(0.035, 0.130, 0.285)),
        material=painted_steel,
        name="rear_post_1",
    )
    frame.visual(
        Box((0.055, 0.34, 0.045)),
        origin=Origin(xyz=(0.035, 0.0, 0.535)),
        material=painted_steel,
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.74, 0.032, 0.040)),
        origin=Origin(xyz=(0.43, -0.115, 0.115)),
        material=rail_steel,
        name="rail_0",
    )
    frame.visual(
        Box((0.74, 0.032, 0.040)),
        origin=Origin(xyz=(0.43, 0.115, 0.115)),
        material=rail_steel,
        name="rail_1",
    )
    for index, y in enumerate((-0.115, 0.115)):
        for x in (0.46, 0.76):
            frame.visual(
                Box((0.035, 0.032, 0.060)),
                origin=Origin(xyz=(x, y, 0.065)),
                material=painted_steel,
                name=f"rail_stand_{index}_{int(x * 100)}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, -0.115, 0.148)),
        material=carriage_red,
        name="shoe_0",
    )
    carriage.visual(
        Box((0.18, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.115, 0.148)),
        material=carriage_red,
        name="shoe_1",
    )
    carriage.visual(
        Box((0.18, 0.19, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1735)),
        material=carriage_red,
        name="slide_deck",
    )
    carriage.visual(
        Box((0.070, 0.120, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.2035)),
        material=carriage_red,
        name="shoulder_base",
    )
    carriage.visual(
        Box((0.060, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, -0.046, 0.248)),
        material=carriage_red,
        name="shoulder_cheek_0",
    )
    carriage.visual(
        Box((0.060, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, 0.046, 0.248)),
        material=carriage_red,
        name="shoulder_cheek_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.026, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_dark,
        name="shoulder_pin",
    )
    upper_arm.visual(
        Box((0.270, 0.038, 0.034)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=arm_blue,
        name="upper_bar",
    )
    upper_arm.visual(
        Box((0.070, 0.026, 0.022)),
        origin=Origin(xyz=(0.280, -0.030, 0.0)),
        material=arm_blue,
        name="elbow_web_0",
    )
    upper_arm.visual(
        Box((0.070, 0.026, 0.022)),
        origin=Origin(xyz=(0.280, 0.030, 0.0)),
        material=arm_blue,
        name="elbow_web_1",
    )
    upper_arm.visual(
        Box((0.055, 0.014, 0.070)),
        origin=Origin(xyz=(0.340, -0.044, 0.0)),
        material=arm_blue,
        name="elbow_cheek_0",
    )
    upper_arm.visual(
        Box((0.055, 0.014, 0.070)),
        origin=Origin(xyz=(0.340, 0.044, 0.0)),
        material=arm_blue,
        name="elbow_cheek_1",
    )

    tip = model.part("tip")
    tip.visual(
        Cylinder(radius=0.023, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_dark,
        name="elbow_pin",
    )
    tip.visual(
        Box((0.220, 0.033, 0.030)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=arm_blue,
        name="tip_bar",
    )
    tip.visual(
        Box((0.070, 0.060, 0.026)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=arm_blue,
        name="tip_pad",
    )
    tip.visual(
        Cylinder(radius=0.018, length=0.065),
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="rubber_tip",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.38),
    )
    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.45, upper=1.10),
    )
    model.articulation(
        "upper_arm_to_tip",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tip,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    tip = object_model.get_part("tip")

    slide = object_model.get_articulation("frame_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_tip")

    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="shoe_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=1e-6,
        name="carriage shoe rides on fork rail",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="shoe_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=1e-6,
        name="opposite shoe rides on fork rail",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="shoe_0",
        elem_b="rail_0",
        min_overlap=0.16,
        name="carriage is retained on rail at rest",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.38}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="shoe_0",
            elem_b="rail_0",
            min_overlap=0.16,
            name="carriage remains on rail when extended",
        )
        extended_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic carriage slides forward",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.35,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_tip_origin = ctx.part_world_position(tip)
    with ctx.pose({shoulder: 0.60}):
        raised_tip_origin = ctx.part_world_position(tip)
    ctx.check(
        "shoulder revolute raises elbow pivot",
        rest_tip_origin is not None
        and raised_tip_origin is not None
        and raised_tip_origin[2] > rest_tip_origin[2] + 0.12,
        details=f"rest={rest_tip_origin}, raised={raised_tip_origin}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0}):
        low_tip_aabb = ctx.part_element_world_aabb(tip, elem="rubber_tip")
    with ctx.pose({shoulder: 0.35, elbow: 0.65}):
        high_tip_aabb = ctx.part_element_world_aabb(tip, elem="rubber_tip")

    low_tip_z = None
    high_tip_z = None
    if low_tip_aabb is not None:
        low_tip_z = (low_tip_aabb[0][2] + low_tip_aabb[1][2]) / 2.0
    if high_tip_aabb is not None:
        high_tip_z = (high_tip_aabb[0][2] + high_tip_aabb[1][2]) / 2.0
    ctx.check(
        "elbow revolute articulates the tip",
        low_tip_z is not None and high_tip_z is not None and high_tip_z > low_tip_z + 0.06,
        details=f"low_tip_z={low_tip_z}, high_tip_z={high_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
