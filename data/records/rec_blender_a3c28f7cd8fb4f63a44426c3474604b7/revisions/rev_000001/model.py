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
    model = ArticulatedObject(name="milkshake_spindle_blender")

    model.material("stainless", rgba=(0.79, 0.81, 0.83, 1.0))
    model.material("cast_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.20, 0.22, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="stainless",
        name="base_plinth",
    )
    frame.visual(
        Box((0.135, 0.105, 0.28)),
        origin=Origin(xyz=(-0.02, 0.0, 0.185)),
        material="cast_metal",
        name="body_shell",
    )
    frame.visual(
        Box((0.07, 0.06, 0.31)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material="stainless",
        name="support_column",
    )
    frame.visual(
        Box((0.05, 0.095, 0.025)),
        origin=Origin(xyz=(-0.005, 0.0, 0.6225)),
        material="cast_metal",
        name="hinge_cap",
    )
    for index, y in enumerate((-0.036, 0.036)):
        frame.visual(
            Box((0.032, 0.012, 0.048)),
            origin=Origin(xyz=(0.034, y, 0.652)),
            material="cast_metal",
            name=f"hinge_cheek_{index}",
        )
    frame.visual(
        Box((0.07, 0.01, 0.20)),
        origin=Origin(xyz=(0.045, 0.0, 0.185)),
        material="stainless",
        name="splash_plate",
    )
    frame.visual(
        Box((0.195, 0.055, 0.02)),
        origin=Origin(xyz=(0.1375, 0.0, 0.145)),
        material="cast_metal",
        name="rest_bracket",
    )
    frame.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(0.24, 0.0, 0.146)),
        material="black_rubber",
        name="cup_rest",
    )
    for x in (-0.065, 0.065):
        for y in (-0.075, 0.075):
            frame.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, 0.004)),
                material="black_rubber",
                name=f"foot_{1 if x > 0 else 0}_{1 if y > 0 else 0}",
            )

    motor_arm = model.part("motor_arm")
    motor_arm.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_trim",
        name="hinge_barrel",
    )
    motor_arm.visual(
        Box((0.038, 0.05, 0.042)),
        origin=Origin(xyz=(0.03, 0.0, -0.006)),
        material="cast_metal",
        name="hinge_neck",
    )
    motor_arm.visual(
        Cylinder(radius=0.031, length=0.16),
        origin=Origin(xyz=(0.105, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="stainless",
        name="motor_can",
    )
    motor_arm.visual(
        Box((0.12, 0.075, 0.05)),
        origin=Origin(xyz=(0.098, 0.0, -0.03)),
        material="cast_metal",
        name="gearbox",
    )
    motor_arm.visual(
        Box((0.085, 0.062, 0.03)),
        origin=Origin(xyz=(0.087, 0.0, 0.02)),
        material="stainless",
        name="top_cover",
    )
    motor_arm.visual(
        Cylinder(radius=0.027, length=0.032),
        origin=Origin(xyz=(0.194, 0.0, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="stainless",
        name="nose_cap",
    )
    motor_arm.visual(
        Box((0.04, 0.058, 0.016)),
        origin=Origin(xyz=(0.187, 0.0, -0.035)),
        material="cast_metal",
        name="bearing_cap",
    )
    for index, y in enumerate((-0.023, 0.023)):
        motor_arm.visual(
            Box((0.032, 0.012, 0.08)),
            origin=Origin(xyz=(0.188, y, -0.058)),
            material="cast_metal",
            name=f"spindle_yoke_{index}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.011, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material="dark_trim",
        name="drive_hub",
    )
    spindle.visual(
        Cylinder(radius=0.0055, length=0.378),
        origin=Origin(xyz=(0.0, 0.0, -0.189)),
        material="steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.01, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.366)),
        material="steel",
        name="lower_collar",
    )
    spindle.visual(
        Cylinder(radius=0.02, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material="steel",
        name="agitator",
    )
    spindle.visual(
        Box((0.032, 0.004, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material="steel",
        name="agitator_blade_x",
    )
    spindle.visual(
        Box((0.004, 0.032, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material="steel",
        name="agitator_blade_y",
    )

    model.articulation(
        "frame_to_motor_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=motor_arm,
        origin=Origin(xyz=(0.038, 0.0, 0.652)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "motor_arm_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=motor_arm,
        child=spindle,
        origin=Origin(xyz=(0.195, 0.0, -0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    motor_arm = object_model.get_part("motor_arm")
    arm_hinge = object_model.get_articulation("frame_to_motor_arm")

    ctx.expect_overlap(
        spindle,
        frame,
        axes="xy",
        elem_a="agitator",
        elem_b="cup_rest",
        min_overlap=0.03,
        name="spindle stays centered above the cup rest",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="z",
        positive_elem="agitator",
        negative_elem="cup_rest",
        min_gap=0.035,
        max_gap=0.07,
        name="agitator clears the cup rest at operating height",
    )

    rest_spindle_pos = ctx.part_world_position(spindle)
    with ctx.pose({arm_hinge: arm_hinge.motion_limits.upper}):
        ctx.expect_gap(
            spindle,
            frame,
            axis="z",
            positive_elem="agitator",
            negative_elem="cup_rest",
            min_gap=0.13,
            name="raised arm lifts the agitator well above the cup rest",
        )
        raised_spindle_pos = ctx.part_world_position(spindle)

    ctx.check(
        "spindle tip swings upward when the arm opens",
        rest_spindle_pos is not None
        and raised_spindle_pos is not None
        and raised_spindle_pos[2] > rest_spindle_pos[2] + 0.10,
        details=f"rest={rest_spindle_pos}, raised={raised_spindle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
