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
    model = ArticulatedObject(name="turnstile")

    base = model.part("base")
    base.visual(Box((1.6, 1.6, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="base_plate")

    frame = model.part("frame")
    frame.visual(Box((1.6, 0.2, 1.2)), origin=Origin(xyz=(0.0, 0.7, 0.65)), name="left_wall")
    frame.visual(Box((1.6, 0.2, 1.2)), origin=Origin(xyz=(0.0, -0.7, 0.65)), name="right_wall")
    frame.visual(Box((1.6, 1.6, 0.1)), origin=Origin(xyz=(0.0, 0.0, 1.3)), name="top_wall")
    
    model.articulation(
        "base_to_frame",
        ArticulationType.FIXED,
        parent=base,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    column = model.part("column")
    column.visual(Cylinder(radius=0.1, length=0.75), origin=Origin(xyz=(0.0, 0.0, 0.425)), name="column_body")
    
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(Cylinder(radius=0.12, length=0.2), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="rotor_hub")

    model.articulation(
        "column_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.9)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    arm_length = 0.46
    arm_radius = 0.02
    # Embed the arms slightly into the rotor hub by starting at r=0.11 instead of 0.12
    arm_center = 0.11 + arm_length / 2

    # Arm 0
    arm_0 = model.part("arm_0")
    arm_0.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(xyz=(arm_center, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="arm_0_tube"
    )
    model.articulation(
        "rotor_to_arm_0",
        ArticulationType.FIXED,
        parent=rotor,
        child=arm_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Arm 1
    arm_1 = model.part("arm_1")
    arm_1.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(xyz=(arm_center, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="arm_1_tube"
    )
    model.articulation(
        "rotor_to_arm_1",
        ArticulationType.FIXED,
        parent=rotor,
        child=arm_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
    )

    # Arm 2
    arm_2 = model.part("arm_2")
    arm_2.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(xyz=(arm_center, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="arm_2_tube"
    )
    model.articulation(
        "rotor_to_arm_2",
        ArticulationType.FIXED,
        parent=rotor,
        child=arm_2,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 4.0 * math.pi / 3.0)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    for i in range(3):
        ctx.allow_overlap(
            "rotor",
            f"arm_{i}",
            reason="The arms are embedded into the rotor hub for structural support.",
        )
        ctx.expect_overlap(f"arm_{i}", "rotor", axes="xy", min_overlap=0.005)
    
    ctx.expect_gap("rotor", "column", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_overlap("rotor", "column", axes="xy", name="rotor centered on column")

    return ctx.report()

object_model = build_object_model()