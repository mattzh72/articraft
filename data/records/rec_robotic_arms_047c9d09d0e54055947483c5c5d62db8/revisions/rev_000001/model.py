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
    model = ArticulatedObject(name="robotic_arm")

    # Base
    base = model.part("base")
    base.visual(
        Box((0.25, 0.25, 0.4)),
        origin=Origin(xyz=(0.0, -0.2, 0.2)),
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.12, height=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        name="shoulder_housing",
    )
    base.visual(
        Box((0.15, 0.2, 0.15)),
        origin=Origin(xyz=(0.0, -0.1, 0.325)),
        name="bracket",
    )

    # Upper Arm
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.12, height=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="base_flange",
    )
    upper_arm.visual(
        Box((0.1, 0.08, 0.4)),
        origin=Origin(xyz=(0.0, -0.05, 0.24)),
        name="upright",
    )
    upper_arm.visual(
        Cylinder(radius=0.07, height=0.1),
        origin=Origin(xyz=(0.0, -0.05, 0.44), rpy=(math.pi / 2, 0.0, 0.0)),
        name="elbow_barrel",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-3.14, upper=3.14),
    )

    # Forearm
    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.07, height=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.4, 0.06, 0.06)),
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
        name="link",
    )

    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.04, 0.44)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.5, upper=1.5),
    )

    # Wrist Head
    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.05, height=0.04),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="base_cylinder",
    )
    wrist_head.visual(
        Cylinder(radius=0.07, height=0.02),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.06, 0.08, 0.04)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        name="tool_block",
    )

    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.4, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        positive_elem="base_flange",
        negative_elem="shoulder_housing",
        max_gap=0.001,
        max_penetration=0.001,
    )

    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_barrel",
        max_gap=0.001,
        max_penetration=0.001,
    )

    ctx.expect_gap(
        wrist_head,
        forearm,
        axis="x",
        positive_elem="base_cylinder",
        negative_elem="link",
        max_gap=0.001,
        max_penetration=0.001,
    )

    return ctx.report()


object_model = build_object_model()
