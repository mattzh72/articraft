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
    model = ArticulatedObject(name="offset_rotary_stack")

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.20, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_body",
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        Cylinder(radius=0.20, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="lower_body",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    side_arm = model.part("side_arm")
    side_arm.visual(
        Box((0.05, 0.05, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        name="arm_body",
    )

    model.articulation(
        "lower_to_arm",
        ArticulationType.FIXED,
        parent=lower_platform,
        child=side_arm,
        origin=Origin(xyz=(0.15, 0.0, 0.02)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.08, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="upper_body",
    )

    model.articulation(
        "arm_to_upper",
        ArticulationType.REVOLUTE,
        parent=side_arm,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_platform")
    arm = object_model.get_part("side_arm")
    upper = object_model.get_part("upper_stage")

    ctx.expect_contact(lower, base, name="lower_platform_contacts_base")
    ctx.expect_contact(arm, lower, name="arm_contacts_lower_platform")
    ctx.expect_contact(upper, arm, name="upper_stage_contacts_arm")

    return ctx.report()

object_model = build_object_model()
