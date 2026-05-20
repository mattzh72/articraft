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
    model = ArticulatedObject(name="slide_chain")

    # 1. Fixed base rail
    base = model.part("base")
    base.visual(
        Box((1.0, 0.1, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_rail",
    )

    # 2. Carriage (slides on base)
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.2, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="carriage_plate",
    )
    # Guides wrapping the base rail sides
    carriage.visual(
        Box((0.2, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.06, -0.015)),
        name="carriage_guide_left",
    )
    carriage.visual(
        Box((0.2, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.06, -0.015)),
        name="carriage_guide_right",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.35, upper=0.35),
    )

    # 3. Arm (hinged on carriage)
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="arm_hub",
    )
    arm.visual(
        Box((0.5, 0.06, 0.04)),
        origin=Origin(xyz=(0.25, 0.0, 0.02)),
        name="arm_beam",
    )

    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.57, upper=1.57),
    )

    # 4. Tip slide (slides on arm)
    tip = model.part("tip")
    tip.visual(
        Box((0.1, 0.1, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="tip_plate",
    )
    # Guides wrapping the arm beam sides
    tip.visual(
        Box((0.1, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.04, -0.015)),
        name="tip_guide_left",
    )
    tip.visual(
        Box((0.1, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.04, -0.015)),
        name="tip_guide_right",
    )

    model.articulation(
        "arm_to_tip",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tip,
        origin=Origin(xyz=(0.1, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    tip = object_model.get_part("tip")

    # Check that plates sit exactly on their parents
    ctx.expect_gap(carriage, base, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="carriage_plate", negative_elem="base_rail")
    ctx.expect_gap(arm, carriage, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="arm_hub", negative_elem="carriage_plate")
    ctx.expect_gap(tip, arm, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="tip_plate", negative_elem="arm_beam")

    # The guides should wrap the rails
    ctx.expect_contact(carriage, base, elem_a="carriage_guide_left", elem_b="base_rail")
    ctx.expect_contact(carriage, base, elem_a="carriage_guide_right", elem_b="base_rail")
    ctx.expect_contact(tip, arm, elem_a="tip_guide_left", elem_b="arm_beam")
    ctx.expect_contact(tip, arm, elem_a="tip_guide_right", elem_b="arm_beam")

    return ctx.report()


object_model = build_object_model()
