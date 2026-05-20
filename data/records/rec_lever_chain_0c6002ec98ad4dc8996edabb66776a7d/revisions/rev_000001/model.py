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
    model = ArticulatedObject(name="boxed_lever_chain")

    # Base lug
    base_lug = model.part("base_lug")
    base_lug.visual(
        Box((0.1, 0.1, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="base_block",
    )
    base_lug.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        name="base_boss",
    )
    base_lug.visual(
        Cylinder(radius=0.015, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        name="base_pin",
    )
    base_lug.visual(
        Cylinder(radius=0.02, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.1425)),
        name="base_cap",
    )

    # Lever 0
    lever_0 = model.part("lever_0")
    lever_0.visual(
        Box((0.36, 0.06, 0.02)),
        origin=Origin(xyz=(0.15, 0.0, 0.01)),
        name="lever_0_bar",
    )
    lever_0.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.3, 0.0, 0.03)),
        name="lever_0_boss",
    )
    lever_0.visual(
        Cylinder(radius=0.015, length=0.02),
        origin=Origin(xyz=(0.3, 0.0, 0.05)),
        name="lever_0_pin",
    )
    lever_0.visual(
        Cylinder(radius=0.02, length=0.005),
        origin=Origin(xyz=(0.3, 0.0, 0.0625)),
        name="lever_0_cap",
    )

    model.articulation(
        "base_to_lever_0",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=lever_0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.0, upper=2.0),
    )

    # Lever 1
    lever_1 = model.part("lever_1")
    lever_1.visual(
        Box((0.36, 0.06, 0.02)),
        origin=Origin(xyz=(0.15, 0.0, 0.01)),
        name="lever_1_bar",
    )
    lever_1.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.3, 0.0, 0.03)),
        name="lever_1_boss",
    )
    lever_1.visual(
        Cylinder(radius=0.015, length=0.02),
        origin=Origin(xyz=(0.3, 0.0, 0.05)),
        name="lever_1_pin",
    )
    lever_1.visual(
        Cylinder(radius=0.02, length=0.005),
        origin=Origin(xyz=(0.3, 0.0, 0.0625)),
        name="lever_1_cap",
    )

    model.articulation(
        "lever_0_to_lever_1",
        ArticulationType.REVOLUTE,
        parent=lever_0,
        child=lever_1,
        origin=Origin(xyz=(0.3, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.5, upper=2.5),
    )

    # Lever 2
    lever_2 = model.part("lever_2")
    lever_2.visual(
        Box((0.36, 0.06, 0.02)),
        origin=Origin(xyz=(0.15, 0.0, 0.01)),
        name="lever_2_bar",
    )
    lever_2.visual(
        Box((0.04, 0.02, 0.02)),
        origin=Origin(xyz=(0.35, 0.0, 0.01)),
        name="end_tab",
    )

    model.articulation(
        "lever_1_to_lever_2",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=lever_2,
        origin=Origin(xyz=(0.3, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.5, upper=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_lug = object_model.get_part("base_lug")
    lever_0 = object_model.get_part("lever_0")
    lever_1 = object_model.get_part("lever_1")
    lever_2 = object_model.get_part("lever_2")

    # Allow overlaps for the captured pins
    ctx.allow_overlap(
        base_lug,
        lever_0,
        elem_a="base_pin",
        elem_b="lever_0_bar",
        reason="The base pin is captured inside the lever 0 bar.",
    )
    ctx.allow_overlap(
        lever_0,
        lever_1,
        elem_a="lever_0_pin",
        elem_b="lever_1_bar",
        reason="The lever 0 pin is captured inside the lever 1 bar.",
    )
    ctx.allow_overlap(
        lever_1,
        lever_2,
        elem_a="lever_1_pin",
        elem_b="lever_2_bar",
        reason="The lever 1 pin is captured inside the lever 2 bar.",
    )

    # Verify seating and gaps
    ctx.expect_gap(lever_0, base_lug, axis="z", positive_elem="lever_0_bar", negative_elem="base_boss", max_penetration=0.0, max_gap=0.001)
    ctx.expect_gap(lever_1, lever_0, axis="z", positive_elem="lever_1_bar", negative_elem="lever_0_boss", max_penetration=0.0, max_gap=0.001)
    ctx.expect_gap(lever_2, lever_1, axis="z", positive_elem="lever_2_bar", negative_elem="lever_1_boss", max_penetration=0.0, max_gap=0.001)

    return ctx.report()


object_model = build_object_model()