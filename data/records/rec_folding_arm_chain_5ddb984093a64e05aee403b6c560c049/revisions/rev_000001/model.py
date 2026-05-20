from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_folding_arm")

    base = model.part("base_plate")
    base.visual(
        Box((0.20, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_visual",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Box((0.30, 0.06, 0.04)),
        origin=Origin(xyz=(0.15, 0.0, 0.02)),
        name="link_1_visual",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        Box((0.30, 0.06, 0.04)),
        origin=Origin(xyz=(0.15, 0.0, 0.02)),
        name="link_2_visual",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        Box((0.30, 0.04, 0.02)),
        origin=Origin(xyz=(0.15, 0.0, 0.01)),
        name="link_3_visual",
    )
    link_3.visual(
        Box((0.05, 0.04, 0.02)),
        origin=Origin(xyz=(0.325, 0.0, 0.01)),
        name="end_tab",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-3.14, upper=3.14),
    )

    model.articulation(
        "link_1_to_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.30, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-3.14, upper=3.14),
    )

    model.articulation(
        "link_2_to_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.30, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-3.14, upper=3.14),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")

    ctx.expect_gap(link_1, base, axis="z", min_gap=0.0, max_gap=0.001)
    ctx.expect_gap(link_2, link_1, axis="z", min_gap=0.0, max_gap=0.001)
    ctx.expect_gap(link_3, link_2, axis="z", min_gap=0.0, max_gap=0.001)

    return ctx.report()

object_model = build_object_model()
