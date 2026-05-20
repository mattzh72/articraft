from __future__ import annotations

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
    model = ArticulatedObject(name="carriage_block_gripper")

    body = model.part("body")
    body.visual(
        Box((0.16, 0.06, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="base_plate",
    )
    body.visual(
        Box((0.16, 0.02, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        name="guide_rail",
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        Box((0.04, 0.06, 0.01)),
        origin=Origin(xyz=(-0.05, 0.0, 0.025)),
        name="top_plate",
    )
    left_carriage.visual(
        Box((0.04, 0.02, 0.01)),
        origin=Origin(xyz=(-0.05, -0.02, 0.015)),
        name="front_skirt",
    )
    left_carriage.visual(
        Box((0.04, 0.02, 0.01)),
        origin=Origin(xyz=(-0.05, 0.02, 0.015)),
        name="rear_skirt",
    )
    left_carriage.visual(
        Box((0.02, 0.04, 0.02)),
        origin=Origin(xyz=(-0.04, 0.0, 0.04)),
        name="finger_base",
    )
    left_carriage.visual(
        Box((0.01, 0.02, 0.03)),
        origin=Origin(xyz=(-0.035, 0.0, 0.065)),
        name="finger_tip",
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        Box((0.04, 0.06, 0.01)),
        origin=Origin(xyz=(0.05, 0.0, 0.025)),
        name="top_plate",
    )
    right_carriage.visual(
        Box((0.04, 0.02, 0.01)),
        origin=Origin(xyz=(0.05, -0.02, 0.015)),
        name="front_skirt",
    )
    right_carriage.visual(
        Box((0.04, 0.02, 0.01)),
        origin=Origin(xyz=(0.05, 0.02, 0.015)),
        name="rear_skirt",
    )
    right_carriage.visual(
        Box((0.02, 0.04, 0.02)),
        origin=Origin(xyz=(0.04, 0.0, 0.04)),
        name="finger_base",
    )
    right_carriage.visual(
        Box((0.01, 0.02, 0.03)),
        origin=Origin(xyz=(0.035, 0.0, 0.065)),
        name="finger_tip",
    )

    model.articulation(
        "left_carriage_joint",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.03),
    )

    model.articulation(
        "right_carriage_joint",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    
    ctx.expect_gap(left_carriage, body, axis="z", positive_elem="top_plate", negative_elem="guide_rail", max_penetration=1e-5, max_gap=0.001)
    ctx.expect_gap(right_carriage, body, axis="z", positive_elem="top_plate", negative_elem="guide_rail", max_penetration=1e-5, max_gap=0.001)
    
    with ctx.pose(left_carriage_joint=0.03, right_carriage_joint=0.03):
        ctx.expect_gap(right_carriage, left_carriage, axis="x", positive_elem="finger_tip", negative_elem="finger_tip", max_penetration=1e-5, max_gap=0.001)

    return ctx.report()


object_model = build_object_model()
