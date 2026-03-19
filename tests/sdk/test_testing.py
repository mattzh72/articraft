from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
)
from sdk import (
    TestContext as SDKTestContext,
)


def _build_joint_origin_model(*, joint_z: float) -> ArticulatedObject:
    model = ArticulatedObject(name="joint_origin_tolerance")

    base = model.part("base")
    base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    child = model.part("child")
    child.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    model.articulation(
        "base_to_child",
        ArticulationType.REVOLUTE,
        parent=base,
        child=child,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.0),
    )
    return model


def test_articulation_origin_tolerance_is_truncated_to_three_decimals() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.115), geometry_source="visual")

    assert ctx.check_articulation_origin_near_geometry(tol=0.0159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.015)",)


def test_articulation_origin_tolerance_is_capped_at_point_one_five() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.25), geometry_source="visual")

    assert ctx.check_articulation_origin_near_geometry(tol=0.159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.15)",)
