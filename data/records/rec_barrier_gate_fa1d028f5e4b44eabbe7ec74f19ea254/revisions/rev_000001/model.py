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
    model = ArticulatedObject(name="parking_barrier")

    housing = model.part("housing")
    # Base plate
    housing.visual(
        Box((0.5, 0.5, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_plate",
    )
    # Main column
    housing.visual(
        Box((0.3, 0.3, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="column",
    )
    # Top cap
    housing.visual(
        Box((0.32, 0.32, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        name="cap",
    )
    # Hinge pin extending to the side (+Y)
    # Column is [-0.15, 0.15] in Y.
    # Hinge pin centered at Y=0.2, length 0.15 -> spans [0.125, 0.275]
    housing.visual(
        Cylinder(radius=0.04, length=0.15),
        origin=Origin(xyz=(0.0, 0.2, 0.9), rpy=(math.pi / 2, 0.0, 0.0)),
        name="hinge_pin",
    )

    boom = model.part("boom")
    # Boom arm extending from X=-0.5 to X=2.5
    # Centered at Y=0.23, width 0.05 -> spans [0.205, 0.255]
    boom.visual(
        Box((3.0, 0.05, 0.1)),
        origin=Origin(xyz=(1.0, 0.0, 0.0)),
        name="arm",
    )
    # Counterweight at the back
    # Centered at X=-0.3, width 0.08 -> spans Y [0.19, 0.27]
    boom.visual(
        Box((0.4, 0.08, 0.2)),
        origin=Origin(xyz=(-0.3, 0.0, 0.0)),
        name="counterweight",
    )

    model.articulation(
        "housing_to_boom",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=boom,
        # Pivot point
        origin=Origin(xyz=(0.0, 0.23, 0.9)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=math.pi / 2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    boom = object_model.get_part("boom")
    joint = object_model.get_articulation("housing_to_boom")

    # The boom arm is intentionally mounted directly onto the hinge pin.
    ctx.allow_overlap(
        boom,
        housing,
        elem_a="arm",
        elem_b="hinge_pin",
        reason="The boom arm mounts directly over the hinge pin.",
    )
    ctx.allow_overlap(
        boom,
        housing,
        elem_a="counterweight",
        elem_b="hinge_pin",
        reason="The counterweight also surrounds the hinge pin.",
    )

    # At rest (closed), the boom overlaps the housing in X
    ctx.expect_overlap(housing, boom, axes="x", min_overlap=0.1, name="boom overlaps housing in X when down")
    
    # Check gap in Y to ensure boom is mounted on the side
    ctx.expect_gap(boom, housing, axis="y", min_gap=0.0, max_gap=0.1, positive_elem="arm", negative_elem="column", name="boom is mounted on side")

    with ctx.pose({joint: math.pi / 2}):
        boom_aabb = ctx.part_world_aabb(boom)
        ctx.check(
            "boom_raised_upwards",
            boom_aabb is not None and boom_aabb[1][2] > 2.0,
            "Boom should be raised upwards, not downwards",
        )

    return ctx.report()


object_model = build_object_model()
