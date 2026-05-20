from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elbow_arm_module")

    powder_black = Material("powder_black", color=(0.02, 0.022, 0.025, 1.0))
    cast_gray = Material("cast_gray", color=(0.42, 0.44, 0.46, 1.0))
    arm_orange = Material("arm_orange", color=(0.95, 0.38, 0.08, 1.0))
    dark_rubber = Material("dark_rubber", color=(0.01, 0.012, 0.014, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.72, 0.72, 0.68, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.23, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=powder_black,
        name="round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.47),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=cast_gray,
        name="column",
    )
    pedestal.visual(
        Box((0.16, 0.33, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=cast_gray,
        name="yoke_bridge",
    )
    for suffix, y in (("0", -0.13), ("1", 0.13)):
        pedestal.visual(
            Box((0.145, 0.055, 0.210)),
            origin=Origin(xyz=(0.0, y, 0.625)),
            material=cast_gray,
            name=f"shoulder_cheek_{suffix}",
        )
        pedestal.visual(
            Cylinder(radius=0.052, length=0.026),
            origin=Origin(xyz=(0.0, y * 1.24, 0.625), rpy=(math.pi / 2, 0.0, 0.0)),
            material=brushed_steel,
            name=f"shoulder_cap_{suffix}",
        )
    pedestal.visual(
        Cylinder(radius=0.026, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.625), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="shoulder_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.160),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=arm_orange,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.075, 0.140, 0.040)),
        origin=Origin(xyz=(0.0875, 0.0, 0.0)),
        material=arm_orange,
        name="shoulder_web",
    )
    for suffix, y in (("0", -0.085), ("1", 0.085)):
        upper_arm.visual(
            Box((0.49, 0.040, 0.055)),
            origin=Origin(xyz=(0.335, y, 0.0)),
            material=arm_orange,
            name=f"upper_rail_{suffix}",
        )
        upper_arm.visual(
            Box((0.120, 0.050, 0.070)),
            origin=Origin(xyz=(0.610, math.copysign(0.105, y), 0.0)),
            material=arm_orange,
            name=f"elbow_fork_{suffix}",
        )
        upper_arm.visual(
            Cylinder(radius=0.075, length=0.050),
            origin=Origin(xyz=(0.660, math.copysign(0.112, y), 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=arm_orange,
            name=f"elbow_hub_{suffix}",
        )
    upper_arm.visual(
        Cylinder(radius=0.023, length=0.245),
        origin=Origin(xyz=(0.660, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.060, length=0.130),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=arm_orange,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.505, 0.070, 0.055)),
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        material=arm_orange,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.045, 0.225, 0.180)),
        origin=Origin(xyz=(0.570, 0.0, 0.0)),
        material=cast_gray,
        name="end_plate",
    )
    for iy, y in enumerate((-0.066, 0.066)):
        for iz, z in enumerate((-0.050, 0.050)):
            forearm.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(0.596, y, z), rpy=(0.0, math.pi / 2, 0.0)),
                material=dark_rubber,
                name=f"plate_hole_{iy}_{iz}",
            )

    travel = MotionLimits(
        effort=80.0,
        velocity=1.6,
        lower=math.radians(-120.0),
        upper=math.radians(135.0),
    )
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=travel,
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.660, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=travel,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        pedestal,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The steel shoulder pin is intentionally captured through the rotating shoulder hub.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The steel elbow pin is intentionally captured through the forearm elbow hub.",
    )

    for joint in (shoulder, elbow):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has broad elbow-arm travel",
            abs(limits.lower - math.radians(-120.0)) < 1e-6
            and abs(limits.upper - math.radians(135.0)) < 1e-6,
            details=f"limits={limits}",
        )
        ctx.check(
            f"{joint.name} axis is horizontal",
            tuple(round(v, 6) for v in joint.axis) == (0.0, -1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_within(
        pedestal,
        upper_arm,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.001,
        name="shoulder pin is concentric in hub",
    )
    ctx.expect_overlap(
        pedestal,
        upper_arm,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.14,
        name="shoulder pin spans hub width",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.001,
        name="elbow pin is concentric in hub",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.11,
        name="elbow pin spans hub width",
    )

    rest_upper = ctx.part_world_aabb(upper_arm)
    rest_forearm = ctx.part_world_aabb(forearm)
    with ctx.pose({shoulder: math.radians(45.0), elbow: math.radians(35.0)}):
        raised_upper = ctx.part_world_aabb(upper_arm)
        raised_forearm = ctx.part_world_aabb(forearm)
    ctx.check(
        "positive shoulder and elbow motion lift the serial arm",
        rest_upper is not None
        and rest_forearm is not None
        and raised_upper is not None
        and raised_forearm is not None
        and raised_upper[1][2] > rest_upper[1][2] + 0.15
        and raised_forearm[1][2] > rest_forearm[1][2] + 0.25,
        details=f"rest_upper={rest_upper}, raised_upper={raised_upper}, rest_forearm={rest_forearm}, raised_forearm={raised_forearm}",
    )

    return ctx.report()


object_model = build_object_model()
