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
    model = ArticulatedObject(name="three_branch_rotary_fixture")

    anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    hub_orange = Material("orange_hub_blocks", rgba=(0.95, 0.42, 0.12, 1.0))
    arm_blue = Material("blue_arms", rgba=(0.08, 0.23, 0.75, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.16, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=anodized,
        name="round_base",
    )
    mast.visual(
        Cylinder(radius=0.045, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=steel,
        name="central_mast",
    )
    mast.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=anodized,
        name="top_cap",
    )

    branch_specs = (
        ("arm_0", 0.0, 0.42, "pad"),
        ("arm_1", 2.0 * math.pi / 3.0, 0.61, "fork"),
        ("arm_2", 4.0 * math.pi / 3.0, 0.80, "pad"),
    )

    pivot_radius = 0.165
    for index, (arm_name, yaw, height, end_style) in enumerate(branch_specs):
        direction_x = math.cos(yaw)
        direction_y = math.sin(yaw)
        pivot = (pivot_radius * direction_x, pivot_radius * direction_y, height)

        mast.visual(
            Box((0.205, 0.075, 0.036)),
            origin=Origin(
                xyz=(0.080 * direction_x, 0.080 * direction_y, height - 0.038),
                rpy=(0.0, 0.0, yaw),
            ),
            material=hub_orange,
            name=f"hub_{index}_lower_web",
        )
        mast.visual(
            Box((0.130, 0.020, 0.115)),
            origin=Origin(
                xyz=(
                    pivot[0] - 0.014 * direction_x - 0.095 * math.sin(yaw),
                    pivot[1] - 0.014 * direction_y + 0.095 * math.cos(yaw),
                    height,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=hub_orange,
            name=f"hub_{index}_cheek_0",
        )
        mast.visual(
            Box((0.130, 0.020, 0.115)),
            origin=Origin(
                xyz=(
                    pivot[0] - 0.014 * direction_x + 0.095 * math.sin(yaw),
                    pivot[1] - 0.014 * direction_y - 0.095 * math.cos(yaw),
                    height,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=hub_orange,
            name=f"hub_{index}_cheek_1",
        )
        mast.visual(
            Box((0.034, 0.205, 0.105)),
            origin=Origin(
                xyz=(pivot[0] - 0.072 * direction_x, pivot[1] - 0.072 * direction_y, height),
                rpy=(0.0, 0.0, yaw),
            ),
            material=hub_orange,
            name=f"hub_{index}_rear_bridge",
        )

        arm = model.part(arm_name)
        arm.visual(
            Cylinder(radius=0.034, length=0.040),
            origin=Origin(),
            material=steel,
            name="pivot_barrel",
        )
        arm.visual(
            Box((0.540, 0.044, 0.034)),
            origin=Origin(xyz=(0.300, 0.0, 0.0)),
            material=arm_blue,
            name="branch_beam",
        )
        arm.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="root_pin_boss",
        )

        if end_style == "fork":
            arm.visual(
                Box((0.045, 0.140, 0.034)),
                origin=Origin(xyz=(0.565, 0.0, 0.0)),
                material=arm_blue,
                name="fork_crosshead",
            )
            arm.visual(
                Box((0.125, 0.028, 0.034)),
                origin=Origin(xyz=(0.640, 0.052, 0.0)),
                material=arm_blue,
                name="fork_tine_0",
            )
            arm.visual(
                Box((0.125, 0.028, 0.034)),
                origin=Origin(xyz=(0.640, -0.052, 0.0)),
                material=arm_blue,
                name="fork_tine_1",
            )
            arm.visual(
                Box((0.035, 0.022, 0.036)),
                origin=Origin(xyz=(0.705, 0.052, 0.0)),
                material=rubber,
                name="fork_tip_0",
            )
            arm.visual(
                Box((0.035, 0.022, 0.036)),
                origin=Origin(xyz=(0.705, -0.052, 0.0)),
                material=rubber,
                name="fork_tip_1",
            )
        else:
            arm.visual(
                Cylinder(radius=0.060, length=0.018),
                origin=Origin(xyz=(0.595, 0.0, -0.026)),
                material=rubber,
                name="round_end_pad",
            )
            arm.visual(
                Box((0.070, 0.080, 0.028)),
                origin=Origin(xyz=(0.552, 0.0, -0.004)),
                material=arm_blue,
                name="pad_neck",
            )

        model.articulation(
            f"mast_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=arm,
            origin=Origin(xyz=pivot, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-2.0 * math.pi / 3.0 / 2.0,
                upper=2.0 * math.pi / 3.0 / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arms = [object_model.get_part(f"arm_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"mast_to_arm_{i}") for i in range(3)]

    for i, joint in enumerate(joints):
        ctx.check(
            f"arm {i} has 120 degree travel",
            abs((joint.motion_limits.upper - joint.motion_limits.lower) - (2.0 * math.pi / 3.0)) < 1e-6,
            details=f"limits={joint.motion_limits}",
        )
        with ctx.pose({joint: joint.motion_limits.upper}):
            moved = ctx.part_world_position(arms[i])
        ctx.check(
            f"arm {i} upper pose remains articulated",
            moved is not None,
            details="pose query returned no world position",
        )

    ctx.expect_origin_gap(arms[1], arms[0], axis="z", min_gap=0.17, name="lower and middle hubs are vertically staggered")
    ctx.expect_origin_gap(arms[2], arms[1], axis="z", min_gap=0.17, name="middle and upper hubs are vertically staggered")

    return ctx.report()


object_model = build_object_model()
