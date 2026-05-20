from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_laptop_stand")

    graphite = model.material("graphite_powdercoat", rgba=(0.08, 0.085, 0.09, 1.0))
    tray_gray = model.material("dark_anodized_tray", rgba=(0.18, 0.19, 0.20, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    # Object frame: +X rearward, -X toward the user/front retaining lips,
    # +Y left, +Z upward.
    base_depth = 0.48
    base_width = 0.36
    base_thickness = 0.018
    lower_x = -0.10
    lower_z = 0.060

    arm_y = 0.168
    arm_dx = 0.150
    arm_dz = 0.150
    arm_length = math.hypot(arm_dx, arm_dz)
    arm_angle = math.atan2(arm_dz, arm_dx)
    axle_radius = 0.009
    axle_length = 0.380

    base = model.part("base")
    base.visual(
        Box((base_depth, base_width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=graphite,
        name="base_plate",
    )
    pillow_height = lower_z - axle_radius - base_thickness
    for pillow_name, y in (("lower_pillow_0", -0.115), ("lower_pillow_1", 0.115)):
        base.visual(
            Box((0.055, 0.038, pillow_height)),
            origin=Origin(xyz=(lower_x, y, base_thickness + pillow_height / 2.0)),
            material=graphite,
            name=pillow_name,
        )
    for foot_name, x, y in (
        ("rubber_foot_0", -0.185, -0.130),
        ("rubber_foot_1", -0.185, 0.130),
        ("rubber_foot_2", 0.185, -0.130),
        ("rubber_foot_3", 0.185, 0.130),
    ):
        base.visual(
            Box((0.055, 0.040, 0.004)),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=foot_name,
        )
    support_arms = model.part("support_arms")
    support_arms.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="lower_axle",
    )
    support_arms.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(xyz=(arm_dx, 0.0, arm_dz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="upper_axle",
    )
    for arm_name, y in (("side_arm_0", -arm_y), ("side_arm_1", arm_y)):
        support_arms.visual(
            Box((arm_length, 0.018, 0.020)),
            origin=Origin(
                xyz=(arm_dx / 2.0, y, arm_dz / 2.0),
                rpy=(0.0, -arm_angle, 0.0),
            ),
            material=aluminum,
            name=arm_name,
        )

    tray = model.part("tray")
    plate_center_z = 0.041
    plate_depth = 0.340
    plate_width = 0.300
    tray.visual(
        Box((plate_depth, plate_width, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, plate_center_z)),
        material=tray_gray,
        name="tray_plate",
    )
    # Two low bearing blocks tie the rigid tray to the upper horizontal pivot.
    for bearing_name, y in (("upper_bearing_0", -0.132), ("upper_bearing_1", 0.132)):
        tray.visual(
            Box((0.058, 0.034, plate_center_z - 0.006 - axle_radius)),
            origin=Origin(
                xyz=(0.0, y, axle_radius + (plate_center_z - 0.006 - axle_radius) / 2.0)
            ),
            material=tray_gray,
            name=bearing_name,
        )
    # Two separate front lips keep the laptop from sliding forward.
    for lip_name, y in (("front_lip_0", -0.075), ("front_lip_1", 0.075)):
        tray.visual(
            Box((0.026, 0.070, 0.040)),
            origin=Origin(xyz=(-0.150, y, 0.067)),
            material=tray_gray,
            name=lip_name,
        )
    for pad_name, y in (("rubber_pad_0", -0.055), ("rubber_pad_1", 0.055)):
        tray.visual(
            Box((0.220, 0.020, 0.004)),
            origin=Origin(xyz=(0.035, y, 0.049)),
            material=rubber,
            name=pad_name,
        )

    drive = model.articulation(
        "base_to_support_arms",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support_arms,
        origin=Origin(xyz=(lower_x, 0.0, lower_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    model.articulation(
        "support_arms_to_tray",
        ArticulationType.REVOLUTE,
        parent=support_arms,
        child=tray,
        origin=Origin(xyz=(arm_dx, 0.0, arm_dz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.55),
        mimic=Mimic(joint=drive.name, multiplier=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    support_arms = object_model.get_part("support_arms")
    tray = object_model.get_part("tray")
    drive = object_model.get_articulation("base_to_support_arms")

    ctx.expect_gap(
        support_arms,
        base,
        axis="z",
        positive_elem="lower_axle",
        negative_elem="lower_pillow_0",
        min_gap=-1e-6,
        max_gap=0.002,
        name="lower axle is seated on a base pillow",
    )
    ctx.expect_gap(
        tray,
        support_arms,
        axis="z",
        positive_elem="upper_bearing_0",
        negative_elem="upper_axle",
        min_gap=-1e-6,
        max_gap=0.002,
        name="tray bearing is seated on upper pivot",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_plate",
        negative_elem="base_plate",
        min_gap=0.18,
        name="tray is elevated over the desktop base",
    )

    left_arm_aabb = ctx.part_element_world_aabb(support_arms, elem="side_arm_1")
    right_arm_aabb = ctx.part_element_world_aabb(support_arms, elem="side_arm_0")
    if left_arm_aabb is not None and right_arm_aabb is not None:
        left_center_y = (left_arm_aabb[0][1] + left_arm_aabb[1][1]) / 2.0
        right_center_y = (right_arm_aabb[0][1] + right_arm_aabb[1][1]) / 2.0
        ctx.check(
            "side arms are symmetric counterparts",
            abs(left_center_y + right_center_y) < 0.002,
            details=f"left_y={left_center_y}, right_y={right_center_y}",
        )
    else:
        ctx.fail("side arms are symmetric counterparts", "side arm AABBs were unavailable")

    for lip_name in ("front_lip_0", "front_lip_1"):
        lip_aabb = ctx.part_element_world_aabb(tray, elem=lip_name)
        plate_aabb = ctx.part_element_world_aabb(tray, elem="tray_plate")
        if lip_aabb is not None and plate_aabb is not None:
            ctx.check(
                f"{lip_name} rises above tray surface",
                lip_aabb[1][2] > plate_aabb[1][2] + 0.030,
                details=f"lip={lip_aabb}, plate={plate_aabb}",
            )
        else:
            ctx.fail(f"{lip_name} rises above tray surface", "element AABBs were unavailable")

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({drive: 0.55}):
        raised_pos = ctx.part_world_position(tray)
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            positive_elem="tray_plate",
            negative_elem="base_plate",
            min_gap=0.23,
            name="raised tray clears the base",
        )
        ctx.expect_gap(
            tray,
            support_arms,
            axis="z",
            positive_elem="upper_bearing_0",
            negative_elem="upper_axle",
            min_gap=-1e-6,
            max_gap=0.002,
            name="upper pivot stays aligned at raised height",
        )
    ctx.check(
        "tray rises when support arms rotate",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.040,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
