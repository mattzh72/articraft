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
    model = ArticulatedObject(name="travel_laptop_stand")

    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_depth = 0.215
    base_width = 0.255
    base_thickness = 0.004
    hinge_radius = 0.006
    hinge_x = -0.095
    hinge_z = 0.010
    lower_pivot_x = 0.090
    lower_pivot_z = 0.014

    platform_depth = 0.232
    platform_width = 0.265
    platform_thickness = 0.005
    platform_rest_angle = 0.50
    upper_pivot_x = 0.160
    upper_pivot_z = -0.010

    leg_length = 0.075
    leg_rest_angle = 1.01
    pivot_radius = 0.007
    lug_length = 0.010
    eye_length = 0.024
    lug_y = 0.017

    base = model.part("base")
    base.visual(
        Box((base_depth, base_width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=dark_gray,
        name="panel",
    )
    base.visual(
        Box((0.070, 0.160, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=black,
        name="center_rib",
    )
    base.visual(
        Box((0.018, 0.180, 0.008)),
        origin=Origin(xyz=(hinge_x + 0.015, 0.0, 0.008)),
        material=black,
        name="hinge_block",
    )
    base.visual(
        Cylinder(radius=hinge_radius, length=0.055),
        origin=Origin(
            xyz=(hinge_x, -0.082, hinge_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="hinge_barrel_0",
    )
    base.visual(
        Cylinder(radius=hinge_radius, length=0.055),
        origin=Origin(
            xyz=(hinge_x, 0.082, hinge_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="hinge_barrel_1",
    )
    base.visual(
        Box((0.028, 0.070, 0.016)),
        origin=Origin(xyz=(lower_pivot_x + 0.021, 0.0, 0.010)),
        material=black,
        name="rear_bridge",
    )
    base.visual(
        Cylinder(radius=pivot_radius, length=lug_length),
        origin=Origin(
            xyz=(lower_pivot_x, -lug_y, lower_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="lower_lug_0",
    )
    base.visual(
        Cylinder(radius=pivot_radius, length=lug_length),
        origin=Origin(
            xyz=(lower_pivot_x, lug_y, lower_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="lower_lug_1",
    )

    foot_specs = (
        ("front_foot_0", (-0.060, -0.085)),
        ("front_foot_1", (-0.060, 0.085)),
        ("rear_foot_0", (0.070, -0.085)),
        ("rear_foot_1", (0.070, 0.085)),
    )
    for name, (x_pos, y_pos) in foot_specs:
        base.visual(
            Box((0.036, 0.020, 0.002)),
            origin=Origin(xyz=(x_pos, y_pos, -0.001)),
            material=rubber,
            name=name,
        )

    platform = model.part("platform")
    platform.visual(
        Box((platform_depth, platform_width, platform_thickness)),
        origin=Origin(xyz=(0.120, 0.0, 0.0085)),
        material=dark_gray,
        name="deck",
    )
    platform.visual(
        Box((0.090, 0.215, 0.012)),
        origin=Origin(xyz=(0.100, 0.0, 0.000)),
        material=black,
        name="spine",
    )
    platform.visual(
        Cylinder(radius=hinge_radius, length=0.102),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="hinge_barrel",
    )
    platform.visual(
        Box((0.030, 0.104, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.004)),
        material=black,
        name="hinge_web",
    )
    platform.visual(
        Box((0.014, 0.190, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, 0.014)),
        material=black,
        name="front_stop",
    )
    platform.visual(
        Box((0.028, 0.060, 0.009)),
        origin=Origin(xyz=(upper_pivot_x + 0.020, 0.0, 0.0015)),
        material=black,
        name="upper_bracket",
    )
    platform.visual(
        Cylinder(radius=pivot_radius, length=lug_length),
        origin=Origin(
            xyz=(upper_pivot_x, -lug_y, upper_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="upper_lug_0",
    )
    platform.visual(
        Cylinder(radius=pivot_radius, length=lug_length),
        origin=Origin(
            xyz=(upper_pivot_x, lug_y, upper_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="upper_lug_1",
    )
    platform.visual(
        Box((0.018, 0.010, 0.008)),
        origin=Origin(xyz=(upper_pivot_x + 0.010, -lug_y, -0.005)),
        material=black,
        name="upper_connector_0",
    )
    platform.visual(
        Box((0.018, 0.010, 0.008)),
        origin=Origin(xyz=(upper_pivot_x + 0.010, lug_y, -0.005)),
        material=black,
        name="upper_connector_1",
    )
    platform.visual(
        Box((0.012, 0.230, 0.010)),
        origin=Origin(xyz=(platform_depth - 0.006, 0.0, 0.014)),
        material=black,
        name="rear_edge",
    )

    support_leg = model.part("support_leg")
    support_leg.visual(
        Cylinder(radius=0.0065, length=eye_length),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="lower_eye",
    )
    support_leg.visual(
        Box((0.070, 0.022, 0.010)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=black,
        name="blade",
    )
    support_leg.visual(
        Cylinder(radius=0.0065, length=eye_length),
        origin=Origin(
            xyz=(-leg_length, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="upper_eye",
    )
    support_leg.visual(
        Box((0.028, 0.024, 0.008)),
        origin=Origin(xyz=(-leg_length + 0.012, 0.0, 0.0)),
        material=dark_gray,
        name="upper_pad",
    )

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(0.0, -platform_rest_angle, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.30,
            upper=0.32,
        ),
    )
    model.articulation(
        "base_to_support_leg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support_leg,
        origin=Origin(xyz=(lower_pivot_x, 0.0, lower_pivot_z), rpy=(0.0, leg_rest_angle, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.58,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    support_leg = object_model.get_part("support_leg")
    platform_hinge = object_model.get_articulation("base_to_platform")
    leg_pivot = object_model.get_articulation("base_to_support_leg")

    ctx.expect_contact(
        support_leg,
        base,
        elem_a="lower_eye",
        elem_b="lower_lug_0",
        name="support leg seats against the first lower lug",
    )
    ctx.expect_contact(
        support_leg,
        base,
        elem_a="lower_eye",
        elem_b="lower_lug_1",
        name="support leg seats against the second lower lug",
    )
    ctx.expect_contact(
        support_leg,
        platform,
        elem_a="upper_eye",
        elem_b="upper_lug_0",
        name="support leg seats against the first upper lug",
    )
    ctx.expect_contact(
        support_leg,
        platform,
        elem_a="upper_eye",
        elem_b="upper_lug_1",
        name="support leg seats against the second upper lug",
    )
    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="rear_edge",
        negative_elem="panel",
        min_gap=0.070,
        name="rear edge of the platform is visibly raised above the base",
    )

    rest_rear = ctx.part_element_world_aabb(platform, elem="rear_edge")
    rest_leg = ctx.part_element_world_aabb(support_leg, elem="upper_eye")

    with ctx.pose({platform_hinge: 0.16, leg_pivot: 0.12}):
        high_rear = ctx.part_element_world_aabb(platform, elem="rear_edge")
        high_leg = ctx.part_element_world_aabb(support_leg, elem="upper_eye")
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem="rear_edge",
            negative_elem="panel",
            min_gap=0.100,
            name="higher setting lifts the rear edge farther above the base",
        )

    with ctx.pose({platform_hinge: -0.20, leg_pivot: -0.28}):
        low_rear = ctx.part_element_world_aabb(platform, elem="rear_edge")
        low_leg = ctx.part_element_world_aabb(support_leg, elem="upper_eye")
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem="rear_edge",
            negative_elem="panel",
            min_gap=0.035,
            name="lower setting still keeps the rear edge off the base",
        )

    ctx.check(
        "platform offers distinct low and high working heights",
        all(aabb is not None for aabb in (rest_rear, high_rear, low_rear))
        and low_rear[1][2] + 0.025 < rest_rear[1][2] < high_rear[1][2] - 0.025,
        details=f"low={low_rear}, rest={rest_rear}, high={high_rear}",
    )
    ctx.check(
        "support leg swings upward as the stand height increases",
        all(aabb is not None for aabb in (rest_leg, high_leg, low_leg))
        and low_leg[1][2] + 0.012 < rest_leg[1][2] < high_leg[1][2] - 0.003,
        details=f"low={low_leg}, rest={rest_leg}, high={high_leg}",
    )

    return ctx.report()


object_model = build_object_model()
