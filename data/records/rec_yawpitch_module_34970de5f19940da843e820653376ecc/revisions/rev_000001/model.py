from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pedestal_yaw_pitch_fixture")

    dark_steel = Material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.48, 0.50, 0.51, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bearing_black = Material("bearing_black", rgba=(0.02, 0.025, 0.028, 1.0))
    cradle_blue = Material("cradle_blue", rgba=(0.05, 0.18, 0.38, 1.0))
    rubber_pad = Material("rubber_pad", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.305),
        origin=Origin(xyz=(0.0, 0.0, 0.1875)),
        material=satin_steel,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3575)),
        material=bearing_black,
        name="top_bearing",
    )
    for i, (x, y) in enumerate(
        ((0.115, 0.115), (-0.115, 0.115), (-0.115, -0.115), (0.115, -0.115))
    ):
        base.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=black_rubber,
            name=f"foot_{i}",
        )

    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=0.130, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_steel,
        name="turntable_disk",
    )
    platform.visual(
        Box((0.110, 0.205, 0.035)),
        origin=Origin(xyz=(0.018, 0.0, 0.055)),
        material=satin_steel,
        name="fork_saddle",
    )
    for i, y, arm_name, collar_name in (
        (0, -0.085, "fork_arm_0", "bearing_collar_0"),
        (1, 0.085, "fork_arm_1", "bearing_collar_1"),
    ):
        platform.visual(
            Box((0.105, 0.030, 0.240)),
            origin=Origin(xyz=(0.018, y, 0.156)),
            material=satin_steel,
            name=arm_name,
        )
        collar_y = -0.109 if y < 0.0 else 0.109
        platform.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(xyz=(0.018, collar_y, 0.185), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name=collar_name,
        )
    platform.visual(
        Box((0.105, 0.030, 0.010)),
        origin=Origin(xyz=(0.098, 0.0, 0.045)),
        material=bearing_black,
        name="yaw_index_tab",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Box((0.220, 0.095, 0.065)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=cradle_blue,
        name="cradle_body",
    )
    cradle.visual(
        Cylinder(radius=0.027, length=0.210),
        origin=Origin(xyz=(0.050, 0.0, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_pad,
        name="tool_channel",
    )
    cradle.visual(
        Box((0.150, 0.082, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, 0.049)),
        material=rubber_pad,
        name="upper_clamp",
    )
    for i, y, trunnion_name in (
        (0, -0.072, "trunnion_0"),
        (1, 0.072, "trunnion_1"),
    ):
        cradle.visual(
            Cylinder(radius=0.016, length=0.054),
            origin=Origin(xyz=(0.018, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=trunnion_name,
        )

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "platform_to_cradle",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=cradle,
        origin=Origin(xyz=(0.018, 0.0, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("base_to_platform")
    pitch = object_model.get_articulation("platform_to_cradle")

    ctx.allow_overlap(
        platform,
        cradle,
        elem_a="fork_arm_0",
        elem_b="trunnion_0",
        reason="The cradle trunnion is intentionally captured through the first fork cheek as the pitch axle.",
    )
    ctx.allow_overlap(
        platform,
        cradle,
        elem_a="fork_arm_1",
        elem_b="trunnion_1",
        reason="The cradle trunnion is intentionally captured through the second fork cheek as the pitch axle.",
    )

    ctx.expect_contact(
        platform,
        base,
        elem_a="turntable_disk",
        elem_b="top_bearing",
        contact_tol=1e-4,
        name="turntable rests on top bearing",
    )
    for i in (0, 1):
        ctx.expect_within(
            cradle,
            platform,
            axes="xz",
            inner_elem=f"trunnion_{i}",
            outer_elem=f"fork_arm_{i}",
            margin=0.0,
            name=f"trunnion_{i} centered in fork cheek",
        )
        ctx.expect_overlap(
            cradle,
            platform,
            axes="y",
            elem_a=f"trunnion_{i}",
            elem_b=f"fork_arm_{i}",
            min_overlap=0.020,
            name=f"trunnion_{i} retained through fork cheek",
        )

    ctx.check(
        "yaw joint is vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "pitch joint is horizontal",
        pitch.axis == (0.0, 1.0, 0.0),
        details=f"axis={pitch.axis}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[j] + high[j]) * 0.5 for j in range(3))

    rest_body = aabb_center(ctx.part_element_world_aabb(cradle, elem="cradle_body"))
    with ctx.pose({pitch: 0.60}):
        pitched_body = aabb_center(ctx.part_element_world_aabb(cradle, elem="cradle_body"))
    ctx.check(
        "positive pitch tips the forward cradle downward",
        rest_body is not None
        and pitched_body is not None
        and pitched_body[2] < rest_body[2] - 0.010,
        details=f"rest={rest_body}, pitched={pitched_body}",
    )

    rest_tab = aabb_center(ctx.part_element_world_aabb(platform, elem="yaw_index_tab"))
    with ctx.pose({yaw: 1.0}):
        yawed_tab = aabb_center(ctx.part_element_world_aabb(platform, elem="yaw_index_tab"))
    ctx.check(
        "yaw rotates the lower platform about the column",
        rest_tab is not None
        and yawed_tab is not None
        and yawed_tab[1] > rest_tab[1] + 0.050,
        details=f"rest={rest_tab}, yawed={yawed_tab}",
    )

    return ctx.report()


object_model = build_object_model()
