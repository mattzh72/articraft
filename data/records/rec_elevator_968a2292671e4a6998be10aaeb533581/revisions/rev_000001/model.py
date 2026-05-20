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
    model = ArticulatedObject(name="single_post_vehicle_lift")

    blue = Material("painted_blue_steel", color=(0.05, 0.18, 0.55, 1.0))
    dark = Material("dark_safety_steel", color=(0.08, 0.09, 0.10, 1.0))
    yellow = Material("yellow_safety_arms", color=(0.95, 0.72, 0.05, 1.0))
    chrome = Material("polished_hydraulic_chrome", color=(0.78, 0.82, 0.84, 1.0))
    rubber = Material("black_rubber_grip", color=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.45, 1.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="floor_base",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=blue,
        name="outer_post",
    )
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.0, 0.175, 0.42, 0.07),
            (0.0, -0.175, 0.42, 0.07),
            (0.175, 0.0, 0.07, 0.28),
            (-0.175, 0.0, 0.07, 0.28),
        )
    ):
        base.visual(
            Box((sx, sy, 0.08)),
            origin=Origin(xyz=(x, y, 0.64)),
            material=dark,
            name=f"post_cap_{idx}",
        )
    # Four welded foot ribs tie the vertical post visibly into the floor base.
    for idx, (x, y, yaw) in enumerate(
        (
            (0.38, 0.0, 0.0),
            (-0.38, 0.0, 0.0),
            (0.0, 0.38, math.pi / 2.0),
            (0.0, -0.38, math.pi / 2.0),
        )
    ):
        base.visual(
            Box((0.46, 0.08, 0.11)),
            origin=Origin(xyz=(x, y, 0.135), rpy=(0.0, 0.0, yaw)),
            material=blue,
            name=f"base_rib_{idx}",
        )

    platform = model.part("platform")
    platform.visual(
        Box((2.65, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark,
        name="wide_deck",
    )
    platform.visual(
        Box((2.46, 0.10, 0.09)),
        origin=Origin(xyz=(0.0, 0.46, 0.18)),
        material=blue,
        name="deck_beam_0",
    )
    platform.visual(
        Box((2.46, 0.10, 0.09)),
        origin=Origin(xyz=(0.0, -0.46, 0.18)),
        material=blue,
        name="deck_beam_1",
    )
    platform.visual(
        Cylinder(radius=0.10, length=0.77),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=chrome,
        name="piston_rod",
    )
    platform.visual(
        Cylinder(radius=0.20, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=blue,
        name="ram_saddle",
    )
    platform.visual(
        Box((2.55, 0.08, 0.085)),
        origin=Origin(xyz=(0.0, 0.63, 0.3025)),
        material=blue,
        name="hinge_mount_0",
    )
    platform.visual(
        Box((2.55, 0.08, 0.085)),
        origin=Origin(xyz=(0.0, -0.63, 0.3025)),
        material=blue,
        name="hinge_mount_1",
    )
    # Raised non-slip strips make the broad deck read as a vehicle platform.
    for idx, x in enumerate((-0.95, -0.55, -0.15, 0.25, 0.65, 1.05)):
        platform.visual(
            Box((0.05, 1.04, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.346)),
            material=rubber,
            name=f"grip_strip_{idx}",
        )

    for idx, side in enumerate((1.0, -1.0)):
        arm = model.part(f"side_arm_{idx}")
        arm.visual(
            Cylinder(radius=0.035, length=2.34),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="hinge_barrel",
        )
        arm.visual(
            Box((2.30, 0.06, 0.02)),
            origin=Origin(xyz=(0.0, -side * 0.030, -0.025)),
            material=dark,
            name="hinge_leaf",
        )
        arm.visual(
            Box((2.36, 0.16, 0.06)),
            origin=Origin(xyz=(0.0, side * 0.095, 0.030)),
            material=yellow,
            name="folding_arm",
        )
        arm.visual(
            Box((0.16, 0.22, 0.075)),
            origin=Origin(xyz=(-1.02, side * 0.13, 0.038)),
            material=yellow,
            name="end_lug_0",
        )
        arm.visual(
            Box((0.16, 0.22, 0.075)),
            origin=Origin(xyz=(1.02, side * 0.13, 0.038)),
            material=yellow,
            name="end_lug_1",
        )
        model.articulation(
            f"platform_to_arm_{idx}",
            ArticulationType.REVOLUTE,
            parent=platform,
            child=arm,
            origin=Origin(xyz=(0.0, side * 0.705, 0.380)),
            axis=(side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0),
        )

    model.articulation(
        "base_to_platform",
        ArticulationType.PRISMATIC,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.18, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    arm_0 = object_model.get_part("side_arm_0")
    arm_1 = object_model.get_part("side_arm_1")
    lift = object_model.get_articulation("base_to_platform")
    arm_hinge_0 = object_model.get_articulation("platform_to_arm_0")
    arm_hinge_1 = object_model.get_articulation("platform_to_arm_1")

    ctx.allow_overlap(
        base,
        platform,
        elem_a="outer_post",
        elem_b="piston_rod",
        reason="The chrome piston rod is intentionally represented as sliding inside the simplified solid hydraulic sleeve.",
    )

    ctx.expect_within(
        platform,
        base,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="outer_post",
        margin=0.0,
        name="piston rod remains centered in the hydraulic post",
    )
    ctx.expect_overlap(
        platform,
        base,
        axes="z",
        elem_a="piston_rod",
        elem_b="outer_post",
        min_overlap=0.45,
        name="lowered piston stays deeply inserted in post",
    )

    low_position = ctx.part_world_position(platform)
    with ctx.pose({lift: 0.45}):
        ctx.expect_within(
            platform,
            base,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="outer_post",
            margin=0.0,
            name="raised piston remains centered in the post",
        )
        ctx.expect_overlap(
            platform,
            base,
            axes="z",
            elem_a="piston_rod",
            elem_b="outer_post",
            min_overlap=0.06,
            name="raised piston retains insertion in post",
        )
        high_position = ctx.part_world_position(platform)

    ctx.check(
        "platform rises vertically on the central post",
        low_position is not None
        and high_position is not None
        and high_position[2] > low_position[2] + 0.40,
        details=f"low={low_position}, high={high_position}",
    )

    arm_0_low = ctx.part_world_aabb(arm_0)
    arm_1_low = ctx.part_world_aabb(arm_1)
    with ctx.pose({arm_hinge_0: math.pi / 2.0, arm_hinge_1: math.pi / 2.0}):
        arm_0_folded = ctx.part_world_aabb(arm_0)
        arm_1_folded = ctx.part_world_aabb(arm_1)

    ctx.check(
        "side safety arms fold upward on platform-edge hinges",
        arm_0_low is not None
        and arm_1_low is not None
        and arm_0_folded is not None
        and arm_1_folded is not None
        and arm_0_folded[1][2] > arm_0_low[1][2] + 0.12
        and arm_1_folded[1][2] > arm_1_low[1][2] + 0.12,
        details=f"arm0_low={arm_0_low}, arm0_folded={arm_0_folded}, arm1_low={arm_1_low}, arm1_folded={arm_1_folded}",
    )

    return ctx.report()


object_model = build_object_model()
