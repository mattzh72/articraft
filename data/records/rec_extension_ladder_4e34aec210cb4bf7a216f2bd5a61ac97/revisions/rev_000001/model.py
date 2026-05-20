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
    model = ArticulatedObject(name="extension_ladder")

    base = model.part("base_section")

    # Base Left Rail (E-channel profile)
    base.visual(Box((0.01, 0.09, 3.0)), origin=Origin((-0.215, 0.0, 1.5)), name="base_left_web")
    base.visual(Box((0.04, 0.01, 3.0)), origin=Origin((-0.19, -0.04, 1.5)), name="base_left_back_flange")
    base.visual(Box((0.04, 0.01, 3.0)), origin=Origin((-0.19, 0.0, 1.5)), name="base_left_mid_flange")
    base.visual(Box((0.015, 0.01, 3.0)), origin=Origin((-0.2025, 0.04, 1.5)), name="base_left_front_flange")

    # Base Right Rail (E-channel profile)
    base.visual(Box((0.01, 0.09, 3.0)), origin=Origin((0.215, 0.0, 1.5)), name="base_right_web")
    base.visual(Box((0.04, 0.01, 3.0)), origin=Origin((0.19, -0.04, 1.5)), name="base_right_back_flange")
    base.visual(Box((0.04, 0.01, 3.0)), origin=Origin((0.19, 0.0, 1.5)), name="base_right_mid_flange")
    base.visual(Box((0.015, 0.01, 3.0)), origin=Origin((0.2025, 0.04, 1.5)), name="base_right_front_flange")

    # Base Feet
    base.visual(Box((0.06, 0.05, 0.05)), origin=Origin((-0.195, -0.02, 0.025)), name="base_left_foot")
    base.visual(Box((0.06, 0.05, 0.05)), origin=Origin((0.195, -0.02, 0.025)), name="base_right_foot")

    # Base Rungs
    for i in range(9):
        z = 0.3 * (i + 1)
        base.visual(
            Cylinder(radius=0.015, length=0.42),
            origin=Origin((0.0, -0.02, z), rpy=(0.0, math.pi / 2, 0.0)),
            name=f"base_rung_{i}"
        )

    fly = model.part("fly_section")

    # Fly Left Rail
    fly.visual(Box((0.03, 0.025, 3.0)), origin=Origin((-0.19, 0.02, 1.5)), name="fly_left_rail")

    # Fly Right Rail
    fly.visual(Box((0.03, 0.025, 3.0)), origin=Origin((0.19, 0.02, 1.5)), name="fly_right_rail")

    # Fly Rungs
    for i in range(9):
        z = 0.3 * (i + 1)
        fly.visual(
            Cylinder(radius=0.0125, length=0.38),
            origin=Origin((0.0, 0.02, z), rpy=(0.0, math.pi / 2, 0.0)),
            name=f"fly_rung_{i}"
        )

    model.articulation(
        "slide_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin((0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.1, effort=100.0, velocity=1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    slide_joint = object_model.get_articulation("slide_joint")

    # The fly section is guided within the base section channels with clearance.
    ctx.allow_isolated_part(fly, reason="The fly section slides within the base channels with realistic clearance.")
    ctx.expect_within(fly, base, axes="x", margin=0.0)
    ctx.expect_within(fly, base, axes="y", margin=0.0)

    # Retained insertion check at rest
    ctx.expect_overlap(fly, base, axes="z", min_overlap=2.9)

    # Extended pose
    with ctx.pose({slide_joint: 2.1}):
        ctx.expect_within(fly, base, axes="xy", margin=0.0)
        ctx.expect_overlap(fly, base, axes="z", min_overlap=0.8)
        
        fly_pos = ctx.part_world_position(fly)
        base_pos = ctx.part_world_position(base)
        if fly_pos is not None and base_pos is not None:
            ctx.check("fly_extends_upward", fly_pos[2] > base_pos[2] + 2.0)

    return ctx.report()

object_model = build_object_model()
