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
    model = ArticulatedObject(name="laptop_stand")

    base = model.part("base")
    base.visual(Box((0.26, 0.22, 0.005)), origin=Origin(xyz=(0, 0, 0.0025)), name="base_board")
    base.visual(Box((0.02, 0.02, 0.008)), origin=Origin(xyz=(-0.12, 0.10, 0.004)), name="base_hinge_left")
    base.visual(Box((0.02, 0.02, 0.008)), origin=Origin(xyz=(0.12, 0.10, 0.004)), name="base_hinge_right")
    base.visual(Box((0.01, 0.02, 0.006)), origin=Origin(xyz=(-0.02, 0.05, 0.008)), name="base_prop_bracket_left")
    base.visual(Box((0.01, 0.02, 0.006)), origin=Origin(xyz=(0.02, 0.05, 0.008)), name="base_prop_bracket_right")

    platform = model.part("platform")
    platform.visual(Box((0.26, 0.22, 0.005)), origin=Origin(xyz=(0, -0.10, 0.0055)), name="platform_board")
    platform.visual(Box((0.22, 0.02, 0.008)), origin=Origin(xyz=(0, 0, 0.004)), name="platform_hinge_center")
    platform.visual(Box((0.01, 0.02, 0.006)), origin=Origin(xyz=(-0.025, -0.15, 0)), name="platform_prop_bracket_left")
    platform.visual(Box((0.01, 0.02, 0.006)), origin=Origin(xyz=(0.025, -0.15, 0)), name="platform_prop_bracket_right")

    prop_lower = model.part("prop_lower")
    prop_lower.visual(Box((0.03, 0.07, 0.004)), origin=Origin(xyz=(0, -0.025, 0)), name="prop_lower_tube")
    prop_lower.visual(Box((0.05, 0.004, 0.004)), origin=Origin(xyz=(0, 0, 0)), name="prop_lower_pin")

    prop_upper = model.part("prop_upper")
    prop_upper.visual(Box((0.02, 0.09, 0.003)), origin=Origin(xyz=(0, -0.005, 0)), name="prop_upper_rod")
    prop_upper.visual(Box((0.06, 0.004, 0.004)), origin=Origin(xyz=(0, -0.05, 0)), name="prop_upper_pin")

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0, 0.10, 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    model.articulation(
        "base_to_prop_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=prop_lower,
        origin=Origin(xyz=(0, 0.05, 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    model.articulation(
        "prop_lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=prop_lower,
        child=prop_upper,
        origin=Origin(xyz=(0, -0.05, 0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.06),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    prop_lower = object_model.get_part("prop_lower")
    prop_upper = object_model.get_part("prop_upper")

    ctx.allow_overlap(prop_lower, prop_upper, elem_a="prop_lower_tube", elem_b="prop_upper_rod", reason="Telescoping proxy")
    ctx.allow_overlap(base, prop_lower, elem_a="base_prop_bracket_left", elem_b="prop_lower_pin", reason="Pin rests in bracket")
    ctx.allow_overlap(base, prop_lower, elem_a="base_prop_bracket_right", elem_b="prop_lower_pin", reason="Pin rests in bracket")
    ctx.allow_overlap(platform, prop_upper, elem_a="platform_prop_bracket_left", elem_b="prop_upper_pin", reason="Pin rests in bracket")
    ctx.allow_overlap(platform, prop_upper, elem_a="platform_prop_bracket_right", elem_b="prop_upper_pin", reason="Pin rests in bracket")

    # At rest, verify the closed loop
    with ctx.pose():
        ctx.expect_overlap(prop_upper, platform, axes="xyz", elem_a="prop_upper_pin", elem_b="platform_prop_bracket_left", name="prop arm is seated in left bracket at rest")
        ctx.expect_overlap(prop_upper, platform, axes="xyz", elem_a="prop_upper_pin", elem_b="platform_prop_bracket_right", name="prop arm is seated in right bracket at rest")
        
    # At an elevated pose, verify the closed loop
    theta = 0.5
    phi = 0.722158
    q_slide = 0.008795
    with ctx.pose({"base_to_platform": theta, "base_to_prop_lower": phi, "prop_lower_to_upper": q_slide}):
        ctx.expect_overlap(prop_upper, platform, axes="xyz", elem_a="prop_upper_pin", elem_b="platform_prop_bracket_left", name="prop arm stays seated in left bracket when raised")
        ctx.expect_overlap(prop_upper, platform, axes="xyz", elem_a="prop_upper_pin", elem_b="platform_prop_bracket_right", name="prop arm stays seated in right bracket when raised")

    return ctx.report()

object_model = build_object_model()
