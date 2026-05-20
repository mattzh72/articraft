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
    model = ArticulatedObject(name="gimbal_joystick")

    base = model.part("base")
    # A box base with open top.
    # Dimensions: 0.15 x 0.15 x 0.10, walls 0.01 thick.
    # Base origin at bottom center (0, 0, 0).
    base.visual(Box((0.15, 0.01, 0.10)), origin=Origin(xyz=(0, -0.07, 0.05)), name="front_wall")
    base.visual(Box((0.15, 0.01, 0.10)), origin=Origin(xyz=(0, 0.07, 0.05)), name="back_wall")
    base.visual(Box((0.01, 0.13, 0.10)), origin=Origin(xyz=(-0.07, 0, 0.05)), name="left_wall")
    base.visual(Box((0.01, 0.13, 0.10)), origin=Origin(xyz=(0.07, 0, 0.05)), name="right_wall")
    base.visual(Box((0.15, 0.15, 0.01)), origin=Origin(xyz=(0, 0, 0.005)), name="bottom_wall")

    outer_yoke = model.part("outer_yoke")
    # Pitch yoke. Rotates on X axis.
    # Center of rotation at Z=0.06.
    # Frame size: 0.12 x 0.12 x 0.02.
    # Arms: Y=-0.05, Y=0.05, X=-0.05, X=0.05.
    outer_yoke.visual(Box((0.12, 0.02, 0.02)), origin=Origin(xyz=(0, -0.05, 0)), name="front_arm")
    outer_yoke.visual(Box((0.12, 0.02, 0.02)), origin=Origin(xyz=(0, 0.05, 0)), name="back_arm")
    outer_yoke.visual(Box((0.02, 0.08, 0.02)), origin=Origin(xyz=(-0.05, 0, 0)), name="left_arm")
    outer_yoke.visual(Box((0.02, 0.08, 0.02)), origin=Origin(xyz=(0.05, 0, 0)), name="right_arm")
    # Pins extending to X=-0.07 and X=0.07. They go from X=0.06 to X=0.07 (length 0.01).
    outer_yoke.visual(Cylinder(radius=0.005, length=0.01), origin=Origin(xyz=(-0.065, 0, 0), rpy=(0, 1.5708, 0)), name="left_pin")
    outer_yoke.visual(Cylinder(radius=0.005, length=0.01), origin=Origin(xyz=(0.065, 0, 0), rpy=(0, 1.5708, 0)), name="right_pin")

    inner_cradle = model.part("inner_cradle")
    # Roll cradle. Rotates on Y axis.
    # Body size: 0.06 x 0.06 x 0.02.
    inner_cradle.visual(Box((0.06, 0.06, 0.02)), origin=Origin(xyz=(0, 0, 0)), name="cradle_body")
    # Pins extending to Y=-0.05 and Y=0.05. Length 0.02.
    inner_cradle.visual(Cylinder(radius=0.005, length=0.02), origin=Origin(xyz=(0, -0.04, 0), rpy=(1.5708, 0, 0)), name="front_pin")
    inner_cradle.visual(Cylinder(radius=0.005, length=0.02), origin=Origin(xyz=(0, 0.04, 0), rpy=(1.5708, 0, 0)), name="back_pin")

    lever = model.part("lever")
    lever.visual(Cylinder(radius=0.008, length=0.08), origin=Origin(xyz=(0, 0, 0.04)), name="stick")
    lever.visual(Box((0.04, 0.04, 0.03)), origin=Origin(xyz=(0, 0, 0.095)), name="cap")

    # Articulations
    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0, 0, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.5, upper=0.5),
    )

    model.articulation(
        "yoke_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.5, upper=0.5),
    )

    model.articulation(
        "cradle_to_lever",
        ArticulationType.FIXED,
        parent=inner_cradle,
        child=lever,
        origin=Origin(xyz=(0, 0, 0.01)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_cradle = object_model.get_part("inner_cradle")
    lever = object_model.get_part("lever")

    # Allow overlaps for the hinge pins
    ctx.allow_overlap(
        outer_yoke,
        base,
        elem_a="left_pin",
        elem_b="left_wall",
        reason="Left pin embeds into the base wall for support.",
    )
    ctx.allow_overlap(
        outer_yoke,
        base,
        elem_a="right_pin",
        elem_b="right_wall",
        reason="Right pin embeds into the base wall for support.",
    )
    ctx.allow_overlap(
        inner_cradle,
        outer_yoke,
        elem_a="front_pin",
        elem_b="front_arm",
        reason="Front pin embeds into the yoke for support.",
    )
    ctx.allow_overlap(
        inner_cradle,
        outer_yoke,
        elem_a="back_pin",
        elem_b="back_arm",
        reason="Back pin embeds into the yoke for support.",
    )

    # Check center alignment
    ctx.expect_within(outer_yoke, base, axes="xy", margin=0.0)
    ctx.expect_within(inner_cradle, outer_yoke, axes="xy", margin=0.0)

    # Check lever placement
    ctx.expect_contact(lever, inner_cradle, elem_a="stick", elem_b="cradle_body")

    return ctx.report()

object_model = build_object_model()