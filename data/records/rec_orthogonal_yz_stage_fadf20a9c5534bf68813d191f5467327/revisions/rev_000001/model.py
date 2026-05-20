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
    model = ArticulatedObject(name="yz_stage")

    # Base
    base = model.part("base")
    base.visual(
        Box((0.02, 0.30, 0.12)),
        origin=Origin(xyz=(-0.01, 0.0, 0.20)),
        name="back_plate",
    )
    base.visual(
        Box((0.02, 0.28, 0.04)),
        origin=Origin(xyz=(0.01, 0.0, 0.20)),
        name="y_rail",
    )

    # Y Stage
    y_stage = model.part("y_stage")
    y_stage.visual(
        Box((0.03, 0.10, 0.12)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        name="y_carriage_base",
    )
    y_stage.visual(
        Box((0.02, 0.10, 0.04)),
        origin=Origin(xyz=(-0.01, 0.0, 0.04)),
        name="y_carriage_top",
    )
    y_stage.visual(
        Box((0.02, 0.10, 0.04)),
        origin=Origin(xyz=(-0.01, 0.0, -0.04)),
        name="y_carriage_bottom",
    )
    y_stage.visual(
        Box((0.02, 0.04, 0.20)),
        origin=Origin(xyz=(0.04, 0.0, -0.05)),
        name="z_rail",
    )

    # Z Stage
    z_stage = model.part("z_stage")
    z_stage.visual(
        Box((0.03, 0.08, 0.08)),
        origin=Origin(xyz=(0.015, 0.0, -0.03)),
        name="z_carriage_base",
    )
    z_stage.visual(
        Box((0.02, 0.02, 0.08)),
        origin=Origin(xyz=(-0.01, 0.03, -0.03)),
        name="z_carriage_left",
    )
    z_stage.visual(
        Box((0.02, 0.02, 0.08)),
        origin=Origin(xyz=(-0.01, -0.03, -0.03)),
        name="z_carriage_right",
    )
    z_stage.visual(
        Box((0.04, 0.04, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, -0.07)),
        name="tool_head",
    )

    # Joints
    model.articulation(
        "y_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_stage,
        origin=Origin(xyz=(0.02, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.08, upper=0.08),
    )

    model.articulation(
        "z_joint",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(0.05, 0.0, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.03, upper=0.09),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    y_stage = object_model.get_part("y_stage")
    z_stage = object_model.get_part("z_stage")

    # Verify Y carriage wraps Y rail
    ctx.expect_contact(y_stage, base, elem_a="y_carriage_base", elem_b="y_rail")
    ctx.expect_contact(y_stage, base, elem_a="y_carriage_top", elem_b="y_rail")
    ctx.expect_contact(y_stage, base, elem_a="y_carriage_bottom", elem_b="y_rail")

    # Verify Z carriage wraps Z rail
    ctx.expect_contact(z_stage, y_stage, elem_a="z_carriage_base", elem_b="z_rail")
    ctx.expect_contact(z_stage, y_stage, elem_a="z_carriage_left", elem_b="z_rail")
    ctx.expect_contact(z_stage, y_stage, elem_a="z_carriage_right", elem_b="z_rail")

    return ctx.report()

object_model = build_object_model()