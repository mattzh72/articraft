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
    model = ArticulatedObject(name="xyz_cartesian_stage")

    # Colors
    color_rail = (0.7, 0.7, 0.75, 1.0)
    color_carriage = (0.2, 0.2, 0.2, 1.0)
    color_standoff = (0.1, 0.4, 0.8, 1.0)
    color_plate = (0.8, 0.8, 0.8, 1.0)

    # 1. Base (fixed X-rail)
    base = model.part("base")
    base.visual(
        Box((0.6, 0.1, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="x_rail",
        color=color_rail,
    )

    # 2. X-stage
    x_stage = model.part("x_stage")
    x_stage.visual(
        Box((0.15, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="x_carriage",
        color=color_carriage,
    )
    x_stage.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        name="x_standoff",
        color=color_standoff,
    )
    x_stage.visual(
        Box((0.1, 0.5, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        name="y_rail",
        color=color_rail,
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.2, upper=0.2),
    )

    # 3. Y-stage
    y_stage = model.part("y_stage")
    y_stage.visual(
        Box((0.15, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="y_carriage",
        color=color_carriage,
    )
    y_stage.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        name="y_standoff",
        color=color_standoff,
    )
    y_stage.visual(
        Box((0.1, 0.05, 0.4)),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        name="z_rail",
        color=color_rail,
    )

    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    # 4. Z-stage
    z_stage = model.part("z_stage")
    z_stage.visual(
        Box((0.15, 0.05, 0.15)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        name="z_carriage",
        color=color_carriage,
    )
    z_stage.visual(
        Box((0.08, 0.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        name="z_standoff",
        color=color_standoff,
    )
    z_stage.visual(
        Box((0.2, 0.01, 0.2)),
        origin=Origin(xyz=(0.0, 0.075, 0.0)),
        name="end_effector",
        color=color_plate,
    )

    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, 0.025, 0.27)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.1, upper=0.1),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    z_stage = object_model.get_part("z_stage")

    # X-axis
    ctx.expect_contact(x_stage, base, elem_a="x_carriage", elem_b="x_rail")
    ctx.expect_within(x_stage, base, axes="y", inner_elem="x_carriage", outer_elem="x_rail", margin=0.03)

    # Y-axis
    ctx.expect_contact(y_stage, x_stage, elem_a="y_carriage", elem_b="y_rail")
    ctx.expect_within(y_stage, x_stage, axes="x", inner_elem="y_carriage", outer_elem="y_rail", margin=0.03)

    # Z-axis
    ctx.expect_contact(z_stage, y_stage, elem_a="z_carriage", elem_b="z_rail")
    ctx.expect_within(z_stage, y_stage, axes="x", inner_elem="z_carriage", outer_elem="z_rail", margin=0.03)

    # Test motion bounds
    with ctx.pose(x_axis=0.2):
        ctx.expect_overlap(x_stage, base, axes="x", elem_a="x_carriage", elem_b="x_rail", min_overlap=0.05)
    
    with ctx.pose(y_axis=0.15):
        ctx.expect_overlap(y_stage, x_stage, axes="y", elem_a="y_carriage", elem_b="y_rail", min_overlap=0.05)
    
    with ctx.pose(z_axis=0.1):
        ctx.expect_overlap(z_stage, y_stage, axes="z", elem_a="z_carriage", elem_b="z_rail", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()
