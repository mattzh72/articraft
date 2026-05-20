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
    model = ArticulatedObject(name="xy_stage")

    # Base
    base = model.part("base")
    base.visual(
        Box((0.40, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="base_plate",
    )
    base.visual(
        Box((0.40, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, -0.06, 0.05)),
        name="x_rail_1",
    )
    base.visual(
        Box((0.40, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.06, 0.05)),
        name="x_rail_2",
    )

    # X Carriage
    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.15, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, -0.06, 0.07)),
        name="x_slider_1",
    )
    x_carriage.visual(
        Box((0.15, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.06, 0.07)),
        name="x_slider_2",
    )
    x_carriage.visual(
        Box((0.15, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        name="x_plate",
    )
    x_carriage.visual(
        Box((0.02, 0.20, 0.02)),
        origin=Origin(xyz=(-0.05, 0.0, 0.11)),
        name="y_rail_1",
    )
    x_carriage.visual(
        Box((0.02, 0.20, 0.02)),
        origin=Origin(xyz=(0.05, 0.0, 0.11)),
        name="y_rail_2",
    )

    # Y Table
    y_table = model.part("y_table")
    y_table.visual(
        Box((0.04, 0.10, 0.02)),
        origin=Origin(xyz=(-0.05, 0.0, 0.13)),
        name="y_slider_1",
    )
    y_table.visual(
        Box((0.04, 0.10, 0.02)),
        origin=Origin(xyz=(0.05, 0.0, 0.13)),
        name="y_slider_2",
    )
    y_table.visual(
        Box((0.15, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        name="y_plate",
    )

    # Articulations
    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.125, upper=0.125),
    )

    model.articulation(
        "x_carriage_to_y_table",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.05, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_table = object_model.get_part("y_table")
    
    # Check initial positions
    ctx.expect_contact(x_carriage, base, elem_a="x_slider_1", elem_b="x_rail_1")
    ctx.expect_contact(x_carriage, base, elem_a="x_slider_2", elem_b="x_rail_2")
    
    ctx.expect_contact(y_table, x_carriage, elem_a="y_slider_1", elem_b="y_rail_1")
    ctx.expect_contact(y_table, x_carriage, elem_a="y_slider_2", elem_b="y_rail_2")

    # Check overlaps to ensure sliders stay on rails
    ctx.expect_overlap(x_carriage, base, axes="xy", elem_a="x_slider_1", elem_b="x_rail_1")
    ctx.expect_overlap(y_table, x_carriage, axes="xy", elem_a="y_slider_1", elem_b="y_rail_1")

    # Check poses
    x_joint = object_model.get_articulation("base_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_table")
    
    with ctx.pose({x_joint: 0.125, y_joint: 0.05}):
        ctx.expect_overlap(x_carriage, base, axes="xy", elem_a="x_slider_1", elem_b="x_rail_1")
        ctx.expect_overlap(y_table, x_carriage, axes="xy", elem_a="y_slider_1", elem_b="y_rail_1")

    return ctx.report()


object_model = build_object_model()
