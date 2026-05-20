from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="cnc_table_drill_press")

    cast = model.material("dark_cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    machined = model.material("machined_steel", rgba=(0.63, 0.65, 0.64, 1.0))
    rail = model.material("polished_rail_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    table_mat = model.material("oiled_table", rgba=(0.18, 0.21, 0.22, 1.0))
    black = model.material("black_slot_shadow", rgba=(0.015, 0.017, 0.018, 1.0))
    motor = model.material("black_motor", rgba=(0.03, 0.035, 0.04, 1.0))
    brass = model.material("brass_oiler", rgba=(0.85, 0.60, 0.22, 1.0))

    frame = model.part("machine_frame")
    frame.visual(
        Box((1.15, 0.70, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.04)),
        material=cast,
        name="base_foot",
    )
    frame.visual(
        Cylinder(radius=0.060, length=1.16),
        origin=Origin(xyz=(-0.35, 0.0, 0.655)),
        material=cast,
        name="round_column",
    )
    frame.visual(
        Cylinder(radius=0.115, length=0.045),
        origin=Origin(xyz=(-0.35, 0.0, 0.102)),
        material=cast,
        name="column_flange",
    )
    frame.visual(
        Box((0.58, 0.18, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, 1.10)),
        material=cast,
        name="head_arm",
    )
    frame.visual(
        Box((0.25, 0.24, 0.26)),
        origin=Origin(xyz=(0.22, 0.0, 1.085)),
        material=cast,
        name="head_housing",
    )
    frame.visual(
        Cylinder(radius=0.068, length=0.055),
        origin=Origin(xyz=(0.22, 0.0, 0.9275)),
        material=cast,
        name="quill_collar",
    )
    frame.visual(
        Box((0.16, 0.16, 0.09)),
        origin=Origin(xyz=(0.13, 0.0, 1.26)),
        material=motor,
        name="head_motor_box",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.16),
        origin=Origin(xyz=(0.23, 0.0, 1.26), rpy=(0.0, pi / 2.0, 0.0)),
        material=motor,
        name="head_motor_cap",
    )

    frame.visual(
        Box((0.96, 0.040, 0.055)),
        origin=Origin(xyz=(0.07, -0.18, 0.106)),
        material=cast,
        name="x_rail_pedestal_0",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.96),
        origin=Origin(xyz=(0.07, -0.18, 0.133), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail,
        name="x_rail_0",
    )
    frame.visual(
        Box((0.96, 0.040, 0.055)),
        origin=Origin(xyz=(0.07, 0.18, 0.106)),
        material=cast,
        name="x_rail_pedestal_1",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.96),
        origin=Origin(xyz=(0.07, 0.18, 0.133), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail,
        name="x_rail_1",
    )

    frame.visual(
        Box((0.10, 0.12, 0.10)),
        origin=Origin(xyz=(0.64, -0.31, 0.135)),
        material=motor,
        name="x_motor_box",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.09),
        origin=Origin(xyz=(0.575, -0.31, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="x_coupler",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.83),
        origin=Origin(xyz=(0.13, -0.31, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="x_leadscrew",
    )
    frame.visual(
        Box((0.035, 0.060, 0.065)),
        origin=Origin(xyz=(-0.29, -0.31, 0.112)),
        material=cast,
        name="x_screw_bearing",
    )

    x_saddle = model.part("x_saddle")
    x_saddle.visual(
        Box((0.48, 0.54, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=table_mat,
        name="x_cross_slide",
    )
    x_saddle.visual(
        Box((0.38, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, -0.18, 0.0155)),
        material=machined,
        name="x_bearing_0",
    )
    x_saddle.visual(
        Box((0.38, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.18, 0.0155)),
        material=machined,
        name="x_bearing_1",
    )
    x_saddle.visual(
        Box((0.040, 0.60, 0.032)),
        origin=Origin(xyz=(-0.15, 0.0, 0.071)),
        material=cast,
        name="y_rail_pedestal_0",
    )
    x_saddle.visual(
        Cylinder(radius=0.018, length=0.60),
        origin=Origin(xyz=(-0.15, 0.0, 0.095), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="y_rail_0",
    )
    x_saddle.visual(
        Box((0.040, 0.60, 0.032)),
        origin=Origin(xyz=(0.15, 0.0, 0.071)),
        material=cast,
        name="y_rail_pedestal_1",
    )
    x_saddle.visual(
        Cylinder(radius=0.018, length=0.60),
        origin=Origin(xyz=(0.15, 0.0, 0.095), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="y_rail_1",
    )
    x_saddle.visual(
        Box((0.12, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.30, 0.075)),
        material=cast,
        name="y_motor_bracket",
    )
    x_saddle.visual(
        Box((0.11, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.39, 0.105)),
        material=motor,
        name="y_motor_box",
    )
    x_saddle.visual(
        Cylinder(radius=0.010, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.092), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="y_leadscrew",
    )

    y_table = model.part("y_table")
    y_table.visual(
        Box((0.060, 0.30, 0.025)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0125)),
        material=machined,
        name="y_bearing_0",
    )
    y_table.visual(
        Box((0.060, 0.30, 0.025)),
        origin=Origin(xyz=(0.15, 0.0, 0.0125)),
        material=machined,
        name="y_bearing_1",
    )
    y_table.visual(
        Box((0.42, 0.38, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=table_mat,
        name="top_table",
    )
    for idx, x in enumerate((-0.105, 0.0, 0.105)):
        y_table.visual(
            Box((0.026, 0.34, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.079)),
            material=black,
            name=f"t_slot_{idx}",
        )
    y_table.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.175, -0.155, 0.086)),
        material=brass,
        name="fixture_boss",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=machined,
        name="quill_sleeve",
    )
    quill.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=machined,
        name="spindle",
    )
    quill.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=black,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=rail,
        name="drill_bit",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=x_saddle,
        origin=Origin(xyz=(0.08, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.12, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.10, lower=-0.13, upper=0.13),
    )
    model.articulation(
        "quill_drop",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=quill,
        origin=Origin(xyz=(0.22, 0.0, 0.900)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.08, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("machine_frame")
    x_saddle = object_model.get_part("x_saddle")
    y_table = object_model.get_part("y_table")
    quill = object_model.get_part("quill")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    quill_drop = object_model.get_articulation("quill_drop")

    ctx.expect_contact(
        x_saddle,
        frame,
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        contact_tol=0.001,
        name="x saddle bearing sits on X rail",
    )
    ctx.expect_contact(
        y_table,
        x_saddle,
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        contact_tol=0.001,
        name="Y table bearing sits on Y rail",
    )
    ctx.expect_contact(
        quill,
        frame,
        elem_a="quill_sleeve",
        elem_b="quill_collar",
        contact_tol=0.001,
        name="quill sleeve seats in head collar",
    )

    rest_x = ctx.part_world_position(x_saddle)
    with ctx.pose({x_axis: 0.16}):
        moved_x = ctx.part_world_position(x_saddle)
        ctx.expect_overlap(
            x_saddle,
            frame,
            axes="x",
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            min_overlap=0.20,
            name="X bearing remains retained on rail at travel",
        )
    ctx.check(
        "X axis moves along positive X",
        rest_x is not None and moved_x is not None and moved_x[0] > rest_x[0] + 0.12,
        details=f"rest={rest_x}, moved={moved_x}",
    )

    rest_y = ctx.part_world_position(y_table)
    with ctx.pose({y_axis: 0.11}):
        moved_y = ctx.part_world_position(y_table)
        ctx.expect_overlap(
            y_table,
            x_saddle,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.16,
            name="Y bearing remains retained on rail at travel",
        )
    ctx.check(
        "Y axis moves along positive Y",
        rest_y is not None and moved_y is not None and moved_y[1] > rest_y[1] + 0.08,
        details=f"rest={rest_y}, moved={moved_y}",
    )

    rest_quill = ctx.part_world_position(quill)
    with ctx.pose({quill_drop: 0.16}):
        lowered_quill = ctx.part_world_position(quill)
    ctx.check(
        "quill drops downward",
        rest_quill is not None
        and lowered_quill is not None
        and lowered_quill[2] < rest_quill[2] - 0.12,
        details=f"rest={rest_quill}, lowered={lowered_quill}",
    )

    return ctx.report()


object_model = build_object_model()
