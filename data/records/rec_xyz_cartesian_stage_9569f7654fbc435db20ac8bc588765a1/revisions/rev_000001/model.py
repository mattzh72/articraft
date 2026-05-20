from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    milled = Material("milled_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    rail = Material("hardened_steel_rails", rgba=(0.36, 0.38, 0.40, 1.0))
    stop = Material("black_end_stops", rgba=(0.02, 0.02, 0.018, 1.0))
    accent = Material("blue_axis_covers", rgba=(0.05, 0.20, 0.85, 1.0))

    # Fixed machine base with a pair of long X-axis linear rails.
    base = model.part("base")
    base.visual(Box((0.50, 0.32, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=anodized, name="base_plate")
    base.visual(Box((0.40, 0.026, 0.022)), origin=Origin(xyz=(0.0, -0.09, 0.046)), material=rail, name="x_rail_neg")
    base.visual(Box((0.40, 0.026, 0.022)), origin=Origin(xyz=(0.0, 0.09, 0.046)), material=rail, name="x_rail_pos")
    for x in (-0.225, 0.225):
        base.visual(Box((0.025, 0.24, 0.030)), origin=Origin(xyz=(x, 0.0, 0.050)), material=stop, name=f"x_stop_{'neg' if x < 0 else 'pos'}")

    # X carriage frame: bearing shoes sit on the two X rails and carry the next stage.
    x_carriage = model.part("x_carriage")
    x_carriage.visual(Box((0.20, 0.22, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0385)), material=milled, name="x_cross_plate")
    x_carriage.visual(Box((0.070, 0.046, 0.026)), origin=Origin(xyz=(-0.065, -0.09, 0.013)), material=stop, name="x_bearing_neg_neg")
    x_carriage.visual(Box((0.070, 0.046, 0.026)), origin=Origin(xyz=(-0.065, 0.09, 0.013)), material=stop, name="x_bearing_neg_pos")
    x_carriage.visual(Box((0.070, 0.046, 0.026)), origin=Origin(xyz=(0.065, -0.09, 0.013)), material=stop, name="x_bearing_pos_neg")
    x_carriage.visual(Box((0.070, 0.046, 0.026)), origin=Origin(xyz=(0.065, 0.09, 0.013)), material=stop, name="x_bearing_pos_pos")
    x_carriage.visual(Box((0.070, 0.025, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.056)), material=accent, name="x_axis_cover")

    # Y stage is turned 90 degrees: its visible guide rails run along Y.
    y_stage = model.part("y_stage")
    y_stage.visual(Box((0.22, 0.18, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0385)), material=milled, name="y_carriage_plate")
    y_stage.visual(Box((0.020, 0.205, 0.015)), origin=Origin(xyz=(-0.070, 0.0, 0.0585)), material=rail, name="y_rail_neg")
    y_stage.visual(Box((0.020, 0.205, 0.015)), origin=Origin(xyz=(0.070, 0.0, 0.0585)), material=rail, name="y_rail_pos")
    y_stage.visual(Box((0.046, 0.070, 0.026)), origin=Origin(xyz=(-0.065, -0.050, 0.013)), material=stop, name="y_bearing_neg_neg")
    y_stage.visual(Box((0.046, 0.070, 0.026)), origin=Origin(xyz=(-0.065, 0.050, 0.013)), material=stop, name="y_bearing_neg_pos")
    y_stage.visual(Box((0.046, 0.070, 0.026)), origin=Origin(xyz=(0.065, -0.050, 0.013)), material=stop, name="y_bearing_pos_neg")
    y_stage.visual(Box((0.046, 0.070, 0.026)), origin=Origin(xyz=(0.065, 0.050, 0.013)), material=stop, name="y_bearing_pos_pos")
    y_stage.visual(Box((0.160, 0.020, 0.010)), origin=Origin(xyz=(0.0, -0.0925, 0.071)), material=accent, name="y_axis_cover")

    # Stationary vertical column carried by the Y stage.
    z_column = model.part("z_column")
    z_column.visual(Box((0.16, 0.16, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=anodized, name="column_foot")
    z_column.visual(Box((0.12, 0.030, 0.220)), origin=Origin(xyz=(0.0, 0.065, 0.110)), material=milled, name="vertical_backbone")
    z_column.visual(Box((0.018, 0.012, 0.180)), origin=Origin(xyz=(-0.045, 0.044, 0.115)), material=rail, name="z_rail_neg")
    z_column.visual(Box((0.018, 0.012, 0.180)), origin=Origin(xyz=(0.045, 0.044, 0.115)), material=rail, name="z_rail_pos")
    z_column.visual(Box((0.14, 0.035, 0.025)), origin=Origin(xyz=(0.0, 0.055, 0.2325)), material=stop, name="z_top_stop")

    # The vertical moving slide with a small precision top table.
    top_table = model.part("top_table")
    top_table.visual(Box((0.13, 0.012, 0.100)), origin=Origin(xyz=(0.0, 0.024, 0.050)), material=stop, name="z_slide_plate")
    top_table.visual(Box((0.026, 0.008, 0.045)), origin=Origin(xyz=(-0.045, 0.034, 0.060)), material=rail, name="z_bearing_neg")
    top_table.visual(Box((0.026, 0.008, 0.045)), origin=Origin(xyz=(0.045, 0.034, 0.060)), material=rail, name="z_bearing_pos")
    top_table.visual(Box((0.16, 0.12, 0.018)), origin=Origin(xyz=(0.0, -0.030, 0.109)), material=milled, name="top_table_plate")
    top_table.visual(Box((0.09, 0.035, 0.020)), origin=Origin(xyz=(0.0, -0.002, 0.100)), material=milled, name="table_neck")

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=-0.075, upper=0.075),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.20, lower=-0.075, upper=0.075),
    )
    model.articulation(
        "y_to_column",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_column,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
    )
    model.articulation(
        "column_to_table",
        ArticulationType.PRISMATIC,
        parent=z_column,
        child=top_table,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_stage = object_model.get_part("y_stage")
    z_column = object_model.get_part("z_column")
    top_table = object_model.get_part("top_table")
    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("column_to_table")

    ctx.expect_gap(x_carriage, base, axis="z", min_gap=0.0, max_gap=0.001, positive_elem="x_bearing_neg_neg", negative_elem="x_rail_neg", name="x bearings sit on x rails")
    ctx.expect_gap(y_stage, x_carriage, axis="z", min_gap=0.0, max_gap=0.001, positive_elem="y_bearing_neg_neg", negative_elem="x_cross_plate", name="y stage is stacked on x carriage")
    ctx.expect_gap(z_column, y_stage, axis="z", max_gap=0.001, max_penetration=0.000001, positive_elem="column_foot", negative_elem="y_rail_neg", name="z column is carried by y stage")
    ctx.expect_contact(top_table, z_column, elem_a="z_bearing_neg", elem_b="z_rail_neg", contact_tol=0.001, name="z slide bearings run on z rails")

    ctx.check("x travel is 150 mm", abs((x_joint.motion_limits.upper - x_joint.motion_limits.lower) - 0.150) < 1e-6)
    ctx.check("y travel is 150 mm", abs((y_joint.motion_limits.upper - y_joint.motion_limits.lower) - 0.150) < 1e-6)
    ctx.check("z travel is 120 mm", abs((z_joint.motion_limits.upper - z_joint.motion_limits.lower) - 0.120) < 1e-6)

    rest_table = ctx.part_world_position(top_table)
    with ctx.pose({x_joint: 0.075, y_joint: 0.075, z_joint: 0.120}):
        extended_table = ctx.part_world_position(top_table)
    ctx.check(
        "orthogonal xyz motion directions",
        rest_table is not None
        and extended_table is not None
        and extended_table[0] > rest_table[0] + 0.070
        and extended_table[1] > rest_table[1] + 0.070
        and extended_table[2] > rest_table[2] + 0.115,
        details=f"rest={rest_table}, extended={extended_table}",
    )

    return ctx.report()


object_model = build_object_model()
