from __future__ import annotations

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
    model = ArticulatedObject(name="bench_positioning_axis")

    dark_iron = model.material("dark_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    blue_carriage = model.material("blue_carriage", rgba=(0.05, 0.20, 0.55, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    orange_lift = model.material("orange_lift", rgba=(0.95, 0.42, 0.08, 1.0))

    bench_base = model.part("bench_base")
    bench_base.visual(
        Box((1.10, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_iron,
        name="bench_plate",
    )
    bench_base.visual(
        Box((1.00, 0.026, 0.047)),
        origin=Origin(xyz=(0.0, -0.0675, 0.0565)),
        material=brushed_steel,
        name="x_rail_0",
    )
    bench_base.visual(
        Box((1.00, 0.026, 0.047)),
        origin=Origin(xyz=(0.0, 0.0675, 0.0565)),
        material=brushed_steel,
        name="x_rail_1",
    )
    bench_base.visual(
        Box((0.035, 0.22, 0.12)),
        origin=Origin(xyz=(-0.52, 0.0, 0.095)),
        material=dark_iron,
        name="end_stop_0",
    )
    bench_base.visual(
        Box((0.035, 0.22, 0.12)),
        origin=Origin(xyz=(0.52, 0.0, 0.095)),
        material=dark_iron,
        name="end_stop_1",
    )
    bench_base.visual(
        Box((0.90, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, -0.1135, 0.035)),
        material=brushed_steel,
        name="position_scale",
    )

    base_carriage = model.part("base_carriage")
    base_carriage.visual(
        Box((0.18, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.0675, 0.0175)),
        material=dark_iron,
        name="bearing_shoe_0",
    )
    base_carriage.visual(
        Box((0.18, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.0675, 0.0175)),
        material=dark_iron,
        name="bearing_shoe_1",
    )
    base_carriage.visual(
        Box((0.24, 0.18, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=blue_carriage,
        name="saddle_plate",
    )
    base_carriage.visual(
        Box((0.13, 0.13, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=blue_carriage,
        name="column_foot",
    )
    base_carriage.visual(
        Box((0.075, 0.075, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=dark_iron,
        name="fixed_column",
    )
    base_carriage.visual(
        Box((0.018, 0.012, 0.560)),
        origin=Origin(xyz=(-0.030, 0.043, 0.405)),
        material=brushed_steel,
        name="front_rail_0",
    )
    base_carriage.visual(
        Box((0.018, 0.012, 0.560)),
        origin=Origin(xyz=(0.030, 0.043, 0.405)),
        material=brushed_steel,
        name="front_rail_1",
    )
    base_carriage.visual(
        Box((0.12, 0.012, 0.170)),
        origin=Origin(xyz=(0.0, 0.047, 0.185)),
        material=blue_carriage,
        name="front_gusset",
    )

    column_carriage = model.part("column_carriage")
    column_carriage.visual(
        Box((0.13, 0.035, 0.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange_lift,
        name="slider_block",
    )
    column_carriage.visual(
        Box((0.048, 0.035, 0.720)),
        origin=Origin(xyz=(0.0, 0.006, 0.290)),
        material=orange_lift,
        name="lifted_column",
    )
    column_carriage.visual(
        Cylinder(radius=0.075, length=0.038),
        origin=Origin(xyz=(0.0, 0.006, 0.666)),
        material=black_rubber,
        name="head_pad",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=bench_base,
        child=base_carriage,
        origin=Origin(xyz=(-0.28, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=base_carriage,
        child=column_carriage,
        origin=Origin(xyz=(0.0, 0.0665, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench_base = object_model.get_part("bench_base")
    base_carriage = object_model.get_part("base_carriage")
    column_carriage = object_model.get_part("column_carriage")
    x_slide = object_model.get_articulation("x_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.check(
        "two orthogonal prismatic axes",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and x_slide.axis == (1.0, 0.0, 0.0)
        and z_slide.axis == (0.0, 0.0, 1.0),
        details=f"x={x_slide.axis}, z={z_slide.axis}",
    )
    ctx.expect_gap(
        base_carriage,
        bench_base,
        axis="z",
        positive_elem="bearing_shoe_1",
        negative_elem="x_rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="base carriage sits on X rail",
    )
    ctx.expect_gap(
        column_carriage,
        base_carriage,
        axis="y",
        positive_elem="slider_block",
        negative_elem="front_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="Z carriage bears on column rail",
    )
    ctx.expect_overlap(
        column_carriage,
        base_carriage,
        axes="xz",
        elem_a="slider_block",
        elem_b="front_rail_0",
        min_overlap=0.015,
        name="Z carriage remains engaged on guide",
    )

    rest_x = ctx.part_world_position(base_carriage)
    rest_z = ctx.part_world_position(column_carriage)
    with ctx.pose({x_slide: 0.50, z_slide: 0.34}):
        moved_x = ctx.part_world_position(base_carriage)
        moved_z = ctx.part_world_position(column_carriage)
        ctx.expect_overlap(
            base_carriage,
            bench_base,
            axes="xy",
            elem_a="bearing_shoe_1",
            elem_b="x_rail_1",
            min_overlap=0.020,
            name="base carriage stays captured at X travel",
        )
        ctx.expect_overlap(
            column_carriage,
            base_carriage,
            axes="xz",
            elem_a="slider_block",
            elem_b="front_rail_0",
            min_overlap=0.015,
            name="Z carriage stays captured at full lift",
        )
    ctx.check(
        "X slide translates carriage along bench",
        rest_x is not None and moved_x is not None and moved_x[0] > rest_x[0] + 0.45,
        details=f"rest={rest_x}, moved={moved_x}",
    )
    ctx.check(
        "Z slide lifts head pad",
        rest_z is not None and moved_z is not None and moved_z[2] > rest_z[2] + 0.30,
        details=f"rest={rest_z}, moved={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
